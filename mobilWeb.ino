#include <PID_v1.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define ENCODER_A 32 
#define ENCODER_B 33
#define PPR 600.00
#define CIR 29.85


double target = 100.0;

// Replace with your network credentials
const char* ssid = "H1";
const char* password = "bigmacsatu";

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

volatile int encoder_value = 0;


float distanceValue = 0.0; 

String dist;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);



WebServer server(80);

// Motor Pins
const int motorPWM = 15;  // PWM pin to control speed
const int motorPWM2 = 2;
const int motorA = 25;    // Direction pin A
const int motorB = 33;    // Direction pin B
int speedclass;

double Kp = 1.0;
double Ki = 0.5;
double Kd = 0.2;

double desiredPosition = 100.0;
double currentPosition = 0.0;
double previousError = 0.0;
double integralTerm = 0.0;

TaskHandle_t mainTask;
TaskHandle_t PIDTask;


void encoder_isr() {
  // Reading the current state of encoder A and B
  int A = digitalRead(ENCODER_A);
  int B = digitalRead(ENCODER_B);

  // If the state of A changed, it means the encoder has been rotated
  if ((A == HIGH) != (B == LOW)) {
    encoder_value--;
  } else {
    encoder_value++;
  }
}


// To convert from pulse to centimeter
float encToDistance(int encoder_value)
{
  return ((encoder_value / PPR) * CIR);
}

void setup() {
  //Wire LCD
  Wire.begin();


  //Serial Baud Rate
  Serial.begin(115200);

  //MotorPWM back and forth
  pinMode(motorPWM, OUTPUT);
  pinMode(motorPWM2, OUTPUT);

  //i2c
  Serial.println("\nI2C Scanner");

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.println(WiFi.localIP());

  // Setup web server
  server.on("/", handleRoot);
  server.on("/speed", handleSpeed);
  server.on("/direction", handleDirection);
  server.onNotFound(handleNotFound);


  server.begin();
  Serial.println("HTTP server started");

  // initialize LCD
  lcd.begin();
  // turn on LCD backlight                      
  lcd.backlight();


  //Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);

  // Attaching the ISR to encoder A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_isr, CHANGE);


  //DualCore
    xTaskCreatePinnedToCore(
                    PID,   /* Task function. */
                    "PIDTask",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &PIDTask,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
    delay(500); 
}

void loop() {
  //Client handler
  server.handleClient();

  //LCD Handler
  // byte error, address;
  // int nDevices;
  // Serial.println("Scanning...");
  // nDevices = 0;
  // for(address = 1; address < 127; address++ ) {
  //   Wire.beginTransmission(address);
  //   error = Wire.endTransmission();
  //   if (error == 0) {
  //     Serial.print("I2C device found at address 0x");
  //     if (address<16) {
  //       Serial.print("0");
  //     }
  //     Serial.println(address,HEX);
  //     nDevices++;
  //   }
  //   else if (error==4) {
  //     Serial.print("Unknow error at address 0x");
  //     if (address<16) {
  //       Serial.print("0");
  //     }
  //     Serial.println(address,HEX);
  //   }    
  // }
  // if (nDevices == 0) {
  //   Serial.println("No I2C devices found\n");
  // }
  // else {
  //   Serial.println("done\n");
  // }
  // delay(5000);


  //LCD Display
   // set cursor to first column, first row
  lcd.setCursor(0, 0);
  // print message
  lcd.print("GROUP 5");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  delay(100);

  distanceValue = encToDistance(encoder_value);
  Serial.println("Encoder value: " + String(encoder_value) + " To Distance: " + String(distanceValue));
  // PID();

}

void handleRoot() {
  String html = "<html><head><title>Motor Control</title></head><body>";
  html += "<h1>Motor Control</h1>";
  html += "<form action=\"/speed\" method=\"GET\">";
  html += "<label>Speed:</label>";
  html += "<input type=\"range\" name=\"speed\" min=\"0\" max=\"255\">";
  html += "<input type=\"submit\" value=\"Set Speed\">";
  html += "</form>";
  html += "<form action=\"/direction\" method=\"GET\">";
  html += "<label>Direction:</label>";
  html += "<input type=\"radio\" name=\"direction\" value=\"forward\" checked>Forward";
  html += "<input type=\"radio\" name=\"direction\" value=\"backward\">Backward";
  html += "<input type=\"radio\" name=\"direction\" value=\"stop\">Stop";
  html += "<input type=\"submit\" value=\"Set Direction\">";
  html += "</form>";
  html += "<h2>Encoder Value : </h2>" + String(encoder_value) + "<h2> To Distance : </h2>" + String(distanceValue);
  html += "<h2>Speed Value : </h2>" + String(speedclass);
  html += "</body></html>";

  server.send(200, "text/html", html);
  delay(500);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void handleSpeed() {
  int speed = server.arg("speed").toInt();
  speedclass = speed;
  //Serial.println(speed);
  String html = "<html><head><title>Speed Value</title></head><body>";
  html += "<h3>Speed Value : </h3>" + String(speedclass) ;
  html += "<a href=\"/\">Back to Home menu<a>";
  html += "</body></html>";
  server.send(200, "text/html", html);

}

void handleDirection() {
  String direction = server.arg("direction");
    if (direction == "forward") {
      analogWrite(motorPWM, speedclass);
      analogWrite(motorPWM2, 0);
      delay(500);
      //Serial.println(speed);
    } else if (direction == "backward") {
      analogWrite(motorPWM, 0);
      analogWrite(motorPWM2, speedclass);
      delay(500);
      //Serial.println(speedclass);
    }
    else
    {
      analogWrite(motorPWM, 0);
      analogWrite(motorPWM2, 0);
      delay(500);
      //Serial.println(speedclass);    
    }
 
  server.send(200, "text/plain", "Direction set to " + direction + "&" + "speed" + String(speedclass));
}

void PID(void * pvParameters)
{
  while (true) {
  // Read current position from the sensor
  currentPosition = distanceValue;

  // Calculate error
  double error = desiredPosition - currentPosition;

  // Calculate PID terms
  double proportionalTerm = Kp * error;
  integralTerm += Ki * error;
  double derivativeTerm = Kd * (error - previousError);

  // Calculate control output
  double controlOutput = proportionalTerm + integralTerm + derivativeTerm;

  // Apply control output to the motor or actuator
  applyControlOutput(controlOutput);

  // Update variables for next iteration
  previousError = error;

  // Add some delay between iterations
  delay(10);
}
}


void applyControlOutput(double output)
{
  if(currentPosition > 0)
  {
    speedclass = speedclass;
  }

  else if(currentPosition < 0 )
  {
   speedclass = -speedclass;
  }

  else 
  {
    speedclass = 0;
  }
  
}



