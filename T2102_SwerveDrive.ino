#include <Servo.h>
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#define OLED_ADDR 0x3C  // Replace with your OLED's I2C address

Adafruit_SSD1306 display(OLED_ADDR);

int sensorValueA0 = 0;  // variable to store the value coming from the sensor
int sensorValueA1 = 0;  // variable to store the value coming from the sensor

int lowThreshold = 200;
int highThreshold = 760;
float lineSlope = 0.342205323;
float lineIntercept = 169;
float sensorValueA0Component = 0;
float sensorValueA1Component = 0;
float heading = 0;

const int steering_motor_speed_PWM_pin = 5;
const int steering_motor_direction_A_pin = 6;
const int steering_motor_direction_B_pin = 7;
const int heading_motor_direction_A_pin = 8;
const int heading_motor_direction_B_pin = 9;
const int heading_motor_speed_PWM_pin = 10;

const int heading_motor_encoder_A_pin = 11;
const int heading_motor_encoder_B_pin = 12;

Servo steering_motor_speed_PWM;
Servo heading_motor_speed_PWM;

volatile int encoderPos = 0;  // Encoder position (volatile for interrupt)
int lastEncoded = 0;          // Used to track last encoder state

int temp_motor_speed = 0;
int temp_motor_speed_increment = +1;


//=============================================================================

void setup() {
  Serial.begin(9600);
  initializeDisplay();
  configure_pins();
}

//=============================================================================

void loop() {

  sensorValueA0 = returnSensor(A0);
  sensorValueA1 = returnSensor(A1);
  heading = calculateHeading(sensorValueA0, sensorValueA1);
  displaySensorValuesAndHeading(sensorValueA0, sensorValueA1, heading);



  temp_motor_speed = temp_motor_speed + temp_motor_speed_increment;
  steering_motor_speed_PWM.write(temp_motor_speed);
  Serial.print ("Speed: ");
  Serial.println (temp_motor_speed);
  if (temp_motor_speed >= 180){
    temp_motor_speed_increment = -1;
  }
  if (temp_motor_speed <= 2){
    temp_motor_speed_increment = 1;
  }
  delay(250);

  
//  steering_motor_speed_PWM.write(0);
//  heading_motor_speed_PWM.write(0);

  static int lastReportedPos = 0;  // Keep track of last reported position
  if (encoderPos != lastReportedPos) {
    Serial.print("Position: ");
    Serial.println(encoderPos);
    lastReportedPos = encoderPos;
  }
}
//========================================================================

//------------------------------------------------------------------------

void initializeDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);  // Initialize the display
  display.clearDisplay();                          // Clear the display
  display.setTextColor(WHITE);                     // Set text color
  display.setTextSize(2);                          // Set text size (Was 2)
  display.clearDisplay();                          // Clear the display
  delay(100);                                      // Wait for 2 seconds
}

//------------------------------------------------------------------------
float returnSensor(int sensorPin) {
  return analogRead(sensorPin);
}

//------------------------------------------------------------------------
void displaySensorValuesAndHeading(int sensorValueA0, int sensorValueA1, float heading) {

  String sensorValuesString = "";
  String headingString = "";

  headingString = "Heading:    " + String(heading);
  display.setCursor(0, 0);       // Set cursor position
  display.print(headingString);  // Print text

  //  Uncomment to see individual sensors
  //  sensorValuesString = "Sen:" + String(sensorValueA0) + "  " + String(sensorValueA1);
  //  display.setCursor(0, 17); // Set cursor position
  //  display.print(sensorValuesString); // Print text

  display.display();       // Update the display
  delay(10);               // Wait for 2 seconds
  display.clearDisplay();  // Clear the display
  delay(1);                // Wait for 2 seconds
}

//------------------------------------------------------------------------

float calculateHeading(float sensorValueA0, float sensorValueA1) {

  // Using mid range of Sensor on A0 for wheel heading between + 90 and -90 degrees

  if ((sensorValueA0 > lowThreshold) && (sensorValueA0 < highThreshold)) {
    sensorValueA0Component = lineSlope * sensorValueA0 - lineIntercept;
    sensorValueA1Component = 0;
  } else {
    //  Using mid range of Sensor on A1 for the wheel heading of 90 to 180 and -90 to -180
    sensorValueA0Component = 0;

    if (sensorValueA1 < 500) {
      sensorValueA1Component = lineSlope * sensorValueA1 + 12.66;
    } else {
      sensorValueA1Component = lineSlope * sensorValueA1 - 351.1;
    }
  }
  heading = sensorValueA0Component + sensorValueA1Component;
  return heading;
}

//------------------------------------------------------------------------
void configure_pins() {

  // pinMode(ledPin, OUTPUT);
  pinMode(steering_motor_speed_PWM_pin, OUTPUT);
  pinMode(steering_motor_direction_A_pin, OUTPUT);
  pinMode(steering_motor_direction_B_pin, OUTPUT);
  pinMode(heading_motor_direction_A_pin, OUTPUT);
  pinMode(heading_motor_direction_B_pin, OUTPUT);
  pinMode(heading_motor_speed_PWM_pin, OUTPUT);

  digitalWrite(steering_motor_direction_A_pin, HIGH);
  digitalWrite(steering_motor_direction_B_pin, LOW);
  digitalWrite(heading_motor_direction_A_pin, HIGH);
  digitalWrite(heading_motor_direction_B_pin, LOW);


  steering_motor_speed_PWM.attach(steering_motor_speed_PWM_pin);
  heading_motor_speed_PWM.attach(heading_motor_speed_PWM_pin);


  pinMode(heading_motor_encoder_A_pin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(heading_motor_encoder_B_pin, INPUT_PULLUP);  // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(heading_motor_encoder_A_pin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(heading_motor_encoder_B_pin), updateEncoder, CHANGE);
}

//------------------------------------------------------------------------
void updateEncoder() {
  int MSB = digitalRead(heading_motor_encoder_A_pin);
  int LSB = digitalRead(heading_motor_encoder_B_pin);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncoded << 2) | encoded;

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
    encoderPos++;
  }
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
    encoderPos--;
  }
  lastEncoded = encoded;
}

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------
