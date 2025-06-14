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
int analogControl = 0;

// Arduino Pin Assignments
// D13   - Not Used
// 3.3v  - Not Used
// Ref   - Not Used

//  A0 - [D14] - A0 - Wheel heading Sensor - Base
//  A1 - [D15] - A1 -  Wheel heading Sensor - Inverted
const int control_potentiometer_ground_pin = 16 // D16 [A2]
const int control_potentiometer_5V_pin = 17  // D17 [A3]
//  A4 - [D18] - SDA - OLED Display
//  A5 - [D19] - SDC - OLED Display
//  A6 
//  A7 - Control Potentiometer - Analog signal

//  5v - Used
//  Reset - Not used
//  Ground - Used
//  Vin - Not Used

// - - - - - - - - - - - - 

// D1 - Not usable
// D0 - Not usable
// Reset - Not used
// Ground

const int heading_motor_encoder_A_pin = 2;  // D2 (supports interupts)
const int heading_motor_encoder_B_pin = 3;  // D3 (supports interupts)

//   D4
const int steering_motor_speed_PWM_pin = 5;     // D5
const int steering_motor_direction_A_pin = 6;   // D6
const int steering_motor_direction_B_pin = 7;   // D7
const int heading_motor_direction_A_pin = 8;    // D8
const int heading_motor_direction_B_pin = 9;    // D9
const int wheel_rotation_motor_speed_PWM_pin = 10;  // D10 
// D11
// D12

//==============================================

volatile int encoderPos = 0;  // Encoder position (volatile for interrupt)
int lastEncoded = 0;          // Used to track last encoder state

int temp_motor_speed = 60;
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

  temp_motor_speed = get_speed_control_value();

  analogWrite(wheel_rotation_motor_speed_PWM_pin, temp_motor_speed);
//   analogWrite(steering_motor_speed_PWM_pin, temp_motor_speed);

// - Temporary code for wheel drive - motor encoder   
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

//  headingString = "Heading:  " + String(heading)
  headingString = "Heading:  " + String(heading)  + " " + String(temp_motor_speed);
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
  pinMode(wheel_rotation_motor_speed_PWM_pin, OUTPUT);

  digitalWrite(steering_motor_direction_A_pin, HIGH);
  digitalWrite(steering_motor_direction_B_pin, LOW);
  digitalWrite(heading_motor_direction_A_pin, HIGH);
  digitalWrite(heading_motor_direction_B_pin, LOW);

  pinMode(steering_motor_speed_PWM_pin, OUTPUT);
  pinMode(wheel_rotation_motor_speed_PWM_pin, OUTPUT);

  pinMode(heading_motor_encoder_A_pin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(heading_motor_encoder_B_pin, INPUT_PULLUP);  // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(heading_motor_encoder_A_pin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(heading_motor_encoder_B_pin), updateEncoder, CHANGE);

  pinMode(control_potentiometer_ground_pin, OUTPUT);
  pinMode(control_potentiometer_5V_pin, OUTPUT);
  digitalWrite(control_potentiometer_ground_pin, LOW);
  digitalWrite(control_potentiometer_5V_pin, HIGH);


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

  float get_speed_control_value(){
    analogControl  = analogRead(A7);
    temp_motor_speed = map ( analogControl, 0, 1023, 60, 120);
    Serial.print ("Motor Speed:   Analog In: ")
    Serial.print (analogControl);
    Serial.print ("  Mapped:");
    Serial.println (temp_motor_speed);
    return temp_motor_speed;
  }



//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------
