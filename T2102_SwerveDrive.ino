#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#define OLED_ADDR 0x3C  // Replace with your OLED's I2C address
#include "RotaryEncoder.h"

Adafruit_SSD1306 display(OLED_ADDR);

int sensorValueA0 = 0;  // variable to store the value coming from the sensor
int sensorValueA1 = 0;  // variable to store the value coming from the sensor

int lowThreshold = 200;
int highThreshold = 760;
float lineSlope = 0.342205323;
float lineIntercept = 169;
float sensorValueA0Component = 0;
float sensorValueA1Component = 0;
float current_heading = 0;
int analogControl = 0;
int joystick_x_value = 0;
int joystick_y_value = 0;

float y_control_value = 0;
float x_control_value = 0;
float joystick_y_middle_value = 510;
float joystick_x_middle_value = 510;
float joystick_deadzone = 5;
float motor_speed = 0;
float desired_wheel_heading_value = 0;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Rotary Encoder Test Code
int Counter1 = 0, LastCount1 = 0;                  //uneeded just for test
void RotaryChanged();                              //we need to declare the func above so Rotary goes to the one below
RotaryEncoder Rotary1(&RotaryChanged, 11, 12, 4);  // Pins  (DT),  (CLK),  (SW)
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


// Arduino Pin Assignments
// D13   - Not Used
// 3.3v  - Not Used
// Ref   - Not Used

//  A0 - [D14] - A0 - Wheel heading Sensor - Base
//  A1 - [D15] - A1 -  Wheel heading Sensor - Inverted

// D16 -
const int joystickSwitch_pin = 17;  // D17 [A3]
//  A4 - [D18] - SDA - OLED Display
//  A5 - [D19] - SDC - OLED Display
//  A6 - Joystick - Y movement value
//  A7 - Joystick - X movement value

//  5v - Used
//  Reset - Not used
//  Ground - Used
//  Vin - Not Used

// - - - - - - - - - - - - - - - -

// D1 - Not usable
// D0 - Not usable
// Reset - Not used
// Ground

const int heading_motor_encoder_A_pin = 2;  // D2 (supports interupts)
const int heading_motor_encoder_B_pin = 3;  // D3 (supports interupts)

const int rotary_encoder_switch_pin = 4;             // D4
const int steering_motor_speed_PWM_pin = 5;          // D5
const int steering_motor_direction_A_pin = 6;        // D6
const int steering_motor_direction_B_pin = 7;        // D7
const int wheel_rotation_motor_direction_A_pin = 8;  // D8
const int wheel_rotation_motor_direction_B_pin = 9;  // D9
const int wheel_rotation_motor_speed_PWM_pin = 10;   // D10
const int rotary_encoder_data_pin = 11;              // D11 - (NEEDS INTERUPT SUPPORT) - Not working
const int rotary_encoder_clock_pin = 12;             // D12 - (NEEDS INTERUPT SUPPORT) - Not working//

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

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // - - - Rotary Encoder Test Code --
  Rotary1.setup();
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
}

//=============================================================================

void loop() {

  // Wheel Heading sensor
  sensorValueA0 = returnSensor(A0);
  sensorValueA1 = returnSensor(A1);
  current_heading = calculateHeading(sensorValueA0, sensorValueA1);
  displaySensorValuesAndHeading(sensorValueA0, sensorValueA1, current_heading);

  // Read joystick and use it to drive wheels
  y_control_value = get_joystick_y_control_value();  //   Returned value range:  0-1023
  motor_speed = calculate_motor_speed_value(y_control_value);
  set_right_front_wheel_speed(motor_speed);

  x_control_value = get_joystick_x_control_value();  //   Returned value range:  0-1023
  // When the joystick to angled left (510 to 1023), the wheels will turn to left 0 to -90
  // When the joystick to angled right (510 to 0), the wheels will turn to 0 to 90 right
  // When the joystick to centered (510), the wheels will turn to center

  desired_wheel_heading_value = convert_joystick_to_heading_value(x_control_value);
  set_right_front_wheel_heading(desired_wheel_heading_value, current_heading);
  //  analogWrite(steering_motor_speed_PWM_pin, get_joystick_x_control_value());



  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // - Temporary code for wheel drive - motor encoder
  static int lastReportedPos = 0;  // Keep track of last reported position
  if (encoderPos != lastReportedPos) {
    Serial.print("Position: ");
    Serial.println(encoderPos);
    lastReportedPos = encoderPos;
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

  // - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // Rotary Encoder Test Code

  if (Rotary1.GetButtonDown())
    Serial.println("Button 1 down");

  if (LastCount1 != Counter1) {
    Serial.print("Counter1:  ");
    Serial.println(Counter1);
    LastCount1 = Counter1;
  }

  // - - - - - - - - - - - - - - - - - - - - - - - - - - -
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
  headingString = "Heading:  " + String(heading) + " " + String(temp_motor_speed);
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
  int heading = sensorValueA0Component + sensorValueA1Component;
  return heading;
}

//------------------------------------------------------------------------
void configure_pins() {

  // pinMode(ledPin, OUTPUT);
  pinMode(steering_motor_speed_PWM_pin, OUTPUT);
  pinMode(steering_motor_direction_A_pin, OUTPUT);
  pinMode(steering_motor_direction_B_pin, OUTPUT);
  pinMode(wheel_rotation_motor_direction_A_pin, OUTPUT);
  pinMode(wheel_rotation_motor_direction_B_pin, OUTPUT);
  pinMode(wheel_rotation_motor_speed_PWM_pin, OUTPUT);

  digitalWrite(steering_motor_direction_A_pin, HIGH);
  digitalWrite(steering_motor_direction_B_pin, LOW);
  digitalWrite(wheel_rotation_motor_direction_A_pin, HIGH);
  digitalWrite(wheel_rotation_motor_direction_B_pin, LOW);

  pinMode(steering_motor_speed_PWM_pin, OUTPUT);
  pinMode(wheel_rotation_motor_speed_PWM_pin, OUTPUT);

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

float get_joystick_x_control_value() {
  analogControl = analogRead(A6);
  int motor_speed = map(analogControl, 0, 1023, -254, 255);
  Serial.print("X Axis:   Analog In: ");
  Serial.print(analogControl);
  Serial.print("  Mapped:");
  Serial.print(motor_speed);
  Serial.print("                   ");
  return motor_speed;
}

float get_joystick_y_control_value() {  // Forward speed
  motor_speed = analogRead(A7);         // 0-1023
  return motor_speed;
}

//float get_joystick_y_control_value() {    // Forward speed
//  analogControl  = analogRead(A7);        // 0-1023
//  int  motor_speed = map ( analogControl, 0, 1023, -254, 254);
//  Serial.print ("Y Axis:   Analog In: ");
//  Serial.print (analogControl);
//  Serial.print ("  Mapped:");
//  Serial.println (motor_speed);
//  return motor_speed;
//}

//------------------------------------------------------------------------
// Rotary Encoder Test Code
void RotaryChanged() {
  const unsigned int state1 = Rotary1.GetState();
  if (state1 & DIR_CW)
    Counter1++;
  if (state1 & DIR_CCW)
    Counter1--;
}
//------------------------------------------------------------------------
void set_drive_wheel_rotation_direction(String direction) {
  if (direction == "forward") {
    digitalWrite(wheel_rotation_motor_direction_A_pin, HIGH);
    digitalWrite(wheel_rotation_motor_direction_B_pin, LOW);
  } else if (direction == "reverse") {
    digitalWrite(wheel_rotation_motor_direction_A_pin, LOW);
    digitalWrite(wheel_rotation_motor_direction_B_pin, HIGH);
  } else {
    Serial.println("Error in direction selection");
  }
}

//------------------------------------------------------------------------
void set_steering_motor_direction(String direction) {
  if (direction == "cw") {

    digitalWrite(steering_motor_direction_A_pin, HIGH);
    digitalWrite(steering_motor_direction_B_pin, LOW);
  } else if (direction == "ccw") {
    digitalWrite(steering_motor_direction_A_pin, LOW);
    digitalWrite(steering_motor_direction_B_pin, HIGH);

  } else {
    Serial.println("Error in direction selection");
  }
}

//------------------------------------------------------------------------
float calculate_motor_speed_value(int y_control_value) {

  // Forward movement >> input range 510 to 103 >> output range 0 254
  if (y_control_value > (joystick_y_middle_value + joystick_deadzone)) {
    set_drive_wheel_rotation_direction("forward");
    motor_speed = map(y_control_value, joystick_y_middle_value, 1023, 0, 254);
    //    Serial.print (" Forward> ");

    // Reverse movement >> input range 0 to 510 >> output range 254 to 0
  } else if (y_control_value < (joystick_y_middle_value - joystick_deadzone)) {
    set_drive_wheel_rotation_direction("reverse");
    motor_speed = map(y_control_value, 0, joystick_y_middle_value, 254, 0);
    //    Serial.print (" Reverse> ");

  } else if ((y_control_value > (joystick_y_middle_value - joystick_deadzone)) && (y_control_value < (joystick_y_middle_value + joystick_deadzone))) {
    motor_speed = 0;
    //    Serial.print (" Deadzone> ");
  }

  //  Serial.print ("Y Axis:   Analog In: ");
  //  Serial.print (y_control_value);
  //  Serial.print ("  Mapped:");
  //  Serial.println (motor_speed);

  return motor_speed;
}
//------------------------------------------------------------------------
void set_right_front_wheel_speed(float motor_speed) {
  analogWrite(wheel_rotation_motor_speed_PWM_pin, motor_speed);
}
//------------------------------------------------------------------------

float convert_joystick_to_heading_value(int joystick_x_value) {

  // When the joystick to angled left (510 to 1023), the wheels will turn to left 0 to -90
  // When the joystick to angled right (510 to 0), the wheels will turn to 0 to 90 right
  // When the joystick to centered (510), the wheels will turn to center

//  float joystick_y_middle_value = 510;
//float joystick_x_middle_value = 510;
//float joystick_deadzone = 5;

  float heading_value = 0;

  // When the joystick to angled left (510 to 1023), the wheels will turn to left 0 to -90
//  if (joystick_x_value > (joystick_x_middle_value + joystick_deadzone)) {
  if (joystick_x_value > 515) {
    set_steering_motor_direction("ccw");
    heading_value = map(joystick_x_value, joystick_x_middle_value, 1023, 0, -90);
    Serial.print(" CCW > ");

    // When the joystick to angled right (510 to 0), the wheels will turn to 0 to 90 right
  } else if (joystick_x_value < 505) {
//  } else if (joystick_x_value < (joystick_x_middle_value - joystick_deadzone)) {
    set_steering_motor_direction("cw");
    heading_value = map(joystick_x_value, joystick_x_middle_value, 0, 0, 90);
    Serial.print(" CW > ");

    // When the joystick to centered (510), the wheels will turn to center
//  } else if ((joystick_x_value > (joystick_x_middle_value - joystick_deadzone)) && (joystick_x_value < (joystick_x_middle_value + joystick_deadzone))) {
  } else if ((joystick_x_value > 505) && (joystick_x_value < 515)) {
    heading_value = 0;
    Serial.print(" Deadzone > ");
  }

  Serial.print("X Axis:   Analog In: ");
  Serial.print(joystick_x_value);
  Serial.print("  Mapped heading: ");
  Serial.println(heading_value);

  return heading_value;
}

//------------------------------------------------------------------------

void set_right_front_wheel_heading(float desired_wheel_heading_value, float current_heading) {
  // Rotate left from center is negative heading 
  // Rotate right from center is positive heading

 // IF current is at 0 and desired is +30 (right) THEN hd = 30 (turning right)
 // IF current is at 0 and desired is -30 (left) THEN hd = -30 (turning left)
 // IF current is at 30 (right) and desired is -30 (left) THEN hd = -60 (turning left)
 // IF current is at -30 (left) and desired is + 30 (right) THEN hd = 60 (turning right)
 // IF current is at 30 
 // ... IF hd is positive, set motor to turn wheel heading cw
 // ... IF hd is negative, set motor to turn wheel heading ccw

  Serial.print("Current: ");
  Serial.print(current_heading);
  Serial.print("  desire: ");
  Serial.print(desired_wheel_heading_value);
 
  float heading_alignment_tolerance = 5;
  float heading_change_speed = 120;

  float heading_difference = desired_wheel_heading_value - current_heading;
  Serial.print("   Hd diff: ");
  Serial.print(heading_difference);
  Serial.print("      ");

  // No need to change heading
  if ( abs(heading_difference) < heading_alignment_tolerance ){
    // No need to change heading
    Serial.print("heading is good   ");
    heading_change_speed = 0;
  }
  else if (heading_difference > heading_alignment_tolerance){
    Serial.print("Need to turn right");
    heading_change_speed = 120;

  } else{
    Serial.print("Need to turn Left ");
    heading_change_speed = -120;

  }

  Serial.print("   heading_change_speed: ");
  Serial.println(heading_change_speed);
  analogWrite(steering_motor_speed_PWM_pin, heading_change_speed);
}


//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------
