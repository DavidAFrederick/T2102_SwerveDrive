//  June 30, 2025  17:30
//
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#define OLED_ADDR 0x3C  // Replace with your OLED's I2C address
#include "RotaryEncoder.h"

Adafruit_SSD1306 display(OLED_ADDR);

int FL_sensorValueA0 = 0;  
int FL_sensorValueA1 = 0;  
int FR_sensorValueA0 = 0;  
int FR_sensorValueA1 = 0;  
int BL_sensorValueA0 = 0;  
int BL_sensorValueA1 = 0;  
int BR_sensorValueA0 = 0;  
int BR_sensorValueA1 = 0;  

float FL_sensorValueA0Component = 0;
float FL_sensorValueA1Component = 0;
float FR_sensorValueA0Component = 0;
float FR_sensorValueA1Component = 0;
float BL_sensorValueA0Component = 0;
float BL_sensorValueA1Component = 0;
float BR_sensorValueA0Component = 0;
float BR_sensorValueA1Component = 0;


int lowThreshold = 200;
int highThreshold = 760;
float lineSlope = 0.342205323;
float lineIntercept = 169;
float FL_current_heading = 0;
int analogControl = 0;
int joystick_x_value = 0;
int joystick_y_value = 0;
float y_control_value = 0;
float x_control_value = 0;
float joystick_y_middle_value = 510;
float joystick_x_middle_value = 510;
float joystick_deadzone = 5;


float FL_motor_speed = 0;
float FR_motor_speed = 0;
float BL_motor_speed = 0;
float BR_motor_speed = 0;
float FL_desired_wheel_heading_value = 0;
float FR_desired_wheel_heading_value = 0;
float BL_desired_wheel_heading_value = 0;
float BR_desired_wheel_heading_value = 0;

int loop_counter = 0;
long nextTime = 0;

// int temp_FL_motor_speed = 60;
// int temp_FL_motor_speed_increment = +1;
bool debugflag = false;
bool homeWheel = false;

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
/*

DIO 54 
PWM 15 -  15 PWM pins: 2 through 13, and 44, 45, and 46.
Interrupts 6 - interrupt pins: 2, 3, 18, 19, 20, and 21.   (Used on one wheel and the rotary controller)
16 analog input pins

Needed:
DIO: 20
Interupts 12 - 2 for each wheel encoder, 4 for rotary controls
PWM 8
Analog 8

// - - - - - >> MEGA PIN ASSIGNMENTS << - - - - - - - - - - - - - - - 
Analog
A0 = Heading Sensor A = Module White (BL) - Back Left (BL)  - []
A1 = Heading Sensor B = Module White (BL) - []
A2 = Heading Sensor A = Module Blue (FL) - Front Left (FL) - [] 
A3 = Heading Sensor B = Module Blue (FL)  - []
A4 = Heading Sensor A = Module Black (FR) - Front Right (FR)  - []
A5 = Heading Sensor B = Module Black (FR)  - []
A6 = Heading Sensor A = Module Silver (BR) - Back Right (BR)  - []
A7 = Heading Sensor B = Module Silver (BR)  - []

A8 = Robot Controller Joystick - X axis - []
A9 = Robot Controller Joystick - Y axis - []
D53 = Robot Controller Joystick - Push Switch - []

Digital
D0  - Can't be used -  TX/RX
D1  - Can't be used -  TX/RX
D2  - Rotary Encoder Data - Interupt - []
D3  - Rotary Encoder Clock - Interupt - []
D4  - Rotary Encoder Switch - []

D5  - PWM - Motor Controller - EN-B - Silver (BR)  - Drive - [GRAY]
D6  - PWM - Motor Controller - EN-A - Blue (FL)  - Heading - []
D7  - PWM - Motor Controller - EN-B - Blue (FL)  - Drive - []

D8  - PWM - Motor Controller - EN-B - White (BL) - Drive - [GRAY]
D9  - DIO - Motor Controller - IN-4 - White (BL) - Drive - [PURPLE]
D10 - DIO - Motor Controller - IN-3 - White (BL) - Drive - [BLUE]
D11 - DIO - Motor Controller - IN-2 - White (BL) - Heading - [GREEN]
D12 - DIO - Motor Controller - IN-1 - White (BL) - Heading - [YELLOW]
D13 - PWM - Motor Controller - EN-A - White (BL) - Heading - [ORANGE]

D18 - Interrupt - Motor Encoder - C1 - [BROWN] - White (BL)
D19 - Interrupt - Motor Encoder - C2 - [WHITE] - White (BL)

D20 - Interrupt - SDA - OLED - [] - Not sure how PINS are identified for i2C
D21 - Interrupt - SCL - OLED  - []

D23 - DIO - Motor Controller - IN-1 - Blue (FL) - Heading - []
D25 - DIO - Motor Controller - IN-2 - Blue (FL) - Heading - []
D27 - DIO - Motor Controller - IN-3 - Blue (FL) - Drive - []
D29 - DIO - Motor Controller - IN-4 - Blue (FL) - Drive - []

D33 - DIO - Motor Controller - IN-1 - Black (FR) - Heading - []
D35 - DIO - Motor Controller - IN-2 - Black (FR) - Heading - []
D37 - DIO - Motor Controller - IN-3 - Black (FR) - Drive - []
D39 - DIO - Motor Controller - IN-4 - Black (FR) - Drive - []

D44 - PWM - Motor Controller - EN-A - Black (FR) - Heading - []
D46 - PWM - Motor Controller - EN-B - Black (FR) - Drive - []

D43 - DIO - Motor Controller - IN-1 - Silver  (BR)  - Heading - [ORANGE]
D45 - PWM - Motor Controller - EN-A - Silver  (BR)  - Heading - [YELLOW]
D47 - DIO - Motor Controller - IN-2 - Silver  (BR)  - Heading - [GREEN]
D49 - DIO - Motor Controller - IN-3 - Silver  (BR)  - Drive - [BLUE]
D51 - DIO - Motor Controller - IN-4 - Silver  (BR)  - Drive - [PURPLE]

D53 = Robot Controller Joystick - Push Switch - []


*/
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Rotary Encoder Test Code
int Counter1 = 0, LastCount1 = 0;                // Not needed just for test
void RotaryChanged();                            // we need to declare the func above so Rotary goes to the one below
// RotaryEncoder Rotary1(&RotaryChanged, 2, 3, 4);  // + Pins  (DT),  (CLK),  (SW)
RotaryEncoder Rotary1(&RotaryChanged, rotary_encoder_data_pin, rotary_encoder_clock_pin, 
  rotary_encoder_switch_pin);  // + Pins  (DT),  (CLK),  (SW)

volatile int encoderPos = 0;  // Encoder position (volatile for interrupt)
int lastEncoded = 0;          // Used to track last encoder state
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Arduino Pin Assignments
// D13   - Not Used
// 3.3v  - Not Used
// Ref   - Not Used

//  A0 - [D14] - A0 - Wheel heading Sensor - Base
//  A1 - [D15] - A1 -  Wheel heading Sensor - Inverted

//  D16 -
//  D17 - joystickSwitch_pin = 17;  // D17 [A3]
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

//- - (Name to Pin Assignments) - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

// TEMP:  D18 - Interrupt - Motor Encoder - C1 - [BROWN] - White (BL)
// TEMP:  D19 - Interrupt - Motor Encoder - C2 - [WHITE] - White (BL)

//  interrupt pins: 2, 3, 18, 19, 20, and 21. 

const int rotary_encoder_data_pin = 2;              // D2   (Interrupt)
const int rotary_encoder_clock_pin = 3;             // D3   (Interrupt)
const int rotary_encoder_switch_pin = 4;            //+ D4

const int joystickSwitch_pin = ;  // D


const int BL_rotation_motor_encoder_A_pin = 19;  //+  (supports interupts) - Only White (BL) - One wheel
const int BL_rotation_motor_encoder_B_pin = 18;  //+  (supports interupts)

const int FL_steering_motor_speed_PWM_pin = 6;           // EN-A  Blue Corner
const int FL_steering_motor_direction_A_pin = 23;        // IN-1
const int FL_steering_motor_direction_B_pin = 25;        // IN-2
const int FL_wheel_rotation_motor_direction_A_pin = 27;  // IN-3
const int FL_wheel_rotation_motor_direction_B_pin = 29;  // IN-4
const int FL_wheel_rotation_motor_speed_PWM_pin = 7;     // EN-B

const int FR_steering_motor_speed_PWM_pin = 44;          // EN-A    Black Corner
const int FR_steering_motor_direction_A_pin = 33;        // IN-1
const int FR_steering_motor_direction_B_pin = 35;        // IN-2
const int FR_wheel_rotation_motor_direction_A_pin = 37;  // IN-3
const int FR_wheel_rotation_motor_direction_B_pin = 39;  // IN-4
const int FR_wheel_rotation_motor_speed_PWM_pin = 46;    // EN-B

const int BL_steering_motor_speed_PWM_pin = 13;          //     White Corner
const int BL_steering_motor_direction_A_pin = 12;        // 
const int BL_steering_motor_direction_B_pin = 11;        // 
const int BL_wheel_rotation_motor_direction_A_pin = 10;  // 
const int BL_wheel_rotation_motor_direction_B_pin = 9;   // 
const int BL_wheel_rotation_motor_speed_PWM_pin = 8;     // 

const int BR_steering_motor_speed_PWM_pin = 45;          // EN-A    Silver Corner
const int BR_steering_motor_direction_A_pin = 43;        // IN-1
const int BR_steering_motor_direction_B_pin = 47;        // IN-2
const int BR_wheel_rotation_motor_direction_A_pin = 49;  // IN-3
const int BR_wheel_rotation_motor_direction_B_pin = 51;  // IN-4
const int BR_wheel_rotation_motor_speed_PWM_pin = 5;     // EN-B


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

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //  Enable the debug flag once per second (1000) to print status and loops per second count
  // loop_counter = loop_counter + 1;   // Moved to inside of the loop
  if (millis() > nextTime) {
    nextTime += 1000;  // Number of milliseconds between prints to monitor
    Serial.print("Number of loops per second: ");
    Serial.println(loop_counter);
    loop_counter = 0;
    debugflag = true;  // Control printing for debugging
  } else {
    debugflag = false;
    loop_counter = loop_counter + 1;
  }
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  //
  //     [ FL ]      [ FR ]
  //
  //             top
  //     
  //     [ BL ]      [ BR ]
  //

  // Wheel Heading sensor
  FL_current_heading = readCurrentHeading(A0,A1);    // Update to pass parameters
  if (debugflag) displaySensorValuesAndHeading(FL_sensorValueA0, FL_sensorValueA1, FL_current_heading);

  // Read joystick and use it to drive wheels
  y_control_value = get_joystick_y_control_value();  //   Returned value range:  0-1023
  FL_motor_speed = calculate_FL_motor_speed_value(y_control_value);
  set_right_front_wheel_speed(FL_motor_speed);

  //  When the joystick button is pressed, drive the wheel direction to zero.
  if (JoystickButtonPressed()) {
    homeWheel =  home_wheels_when_joystick_pressed();
  }

  // Do not steer the wheel based on joystick once the home joystick button is pressed
  if (homeWheel == false) {
    // Used for wheel steering
    x_control_value = get_joystick_x_control_value();

    FL_desired_wheel_heading_value = convert_joystick_to_heading_value(x_control_value);
    set_right_front_wheel_heading(FL_desired_wheel_heading_value, FL_current_heading);
  }


  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // - Temporary code for wheel drive - motor encoder
  static int lastReportedPos = 0;  // Keep track of last reported position
  if (encoderPos != lastReportedPos) {
    //    Serial.print("Position: ");
    //    Serial.println(encoderPos);
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

// Initialize the OLED Display
void initializeDisplay() {
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);  // Initialize the display
  display.clearDisplay();                          // Clear the display
  display.setTextColor(WHITE);                     // Set text color
  display.setTextSize(2);                          // Set text size (Was 2)
  display.clearDisplay();                          // Clear the display
  delay(100);                                      // Wait for 2 seconds
}

//------------------------------------------------------------------------

// Performs an analog read of the specified pin
float returnSensor(int sensorPin) {
  return analogRead(sensorPin);
}
//------------------------------------------------------------------------

/*
  Displays heading and raw sensors values (disabled) on the OLED.  SLOW
*/
void displaySensorValuesAndHeading(int FL_sensorValueA0, int FL_sensorValueA1, float heading) {

  String sensorValuesString = "";
  String headingString = "";

  headingString = "Heading:  " + String(heading);
  display.setCursor(0, 0);       // Set cursor position
  display.print(headingString);  // Print text

  //  Uncomment to see individual sensors
  //  sensorValuesString = "Sen:" + String(FL_sensorValueA0) + "  " + String(FL_sensorValueA1);
  //  display.setCursor(0, 17); // Set cursor position
  //  display.print(sensorValuesString); // Print text

  display.display();       // Update the display
  delay(10);               //
  display.clearDisplay();  // Clear the display
  delay(1);                //
}

//------------------------------------------------------------------------

/*
  Calculate the heading of the wheel based on two rotational position sensors.
  Need two sensors to cover the 30 degree blank segment
*/
float calculateHeading(float sensorValueA0, float sensorValueA1) {
  float sensorValueA0Component = 0;
  float sensorValueA1Component = 0;

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

// Assigns the modes of each pin on the arduino
void configure_pins() {

  // Signals to Motor Controller
  pinMode(FL_steering_motor_speed_PWM_pin, OUTPUT);
  pinMode(FL_steering_motor_direction_A_pin, OUTPUT);
  pinMode(FL_steering_motor_direction_B_pin, OUTPUT);
  pinMode(FL_wheel_rotation_motor_direction_A_pin, OUTPUT);
  pinMode(FL_wheel_rotation_motor_direction_B_pin, OUTPUT);
  pinMode(FL_wheel_rotation_motor_speed_PWM_pin, OUTPUT);

  digitalWrite(FL_steering_motor_direction_A_pin, HIGH);
  digitalWrite(FL_steering_motor_direction_B_pin, LOW);
  digitalWrite(FL_wheel_rotation_motor_direction_A_pin, HIGH);
  digitalWrite(FL_wheel_rotation_motor_direction_B_pin, LOW);

  //  pinMode(FL_steering_motor_speed_PWM_pin, OUTPUT);  // Why duplicated
  //  pinMode(FL_wheel_rotation_motor_speed_PWM_pin, OUTPUT);

  // Signals to Wheel Rotation Drive Motor Encoder
  pinMode(BL_rotation_motor_encoder_A_pin, INPUT_PULLUP);  // Enable internal pull-up resistor
  pinMode(BL_rotation_motor_encoder_B_pin, INPUT_PULLUP);  // Enable internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(BL_rotation_motor_encoder_A_pin), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_rotation_motor_encoder_B_pin), updateEncoder, CHANGE);

  // Joystick
  pinMode(joystickSwitch_pin, INPUT_PULLUP);
}

//------------------------------------------------------------------------

/*
  This function is an interrupt call to handle the motor encoder.
  Increments or decrements a counter
*/

void updateEncoder() {
  int MSB = digitalRead(BL_rotation_motor_encoder_A_pin);
  int LSB = digitalRead(BL_rotation_motor_encoder_B_pin);
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

// Read the joystick X position (lateral) and return 0 (full left) to 1023 (Full Right)
float get_joystick_x_control_value() {
  analogControl = analogRead(A6);
  return analogControl;
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
// Read the joystick y position (forward/backward) and return 1023 (full forward) to 0 (Full reverse)
float get_joystick_y_control_value() {  // Forward speed
  FL_motor_speed = analogRead(A7);         // 0-1023
  return FL_motor_speed;
}

//float get_joystick_y_control_value() {    // Forward speed
//  analogControl  = analogRead(A7);        // 0-1023
//  int  FL_motor_speed = map ( analogControl, 0, 1023, -254, 254);
//  Serial.print ("Y Axis:   Analog In: ");
//  Serial.print (analogControl);
//  Serial.print ("  Mapped:");
//  Serial.println (FL_motor_speed);
//  return FL_motor_speed;
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

// Controls the direction of rotation of the wheel drive motor

void set_drive_wheel_rotation_direction(String direction) {
  if (direction == "forward") {
    digitalWrite(FL_wheel_rotation_motor_direction_A_pin, HIGH);
    digitalWrite(FL_wheel_rotation_motor_direction_B_pin, LOW);
  } else if (direction == "reverse") {
    digitalWrite(FL_wheel_rotation_motor_direction_A_pin, LOW);
    digitalWrite(FL_wheel_rotation_motor_direction_B_pin, HIGH);
  } else {
    Serial.println("Error in direction selection");
  }
}

//------------------------------------------------------------------------

// Controls the direction of rotation of the wheel heading drive motor

void set_steering_motor_direction(String direction) {
  if (direction == "cw") {

    digitalWrite(FL_steering_motor_direction_A_pin, HIGH);
    digitalWrite(FL_steering_motor_direction_B_pin, LOW);
  } else if (direction == "ccw") {
    digitalWrite(FL_steering_motor_direction_A_pin, LOW);
    digitalWrite(FL_steering_motor_direction_B_pin, HIGH);

  } else {
    Serial.println("Error in direction selection");
  }
}

//------------------------------------------------------------------------

/*
  Calculate motor speed and direction from the input joystick position value ranging
  from 1023 (full forward) to 0 (full reverse). The motor output is of range 0 to 254.
  Must set the motor direction separately.
*/

float calculate_FL_motor_speed_value(int y_control_value) {

  // Forward movement >> input range 510 to 1023 >> output range 0 254
  if (y_control_value > (joystick_y_middle_value + joystick_deadzone)) {
    set_drive_wheel_rotation_direction("forward");
    FL_motor_speed = map(y_control_value, joystick_y_middle_value, 1023, 0, 254);
    //    Serial.print (" Forward> ");

    // Reverse movement >> input range 0 to 510 >> output range 254 to 0
  } else if (y_control_value < (joystick_y_middle_value - joystick_deadzone)) {
    set_drive_wheel_rotation_direction("reverse");
    FL_motor_speed = map(y_control_value, 0, joystick_y_middle_value, 254, 0);  // ERROR???
    // FL_motor_speed = map(y_control_value, 0, joystick_y_middle_value, 254, 0);  // Updated
    //    Serial.print (" Reverse> ");

  } else if ((y_control_value > (joystick_y_middle_value - joystick_deadzone)) && (y_control_value < (joystick_y_middle_value + joystick_deadzone))) {
    FL_motor_speed = 0;
    //    Serial.print (" Deadzone> ");
  }

  if (debugflag && false) {
    //  Serial.print ("Y Axis:   Analog In: ");
    //  Serial.print (y_control_value);
    //  Serial.print ("  Mapped:");
    //  Serial.println (FL_motor_speed);
  }
  return FL_motor_speed;
}
//------------------------------------------------------------------------
// Need to expand this code to support forward and reverse operations.  Copy from wheel heading speed control
void set_right_front_wheel_speed(float FL_motor_speed) {
  analogWrite(FL_wheel_rotation_motor_speed_PWM_pin, FL_motor_speed);
}
//------------------------------------------------------------------------

/*
    When the joystick to angled left (510 to 1023), the wheels will turn to left 0 to -90.
    When the joystick to angled right (510 to 0), the wheels will turn to 0 to 90 right.
    When the joystick to centered (510), the wheels will turn to center.
*/
float convert_joystick_to_heading_value(int joystick_x_value) {

  float heading_value = 0;

  // When the joystick to angled left (510 to 1023), the wheels will turn to left 0 to -90
  if (joystick_x_value > (joystick_x_middle_value + joystick_deadzone)) {
    // if (joystick_x_value > 515) {
    // set_steering_motor_direction("ccw");  ////  ERROR - THESE DON'T BELONG HERE, WE ARE NOT MOVING THE MOTOR
    heading_value = map(joystick_x_value, joystick_x_middle_value, 1023, 0, -90);
    //    Serial.print(" CCW > ");

    // When the joystick to angled right (510 to 0), the wheels will turn to 0 to 90 right
    // } else if (joystick_x_value < 505) {
  } else if (joystick_x_value < (joystick_x_middle_value - joystick_deadzone)) {
    // set_steering_motor_direction("cw");  ////  ERROR - THESE DON'T BELONG HERE, WE ARE NOT MOVING THE MOTOR
    heading_value = map(joystick_x_value, joystick_x_middle_value, 0, 0, 90);
    //    Serial.print(" CW > ");

    // When the joystick to centered (510), the wheels will turn to center
  } else if ((joystick_x_value > (joystick_x_middle_value - joystick_deadzone)) && (joystick_x_value < (joystick_x_middle_value + joystick_deadzone))) {
    // } else if ((joystick_x_value > 505) && (joystick_x_value < 515)) {
    heading_value = 0;
    //    Serial.print(" Deadzone > ");
  }

  return heading_value;
}

//------------------------------------------------------------------------

// Drive the wheel to the desired value based on the current position

void set_right_front_wheel_heading(float desired_wheel_heading_value, float FL_current_heading) {
  // Rotate left from center is negative heading
  // Rotate right from center is positive heading

  // IF current is at 0 and desired is +30 (right) THEN hd = 30 (turning right)
  // IF current is at 0 and desired is -30 (left) THEN hd = -30 (turning left)
  // IF current is at 30 (right) and desired is -30 (left) THEN hd = -60 (turning left)
  // IF current is at -30 (left) and desired is + 30 (right) THEN hd = 60 (turning right)
  // IF current is at 30
  // ... IF hd is positive, set motor to turn wheel heading cw
  // ... IF hd is negative, set motor to turn wheel heading ccw

  float heading_alignment_tolerance = 8;
  float heading_change_speed = 150;  // 120;

  float heading_difference = desired_wheel_heading_value - FL_current_heading;

  int option = 0;

  if (abs(FL_current_heading) > 160) {
    runWheelSteeringMotor(0);
    Serial.println("Reached +/- 170 degree limit");
  }

  // Check to see if within the tolerance window
  if (abs(heading_difference) <= heading_alignment_tolerance) {
    //    Serial.print("heading is good   ");
    option = 1;
    heading_change_speed = 0;
  } else  //  Heading outside of tolerance window
  {
    if (heading_difference >= 0) {
      //    Serial.print("Need to turn right");
      heading_change_speed = -heading_change_speed;
      option = 2;
    } else if (heading_difference < 0) {
      //    Serial.print("Need to turn right");
      heading_change_speed = heading_change_speed;
      option = 3;
    }
  }

  if (debugflag && false) {
    Serial.print("Ct: ");
    Serial.print(FL_current_heading);
    Serial.print(" Dd: ");
    Serial.print(desired_wheel_heading_value);
    Serial.print(" diff: ");
    Serial.print(heading_difference);
    Serial.print(" HC_speed: ");
    Serial.print(heading_change_speed);
    Serial.print(" case: ");
    Serial.println(option);
  }
  runWheelSteeringMotor(heading_change_speed);
}

//------------------------------------------------------------------------

//  Released = 0 and pressed = 1

bool JoystickButtonPressed() {
  int pinInputValue = !digitalRead(joystickSwitch_pin);
  return pinInputValue;
}

//------------------------------------------------------------------------

// Stop the motor

void stopWheelSteeringMotor() {
  analogWrite(FL_steering_motor_speed_PWM_pin, 0);
}
//------------------------------------------------------------------------

/*
  Input range is -254 to + 254
  If input is greater than zero, set motor to forward and run the motor
  If input is less than zero, set motor to reverse direction, negate the input and run the motor
*/
void runWheelSteeringMotor(float local_motorSpeed) {

  if (debugflag && false) {
    Serial.print("Speed ");
    Serial.println(local_motorSpeed);
  }

  if (local_motorSpeed >= 0) {
    set_steering_motor_direction("ccw");
    analogWrite(FL_steering_motor_speed_PWM_pin, local_motorSpeed);
  } else if (local_motorSpeed < 0) {
    set_steering_motor_direction("cw");
    analogWrite(FL_steering_motor_speed_PWM_pin, -local_motorSpeed);
  }
}

//------------------------------------------------------------------------

// Rotates the wheel to zero degree position.  Loops until complete.

bool driveMotorToHome() {  //  Returns true when completed
  bool homingDone = false;
  float local_heading_tolerance = 5;
  float local_heading = readCurrentHeading(A0,A1);
  float homing_speed = 120;
  Serial.println("Homing Steering motor - Started");

  while (!homingDone) {
    local_heading = readCurrentHeading(A0,A1);
//    Serial.print("Homing Steering motor - In process:  ");
//    Serial.println(local_heading);

    if (abs(local_heading) < local_heading_tolerance) {
      homingDone = true;
      stopWheelSteeringMotor();

      Serial.print("Homing Steering motor - Complete 1:  Current Heading: ");
      local_heading = readCurrentHeading(A0,A1);
      Serial.println(local_heading);
      Serial.println ("Holding for 1 second");
      delay(1000);
      Serial.println ("Release hold");
    }
    else if (local_heading >= 0) {
      runWheelSteeringMotor(homing_speed);  // 0-254 is CountClockwise, -254 to 0 is Clockwise
//      Serial.print("Homing Steering motor - Positive  ");
    } else {
      runWheelSteeringMotor(-homing_speed);  // 0-254 is CountClockwise, -254 to 0 is Clockwise
//      Serial.print("Homing Steering motor - Negative  ");
    }
  }

  Serial.println("Homing Steering motor - Complete");
  return homingDone;
}

//------------------------------------------------------------------------

// Reads the raw values for the two rotational position sensors
float readCurrentHeading(int sensor_name_A, int sensor_name_B) {
  FL_sensorValueA0 = returnSensor(sensor_name_A);
  FL_sensorValueA1 = returnSensor(sensor_name_B);
  return calculateHeading(FL_sensorValueA0, FL_sensorValueA1);
}
//------------------------------------------------------------------------

bool home_wheels_when_joystick_pressed() {
  homeWheel = true;
  stopWheelSteeringMotor();
  driveMotorToHome();
  Serial.println("Homing Complete");
  delay(2000);
  homeWheel = false;
  return homeWheel;
}
//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

//------------------------------------------------------------------------

// No printing:  About 1700 loops per second
// Display on OLED: about 6 loops per second
// Disabled OLED and enabled printing at 9600:  19 Loops per second
// Disabled OLED and enabled printing
//------------------------------------------------------------------------
