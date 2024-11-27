// PS2 Tank by Igor Fonseca @2019
// Controls a robotic tank using a PS2 joystick, using D-pad buttons
// based on an example using the PS2X library by Bill Porter 2011
// All text above must be included in any redistribution.

// include libraries
#include <PS2X_lib.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
#include <Servo.h>

// These are used to set the direction of the bridge driver.
#define ENA 3      //ENA
#define MOTORA_1 4 //IN3
#define MOTORA_2 5 //IN4
#define MOTORB_1 8 //IN1
#define MOTORB_2 7 //IN2
#define ENB 6      //ENB
#define DIR 46
#define STEP 47
#define SERVO_1 21
#define SERVO_2 20
#define SERVO_3 19
#define SERVO_4 18

PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// Variables for servo control
int servo1StartPosition = 90;  // initial start position for servo 1
int servo2StartPosition = 90;  // initial start position for servo 2
int servo3Position = 90; // initial position for servo 3
int servo4Direction = 0; // 0 = one direction, 1 = the other

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you conect the controller, 
//or call config_gamepad(pins) again after connecting the controller.
int error = 0; 
byte type = 0;
byte vibrate = 0;

void setup(){

 // Configure output pins
 pinMode(ENA, OUTPUT);
 pinMode(MOTORA_1, OUTPUT);
 pinMode(MOTORA_2, OUTPUT);
 pinMode(ENB, OUTPUT);
 pinMode(MOTORB_1, OUTPUT);
 pinMode(MOTORB_2, OUTPUT);

 // servo setup
 servo1.attach(SERVO_1);
 servo2.attach(SERVO_2);
 servo3.attach(SERVO_3);
 servo4.attach(SERVO_4);

 // Initialize servos to their current positions instead of 0
 servo1.write(servo1StartPosition);
 servo2.write(servo2StartPosition);
 servo3.write(servo3Position);  // Initialize servo3 to its starting position

 // Disable both motors
 digitalWrite(ENA,0);
 digitalWrite(ENB,0);

 // Start serial communication
 Serial.begin(57600);
  
 error = ps2x.config_gamepad(13,11,10,12, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
   
 // Check for the type of controller
 type = ps2x.readType();
 switch(type) {
  case 0:
    Serial.println("Unknown Controller type");
    break;
  case 1:
    Serial.println("DualShock Controller Found");
    break;
  case 2:
    Serial.println("GuitarHero Controller Found");
    break;
  }
}

// Main loop
void loop(){
   
 if(error == 1) //skip loop if no controller found
  return; 
   
 else { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); // disable vibration of the controller

     // Perform movements based on D-pad buttons
     
     // MOVE FORWARD
     if(ps2x.Button(PSB_PAD_UP)) {
        digitalWrite(MOTORA_1,HIGH);
        digitalWrite(MOTORA_2,LOW);
        digitalWrite(MOTORB_1,LOW);
        digitalWrite(MOTORB_2,HIGH);
        analogWrite(ENB, 1023);
        analogWrite(ENA, 930);
        Serial.println("Move forward");
      }
      // TURN RIGHT
      if(ps2x.Button(PSB_PAD_RIGHT)){
        digitalWrite(MOTORA_1,LOW);
        digitalWrite(MOTORA_2,HIGH);
        digitalWrite(MOTORB_1,LOW);
        digitalWrite(MOTORB_2,HIGH);
        analogWrite(ENB, 1023);
        analogWrite(ENA, 930);
        Serial.println("Turn right");
      }
      // TURN LEFT
      if(ps2x.Button(PSB_PAD_LEFT)){
        digitalWrite(MOTORA_1,HIGH);
        digitalWrite(MOTORA_2,LOW);
        digitalWrite(MOTORB_1,HIGH);
        digitalWrite(MOTORB_2,LOW);
        analogWrite(ENB, 1023);
        analogWrite(ENA, 930);
        Serial.println("Turn left");
      }
      // MOVE BACK
      if(ps2x.Button(PSB_PAD_DOWN)){
        digitalWrite(MOTORA_1,LOW);
        digitalWrite(MOTORA_2,HIGH);
        digitalWrite(MOTORB_1,HIGH);
        digitalWrite(MOTORB_2,LOW);
        analogWrite(ENB, 1023);
        analogWrite(ENA, 930);         
        Serial.println("Move back");
      }  
      if (!ps2x.Button(PSB_PAD_DOWN) && !ps2x.Button(PSB_PAD_UP) && !ps2x.Button(PSB_PAD_RIGHT) && !ps2x.Button(PSB_PAD_LEFT)) {
        analogWrite(ENB, 0);
        analogWrite(ENA, 0);
      }

      // Control servos with right joystick (Y axis)
      int rightY = ps2x.Analog(PSS_RY); // Get right joystick Y axis value (up/down)
      Serial.print("Right Stick Y: ");
      Serial.println(rightY);
      
      // If the right joystick is at the center (127), do nothing
      if (rightY != 127) {
        // Map the joystick input to servo angles (0-255 to 0-180 degrees)
        int servoAngle = map(rightY, 0, 255, 0, 180);  // Up -> 0 degrees, Down -> 180 degrees
        
        // Slowly change the angle to make the movement smoother (increase/decrease gradually)
        int currentAngle1 = servo1.read();
        int currentAngle2 = servo2.read();

        // Adjust the angle speed by limiting how much it changes per loop
        if (abs(currentAngle1 - servoAngle) > 1) {
          if (currentAngle1 < servoAngle) {
            servo1.write(currentAngle1 + 3);  // Increment gradually
          } else {
            servo1.write(currentAngle1 - 3);  // Decrement gradually
          }
        }

        if (abs(currentAngle2 - servoAngle) > 1) {
          if (currentAngle2 < servoAngle) {
            servo2.write(currentAngle2 + 3);  // Increment gradually
          } else {
            servo2.write(currentAngle2 - 3);  // Decrement gradually
          }
        }

        Serial.print("Servo 1 and Servo 2 angle: ");
        Serial.println(servoAngle);
      }

      // Control Servo 3 based on R2 and R1 buttons
      if (ps2x.Button(PSB_R2)) {
        // Rotate in one direction when R2 is pressed
        servo3Position += 3;  // Adjust the position by 1 degree
        if (servo3Position > 180) servo3Position = 180;  // Limit the max angle to 180
        servo3.write(servo3Position);  // Apply the position
        Serial.print("Servo 3 Position: ");
        Serial.println(servo3Position);
      }
      
      if (ps2x.Button(PSB_R1)) {
        // Rotate in the opposite direction when R1 is pressed
        servo3Position -= 3;  // Adjust the position by -1 degree
        if (servo3Position < 0) servo3Position = 0;  // Limit the min angle to 0
        servo3.write(servo3Position);  // Apply the position
        Serial.print("Servo 3 Position: ");
        Serial.println(servo3Position);
      }

      // Control Servo 4 based on Green Triangle and Square buttons
    if (ps2x.ButtonPressed(PSB_GREEN)) {
        // Reverse the direction of servo 4 each time the Green Triangle is pressed
        servo4Direction = 1 - servo4Direction; // Toggle between 0 and 1
        if (servo4Direction == 0) {
            servo4.write(0);  // Set to one direction (0 degrees)
        } else {
            servo4.write(180);  // Set to the other direction (180 degrees)
        }
        Serial.print("Servo 4 Direction: ");
        Serial.println(servo4Direction);
    }

    if (ps2x.ButtonPressed(PSB_RED)) {
        // Set servo 4 to another fixed position based on the Square button
        servo4.write(90);  // Set to a neutral position (90 degrees)
        Serial.println("Servo 4 set to 90 degrees");
    }

    // Fix: Correct the missing closing parenthesis and add a semicolon
    delay(50); // Small delay to make the loop run smoothly

 }
}
