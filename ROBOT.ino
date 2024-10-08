// PS2 Tank by Igor Fonseca @2019
// Controls a robotic tank using a PS2 joystick, using left analog stick
// based on an example using the PS2X library by Bill Porter 2011
// All text above must be included in any redistribution.

// include libraries
#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <Stepper.h>

// These are used to set the direction of the bridge driver.
#define ENA 3      //ENA
#define MOTORA_1 4 //IN3
#define MOTORA_2 5 //IN4
#define MOTORB_1 8 //IN1
#define MOTORB_2 7 //IN2
#define ENB 6      //ENB
#define front_led 46 //rear led always en
#define SERVO1_PIN 39
#define SERVO2_PIN 40
#define STEPPER_IN1 18
#define STEPPER_IN2 19
#define STEPPER_IN3 20
#define STEPPER_IN4 21

int motor_right_speed = 0;
int motor_left_speed = 0;
int fr_lmp = 0;


const int stepsPerRevolution = 2038;

int servo1_angle = 0;
int servo2_angle = 0;

PS2X ps2x; // create PS2 Controller Class
Servo servo1;
Servo servo2;
Stepper stepper1 = Stepper(stepsPerRevolution, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

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
 pinMode(front_led, OUTPUT);

 servo1.attach(SERVO1_PIN);
 servo2.attach(SERVO2_PIN);

 // Disable both motors
 digitalWrite(ENA,0);
 digitalWrite(ENB,0);
 digitalWrite(front_led, LOW);
 
 // Start serial communication
 Serial.begin(57600);
  
 error = ps2x.config_gamepad(13,11,10,12, true, true);   //setup pins and settings:  GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
 
 // Check for error
 if(error == 0){
  Serial.println("Found Controller, configured successful");
 }
   
 else if(error == 1)
  Serial.println("No controller found, check wiring or reset the Arduino");
   
 else if(error == 2)
  Serial.println("Controller found but not accepting commands");
  
 else if(error == 3)
  Serial.println("Controller refusing to enter Pressures mode, may not support it.");
   
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
// Main loop
void loop(){
   
 if(error == 1) //skip loop if no controller found
  return; 
  
 else { //DualShock Controller
  
    ps2x.read_gamepad(false, vibrate); // disable vibration of the controller   
    
    int nJoyLX = ps2x.Analog(PSS_LX); // read x-joystick
    int nJoyLY = ps2x.Analog(PSS_LY); // read y-joystick

    int nJoyRX = ps2x.Analog(PSS_RX);
    int nJoyRY = ps2x.Analog(PSS_RY);


    nJoyRY = map(nJoyRY, 0, 255, -1023, 1023);

    if (ps2x.Button(PSB_GREEN) && fr_lmp == 0) {
        digitalWrite(front_led, HIGH);
        fr_lmp = 1;
    } else if (ps2x.Button(PSB_GREEN) && fr_lmp == 1) {
        fr_lmp = 0;
        digitalWrite(front_led, LOW);
    }
    delay(30);
    
    nJoyLX = map(nJoyLX, 0, 255, -1023, 1023); // X-axis: left/right
    nJoyLY = map(nJoyLY, 0, 255, -1023, 1023); // Y-axis: forward/backward

    nJoyRX = map(nJoyRX, 0, 255, -1023, 1023);
    nJoyRY = map(nJoyRY, 0, 255, -1023, 1023);

    // Якщо джойстик максимально нахилений вперед або назад, ігнорувати невеликі зміщення по осі X
    if (abs(nJoyLY) > 900 && abs(nJoyLX) < 50) {
        nJoyLX = 0; // Ігнорувати X, якщо рух сильно вперед і X незначний
    }
  
    // OUTPUTS
    int nMotMixL; // Motor (left) mixed output
    int nMotMixR; // Motor (right) mixed output
  
    // CONFIG
    float fPivYLimit = 1023.0;
        
    // TEMP VARIABLES
    float nMotPremixL;    // Motor (left) premixed output
    float nMotPremixR;    // Motor (right) premixed output
    int nPivSpeed;        // Pivot Speed
    float fPivScale;      // Balance scale between drive and pivot
  
    if (nJoyRY > 50) {    /// mid: -1063 Top: -9231 Bot: 7185
      servo1_angle += 3;
      servo2_angle += 3;
      Serial.println("Right stick moved");
      Serial.println(nJoyRY);
    } else if (nJoyRY < -1063){
      servo1_angle -= 3;
      servo2_angle -= 3;
      Serial.println("Right stick moved");
      Serial.println(nJoyRY);  
    }

    servo1_angle = constrain(servo1_angle, 0, 180);
    servo2_angle = constrain(servo2_angle, 0, 180);
  
  // Update servo positions
    servo1.write(servo1_angle);
    servo2.write(servo2_angle);

    // Calculate Drive Turn output due to Joystick X input
    if (nJoyLY >= 0) {
      // Forward
      nMotPremixL = (nJoyLX >= 0) ? (1023.0 + nJoyLX) : 1023.0;
      nMotPremixR = (nJoyLX >= 0) ? 1023.0 : (1023.0 - nJoyLX);
    } else {
      // Reverse
      nMotPremixL = (nJoyLX >= 0) ? 1023.0 : (1023.0 - nJoyLX);
      nMotPremixR = (nJoyLX >= 0) ? (1023.0 + nJoyLX) : 1023.0;
    }
  
    // Scale Drive output due to Joystick Y input (throttle)
    nMotPremixL = nMotPremixL * nJoyLY / 1023.0;
    nMotPremixR = nMotPremixR * nJoyLY / 1023.0;
  
    // Calculate pivot amount
    nPivSpeed = nJoyLX;
    fPivScale = (abs(nJoyLY) > fPivYLimit) ? 0.0 : (1.0 - abs(nJoyLY) / fPivYLimit);
  
    // Calculate final mix of Drive and Pivot
    nMotMixL = (1.0 - fPivScale) * nMotPremixL + fPivScale * (nPivSpeed);
    nMotMixR = (1.0 - fPivScale) * nMotPremixR + fPivScale * (-nPivSpeed);
    
    motor_left_speed = nMotMixL;
    motor_right_speed = nMotMixR;
  
    // Motor control
    if (motor_right_speed > 50) {
      digitalWrite(MOTORB_1, HIGH);
      digitalWrite(MOTORB_2, LOW);
    } else if (motor_right_speed < -50) {
      digitalWrite(MOTORB_1, LOW);
      digitalWrite(MOTORB_2, HIGH);
    } else {
      digitalWrite(MOTORB_1, LOW);
      digitalWrite(MOTORB_2, LOW);
    }
  
    if (motor_left_speed > 50) {
      digitalWrite(MOTORA_1, LOW);
      digitalWrite(MOTORA_2, HIGH);
    } else if (motor_left_speed < -50) {
      digitalWrite(MOTORA_1, HIGH);
      digitalWrite(MOTORA_2, LOW);
    } else {
      digitalWrite(MOTORA_1, LOW);
      digitalWrite(MOTORA_2, LOW);
    }
    
    analogWrite(ENA, abs(motor_left_speed));
    analogWrite(ENB, abs(motor_right_speed));
    if (abs(motor_left_speed) > 50 || abs(motor_right_speed) > 50) {
      Serial.println("Moving!");
    }
    
    delay(50);
 }    
}
