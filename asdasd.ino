// PS2 Tank by Igor Fonseca @2019 
// Controls a robotic tank using a PS2 joystick, using left analog stick 
// based on an example using the PS2X library by Bill Porter 2011 
// All text above must be included in any redistribution.

// include libraries
#include <PS2X_lib.h>  //for v1.6
#include <Servo.h>
#include <AccelStepper.h>

// Pin Definitions
#define ENA 3      // Motor A enable
#define MOTORA_1 4 // Motor A IN3
#define MOTORA_2 5 // Motor A IN4
#define MOTORB_1 8 // Motor B IN1
#define MOTORB_2 7 // Motor B IN2
#define ENB 6      // Motor B enable
#define FRONT_LED 46 // Front LED
#define SERVO1_PIN 39
#define SERVO2_PIN 40
#define STEPPER_IN1 18
#define STEPPER_IN2 19
#define STEPPER_IN3 20
#define STEPPER_IN4 21
#define MotorInterfaceType 4
#define trigPin 48
#define echoPin 49

long duration, cm, inches;
const int stepsPerRevolution = 2038;
int motor_right_speed = 0;
int motor_left_speed = 0;
int fr_lmp = 0;
bool autopilot = false; // Стан автопілота

PS2X ps2x; // PS2 Controller Class
Servo servo1, servo2;

AccelStepper myStepper(MotorInterfaceType, STEPPER_IN1, STEPPER_IN3, STEPPER_IN2, STEPPER_IN4);

int error = 0;
byte type = 0;
byte vibrate = 0;

int servo1_angle = 90;
int servo2_angle = 90;

int distance_threashhold = 15;

void setup() {
  myStepper.setMaxSpeed(3000);
  myStepper.setAcceleration(5000);
  myStepper.setSpeed(2000);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(MOTORA_1, OUTPUT);
  pinMode(MOTORA_2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOTORB_1, OUTPUT);
  pinMode(MOTORB_2, OUTPUT);
  pinMode(FRONT_LED, OUTPUT);

  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);

  digitalWrite(ENA, 0);
  digitalWrite(ENB, 0);
  digitalWrite(FRONT_LED, LOW);

  Serial.begin(57600);

  error = ps2x.config_gamepad(13, 11, 10, 12, true, true);

  if (error == 0) {
    Serial.println("Controller found, configured successfully");
  } else {
    Serial.println(error == 1 ? "No controller found" : "Controller found but not responding");
  }

  type = ps2x.readType();
  Serial.println(type == 1 ? "DualShock Controller Found" : (type == 2 ? "GuitarHero Controller Found" : "Unknown Controller"));
}

void loop() {
  if (error == 1) return;

  ps2x.read_gamepad(false, vibrate);
  handleJoystick();
  handleLED();
  handleServos();
  handleStepper();
  handleMotors();
  handleSonar();
  handleAutopilot();  // Додаємо функцію автопілота
}

// Управління джойстиком
void handleJoystick() {
  int nJoyLX = map(ps2x.Analog(PSS_LX), 0, 255, -1023, 1023);
  int nJoyLY = map(ps2x.Analog(PSS_LY), 0, 255, -1023, 1023);

  if (abs(nJoyLY) > 900 && abs(nJoyLX) < 50) nJoyLX = 0;

  float fPivYLimit = 1023.0;
  float nMotPremixL = nJoyLX;
  float nMotPremixR = nJoyLX;

  nMotPremixL += nJoyLY;
  nMotPremixR -= nJoyLY;

  motor_left_speed = nMotPremixL;
  motor_right_speed = nMotPremixR;
}

// Фари
void handleLED() {
  if (ps2x.Button(PSB_GREEN)) {
    fr_lmp = !fr_lmp;
    digitalWrite(FRONT_LED, fr_lmp ? HIGH : LOW);
    delay(30);
  }
}

// Управління маніпулятором
void handleServos() {
  int nJoyRY = map(ps2x.Analog(PSS_RY), 0, 255, -1023, 1023);

  if (nJoyRY > 50) {
    servo1_angle += 3;
    servo2_angle += 3;
    Serial.println(nJoyRY);
  } else if (nJoyRY <= -50) {
    servo1_angle -= 3;
    servo2_angle -= 3;
    Serial.println(nJoyRY);
  }

  servo1_angle = constrain(servo1_angle, 0, 180);
  servo2_angle = constrain(servo2_angle, 0, 180);

  servo1.write(servo1_angle);
  servo2.write(servo2_angle);
}

// Управління кроковим двигуном (обертання маніпулятора)
void handleStepper() {
  int nJoyRX = map(ps2x.Analog(PSS_RX), 0, 255, -1023, 1023);

  if (nJoyRX > 50) {
    myStepper.moveTo(myStepper.currentPosition() + 700);
    Serial.println(nJoyRX);
  } else if (nJoyRX < -50) {
    myStepper.moveTo(myStepper.currentPosition() - 700);
    Serial.println(nJoyRX);
  }

  myStepper.run();
}

// Управління двигунами
void handleMotors() {
  controlMotor(MOTORA_1, MOTORA_2, motor_right_speed); // Right motor control on left
  controlMotor(MOTORB_1, MOTORB_2, motor_left_speed);  // Left motor control on right

  analogWrite(ENA, abs(motor_left_speed));  // Speed control for left motor
  analogWrite(ENB, abs(motor_right_speed)); // Speed control for right motor

  if (abs(motor_left_speed) > 50 || abs(motor_right_speed) > 50) {
    Serial.println("Moving!");
  }

  delay(50);
}

void controlMotor(int pin1, int pin2, int speed) {
  if (speed > 50) {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  } else if (speed < -50) {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  } else {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

// Управління автопілотом
void handleAutopilot() {
  if (ps2x.ButtonPressed(PSB_RED)) { // Натискання червоної кнопки вмикає/вимикає автопілот
    autopilot = !autopilot;
    Serial.println(autopilot ? "Autopilot ON" : "Autopilot OFF");
  }

  if (autopilot) {
    handleSonar(); // Використовуємо сонар для уникнення перешкод
    if (cm >= distance_threashhold) {
      // Рух вперед, якщо немає перешкод
      motor_left_speed = 200;
      motor_right_speed = 200;
    } else {
      // Повертаємо, щоб уникнути перешкоду
      motor_left_speed = -200;
      motor_right_speed = 200;
      delay(500); // Короткий розворот
    }
    handleMotors(); // Оновлюємо стан моторів
  }
}

void handleSonar() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH);

  cm = (duration / 2) / 29.1; // Перетворення часу у відстань в сантиметрах
  inches = (duration / 2) / 74; // Перетворення часу у відстань в дюймах

  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();

  if (cm < distance_threashhold) {
    StopMotors(); // Зупиняємо танк, якщо перешкода занадто близько
  }

  delay(250);
}

void StopMotors() {
  motor_left_speed = 0;
  motor_right_speed = 0;

  handleMotors();
}
