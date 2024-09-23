# PS2 Tank Robot Controller

**Author**: [SACEIT](https://saceit.org.ua)
**Based on the PS2X library example by Bill Porter (2011)**

## Description

This project uses a PlayStation 2 joystick to control a robotic tank. The system allows you to control the tank's movement, steering, servos, stepper motor, and also includes an ultrasonic sensor to detect obstacles. Additionally, an **autopilot mode** is implemented, allowing the robot to automatically avoid obstacles by using the ultrasonic sensor to measure distances.

## Features

- **Tank Control**: Use the left analog stick of the PS2 joystick to control the tank's movement (forward, backward, left, right).
- **Headlights**: Control the robot's front LED using the green button on the PS2 joystick.
- **Manipulator Control**: Use the right analog stick to control two servos responsible for the manipulator's movement.
- **Stepper Motor Control**: Rotate the manipulator using the right analog stick's horizontal axis (X).
- **Obstacle Detection**: The ultrasonic sensor detects obstacles, and if something is closer than a set threshold, the robot stops or turns.
- **Autopilot Mode**: Activated by pressing the red button on the PS2 controller. In autopilot mode, the robot moves forward while avoiding obstacles using the ultrasonic sensor.

## Components

- **PS2 Controller**: For controlling the robot remotely.
- **Motor Driver**: Controls the left and right motors for movement.
- **Servos**: Two servos to control the robot's manipulator.
- **Stepper Motor**: Rotates the manipulator.
- **Ultrasonic Sensor**: Detects obstacles and helps avoid collisions in autopilot mode.
- **LED**: Controls the front LED to simulate headlights.

## Pin Setup

| Component           | Pin Number |
|---------------------|------------|
| Motor A Enable       | 3          |
| Motor A IN3          | 4          |
| Motor A IN4          | 5          |
| Motor B IN1          | 8          |
| Motor B IN2          | 7          |
| Motor B Enable       | 6          |
| Front LED            | 46         |
| Servo 1              | 39         |
| Servo 2              | 40         |
| Stepper Motor IN1    | 18         |
| Stepper Motor IN2    | 19         |
| Stepper Motor IN3    | 20         |
| Stepper Motor IN4    | 21         |
| Ultrasonic Trig Pin  | 48         |
| Ultrasonic Echo Pin  | 49         |

## Libraries Used

- **PS2X_lib**: Library for interfacing with PS2 controllers.
- **Servo**: Standard Arduino library to control servo motors.
- **AccelStepper**: Library for controlling stepper motors.

## How to Use

1. Connect the components to the correct pins on your Arduino board as specified in the **Pin Setup** section.
2. Upload the provided code to the Arduino.
3. Use the PS2 joystick to control the robot:
   - **Left analog stick** controls movement.
   - **Right analog stick** controls the manipulator.
   - **Green button** toggles the front LED.
   - **Red button** activates/deactivates the autopilot mode.

In **autopilot mode**, the robot will move forward and avoid obstacles detected by the ultrasonic sensor.

## Autopilot Functionality

When autopilot is activated:
- If the ultrasonic sensor detects an obstacle within the set distance threshold, the robot will turn to avoid the obstacle.
- The robot will continue moving forward once the path is clear.

## Requirements

- Arduino board
- Motor driver (e.g., L298N)
- PlayStation 2 controller
- Servo motors
- Stepper motor
- Ultrasonic sensor (e.g., HC-SR04)
- PS2X_lib for Arduino (available [here](https://github.com/madsci1016/Arduino-PS2X))

## License

This project is open source, and redistribution must include attribution to the original authors. Any modifications must retain the original authorship credits in accordance with the terms of the project.
