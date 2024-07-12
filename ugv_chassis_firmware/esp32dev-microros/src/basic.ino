// // https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// // https://github.com/micro-ROS/micro_ros_platformio
// // https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html

/*** CNC v3 connections : STEP/DIR
 * Red Black
 * Green Orange
 * White Blue
 * Yellow Violet
****/

// #include <Arduino.h>
// #include <AccelStepper.h>
// const double ANGLES_PER_STEP = 1.8;

// // X
// const int REAR_LEFT_STEP = 4; // Red:24
// const int REAR_LEFT_DIR = 16; // Black:23
// AccelStepper rearLeftWheel(AccelStepper::DRIVER, REAR_LEFT_STEP, REAR_LEFT_DIR);

// // Z
// const int rear_right_step = 18; // White:20
// const int rear_right_dir = 19;  // Blue:19
// AccelStepper rearRightWheel(AccelStepper::DRIVER, rear_right_step, rear_right_dir);

// // Y
// const int front_left_step = 12; // Green:24
// const int front_left_dir = 14;  // Orange:23
// AccelStepper frontLeftWheel(AccelStepper::DRIVER, front_left_step, front_left_dir);

// // A
// const int front_right_step = 25; // Yellow:20
// const int front_right_dir = 33;  // violet:19
// AccelStepper frontRightWheel(AccelStepper::DRIVER, front_right_step, front_right_dir);

// const int MAX_SPEED = 500;
// double speed;
// long previousMillis = 0;
// long interval = 1000;

// double convertRadiansPerSecondToStepsPerSecond(double angularVelocity)
// {
//     const double angularVelocityDegrees = angularVelocity * RAD_TO_DEG;
//     return angularVelocityDegrees / ANGLES_PER_STEP;
// }

// void setSpeed()
// {
//     rearLeftWheel.setSpeed(speed);
//     frontLeftWheel.setSpeed(speed);
//     rearRightWheel.setSpeed(speed);
//     frontRightWheel.setSpeed(speed);
// }

// void setup()
// {
//     Serial.begin(115200);
//     speed = convertRadiansPerSecondToStepsPerSecond(3.14);

//     rearLeftWheel.setMaxSpeed(MAX_SPEED);
//     frontLeftWheel.setMaxSpeed(MAX_SPEED);
//     rearRightWheel.setMaxSpeed(MAX_SPEED);
//     frontRightWheel.setMaxSpeed(MAX_SPEED);

//     setSpeed();
// }

// void loop()
// {
//     rearLeftWheel.runSpeed();
//     frontLeftWheel.runSpeed();
//     rearRightWheel.runSpeed();
//     frontRightWheel.runSpeed();

//     unsigned long currentMillis = millis();

//     if (currentMillis - previousMillis > interval)
//     {
//         previousMillis = currentMillis;
//         speed *= -1;
//         setSpeed();
//     }
// }
