// https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
// https://www.makerstore.com.au/wp-content/uploads/filebase/publications/CNC-Shield-Guide-v1.0.pdf
// https://github.com/makerbase-mks/MKS-SERVO42C/wiki/Menu-instruction

/*** CNC v3 connections :
 ******STEP           DIR  
 *  X  Brown  26      Purple 15
 *  Y  Green  27      Blue    2
 *  Z  White  14      Orange  0
 *  A  Yellow 12      Grey    4
 * 
 ******Black  GND  
****/

// #include <Arduino.h>
// #include <AccelStepper.h>
// const double ANGLES_PER_STEP = 1.8;

// // X
// const int REAR_LEFT_STEP = 26; 
// const int REAR_LEFT_DIR = 15; 
// AccelStepper rearLeftWheel(AccelStepper::DRIVER, REAR_LEFT_STEP, REAR_LEFT_DIR);

// // Z
// const int REAR_RIGHT_STEP = 14; 
// const int REAR_RIGHT_DIR = 0;  
// AccelStepper rearRightWheel(AccelStepper::DRIVER, REAR_RIGHT_STEP, REAR_RIGHT_DIR);

// // Y
// const int FRONT_LEFT_STEP = 27;
// const int FRONT_LEFT_DIR = 2;
// AccelStepper frontLeftWheel(AccelStepper::DRIVER, FRONT_LEFT_STEP, FRONT_LEFT_DIR);

// // A
// const int FRONT_RIGHT_STEP = 12; 
// const int FRONT_RIGHT_DIR = 4;
// AccelStepper frontRightWheel(AccelStepper::DRIVER, FRONT_RIGHT_STEP, FRONT_RIGHT_DIR);

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
