// // https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// // // https://github.com/micro-ROS/micro_ros_platformio

// #include <Arduino.h>
// #include <AccelStepper.h>
// const double ANGLES_PER_STEP = 1.8;

// // X
// const int REAR_LEFT_STEP = 4; //Red:24
// const int REAR_LEFT_DIR = 16; //Black:23
// AccelStepper rearLeftWheel(1, REAR_LEFT_STEP, REAR_LEFT_DIR);

// // Z
// const int rear_right_step = 18; //White:20
// const int rear_right_dir = 19;  //Blue:19
// AccelStepper rearRightWheel(1, rear_right_step, rear_right_dir);

// // Y
// const int front_left_step = 12; //Green:24
// const int front_left_dir = 14;  //Orange:23
// AccelStepper frontLeftWheel(1, front_left_step, front_left_dir);

// // A
// const int front_right_step = 25;  //Yellow:20
// const int front_right_dir = 33;   //violet:19
// AccelStepper frontRightWheel(1, front_right_step, front_right_dir);

// const int MAX_SPEED=500;
// double convertRadiansPerSecondToStepsPerSecond(double angularVelocity)
// {
//     const double angularVelocityDegrees = angularVelocity * RAD_TO_DEG;
//     return angularVelocityDegrees / ANGLES_PER_STEP;
// }

// void setup()
// {
//     Serial.begin(115200);
//     double speed = convertRadiansPerSecondToStepsPerSecond(3.14);

//     rearLeftWheel.setMaxSpeed(MAX_SPEED);
//     rearLeftWheel.setSpeed(speed);


//     frontLeftWheel.setMaxSpeed(MAX_SPEED);
//     frontLeftWheel.setSpeed(speed);


//     rearRightWheel.setMaxSpeed(MAX_SPEED);
//     rearRightWheel.setSpeed(speed);


//     frontRightWheel.setMaxSpeed(MAX_SPEED);
//     frontRightWheel.setSpeed(speed);
// }

// void loop()
// {
//     rearLeftWheel.runSpeed();
//     frontLeftWheel.runSpeed();
//     rearRightWheel.runSpeed();
//     frontRightWheel.runSpeed();
// }
