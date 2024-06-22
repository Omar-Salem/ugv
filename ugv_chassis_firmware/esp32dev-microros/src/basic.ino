// // https://www.hackster.io/514301/micro-ros-on-esp32-using-arduino-ide-1360ca
// // https://github.com/micro-ROS/micro_ros_platformio

// #include <Arduino.h>
// #include <AccelStepper.h>
// const double ANGLES_PER_STEP = 1.8;

// const int right_step = 4;
// const int right_dir = 16;
// AccelStepper right(AccelStepper::DRIVER, right_step, right_dir);

// const int left_step = 12;
// const int left_dir = 14;
// AccelStepper left(AccelStepper::DRIVER, left_step, left_dir);

// AccelStepper z(AccelStepper::DRIVER, 18, 19);

// double convertRadiansPerSecondToStepsPerSecond(double angularVelocity)
// {
//     const double angularVelocityDegrees = angularVelocity * RAD_TO_DEG;
//     return angularVelocityDegrees / ANGLES_PER_STEP;
// }

// void setup()
// {
//     // Serial.begin(115200);
//     double speed = convertRadiansPerSecondToStepsPerSecond(2*3.14);
//     // speed = 200;
//     left.setMaxSpeed(20000);
//     left.setSpeed(speed);

//     right.setMaxSpeed(20000);
//     right.setSpeed(speed);

//     // z.setMaxSpeed(20000);
//     // z.setSpeed(-1000);

//     // accelStepper.setSpeed(-2000);
//     // stepper.setSpeed(480);
// }

// void loop()
// {
//     // left.runSpeed();
//     right.runSpeed();
//     // z.runSpeed();
// }