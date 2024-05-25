#include <AccelStepper.h>

const int stepXPin = 18;
const int dirXPin = 19;
AccelStepper x_motor(1, stepXPin, dirXPin);

const int stepYPin = 4;
const int dirYPin = 16;
AccelStepper y_motor(1, stepYPin, dirYPin);

const int stepZPin = 12;
const int dirZPin = 14;
AccelStepper z_motor(1, stepZPin, dirZPin);

const int stepAPin = 25;
const int dirAPin = 33;
AccelStepper a_motor(1, stepAPin, dirAPin);

void setup() {

    int max = 3000;
    int v = 1000;

    x_motor.setMaxSpeed(max);
    x_motor.setSpeed(v);

    y_motor.setMaxSpeed(max);
    y_motor.setSpeed(v);


    z_motor.setMaxSpeed(max);
    z_motor.setSpeed(v);

    a_motor.setMaxSpeed(max);
    a_motor.setSpeed(v);
}

void loop() {

    x_motor.runSpeed();
    y_motor.runSpeed();
    z_motor.runSpeed();
    a_motor.runSpeed();
}