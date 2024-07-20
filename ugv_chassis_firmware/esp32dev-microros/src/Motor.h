#ifndef MOTOR_H
#define MOTOR_H
#include <AccelStepper.h>
class Motor
{

private:
    const double ANGLES_PER_STEP = 1.8;
    const int MAX_SPEED = 500;
    AccelStepper *steppr;
    double convertRadiansPerSecondToStepsPerSecond(double angularVelocity);

public:
    Motor(int step, int dir);
    void setSpeed(double angularVelocity);
    double getCurrentPosition();
    void run();
};
#endif