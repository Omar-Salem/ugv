//
// Created by omar on 4/2/24.
//

#ifndef UGV_STEPPERMOTOR_H
#define UGV_STEPPERMOTOR_H

#include <AccelStepper.h>

class TwoPinStepperMotor {
private:
    AccelStepper *accelStepper;
    const int FULLSTEP = 4;
    const int STEPS_PER_REVOLUTION = 2048;
    const int MAX_STEPS_PER_SECOND = STEPS_PER_REVOLUTION / FULLSTEP;
    const double ANGLES_PER_STEP = 0.17578125;
    const double CONVERT_DEG_TO_RAD = 0.0174533;
    const double MAX_ANGULAR_VELOCITY = MAX_STEPS_PER_SECOND*ANGLES_PER_STEP * CONVERT_DEG_TO_RAD;//1.570797 r/s
    const double CONVERT_RAD_TO_DEG = 57.29578;
    int directionMultiplier_;
    long commandLastPing_ ;
    const long DEAD_MAN_SWITCH_TIMEOUT_MILLI_SEC=1000;

public:
    TwoPinStepperMotor(byte stepPin,
                        byte dirPin,
                        bool isLeft);

    double getPosition() const;

    void setVelocity(double angularVelocity);

    void move();
};


#endif //UGV_STEPPERMOTOR_H
