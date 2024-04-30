//
// Created by omar on 4/2/24.
//

#include "TwoPinStepperMotor.h"

TwoPinStepperMotor::TwoPinStepperMotor(byte stepPin,
                                       byte dirPin,
                                       bool isLeft) {
    directionMultiplier_ = isLeft ? 1 : -1;
    commandLastPing_ = millis();
    accelStepper = new AccelStepper(1, stepPin, dirPin);

    accelStepper->setMaxSpeed(MAX_STEPS_PER_SECOND);
}


double TwoPinStepperMotor::getPosition() const {
    int currentPosition = accelStepper->currentPosition();
    currentPosition *= ANGLES_PER_STEP;
    return currentPosition * CONVERT_DEG_TO_RAD * directionMultiplier_;
}

void TwoPinStepperMotor::setVelocity(double angularVelocity) {
    commandLastPing_ = millis();
    const double angularVelocityDegrees = angularVelocity * CONVERT_RAD_TO_DEG;
    const double stepsPerSecond = angularVelocityDegrees / ANGLES_PER_STEP;
    accelStepper->setSpeed(stepsPerSecond * directionMultiplier_);
}

void TwoPinStepperMotor::move() {
    // if (millis() - commandLastPing_ >= DEAD_MAN_SWITCH_TIMEOUT_MILLI_SEC) {
    //     accelStepper->setSpeed(0);
    // }
    accelStepper->runSpeed();
}
