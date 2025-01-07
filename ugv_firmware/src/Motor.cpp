
#include <Motor.h>
Motor::Motor(int step, int dir)
{
     steppr = new AccelStepper(AccelStepper::DRIVER, step, dir);
     steppr->setMaxSpeed(MAX_SPEED);
}

double Motor::convertRadiansPerSecondToStepsPerSecond(double angularVelocity)
{
     const double angularVelocityDegrees = angularVelocity * RAD_TO_DEG;
     return angularVelocityDegrees / ANGLES_PER_STEP;
}

void Motor::setSpeed(double angularVelocity)
{
     lastCommandTime = millis();
     auto stepsPerSecond = convertRadiansPerSecondToStepsPerSecond(angularVelocity);
     steppr->setSpeed(stepsPerSecond);
}

double Motor::getCurrentPosition() { return steppr->currentPosition() * ANGLES_PER_STEP * DEG_TO_RAD; }

void Motor::run()
{
     const float elapsedTime = (millis() - lastCommandTime) / 1000;
     if (elapsedTime >= 1)
     {
          return;
     }
     steppr->runSpeed();
}