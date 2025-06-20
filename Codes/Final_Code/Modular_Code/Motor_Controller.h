#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "Motor.h"

class MotorController {
  public:
    MotorController(Motor& leftMotor, Motor& rightMotor);
    void setSpeed(int leftSpeed, int rightSpeed);
    void stop();

  private:
    Motor& _leftMotor;
    Motor& _rightMotor;
};

#endif
