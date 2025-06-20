#include "Motor_Controller.h"

MotorController::MotorController(Motor& leftMotor, Motor& rightMotor)
  : _leftMotor(leftMotor), _rightMotor(rightMotor) {}

void MotorController::setSpeed(int leftSpeed, int rightSpeed) {
  // Determine direction and speed for left motor
  Motor::Direction leftDirection = (leftSpeed >= 0)
    ? Motor::Direction::FORWARD
    : Motor::Direction::BACKWARD;

  Motor::Direction rightDirection = (rightSpeed >= 0)
    ? Motor::Direction::FORWARD
    : Motor::Direction::BACKWARD;

  _leftMotor.setSpeed(abs(leftSpeed), leftDirection);
  _rightMotor.setSpeed(abs(rightSpeed), rightDirection);
}

void MotorController::stop() {
  _leftMotor.stop();
  _rightMotor.stop();
}
