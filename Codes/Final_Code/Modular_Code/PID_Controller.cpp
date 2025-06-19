#include "PID_Controller.h"

PIDController::PIDController(float kp, float ki, float kd)
  : _kp(kp), _ki(ki), _kd(kd), _previousError(0), _integral(0) {}

float PIDController::compute(float error) {
  float derivative = error - _previousError;
  _integral += error;
  float output = _kp * error + _ki * _integral + _kd * derivative;
  _previousError = error;
  return output;
}

void PIDController::reset() {
  _previousError = 0;
  _integral = 0;
}
