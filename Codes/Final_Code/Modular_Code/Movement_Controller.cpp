#include "Movement_Controller.h"

MovementController::MovementController(MotorController& motorCtrl, IRSensor& leftIR, IRSensor& rightIR, PIDController& pidCtrl)
  : motorController(motorCtrl), leftIRSensor(leftIR), rightIRSensor(rightIR), pid(pidCtrl) {}


void MovementController::updateState(bool obstacleDetected, char lastDetectedColor, MovementState& currentStateRef, LastSeen& lastSeenRef) {
  bool leftIR = leftIRSensor.isLineDetected();
  bool rightIR = rightIRSensor.isLineDetected();

  if (leftIR && !rightIR) lastSeenRef = LEFT;
  else if (rightIR && !leftIR) lastSeenRef = RIGHT;

  if (obstacleDetected && lastDetectedColor == 'A') {
    currentStateRef = STOP;
  } else if (obstacleDetected && lastDetectedColor == 'B') {
    currentStateRef = FORWARD;
  } else if (!leftIR && !rightIR) {
    if (lastSeenRef == LEFT) currentStateRef = SEARCH_LEFT;
    else if (lastSeenRef == RIGHT) currentStateRef = SEARCH_RIGHT;
    else currentStateRef = STOP;
  } else {
    currentStateRef = FORWARD;
  }
}

void MovementController::act(MovementState state) {
  switch (state) {
    case STOP:
      motorController.stop();
      break;

    case FORWARD: {
      int error = leftIRSensor.isLineDetected() - rightIRSensor.isLineDetected();
      float correction = pid.compute(error);

      int baseSpeed = 100;
      int leftSpeed = constrain(baseSpeed - correction, 0, 255);
      int rightSpeed = constrain(baseSpeed + correction, 0, 255);
      motorController.setSpeed(leftSpeed, rightSpeed);
      break;
    }

    case SEARCH_LEFT:
      motorController.setSpeed(-60, 60);
      break;

    case SEARCH_RIGHT:
      motorController.setSpeed(60, -60);
      break;
  }
}

// === Missing implementations that must be added ===

MovementState& MovementController::getCurrentStateRef() {
  return currentState;
}

LastSeen& MovementController::getLastSeenRef() {
  return lastSeenLine;
}

MovementState MovementController::getCurrentState() const {
  return currentState;
}

void MovementController::setCurrentState(MovementState state) {
  currentState = state;
}
