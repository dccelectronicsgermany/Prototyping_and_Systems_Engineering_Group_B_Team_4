#include "Obstacle_Handler.h"
#include <Arduino.h>

ObstacleHandler::ObstacleHandler(MotorController& mc, MovementController& mv, ObstacleChecker& checker, IRSensor& leftIR, IRSensor& rightIR)
  : motorController(mc), movementController(mv), obstacleChecker(checker), leftIRSensor(leftIR), rightIRSensor(rightIR) {}

void ObstacleHandler::handleObstacle() {
  Serial.println("Avoiding obstacle: Reversing...");
  reverse(300);

  Serial.println("Pivoting left...");
  pivotLeft(500);

  Serial.println("Moving forward to bypass obstacle...");
  moveForward(500);
  if (searchForLine(500)) return;

  obstacleChecker.check();

  Serial.println("Realigning right...");
  pivotRight(700);
  if (searchForLine(500)) return;

  obstacleChecker.check();

  Serial.println("Moving forward again...");
  moveForward(500);
  if (searchForLine(500)) return;

  obstacleChecker.check();

  Serial.println("Searching for line...");
  if (searchForLine(1500)) return;

  Serial.println("Line not found, initiating search pattern...");

  for (int i = 0; i < 3; i++) {
    Serial.print("Sweep attempt "); Serial.println(i + 1);

    Serial.println(" → Nudge forward");
    motorController.setSpeed(100, 100);
    delay(400);
    motorController.stop();

    bool leftIR = leftIRSensor.isLineDetected();
    bool rightIR = rightIRSensor.isLineDetected();
    if (leftIR || rightIR) {
      Serial.println("Line found during left sweep!");
      motorController.stop();
      movementController.setCurrentState(FORWARD);
      return;
    }

    Serial.println(" → Sweep left");
    motorController.setSpeed(-60, 60);
    delay(500);
    motorController.stop();

    leftIR = leftIRSensor.isLineDetected();
    rightIR = rightIRSensor.isLineDetected();
    if (leftIR || rightIR) {
      Serial.println("Line found during left sweep!");
      motorController.stop();
      movementController.setCurrentState(FORWARD);
      return;
    }

    Serial.println(" → Sweep right");
    motorController.setSpeed(60, -60);
    delay(1000);
    motorController.stop();

    leftIR = leftIRSensor.isLineDetected();
    rightIR = rightIRSensor.isLineDetected();
    if (leftIR || rightIR) {
      Serial.println("Line found during right sweep!");
      motorController.stop();
      movementController.setCurrentState(FORWARD);
      return;
    }

    Serial.println(" → Return to center");
    motorController.setSpeed(-60, 60);
    delay(500);
    motorController.stop();
  }

  Serial.println("Line not found after search pattern. Stopping.");
  movementController.setCurrentState(STOP);
  motorController.stop();
}

void ObstacleHandler::reverse(unsigned long duration) {
  unsigned long startTime = millis();
  motorController.setSpeed(-100, -100);
  while (millis() - startTime < duration) {
    obstacleChecker.check();
    delay(10);
  }
  motorController.stop();
}

void ObstacleHandler::pivotLeft(unsigned long duration) {
  unsigned long startTime = millis();
  motorController.setSpeed(-60, 60);
  while (millis() - startTime < duration) {
    obstacleChecker.check();
    delay(10);
  }
  motorController.stop();
}

void ObstacleHandler::pivotRight(unsigned long duration) {
  unsigned long startTime = millis();
  motorController.setSpeed(60, -60);
  while (millis() - startTime < duration) {
    obstacleChecker.check();
    delay(10);
  }
  motorController.stop();
}

void ObstacleHandler::moveForward(unsigned long duration) {
  unsigned long startTime = millis();
  motorController.setSpeed(50, 50);
  while (millis() - startTime < duration) {
    obstacleChecker.check();
    bool leftIR = leftIRSensor.isLineDetected();
    bool rightIR = rightIRSensor.isLineDetected();
    if (leftIR || rightIR) {
      motorController.stop();
      movementController.setCurrentState(FORWARD);
      return;
    }
    delay(10);
  }
  motorController.stop();
}

bool ObstacleHandler::searchForLine(unsigned long maxDuration) {
  unsigned long startTime = millis();
  motorController.setSpeed(60, 60);
  while (millis() - startTime < maxDuration) {
    bool leftIR = leftIRSensor.isLineDetected();
    bool rightIR = rightIRSensor.isLineDetected();
    if (leftIR || rightIR) {
      motorController.stop();
      if (leftIR && !rightIR) movementController.getLastSeenRef() = LEFT;
      else if (rightIR && !leftIR) movementController.getLastSeenRef() = RIGHT;
      movementController.setCurrentState(FORWARD);
      return true;
    }
    delay(10);
  }
  motorController.stop();
  return false;
}
