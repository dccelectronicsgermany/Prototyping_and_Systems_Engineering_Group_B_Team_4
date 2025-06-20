#include "Obstacle_Checker.h"
#include <Arduino.h>

ObstacleChecker::ObstacleChecker(UltrasonicSensor& leftU, UltrasonicSensor& rightU,
                                 ColorSensor& cs, MotorController& mc,
                                 MovementController& mv, ObstacleHandler& oh)
  : leftUltrasonic(leftU), rightUltrasonic(rightU), colorSensor(cs),
    motorController(mc), movementController(mv), obstacleHandler(oh) {}

void ObstacleChecker::setColorCalibration(const ColorCalibration& a, const ColorCalibration& b) {
  colorA = a;
  colorB = b;
}

bool ObstacleChecker::isObstacleDetected() const {
  return obstacleDetected;
}

char ObstacleChecker::getLastDetectedColor() const {
  return lastDetectedColor;
}

void ObstacleChecker::check() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastCheck < interval) return;

  int distLeft = leftUltrasonic.getDistance();
  int distRight = rightUltrasonic.getDistance();
  bool obstacleNow = (distLeft > 0 && distLeft < 25) || (distRight > 0 && distRight < 25);

  if (obstacleNow && !colorDetectedThisCycle) {
    motorController.stop();
    char detected = colorSensor.detectColor(colorA, colorB);
    lastDetectedColor = detected;
    colorDetectedThisCycle = true;

    if (detected == 'A') {
      Serial.println("Action: STOP for Color A");
      movementController.setCurrentState(STOP);
      return;
    } else if (detected == 'B') {
      Serial.println("Action: AVOID OBSTACLE for Color B");
      obstacleHandler.avoidObstacle(100, 60);
    }
  } else if (!obstacleNow) {
    colorDetectedThisCycle = false;
    lastDetectedColor = 'N';
  }

  obstacleDetected = obstacleNow;
  lastCheck = currentMillis;
}
