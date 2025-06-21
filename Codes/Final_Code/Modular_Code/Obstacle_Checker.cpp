#include "Obstacle_Checker.h"
#include <Arduino.h>

ObstacleChecker::ObstacleChecker(UltrasonicSensor& leftU, UltrasonicSensor& rightU,
                                 ColorSensor& cs)
  : leftUltrasonic(leftU), rightUltrasonic(rightU), colorSensor(cs) {}

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

void ObstacleChecker::check(bool shouldStopBeforeColorDetection) {
  unsigned long currentMillis = millis();
  if (currentMillis - lastCheck < interval) return;

  lastCheck = currentMillis;

  int distanceLeft = leftUltrasonic.getDistance();
  int distanceRight = rightUltrasonic.getDistance();
  bool obstacleNow = (distanceLeft > 0 && distanceLeft < 25) ||
                     (distanceRight > 0 && distanceRight < 25);

  if (obstacleNow && !colorDetectedThisCycle) {
    if (shouldStopBeforeColorDetection) {
      Serial.println("Obstacle detected. Waiting before detecting color...");
      delay(100);  // Give robot time to settle if caller already stopped it
    }

    lastDetectedColor = colorSensor.detectColor(colorA, colorB);
    colorDetectedThisCycle = true;
  } else if (!obstacleNow) {
    colorDetectedThisCycle = false;
    lastDetectedColor = 'N';
  }

  obstacleDetected = obstacleNow;
}
