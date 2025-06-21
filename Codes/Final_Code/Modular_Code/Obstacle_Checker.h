#ifndef OBSTACLE_CHECKER_H
#define OBSTACLE_CHECKER_H

#include "Ultrasonic_Sensor.h"
#include "Color_Sensor.h"

class ObstacleChecker {
public:
  ObstacleChecker(UltrasonicSensor& leftU, UltrasonicSensor& rightU, ColorSensor& cs);

  void setColorCalibration(const ColorCalibration& a, const ColorCalibration& b);
  void check(bool shouldStopBeforeColorDetection = true);

  bool isObstacleDetected() const;
  char getLastDetectedColor() const;

private:
  UltrasonicSensor& leftUltrasonic;
  UltrasonicSensor& rightUltrasonic;
  ColorSensor& colorSensor;

  ColorCalibration colorA;
  ColorCalibration colorB;

  bool obstacleDetected = false;
  bool colorDetectedThisCycle = false;
  char lastDetectedColor = 'N';

  unsigned long lastCheck = 0;
  const unsigned long interval = 200;
};

#endif
