#ifndef OBSTACLE_HANDLER_H
#define OBSTACLE_HANDLER_H

#include "Motor_Controller.h"
#include "Movement_Controller.h"
#include "Obstacle_Checker.h"
#include "IR_Sensor.h"

class ObstacleHandler {
public:
  ObstacleHandler(MotorController& mc,
                  MovementController& mv,
                  ObstacleChecker& checker,
                  IRSensor& leftIR,
                  IRSensor& rightIR);

  void handleObstacle();

private:
  MotorController& motorController;
  MovementController& movementController;
  ObstacleChecker& obstacleChecker;
  IRSensor& leftIRSensor;
  IRSensor& rightIRSensor;

  void reverse(unsigned long duration);
  void pivotLeft(unsigned long duration);
  void pivotRight(unsigned long duration);
  void moveForward(unsigned long duration);
  bool searchForLine(unsigned long maxDuration);
};

#endif
