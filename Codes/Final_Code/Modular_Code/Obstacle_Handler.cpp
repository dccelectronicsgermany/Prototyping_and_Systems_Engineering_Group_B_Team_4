#include "Obstacle_Handler.h"
#include <Arduino.h>

ObstacleHandler::ObstacleHandler(MotorController& motorCtrl, IRSensor& leftIR, IRSensor& rightIR,
                                 MovementState& stateRef, LastSeen& lastSeenRef)
  : motorController(motorCtrl), leftIRSensor(leftIR), rightIRSensor(rightIR),
    currentState(stateRef), lastSeenLine(lastSeenRef) {}

void ObstacleHandler::avoidObstacle(int motorSpeed, int turnSpeed) {
 
}
