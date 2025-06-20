#ifndef OBSTACLE_HANDLER_H
#define OBSTACLE_HANDLER_H

#include "Motor_Controller.h"
#include "IR_Sensor.h"
#include "Movement_Types.h"


class ObstacleHandler {
  public:
    ObstacleHandler(MotorController& motorCtrl, IRSensor& leftIR, IRSensor& rightIR,
                    MovementState& stateRef, LastSeen& lastSeenRef);

    void avoidObstacle(int motorSpeed, int turnSpeed);

  private:
    MotorController& motorController;
    IRSensor& leftIRSensor;
    IRSensor& rightIRSensor;
    MovementState& currentState;
    LastSeen& lastSeenLine;
};

#endif
