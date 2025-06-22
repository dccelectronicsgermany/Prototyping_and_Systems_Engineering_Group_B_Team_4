#ifndef MOVEMENT_CONTROLLER_H
#define MOVEMENT_CONTROLLER_H

#include "Motor_Controller.h"
#include "IR_Sensor.h"
#include "PID_Controller.h"
#include "Color_Sensor.h"
#include "Ultrasonic_Sensor.h"
#include "Movement_Types.h"


class MovementController {
  public:
    MovementController(MotorController& motorCtrl, IRSensor& leftIR, IRSensor& rightIR, PIDController& pidCtrl);
    void updateState(bool obstacleDetected, char lastDetectedColor, MovementState& currentState, LastSeen& lastSeenLine);
    void act(MovementState state);

    MovementState& getCurrentStateRef();
    LastSeen& getLastSeenRef();
    MovementState getCurrentState() const;
    void setCurrentState(MovementState state);

  private:
    MotorController& motorController;
    IRSensor& leftIRSensor;
    IRSensor& rightIRSensor;
    PIDController& pid;
    MovementState currentState = STOP;
    LastSeen lastSeenLine = NONE;
    
    int searchSpeed = 60;
    int baseSpeed = 100;
};

#endif
