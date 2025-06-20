#ifndef OBSTACLE_CHECKER_H
#define OBSTACLE_CHECKER_H

#include "Ultrasonic_Sensor.h"
#include "Color_Sensor.h"
#include "Motor_Controller.h"
#include "Movement_Controller.h"
#include "Obstacle_Handler.h"
#include "Movement_Types.h"

class ObstacleChecker {
  public:
    ObstacleChecker(UltrasonicSensor& leftUltrasonic,
                    UltrasonicSensor& rightUltrasonic,
                    ColorSensor& colorSensor,
                    MotorController& motorController,
                    MovementController& movementController,
                    ObstacleHandler& obstacleHandler);

    void check();
    void setColorCalibration(const ColorCalibration& a, const ColorCalibration& b);
    bool isObstacleDetected() const;
    char getLastDetectedColor() const;

  private:
    UltrasonicSensor& leftUltrasonic;
    UltrasonicSensor& rightUltrasonic;
    ColorSensor& colorSensor;
    MotorController& motorController;
    MovementController& movementController;
    ObstacleHandler& obstacleHandler;

    ColorCalibration colorA;
    ColorCalibration colorB;

    unsigned long lastCheck = 0;
    static constexpr unsigned long interval = 200;

    bool obstacleDetected = false;
    bool colorDetectedThisCycle = false;
    char lastDetectedColor = 'N';
};

#endif
