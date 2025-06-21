#include <Arduino.h>
#include "Motor.h"
#include "Motor_Controller.h"
#include "IR_Sensor.h"
#include "Ultrasonic_Sensor.h"
#include "Color_Sensor.h"
#include "PID_Controller.h"
#include "Movement_Controller.h"
#include "Obstacle_Handler.h"
#include "Movement_Types.h"
#include "Obstacle_Checker.h"

// === IR Sensor Setup ===
constexpr uint8_t irLeftPin = 2;
constexpr uint8_t irRightPin = 4;
IRSensor leftIRSensor(irLeftPin);
IRSensor rightIRSensor(irRightPin);

// === Ultrasonic Sensor Setup ===
constexpr uint8_t trigLeftPin = 6;
constexpr uint8_t echoLeftPin = 7;
constexpr uint8_t trigRightPin = 12;
constexpr uint8_t echoRightPin = 13;
UltrasonicSensor leftUltrasonic(trigLeftPin, echoLeftPin);
UltrasonicSensor rightUltrasonic(trigRightPin, echoRightPin);

// === Color Sensor Setup ===
constexpr uint8_t S0 = A0;
constexpr uint8_t S1 = A1;
constexpr uint8_t S2 = A2;
constexpr uint8_t S3 = A3;
constexpr uint8_t sensorOut = A4;
ColorSensor colorSensor(S0, S1, S2, S3, sensorOut);

// === Motor Setup ===
constexpr uint8_t enA = 3;
constexpr uint8_t in1 = 8;
constexpr uint8_t in2 = 9;
constexpr uint8_t enB = 5;
constexpr uint8_t in3 = 10;
constexpr uint8_t in4 = 11;
Motor leftMotor(enA, in1, in2);
Motor rightMotor(enB, in3, in4);
MotorController motorController(leftMotor, rightMotor);

// === PID Controller Setup ===
PIDController pid(10.0, 0.0, 10.0);

// === Movement & Obstacle Controllers ===
MovementController movementController(motorController, leftIRSensor, rightIRSensor, pid);
ObstacleChecker obstacleChecker(leftUltrasonic, rightUltrasonic, colorSensor);
ObstacleHandler obstacleHandler(motorController, movementController, obstacleChecker,
                                leftIRSensor, rightIRSensor);



// === Color Calibration Data ===
ColorCalibration colorA = {0.595, 0.585};
ColorCalibration colorB = {0.561, 0.541};

void setup() {
  Serial.begin(9600);

  // Initialize hardware components
  leftIRSensor.begin();
  rightIRSensor.begin();
  leftUltrasonic.begin();
  rightUltrasonic.begin();
  colorSensor.begin();
  leftMotor.begin();
  rightMotor.begin();

  motorController.stop();
  obstacleChecker.setColorCalibration(colorA, colorB);
}

void loop() {
  // Check for obstacle
  obstacleChecker.check();

  if (obstacleChecker.isObstacleDetected()) {
    motorController.stop();
    delay(100);
    obstacleChecker.check(false);  // Detect color while stopped

    char color = obstacleChecker.getLastDetectedColor();
    if (color == 'A') {
      Serial.println("Action: STOP for Color A");
      movementController.setCurrentState(STOP);
    } else if (color == 'B') {
      Serial.println("Action: AVOID OBSTACLE for Color B");
      obstacleHandler.handleObstacle();
    }
    return;  // Skip line-following for this cycle
  }

  // No obstacle, continue with line following
  bool leftIR = leftIRSensor.isLineDetected();
  bool rightIR = rightIRSensor.isLineDetected();

  // Update last seen line position
  if (leftIR && !rightIR) movementController.getLastSeenRef() = LEFT;
  else if (rightIR && !leftIR) movementController.getLastSeenRef() = RIGHT;

  // Determine current movement state
  if (!leftIR && !rightIR) {
    LastSeen lastSeen = movementController.getLastSeenRef();
    if (lastSeen == LEFT) movementController.setCurrentState(SEARCH_LEFT);
    else if (lastSeen == RIGHT) movementController.setCurrentState(SEARCH_RIGHT);
    else movementController.setCurrentState(STOP);
  } else {
    movementController.setCurrentState(FORWARD);
  }

  // Execute movement
  movementController.act(movementController.getCurrentState());
}

