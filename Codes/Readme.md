# Arduino Modular Robot System

This project is a modular, extensible robot control system based on:

- Line following (PID)
- Obstacle avoidance (Ultrasonic + Color)
- Modular C++ architecture (IR, Motors, PID, Movement...)

## Structure
- `Motor.h/.cpp`: Motor abstraction
- `IRSensor`, `UltrasonicSensor`, `ColorSensor`: Sensor modules
- `MovementController`: Handles motion logic
- `ObstacleHandler`: Obstacle evasion behavior
- `ObstacleChecker`: Central sensor processor
- `PIDController`: For smooth line following

## Board
- Arduino Nano R4 WiFi

