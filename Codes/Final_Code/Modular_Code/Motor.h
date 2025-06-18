#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
  public:
    enum class Direction {
      FORWARD,
      BACKWARD
    };
    void begin(); 


    Motor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin);
    void setSpeed(int speed, Direction direction);
    void stop();

  private:
    uint8_t _enPin;
    uint8_t _in1Pin;
    uint8_t _in2Pin;
};

#endif
