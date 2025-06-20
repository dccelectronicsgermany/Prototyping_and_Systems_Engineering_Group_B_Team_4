#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include <Arduino.h>

class IRSensor {
  public:
    IRSensor(uint8_t pin);
    void begin();
    bool isLineDetected();

  private:
    uint8_t _pin;
};

#endif
