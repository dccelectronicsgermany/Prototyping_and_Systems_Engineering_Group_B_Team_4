#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include <Arduino.h>

class UltrasonicSensor {
  public:
    void begin();
    UltrasonicSensor(uint8_t trigPin, uint8_t echoPin);
    int getDistance();  // Returns distance in cm

  private:
    uint8_t _trigPin;
    uint8_t _echoPin;
};

#endif
