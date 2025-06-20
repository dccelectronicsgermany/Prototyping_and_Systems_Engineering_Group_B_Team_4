#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>

struct ColorCalibration {
  float ratioRG;
  float ratioRB;
};

class ColorSensor {
  public:
    ColorSensor(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t outPin);
    void begin();
    char detectColor(const ColorCalibration& colorA, const ColorCalibration& colorB);
  
  private:
    uint8_t _s0, _s1, _s2, _s3, _outPin;
    int readFrequency(bool s2, bool s3);
};

#endif
