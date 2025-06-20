#include "IR_Sensor.h"

IRSensor::IRSensor(uint8_t pin) : _pin(pin) {}

void IRSensor::begin() {
  pinMode(_pin, INPUT);
}

bool IRSensor::isLineDetected() {
  return digitalRead(_pin) == HIGH;
}
