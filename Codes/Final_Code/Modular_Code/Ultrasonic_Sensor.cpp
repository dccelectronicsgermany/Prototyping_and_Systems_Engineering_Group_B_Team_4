#include "Ultrasonic_Sensor.h"

void UltrasonicSensor::begin() {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}


UltrasonicSensor::UltrasonicSensor(uint8_t trigPin, uint8_t echoPin)
  : _trigPin(trigPin), _echoPin(echoPin) {
  pinMode(_trigPin, OUTPUT);
  pinMode(_echoPin, INPUT);
}

int UltrasonicSensor::getDistance() {
  digitalWrite(_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigPin, LOW);

  long duration = pulseIn(_echoPin, HIGH, 10000);  // 10ms timeout
  if (duration == 0) return -1;  // No echo received
  int cm = duration * 0.034 / 2;
  if (cm < 2 || cm > 400) return -1;  // Out of range
  return cm;
}
