#include "Motor.h"

Motor::Motor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin)
  : _enPin(enPin), _in1Pin(in1Pin), _in2Pin(in2Pin) {}

void Motor::begin() {
  pinMode(_enPin, OUTPUT);
  pinMode(_in1Pin, OUTPUT);
  pinMode(_in2Pin, OUTPUT);
}

void Motor::setSpeed(int speed, Direction direction) {
  if (direction == Direction::FORWARD) {
    digitalWrite(_in1Pin, HIGH);
    digitalWrite(_in2Pin, LOW);
  } else {
    digitalWrite(_in1Pin, LOW);
    digitalWrite(_in2Pin, HIGH);
  }

  analogWrite(_enPin, constrain(abs(speed), 0, 255));
}

void Motor::stop() {
  analogWrite(_enPin, 0);
  digitalWrite(_in1Pin, LOW);
  digitalWrite(_in2Pin, LOW);
}
