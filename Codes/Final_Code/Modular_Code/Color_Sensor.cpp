#include "Color_Sensor.h"
#include <Arduino.h>

ColorSensor::ColorSensor(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3, uint8_t outPin)
  : _s0(s0), _s1(s1), _s2(s2), _s3(s3), _outPin(outPin) {}

void ColorSensor::begin() {
  pinMode(_s0, OUTPUT);
  pinMode(_s1, OUTPUT);
  pinMode(_s2, OUTPUT);
  pinMode(_s3, OUTPUT);
  pinMode(_outPin, INPUT);

  digitalWrite(_s0, HIGH);
  digitalWrite(_s1, LOW);
}

char ColorSensor::detectColor(const ColorCalibration& colorA, const ColorCalibration& colorB) {
  const int samples = 5;
  long rSum = 0, gSum = 0, bSum = 0;

  for (int i = 0; i < samples; i++) {
    int r = readFrequency(LOW, LOW);
    int g = readFrequency(HIGH, HIGH);
    int b = readFrequency(LOW, HIGH);

    rSum += r;
    gSum += g;
    bSum += b;

    delay(100);
  }

  float rAvg = rSum / (float)samples;
  float gAvg = gSum / (float)samples;
  float bAvg = bSum / (float)samples;

  float ratioRG = rAvg / gAvg;
  float ratioRB = rAvg / bAvg;

  Serial.println("----------------------");
  Serial.print("R="); Serial.print(rAvg, 1);
  Serial.print(" G="); Serial.print(gAvg, 1);
  Serial.print(" B="); Serial.print(bAvg, 1);
  Serial.print(" | RatioRG="); Serial.print(ratioRG, 3);
  Serial.print(" RatioRB="); Serial.println(ratioRB, 3);

  float diffA = abs(ratioRG - colorA.ratioRG) + abs(ratioRB - colorA.ratioRB);
  float diffB = abs(ratioRG - colorB.ratioRG) + abs(ratioRB - colorB.ratioRB);

  if (diffA < diffB) {
    Serial.println("Detected Color: A");
    Serial.println("----------------------");
    return 'A';
  } else {
    Serial.println("Detected Color: B");
    Serial.println("----------------------");
    return 'B';
  }
}

int ColorSensor::readFrequency(bool s2, bool s3) {
  digitalWrite(_s2, s2);
  digitalWrite(_s3, s3);
  delay(50);
  return pulseIn(_outPin, LOW);
}
