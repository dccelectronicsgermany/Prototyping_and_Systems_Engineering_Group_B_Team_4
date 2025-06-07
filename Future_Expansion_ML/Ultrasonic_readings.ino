#define trigPinLeft 6
#define echoPinLeft 7
#define trigPinRight 12
#define echoPinRight 13

void setup() {
  Serial.begin(115200);  // Use 115200 baud for Edge Impulse data forwarder
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
}

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000); // 30 ms timeout
  if (duration == 0) return -1; // no echo received
  long distance = duration * 0.034 / 2; // convert to cm
  return distance;
}

void loop() {
  long leftDist = getDistance(trigPinLeft, echoPinLeft);
  long rightDist = getDistance(trigPinRight, echoPinRight);

  // Output CSV format: leftDistance,rightDistance
  Serial.print(leftDist);
  Serial.print(",");
  Serial.println(rightDist);

  delay(200);  // 5 Hz sample rate
}
