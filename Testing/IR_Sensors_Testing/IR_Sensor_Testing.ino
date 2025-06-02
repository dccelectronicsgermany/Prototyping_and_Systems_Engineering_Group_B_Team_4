// IR sensor pins
const int irLeftPin = 2;
const int irRightPin = 4;

void setup() {
  Serial.begin(9600);
  pinMode(irLeftPin, INPUT);
  pinMode(irRightPin, INPUT);
  Serial.println("IR Sensor Test Started");
}

void loop() {
  int leftValue = digitalRead(irLeftPin);
  int rightValue = digitalRead(irRightPin);

  Serial.print("Left IR Sensor: ");
  Serial.print(leftValue == HIGH ? "Line detected" : "No line");
  Serial.print(" | Right IR Sensor: ");
  Serial.println(rightValue == HIGH ? "Line detected" : "No line");

  delay(1000); // Wait 500ms before next read
}
