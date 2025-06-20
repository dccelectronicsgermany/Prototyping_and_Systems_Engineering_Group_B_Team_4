// ======================= Motor Control Pins =======================
const int enA = 3;
const int in1 = 8;
const int in2 = 9;
const int enB = 5;
const int in3 = 10;
const int in4 = 11;

// ======================= Sensor Pins =======================
// Line IR Sensors
const int irLeft = 2;
const int irRight = 4;

// Ultrasonic Sensors
const int trigLeft = 6;
const int echoLeft = 7;
const int trigRight = 12;
const int echoRight = 13;

// Color Sensor (TCS3200)
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int sensorOut = A4;

// ======================= Control Parameters =======================
float Kp = 5.0;
int baseSpeed = 45; // Reduced for low-speed performance
int minSpeed = 15;   // Prevent motors from stalling

// ======================= Robot State =======================
enum RobotState {
  LINE_FOLLOW,
  OBSTACLE_AVOID,
  STOPPED
};
RobotState currentState = LINE_FOLLOW;

// ======================= Timing Variables =======================
unsigned long lastColorDetectTime = 0;
const unsigned long colorDetectInterval = 200;

// ======================= Setup =======================
void setup() {
  Serial.begin(9600);

  pinMode(enA, OUTPUT); pinMode(in1, OUTPUT); pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT); pinMode(in3, OUTPUT); pinMode(in4, OUTPUT);

  pinMode(irLeft, INPUT); pinMode(irRight, INPUT);
  pinMode(trigLeft, OUTPUT); pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT); pinMode(echoRight, INPUT);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

// ======================= Main Loop =======================
void loop() {
  int leftIR = digitalRead(irLeft);
  int rightIR = digitalRead(irRight);
  int distanceLeft = getDistance(trigLeft, echoLeft);
  int distanceRight = getDistance(trigRight, echoRight);

  switch (currentState) {
    case LINE_FOLLOW:
      if ((distanceLeft > 0 && distanceLeft < 20) || (distanceRight > 0 && distanceRight < 20)) {
        stopMotors();
        Serial.println("Obstacle detected!");
        currentState = OBSTACLE_AVOID;
        break;
      }
      followLine(leftIR, rightIR);
      break;

    case OBSTACLE_AVOID:
      avoidObstacle(distanceLeft, distanceRight);
      currentState = LINE_FOLLOW;
      break;

    case STOPPED:
      stopMotors();
      Serial.println("Robot stopped.");
      break;
  }

  if (millis() - lastColorDetectTime > colorDetectInterval) {
    detectColor();
    lastColorDetectTime = millis();
  }
}

// ======================= Line Following =======================
void followLine(int leftIR, int rightIR) {
  int error = 0;
  if (leftIR == 0 && rightIR == 1) error = 1;
  else if (leftIR == 1 && rightIR == 0) error = -1;

  int correction = Kp * error;
  int leftSpeed = constrain(baseSpeed + correction, minSpeed, 255);
  int rightSpeed = constrain(baseSpeed - correction, minSpeed, 255);

  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);

  Serial.print("L IR: "); Serial.print(leftIR);
  Serial.print(" R IR: "); Serial.print(rightIR);
  Serial.print(" | L Spd: "); Serial.print(leftSpeed);
  Serial.print(" R Spd: "); Serial.println(rightSpeed);
}

// ======================= Distance Measurement =======================
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;

  int cm = duration * 0.034 / 2;
  return (cm < 2 || cm > 400) ? -1 : cm;
}

// ======================= Obstacle Avoidance =======================
void avoidObstacle(int distLeft, int distRight) {
  Serial.println("Avoiding obstacle...");

  moveBackward(90);  delay(150);
  if (distLeft < distRight) turnRight(50);
  else turnLeft(50);
  delay(200);

  moveForward(50); delay(200);

  stopMotors();
  Serial.println("Obstacle avoided.");
}

// ======================= Color Detection =======================
void detectColor() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  int red = pulseIn(sensorOut, LOW, 10000);
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  int green = pulseIn(sensorOut, LOW, 10000);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  int blue = pulseIn(sensorOut, LOW, 10000);

  if (red == 0 || green == 0 || blue == 0) {
    Serial.println("Color sensor timeout");
    return;
  }

  Serial.print("R: "); Serial.print(red);
  Serial.print(" G: "); Serial.print(green);
  Serial.print(" B: "); Serial.println(blue);

  if (red < green && red < blue) {
    Serial.println("Red detected - stopping");
    currentState = STOPPED;
  } else if (green < red && green < blue) {
    Serial.println("Green detected");
  } else if (blue < red && blue < green) {
    Serial.println("Blue detected");
  } else {
    Serial.println("Color unclear");
  }
}

// ======================= Motor Control Functions =======================
void moveForward(int speed) {
  analogWrite(enA, speed); analogWrite(enB, speed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void moveBackward(int speed) {
  analogWrite(enA, speed); analogWrite(enB, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void turnLeft(int speed) {
  analogWrite(enA, speed / 2); analogWrite(enB, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnRight(int speed) {
  analogWrite(enA, speed); analogWrite(enB, speed / 2);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void stopMotors() {
  analogWrite(enA, 0); analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}
