// Motor control pins
const int enA = 3;
const int in1 = 8;
const int in2 = 9;
const int enB = 5;
const int in3 = 10;
const int in4 = 11;

// IR sensors
const int irLeft = 2;
const int irRight = 4;

// Ultrasonic front-left and front-right sensors
const int trigLeft = 6;
const int echoLeft = 7;
const int trigRight = 12;
const int echoRight = 13;

// Color sensor pins (TCS3200 example)
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int sensorOut = A4;

// PID and motion control
float Kp = 100;
int baseSpeed = 180;

// State machine
enum RobotState {
  LINE_FOLLOW,
  OBSTACLE_AVOID,
  STOPPED
};

RobotState currentState = LINE_FOLLOW;

// Timing variables
unsigned long lastColorDetectTime = 0;
const unsigned long colorDetectInterval = 200; // ms

void setup() {
  Serial.begin(9600);

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // IR sensors
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  // Ultrasonic sensors
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // Color sensor
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {
  int leftIR = digitalRead(irLeft);
  int rightIR = digitalRead(irRight);
  int distanceLeft = getDistance(trigLeft, echoLeft);
  int distanceRight = getDistance(trigRight, echoRight);

  switch (currentState) {
    case LINE_FOLLOW:
      // Check for obstacle
      if ((distanceLeft > 0 && distanceLeft < 20) || (distanceRight > 0 && distanceRight < 20)) {
        stopMotors();
        Serial.println("Obstacle detected!");
        currentState = OBSTACLE_AVOID;
        break;
      }

      // Proportional line following
      {
        int error = 0;
        if (leftIR == 0 && rightIR == 1) {
          error = 1;
        } else if (leftIR == 1 && rightIR == 0) {
          error = -1;
        } else {
          error = 0;
        }

        int correction = Kp * error;
        int leftMotorSpeed = constrain(baseSpeed + correction, 0, 255);
        int rightMotorSpeed = constrain(baseSpeed - correction, 0, 255);

        analogWrite(enA, leftMotorSpeed);
        analogWrite(enB, rightMotorSpeed);
        digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
        digitalWrite(in3, HIGH); digitalWrite(in4, LOW);

        Serial.print("Line Follow | L IR: "); Serial.print(leftIR);
        Serial.print(" R IR: "); Serial.print(rightIR);
        Serial.print(" | L Spd: "); Serial.print(leftMotorSpeed);
        Serial.print(" R Spd: "); Serial.println(rightMotorSpeed);
      }
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

  // Non-blocking color detection
  if (millis() - lastColorDetectTime > colorDetectInterval) {
    detectColor();
    lastColorDetectTime = millis();
  }
}

// --- Distance reading ---
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000); // 30ms timeout
  if (duration == 0) return -1;  // no echo received
  int cm = duration * 0.034 / 2;
  if (cm < 2 || cm > 400) return -1;  // out of range
  return cm;
}

// --- Obstacle avoidance logic ---
void avoidObstacle(int distLeft, int distRight) {
  Serial.println("Avoiding obstacle...");
  if (distLeft < distRight) {
    moveBackward(180);
    delay(300);
    turnRight(200);
    delay(400);
  } else {
    moveBackward(180);
    delay(300);
    turnLeft(200);
    delay(400);
  }
  moveForward(200);
  delay(400);
  stopMotors();
  Serial.println("Obstacle avoided.");
}

// --- Color sensor logic (TCS3200) ---
void detectColor() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  int red = pulseIn(sensorOut, LOW, 10000);   // 10ms timeout
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  int green = pulseIn(sensorOut, LOW, 10000);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  int blue = pulseIn(sensorOut, LOW, 10000);

  if (red == 0 || green == 0 || blue == 0) {
    Serial.println("Color sensor timeout");
    return;
  }

  Serial.print("Red: "); Serial.print(red);
  Serial.print(" Green: "); Serial.print(green);
  Serial.print(" Blue: "); Serial.println(blue);

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

// --- Motor controls ---
void moveForward(int speed) {
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void moveBackward(int speed) {
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void turnLeft(int speed) {
  analogWrite(enA, speed / 2);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
}

void turnRight(int speed) {
  analogWrite(enA, speed);
  analogWrite(enB, speed / 2);
  digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
}

void stopMotors() {
  analogWrite(enA, 0); analogWrite(enB, 0);
  digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  digitalWrite(in3, LOW); digitalWrite(in4, LOW);
}
