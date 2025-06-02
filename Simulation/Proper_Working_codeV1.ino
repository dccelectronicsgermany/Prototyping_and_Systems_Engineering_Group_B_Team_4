// Motor control pins
const int enA = 3;  // PWM
const int in1 = 8;
const int in2 = 9;
const int enB = 5;  // PWM
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

// Motor speed (lowered for 12V LiPo)
const int MOTOR_SPEED = 60;

// PID constants
float Kp = 5.0;
float Ki = 0.0;
float Kd = 2.5;

// PID state
float previousError = 0;
float integral = 0;

// Obstacle detection timing
unsigned long lastObstacleCheck = 0;
const unsigned long obstacleCheckInterval = 200;  // ms
bool obstacleDetected = false;

// Movement state enum
enum MovementState { STOP, FORWARD, SEARCH_LEFT, SEARCH_RIGHT };
MovementState currentState = STOP;

// Line lost tracking
enum LastSeen { NONE, LEFT, RIGHT };
LastSeen lastSeenLine = NONE;

void setup() {
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

  stopMotors();
}

void loop() {
  unsigned long currentMillis = millis();

  // Obstacle check
  if (currentMillis - lastObstacleCheck >= obstacleCheckInterval) {
    int distanceLeft = getDistance(trigLeft, echoLeft);
    int distanceRight = getDistance(trigRight, echoRight);
    obstacleDetected = ((distanceLeft > 0 && distanceLeft < 25) ||
                        (distanceRight > 0 && distanceRight < 25));
    lastObstacleCheck = currentMillis;
  }

  // IR readings
  int leftIR = digitalRead(irLeft);
  int rightIR = digitalRead(irRight);

  // Update line last seen direction
  if (leftIR == HIGH && rightIR == LOW) lastSeenLine = LEFT;
  else if (rightIR == HIGH && leftIR == LOW) lastSeenLine = RIGHT;

  // Determine state
  if (obstacleDetected) {
    currentState = STOP;
  } else if (leftIR == LOW && rightIR == LOW) {
    // Both sensors see white (no line)
    if (lastSeenLine == LEFT) currentState = SEARCH_LEFT;
    else if (lastSeenLine == RIGHT) currentState = SEARCH_RIGHT;
    else currentState = STOP;
  } else {
    currentState = FORWARD;
  }

  // Act on state
  actOnState(currentState, leftIR, rightIR);
}

void actOnState(MovementState state, int leftIR, int rightIR) {
  switch (state) {
    case STOP:
      stopMotors();
      break;

    case FORWARD: {
      // Corrected PID logic (line-following)
      int error = leftIR - rightIR;  // Turn toward side that sees line
      float derivative = error - previousError;
      integral += error;
      float correction = Kp * error + Ki * integral + Kd * derivative;
      previousError = error;

      int baseSpeed = MOTOR_SPEED;
      int leftSpeed = constrain(baseSpeed - correction, 0, 255);
      int rightSpeed = constrain(baseSpeed + correction, 0, 255);
      setMotorSpeed(leftSpeed, rightSpeed);
      break;
    }

    case SEARCH_LEFT:
      setMotorSpeed(-80, 80);  // Pivot left slowly
      break;

    case SEARCH_RIGHT:
      setMotorSpeed(80, -80);  // Pivot right slowly
      break;
  }
}

// Read distance using ultrasonic sensor (non-blocking within pulseIn timeout)
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 10000);  // 10ms timeout
  if (duration == 0) return -1;  // no echo received
  int cm = duration * 0.034 / 2;
  if (cm < 2 || cm > 400) return -1;  // out of range
  return cm;
}

// Set direction and speed for both motors
void setMotorSpeed(int leftSpeed, int rightSpeed) {
  // Left motor
  if (leftSpeed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }

  // Right motor
  if (rightSpeed > 0) {
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  }

  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));
}

// Stop all motor movement
void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
