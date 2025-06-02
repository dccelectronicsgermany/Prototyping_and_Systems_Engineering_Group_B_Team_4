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

// Motor speed
const int MOTOR_SPEED = 180;

// Timing variables for non-blocking ultrasonic check
unsigned long lastObstacleCheck = 0;
const unsigned long obstacleCheckInterval = 200;  // ms
bool obstacleDetected = false;

// Movement state enum
enum MovementState { STOP, FORWARD, TURN_LEFT, TURN_RIGHT };
MovementState currentState = STOP;

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

  stopMotors();
}

void loop() {
  unsigned long currentMillis = millis();

  // Check obstacle every obstacleCheckInterval ms (non-blocking)
  if (currentMillis - lastObstacleCheck >= obstacleCheckInterval) {
    int distanceLeft = getDistance(trigLeft, echoLeft);
    int distanceRight = getDistance(trigRight, echoRight);

    obstacleDetected = ((distanceLeft > 0 && distanceLeft < 25) ||
                        (distanceRight > 0 && distanceRight < 25));
    lastObstacleCheck = currentMillis;
  }

  // Determine current state
  if (obstacleDetected) {
    currentState = STOP;
  } else {
    int leftIR = digitalRead(irLeft);
    int rightIR = digitalRead(irRight);

    if ((leftIR == LOW && rightIR == LOW) || (leftIR == HIGH && rightIR == HIGH)) {
      currentState = FORWARD;
    } else if (rightIR == HIGH && leftIR == LOW) {
      currentState = TURN_RIGHT;
    } else if (rightIR == LOW && leftIR == HIGH) {
      currentState = TURN_LEFT;
    } else {
      currentState = STOP;
    }
  }

  // Act on current state
  switch (currentState) {
    case STOP:
      stopMotors();
      Serial.println("Obstacle detected or unknown state! Stopping.");
      break;

    case FORWARD:
      setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
      break;

    case TURN_LEFT:
      setMotorSpeed(MOTOR_SPEED, -MOTOR_SPEED);
      break;

    case TURN_RIGHT:
      setMotorSpeed(-MOTOR_SPEED, MOTOR_SPEED);
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

  // Apply speed with PWM
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
