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

// Color sensor pins
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int sensorOut = A4;

// Motor speed (lowered for 12V LiPo)
const int MOTOR_SPEED = 100;
const int TURN_SPEED = 60;

// PID constants
float Kp = 10.0;
float Ki = 0.0;
float Kd = 5.0;

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

struct ColorCalibration {
  float ratioRG;
  float ratioRB;
};

ColorCalibration colorA = {0.580, 0.595};
ColorCalibration colorB = {0.626, 0.635};

// For one-time color detection per obstacle
bool colorDetectedForThisObstacle = false;
char lastDetectedColor = 'N';  // 'A', 'B', or 'N'

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

  // Color sensor setup
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Serial.begin(9600);
  stopMotors();
}

void loop() {
  unsigned long currentMillis = millis();

  // Obstacle check
  if (currentMillis - lastObstacleCheck >= obstacleCheckInterval) {
    int distanceLeft = getDistance(trigLeft, echoLeft);
    int distanceRight = getDistance(trigRight, echoRight);

    bool obstacleNow = ((distanceLeft > 0 && distanceLeft < 20) ||
                        (distanceRight > 0 && distanceRight < 20));

    if (obstacleNow && !colorDetectedForThisObstacle) {
      stopMotors();
      char detected = detectColor();
      lastDetectedColor = detected;
      colorDetectedForThisObstacle = true;

      if (detected == 'A') {
        Serial.println("Action: STOP for Color A");
        currentState = STOP;
        lastDetectedColor = 'A';
        return;  // Remain stopped
      } 
      else if (detected == 'B') {
        Serial.println("Action: AVOID OBSTACLE for Color B");
        avoidObstacle();
      }
    } 
    else if (!obstacleNow) {
      colorDetectedForThisObstacle = false;
      lastDetectedColor = 'N';
    }

    obstacleDetected = obstacleNow;
    lastObstacleCheck = currentMillis;
  }

  // IR readings
  int leftIR = digitalRead(irLeft);
  int rightIR = digitalRead(irRight);

  // Update line last seen direction
  if (leftIR == HIGH && rightIR == LOW) lastSeenLine = LEFT;
  else if (rightIR == HIGH && leftIR == LOW) lastSeenLine = RIGHT;

  // Determine state
  if (obstacleDetected && lastDetectedColor == 'A') {
    currentState = STOP;
  } else if (obstacleDetected && lastDetectedColor == 'B') {
    // After avoidObstacle() runs, resume normal operation
    currentState = FORWARD;
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

// Returns 'A', 'B', or 'N'
char detectColor() {
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

int readFrequency(bool s2, bool s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(50);
  return pulseIn(sensorOut, LOW);
}

void avoidObstacle() {
  // Step 1: Pivot right to avoid obstacle
  setMotorSpeed(TURN_SPEED, TURN_SPEED/2);
  delay(700);

  // Step 2: Move forward past the obstacle
  setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
  delay(1000);

  // Step 3: Pivot left to realign with the original path
  setMotorSpeed(TURN_SPEED/2, TURN_SPEED);
  delay(700);

  // Step 4: Move forward and look for the line
  unsigned long startTime = millis();
  unsigned long maxDuration = 1500;
  setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);

  while (millis() - startTime < maxDuration) {
    int leftIR = digitalRead(irLeft);
    int rightIR = digitalRead(irRight);

    if (leftIR == HIGH || rightIR == HIGH) {
      stopMotors();

      if (leftIR == HIGH && rightIR == LOW) lastSeenLine = LEFT;
      else if (rightIR == HIGH && leftIR == LOW) lastSeenLine = RIGHT;

      currentState = FORWARD;
      return;
    }
  }

  stopMotors();

  // Step 5: Search pattern (move forward a bit + sweep left/right)
  for (int i = 0; i < 3; i++) {
    // Small forward nudge
    setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
    delay(400);
    stopMotors();

    // Sweep left
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    delay(500);
    stopMotors();

    int leftIR = digitalRead(irLeft);
    int rightIR = digitalRead(irRight);
    if (leftIR == HIGH || rightIR == HIGH) {
      stopMotors();
      currentState = FORWARD;
      return;
    }

    // Sweep right
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
    delay(1000);
    stopMotors();

    leftIR = digitalRead(irLeft);
    rightIR = digitalRead(irRight);
    if (leftIR == HIGH || rightIR == HIGH) {
      stopMotors();
      currentState = FORWARD;
      return;
    }

    // Return to center
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    delay(500);
    stopMotors();
  }

  // If line still not found, let loop handle further searching
  currentState = STOP;
}
