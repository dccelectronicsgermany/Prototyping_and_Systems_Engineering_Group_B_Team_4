// === Motor control pins ===
const int enA = 3;  // PWM
const int in1 = 8;
const int in2 = 9;
const int enB = 5;  // PWM
const int in3 = 10;
const int in4 = 11;

// === IR sensors ===
const int irLeft = 2;
const int irRight = 4;

// === Ultrasonic front-left and front-right sensors ===
const int trigLeft = 6;
const int echoLeft = 7;
const int trigRight = 12;
const int echoRight = 13;

// === Color sensor pins ===
const int S0 = A0;
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int sensorOut = A4;

// === Motor speed settings ===
const float MOTOR_SPEED = 100;
const float TURN_SPEED = 60;

// === PID constants ===
float Kp = 10.0;
float Ki = 0.0;
float Kd = 10.0;
float previousError = 0;
float integral = 0;

// === Obstacle detection state ===
unsigned long lastObstacleCheck = 0;
const unsigned long obstacleCheckInterval = 200;
bool obstacleDetected = false;

// === Movement & line states ===
enum MovementState { STOP, FORWARD, SEARCH_LEFT, SEARCH_RIGHT };
MovementState currentState = STOP;

enum LastSeen { NONE, LEFT, RIGHT };
LastSeen lastSeenLine = NONE;

// === Color calibration ===
struct ColorCalibration {
  float ratioRG;
  float ratioRB;
};
ColorCalibration colorA = {0.595, 0.585};
ColorCalibration colorB = {0.561, 0.541};

bool colorDetectedForThisObstacle = false;
char lastDetectedColor = 'N';  // 'A', 'B', or 'N'

void setup() {
  // Motor
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // IR
  pinMode(irLeft, INPUT);
  pinMode(irRight, INPUT);

  // Ultrasonic
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

  Serial.begin(9600);
  stopMotors();
}

void loop() {
  unsigned long currentMillis = millis();

  // Obstacle detection
  if (currentMillis - lastObstacleCheck >= obstacleCheckInterval) {
    int distanceLeft = getDistance(trigLeft, echoLeft);
    int distanceRight = getDistance(trigRight, echoRight);

    bool obstacleNow = ((distanceLeft > 0 && distanceLeft < 25) ||
                        (distanceRight > 0 && distanceRight < 25));

    if (obstacleNow && !colorDetectedForThisObstacle) {
      stopMotors();
      char detected = detectColor();
      lastDetectedColor = detected;
      colorDetectedForThisObstacle = true;

      if (detected == 'A') {
        Serial.println("Action: STOP for Color A");
        currentState = STOP;
        return;
      } else if (detected == 'B') {
        Serial.println("Action: AVOID OBSTACLE for Color B");
        avoidObstacle();
      }
    } else if (!obstacleNow) {
      colorDetectedForThisObstacle = false;
      lastDetectedColor = 'N';
    }

    obstacleDetected = obstacleNow;
    lastObstacleCheck = currentMillis;
  }

  // IR reading
  int leftIR = digitalRead(irLeft);
  int rightIR = digitalRead(irRight);

  // Track last seen
  if (leftIR == HIGH && rightIR == LOW) lastSeenLine = LEFT;
  else if (rightIR == HIGH && leftIR == LOW) lastSeenLine = RIGHT;

  // Determine movement state
  if (obstacleDetected && lastDetectedColor == 'A') {
    currentState = STOP;
  } else if (obstacleDetected && lastDetectedColor == 'B') {
    currentState = FORWARD;
  } else if (leftIR == LOW && rightIR == LOW) {
    if (lastSeenLine == LEFT) currentState = SEARCH_LEFT;
    else if (lastSeenLine == RIGHT) currentState = SEARCH_RIGHT;
    else currentState = STOP;
  } else {
    currentState = FORWARD;
  }

  // Act on movement state
  actOnState(currentState, leftIR, rightIR);
}

void actOnState(MovementState state, int leftIR, int rightIR) {
  switch (state) {
    case STOP:
      stopMotors();
      break;

    case FORWARD: {
      int error = leftIR - rightIR;
      float derivative = error - previousError;
      integral += error;
      float correction = Kp * error + Ki * integral + Kd * derivative;
      previousError = error;

      int leftSpeed = constrain(MOTOR_SPEED - correction, 0, 255);
      int rightSpeed = constrain(MOTOR_SPEED + correction, 0, 255);
      setMotorSpeed(leftSpeed, rightSpeed);
      break;
    }

    case SEARCH_LEFT:
      setMotorSpeed(-TURN_SPEED, TURN_SPEED);
      break;

    case SEARCH_RIGHT:
      setMotorSpeed(TURN_SPEED, -TURN_SPEED);
      break;
  }
}

void avoidObstacle() {
  // Step 0: Move slightly backward
  Serial.println("Avoiding obstacle: Reversing...");
  setMotorSpeed(-MOTOR_SPEED, -MOTOR_SPEED);
  delay(300);

  stopMotors();

  // Step 1: Pivot left to steer away from obstacle
  Serial.println("Pivoting left...");
  setMotorSpeed(-TURN_SPEED, TURN_SPEED);
  delay(300);

  stopMotors();

  // Step 2: Move forward to clear the obstacle
  Serial.println("Moving forward to bypass obstacle...");
  setMotorSpeed(MOTOR_SPEED/1.8, MOTOR_SPEED/1.8);
  delay(1500);

  stopMotors();

  // Step 3: Pivot right to realign with path
  Serial.println("Realigning right...");
  setMotorSpeed(TURN_SPEED, -TURN_SPEED);
  delay(700);

  stopMotors();

  // Step 4: Move forward again to ensure clearance
  Serial.println("Moving forward again...");
  setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
  delay(500);

  stopMotors();

  // Step 6: Line search
  Serial.println("Searching for line...");

  // Step 4: Move forward and look for the line
  unsigned long startTime = millis();
  unsigned long maxDuration = 1500;
  setMotorSpeed(MOTOR_SPEED/1.8, MOTOR_SPEED/1.8);

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
  Serial.println("Line not found, initiating search pattern...");

  for (int i = 0; i < 3; i++) {
    Serial.print("Sweep attempt "); Serial.println(i + 1);

    Serial.println(" → Nudge forward");
    setMotorSpeed(MOTOR_SPEED, MOTOR_SPEED);
    delay(400);
    stopMotors();

    int leftIR = digitalRead(irLeft);
    int rightIR = digitalRead(irRight);
    if (leftIR == HIGH || rightIR == HIGH) {
      Serial.println("Line found during left sweep!");
      stopMotors();
      currentState = FORWARD;
      return;
    }
    

    Serial.println(" → Sweep left");
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    delay(500);
    stopMotors();

    leftIR = digitalRead(irLeft);
    rightIR = digitalRead(irRight);
    if (leftIR == HIGH || rightIR == HIGH) {
      Serial.println("Line found during left sweep!");
      stopMotors();
      currentState = FORWARD;
      return;
    }

    Serial.println(" → Sweep right");
    setMotorSpeed(TURN_SPEED, -TURN_SPEED);
    delay(1000);
    stopMotors();

    leftIR = digitalRead(irLeft);
    rightIR = digitalRead(irRight);
    if (leftIR == HIGH || rightIR == HIGH) {
      Serial.println("Line found during right sweep!");
      stopMotors();
      currentState = FORWARD;
      return;
    }

    Serial.println(" → Return to center");
    setMotorSpeed(-TURN_SPEED, TURN_SPEED);
    delay(500);
    stopMotors();
  }

  Serial.println("Line not found after search pattern. Stopping.");
  currentState = STOP;
}


// === Helper functions ===

void stopMotors() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void setMotorSpeed(int leftSpeed, int rightSpeed) {
  if (leftSpeed > 0) {
    digitalWrite(in1, HIGH); digitalWrite(in2, LOW);
  } else if (leftSpeed < 0) {
    digitalWrite(in1, LOW); digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW); digitalWrite(in2, LOW);
  }

  if (rightSpeed > 0) {
    digitalWrite(in3, HIGH); digitalWrite(in4, LOW);
  } else if (rightSpeed < 0) {
    digitalWrite(in3, LOW); digitalWrite(in4, HIGH);
  } else {
    digitalWrite(in3, LOW); digitalWrite(in4, LOW);
  }

  analogWrite(enA, abs(leftSpeed));
  analogWrite(enB, abs(rightSpeed));
}

int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 10000);
  if (duration == 0) return -1;
  int cm = duration * 0.034 / 2;
  if (cm < 2 || cm > 400) return -1;
  return cm;
}

char detectColor() {
  const int samples = 5;
  long rSum = 0, gSum = 0, bSum = 0;

  for (int i = 0; i < samples; i++) {
    int r = readFrequency(LOW, LOW);
    int g = readFrequency(HIGH, HIGH);
    int b = readFrequency(LOW, HIGH);
    rSum += r; gSum += g; bSum += b;
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
    return 'A';
  } else {
    Serial.println("Detected Color: B");
    return 'B';
  }
}

int readFrequency(bool s2, bool s3) {
  digitalWrite(S2, s2);
  digitalWrite(S3, s3);
  delay(50);
  return pulseIn(sensorOut, LOW);
}
