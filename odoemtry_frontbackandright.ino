// ===========================================================
// ⚙️ ODOMETRY-BASED 1M FORWARD + 90° RIGHT TURN
// ===========================================================

int FL_IN1 = 23, FL_IN2 = 22, FL_PWM = 2;
int FR_IN1 = 24, FR_IN2 = 25, FR_PWM = 3;
int BL_IN1 = 26, BL_IN2 = 27, BL_PWM = 4;
int BR_IN1 = 29, BR_IN2 = 28, BR_PWM = 5;
int speedVal = 80;        // Normal driving speed
int turnSpeed = 130;      // 🔹 Higher torque speed for turning

#define ENCODER_L_A 34
#define ENCODER_L_B 35
#define ENCODER_R_A 32
#define ENCODER_R_B 33

volatile long leftCount = 0, rightCount = 0;
volatile int lastEncodedLeft = 0, lastEncodedRight = 0;

const float WHEEL_DIAMETER_M = 0.150;
const float WHEEL_CIRCUMFERENCE_M = 3.14159265 * WHEEL_DIAMETER_M;
const float CPR = 1300.0;
const float WHEELBASE_M = 0.50;
const float CALIBRATION_FACTOR = 1.00;
const float METERS_PER_TICK = (WHEEL_CIRCUMFERENCE_M / CPR) * CALIBRATION_FACTOR;

float x = 0, y = 0, theta = 0;

// ================== ENCODERS ==================
void readEncoderLeft() {
  int MSB = digitalRead(ENCODER_L_A);
  int LSB = digitalRead(ENCODER_L_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedLeft << 2) | encoded;
  switch (sum) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: leftCount++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: leftCount--; break;
  }
  lastEncodedLeft = encoded;
}
void readEncoderRight() {
  int MSB = digitalRead(ENCODER_R_A);
  int LSB = digitalRead(ENCODER_R_B);
  int encoded = (MSB << 1) | LSB;
  int sum = (lastEncodedRight << 2) | encoded;
  switch (sum) {
    case 0b0001: case 0b0111: case 0b1110: case 0b1000: rightCount++; break;
    case 0b0010: case 0b0100: case 0b1101: case 0b1011: rightCount--; break;
  }
  lastEncodedRight = encoded;
}

// ================== MOTOR CONTROL ==================
void moveForward(int s) {
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
  analogWrite(FL_PWM, s); analogWrite(FR_PWM, s);
  analogWrite(BL_PWM, s); analogWrite(BR_PWM, s);
}
void moveBackward(int s) {
  digitalWrite(FL_IN1, LOW); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, LOW); digitalWrite(FR_IN2, HIGH);
  digitalWrite(BL_IN1, LOW); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN1, LOW); digitalWrite(BR_IN2, HIGH);
  analogWrite(FL_PWM, s); analogWrite(FR_PWM, s);
  analogWrite(BL_PWM, s); analogWrite(BR_PWM, s);
}
void rotateRight(int s) {
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, LOW);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, LOW);
  digitalWrite(FR_IN1, LOW);  digitalWrite(FR_IN2, HIGH);
  digitalWrite(BR_IN1, LOW);  digitalWrite(BR_IN2, HIGH);
  analogWrite(FL_PWM, s); analogWrite(FR_PWM, s);
  analogWrite(BL_PWM, s); analogWrite(BR_PWM, s);
}
void rotateLeft(int s) {
  digitalWrite(FL_IN1, LOW);  digitalWrite(FL_IN2, HIGH);
  digitalWrite(BL_IN1, LOW);  digitalWrite(BL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, LOW);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, LOW);
  analogWrite(FL_PWM, s); analogWrite(FR_PWM, s);
  analogWrite(BL_PWM, s); analogWrite(BR_PWM, s);
}
void stopMotors() {
  analogWrite(FL_PWM, 0); analogWrite(FR_PWM, 0);
  analogWrite(BL_PWM, 0); analogWrite(BR_PWM, 0);
  digitalWrite(FL_IN1, HIGH); digitalWrite(FL_IN2, HIGH);
  digitalWrite(FR_IN1, HIGH); digitalWrite(FR_IN2, HIGH);
  digitalWrite(BL_IN1, HIGH); digitalWrite(BL_IN2, HIGH);
  digitalWrite(BR_IN1, HIGH); digitalWrite(BR_IN2, HIGH);
}

// ================== UPDATE ODOMETRY ==================
void updateOdometry() {
  static long lastLeft = 0, lastRight = 0;
  noInterrupts();
  long leftNow = leftCount, rightNow = rightCount;
  interrupts();

  long deltaL = leftNow - lastLeft;
  long deltaR = rightNow - lastRight;

  float distL = -deltaL * METERS_PER_TICK;
  float distR =  deltaR * METERS_PER_TICK;

  float deltaD = (distR + distL) / 2.0;
  float deltaTheta = (distR - distL) / WHEELBASE_M;
  float theta_mid = theta + deltaTheta / 2.0;

  x += deltaD * cos(theta_mid);
  y += deltaD * sin(theta_mid);
  theta += deltaTheta;

  if (theta > 3.14159265)  theta -= 2 * 3.14159265;
  if (theta < -3.14159265) theta += 2 * 3.14159265;

  lastLeft = leftNow; lastRight = rightNow;
}

// ================== MOVE + TURN ==================
void moveForwardBy(float distance_m) {
  float startX = x, startY = y;
  moveForward(speedVal);

  while (true) {
    updateOdometry();
    float dx = x - startX;
    float dy = y - startY;
    float traveled = sqrt(dx * dx + dy * dy);
    Serial.print("FORWARD | Traveled: "); Serial.print(traveled, 3);
    Serial.print(" m | X: "); Serial.print(x, 3);
    Serial.print(" | Y: "); Serial.print(y, 3);
    Serial.print(" | Θ: "); Serial.println(theta * 180.0 / 3.14159, 2);

    if (traveled >= distance_m) break;
    delay(100);
  }
  stopMotors();
  delay(500);
}

// 🔹 Stronger rotation with short torque kick
void rotateRightBy90() {
  float startTheta = theta;
  float targetTheta = startTheta - (3.14159265 / 2.0);
  if (targetTheta < -3.14159265) targetTheta += 2 * 3.14159265;

  Serial.println("Turning Right 90°...");
  
  // 🔹 Kickstart turn
  rotateRight(180);
  delay(300);
  
  rotateRight(turnSpeed);
  while (true) {
    updateOdometry();
    Serial.print("TURNING | Θ: "); Serial.println(theta * 180.0 / 3.14159, 2);
    if (fabs(theta - targetTheta) < 0.08) break;  // 🔹 Wider tolerance
    delay(100);
  }
  stopMotors();
  delay(500);
}

void rotateLeftBy90() {
  float startTheta = theta;
  float targetTheta = startTheta + (3.14159265 / 2.0);
  if (targetTheta > 3.14159265) targetTheta -= 2 * 3.14159265;

  Serial.println("Turning Left 90°...");
  rotateLeft(180);
  delay(300);
  
  rotateLeft(turnSpeed);
  while (true) {
    updateOdometry();
    Serial.print("TURNING | Θ: "); Serial.println(theta * 180.0 / 3.14159, 2);
    if (fabs(theta - targetTheta) < 0.08) break;
    delay(100);
  }
  stopMotors();
  delay(500);
}

// ================== MAIN ==================
void setup() {
  Serial.begin(115200);
  pinMode(FL_IN1, OUTPUT); pinMode(FL_IN2, OUTPUT); pinMode(FL_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT); pinMode(FR_IN2, OUTPUT); pinMode(FR_PWM, OUTPUT);
  pinMode(BL_IN1, OUTPUT); pinMode(BL_IN2, OUTPUT); pinMode(BL_PWM, OUTPUT);
  pinMode(BR_IN1, OUTPUT); pinMode(BR_IN2, OUTPUT); pinMode(BR_PWM, OUTPUT);

  pinMode(ENCODER_L_A, INPUT_PULLUP);
  pinMode(ENCODER_L_B, INPUT_PULLUP);
  pinMode(ENCODER_R_A, INPUT_PULLUP);
  pinMode(ENCODER_R_B, INPUT_PULLUP);

  lastEncodedLeft = (digitalRead(ENCODER_L_A) << 1) | digitalRead(ENCODER_L_B);
  lastEncodedRight = (digitalRead(ENCODER_R_A) << 1) | digitalRead(ENCODER_R_B);

  attachInterrupt(digitalPinToInterrupt(ENCODER_L_A), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_B), readEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_A), readEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_B), readEncoderRight, CHANGE);

  delay(1000);
  Serial.println("\n=== Starting Sequence ===");
}

void loop() {
  moveForwardBy(1.0);
  moveBackward(80
    
  );
  delay(1200); // Move roughly 1m back (simple timing)
  stopMotors();

  rotateRightBy90();
  rotateLeftBy90();

  stopMotors();
  Serial.println("\n=== Sequence Complete ===");
  while (1);
}
