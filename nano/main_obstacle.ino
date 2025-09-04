#include <Wire.h>
#include <VL53L0X.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// ===================================================
// === Motor Driver (L298N) Pins ===
// ===================================================
#define IN1 10
#define IN2 9
#define ENA 11
#define BUTTON_PIN 5   // Start button

bool motorRunning = false;
bool lastButtonState = HIGH;

// ===================================================
// === LED Pins (Status Indicators) ===
// ===================================================
#define LED1 2   // Sensors initialized
#define LED2 4   // Reserved
#define LED3 6   // Stable yaw saved (MPU6050)

// ===================================================
// === VL53L0X TOF Sensors ===
// ===================================================
VL53L0X sensorLeft;
VL53L0X sensorRight;
#define XSHUT_LEFT 8
#define XSHUT_RIGHT 7

unsigned long gapCooldownEnd = 0;
unsigned long startupIgnoreEnd = 0;

// ===================================================
// === MPU6050 IMU ===
// ===================================================
MPU6050 mpu;
bool dmpReady = false;
uint8_t fifoBuffer[64];
float ypr[3];
float stableYaw = 0.0;
bool yawSaved = false;
float turnStartYaw = 0.0;
bool turning = false;

enum TurnType { NONE, LEFT_TURN, RIGHT_TURN };
TurnType currentTurn = NONE;

// ===================================================
// === TOF Flags ===
// ===================================================
bool gapLeftActive = false;
bool gapRightActive = false;

// Confirmation counters
int leftGapConfirm = 0;
int rightGapConfirm = 0;

// Track which side triggered obstacle avoidance
char lastObstacle = 'N';

// ===================================================
// === Servo Steering ===
// ===================================================
Servo steering;

static inline int degToUs(float deg) {
  deg = constrain(deg, 0.0f, 270.0f);
  return (int)(500.0f + deg * (2000.0f / 270.0f));
}

const float SERVO_LEFT = 130.0f;
const float SERVO_RIGHT = 50.0f;
const float SERVO_STRAIGHT = 94.5f;
const float SERVO_STRAIGHT_ALT = 98.3f;

void smoothTurn(float targetDeg) {
  int currentUs = steering.readMicroseconds();
  int targetUs = degToUs(targetDeg);
  int step = (targetUs > currentUs) ? 5 : -5;

  for (int us = currentUs; us != targetUs; us += step) {
    steering.writeMicroseconds(us);
    delay(5);
    if ((step > 0 && us >= targetUs) || (step < 0 && us <= targetUs)) break;
  }
  steering.writeMicroseconds(targetUs);
}

// ===================================================
// === F1 Lights Sequence ===
// ===================================================
bool f1Active = false;
unsigned long f1StartTime = 0;

// ===================================================
// === STOP Sequence ===
// ===================================================
bool stopActive = false;
unsigned long stopStartTime = 0;
int stopStep = 0;
bool blinkState = false;
unsigned long lastBlinkTime = 0;

// ===================================================
// === TURN COUNT ===
// ===================================================
int turnCount = 0;

// ===================================================
// === SPEED CONTROL ===
// ===================================================
const int STRAIGHT_SPEED = 110;
const int TURN_SPEED = 90;
int currentSpeed = STRAIGHT_SPEED;

// ===================================================
// === PI Serial Control ===
// ===================================================
char lastPiCommand = 'N';   // 'R', 'L', 'S', or 'N'
bool piTurnActive = false;  // true while obeying Pi

// ===================================================
// === Setup ===
// ===================================================
void setup() {
  steering.attach(3, 500, 2500);
  smoothTurn(96.0f);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  Serial.begin(9600);
  Wire.begin();

  // TOF init
  pinMode(XSHUT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  digitalWrite(XSHUT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  delay(100);

  digitalWrite(XSHUT_LEFT, HIGH);
  delay(50);
  if (sensorLeft.init()) sensorLeft.setAddress(0x30);
  else { Serial.println("❌ Left TOF failed!"); while (1); }

  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(50);
  if (sensorRight.init()) sensorRight.setAddress(0x31);
  else { Serial.println("❌ Right TOF failed!"); while (1); }

  sensorLeft.setMeasurementTimingBudget(33000);
  sensorRight.setMeasurementTimingBudget(33000);
  sensorLeft.startContinuous();
  sensorRight.startContinuous();
  Serial.println("✅ TOF sensors ready");

  // MPU init
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("✅ DMP Initialized");
  } else {
    Serial.println("❌ DMP init failed");
    while (1);
  }

  Serial.println("===============================");
  digitalWrite(LED1, HIGH);

  startupIgnoreEnd = millis() + 3000;
}

float toDegrees(float rawYaw) {
  float yawDeg = rawYaw * 180 / M_PI;
  if (yawDeg < 0) yawDeg += 360;
  return fmod(yawDeg, 360);
}

// ===================================================
// === STOP Handler ===
// ===================================================
void handleStop() {
  unsigned long elapsed = millis() - stopStartTime;

  if (stopStep == 0 && elapsed >= 1000) {
    digitalWrite(LED1, HIGH); stopStep = 1;
  } 
  else if (stopStep == 1 && elapsed >= 2000) {
    digitalWrite(LED2, HIGH); stopStep = 2;
  } 
  else if (stopStep == 2 && elapsed >= 3000) {
    digitalWrite(LED3, HIGH); stopStep = 3;

    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    smoothTurn(SERVO_STRAIGHT);
    lastBlinkTime = millis();
  }

  if (stopStep >= 3 && millis() - lastBlinkTime >= 500) {
    blinkState = !blinkState;
    digitalWrite(LED1, blinkState);
    digitalWrite(LED2, blinkState);
    digitalWrite(LED3, blinkState);
    lastBlinkTime = millis();
  }
}

// ===================================================
// === TOF Obstacle & Gap Avoidance ===
// ===================================================
void handleTOF(float currentYaw) {
  uint16_t leftDist = sensorLeft.readRangeContinuousMillimeters();
  uint16_t rightDist = sensorRight.readRangeContinuousMillimeters();

  Serial.print("📏 TOF | Left: ");
  Serial.print(leftDist);
  Serial.print(" mm | Right: ");
  Serial.print(rightDist);
  Serial.println(" mm");

  // Immediate obstacle avoidance
  if (rightDist < 170) {
    Serial.println("🟠 OBSTACLE RIGHT → TURN LEFT");
    smoothTurn(SERVO_LEFT);
    lastObstacle = 'R';
  }
  if (leftDist < 170) {
    Serial.println("🔵 OBSTACLE LEFT → TURN RIGHT");
    smoothTurn(SERVO_RIGHT);
    lastObstacle = 'L';
  }

  // Recovery
  if (lastObstacle == 'R' && rightDist >= 250) {
    Serial.println("✅ RIGHT CLEARED → STRAIGHT_ALT");
    smoothTurn(SERVO_STRAIGHT_ALT);
    lastObstacle = 'N';
  }
  if (lastObstacle == 'L' && leftDist >= 170) {
    Serial.println("✅ LEFT CLEARED → STRAIGHT");
    smoothTurn(SERVO_STRAIGHT);
    lastObstacle = 'N';
  }

  // GAP detection (skip if turning or Pi controlling)
  if (!turning && !piTurnActive && millis() > gapCooldownEnd && millis() > startupIgnoreEnd) {
    if (leftDist > 850) leftGapConfirm++; else leftGapConfirm = 0;
    if (leftGapConfirm > 0) {
      Serial.println("🚀 GAP LEFT CONFIRMED → INITIATING LEFT TURN");
      turnStartYaw = currentYaw;
      currentTurn = LEFT_TURN;
      turning = true;
      smoothTurn(SERVO_LEFT);
      currentSpeed = TURN_SPEED;
      gapLeftActive = true;
      leftGapConfirm = 0;
      turnCount++;
      Serial.print("🔢 Turn Count: "); Serial.println(turnCount);
    }

    if (rightDist > 850) rightGapConfirm++; else rightGapConfirm = 0;
    if (rightGapConfirm > 0) {
      Serial.println("🚀 GAP RIGHT CONFIRMED → INITIATING RIGHT TURN");
      turnStartYaw = currentYaw;
      currentTurn = RIGHT_TURN;
      turning = true;
      smoothTurn(SERVO_RIGHT);
      currentSpeed = TURN_SPEED;
      gapRightActive = true;
      rightGapConfirm = 0;
      turnCount++;
      Serial.print("🔢 Turn Count: "); Serial.println(turnCount);
    }
  }
}

// ===================================================
// === Main Loop ===
// ===================================================
void loop() {
  if (stopActive) {
    handleStop();
    return;
  }

  // Button start
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH && !f1Active && !motorRunning) {
    Serial.println("🏎️ Button pressed → F1 Lights");
    f1Active = true;
    f1StartTime = millis();
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(500);
  }
  lastButtonState = buttonState;

  if (f1Active) {
    unsigned long elapsed = millis() - f1StartTime;
    if (elapsed < 666) digitalWrite(LED1, HIGH);
    else if (elapsed < 1332) digitalWrite(LED2, HIGH);
    else if (elapsed < 1998) digitalWrite(LED3, HIGH);
    else {
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      motorRunning = true;
      f1Active = false;
      Serial.println("🟢 Lights Out → Motor GO");
    }
  }

  // Motor control
  if (motorRunning) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, currentSpeed);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  // --- Pi Serial Commands ---
  if (Serial.available()) {
    char cmd = Serial.read();

    if (cmd == 'R') {
      Serial.println("📡 Pi → TURN RIGHT");
      smoothTurn(SERVO_RIGHT);
      lastPiCommand = 'R';
      piTurnActive = true;
    }
    else if (cmd == 'L') {
      Serial.println("📡 Pi → TURN LEFT");
      smoothTurn(SERVO_LEFT);
      lastPiCommand = 'L';
      piTurnActive = true;
    }
    else if (cmd == 'S') {
      Serial.println("📡 Pi → STRAIGHT");
      if (lastPiCommand == 'R') {
        smoothTurn(SERVO_STRAIGHT_ALT);
        Serial.println("↔️ Return to STRAIGHT_ALT (after RIGHT)");
      }
      else if (lastPiCommand == 'L') {
        smoothTurn(SERVO_STRAIGHT);
        Serial.println("↔️ Return to STRAIGHT (after LEFT)");
      }
      lastPiCommand = 'S';
      piTurnActive = false;
    }
  }

  // MPU6050 Yaw
  float currentYaw = 0.0;
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    Quaternion q;
    VectorFloat gravity;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentYaw = toDegrees(ypr[0]);
    stableYaw = currentYaw;

    Serial.print("🧭 Current Yaw: ");
    Serial.println(currentYaw);

    if (!yawSaved) {
      static float lastYawCheck = 0;
      static unsigned long calibrationStart = 0;
      static bool calibrating = false;

      if (!calibrating) {
        lastYawCheck = currentYaw;
        calibrationStart = millis();
        calibrating = true;
      }

      if (abs(currentYaw - lastYawCheck) < 0.5) {
        if (millis() - calibrationStart > 1500) {
          stableYaw = currentYaw;
          yawSaved = true;
          calibrating = false;
          Serial.print("✅ Stable Yaw: ");
          Serial.println(stableYaw);
          digitalWrite(LED3, HIGH);
        }
      } else {
        calibrationStart = millis();
        lastYawCheck = currentYaw;
      }
    }

    // Handle yaw-based GAP turns (ignore if Pi is controlling)
    if (turning && !piTurnActive) {
      float targetYaw = turnStartYaw;
      if (currentTurn == RIGHT_TURN) { targetYaw += 85; if (targetYaw >= 360) targetYaw -= 360; }
      else if (currentTurn == LEFT_TURN) { targetYaw -= 90; if (targetYaw < 0) targetYaw += 360; }

      float yawError = fabs(currentYaw - targetYaw);
      if (yawError > 180) yawError = 360 - yawError;

      if (yawError <= 5) {
        Serial.println("✅ TURN COMPLETE → STRAIGHT");
        smoothTurn((currentTurn == RIGHT_TURN) ? SERVO_STRAIGHT_ALT : SERVO_STRAIGHT);
        turning = false;
        currentTurn = NONE;
        gapLeftActive = false;
        gapRightActive = false;
        gapCooldownEnd = millis() + 1000;
        currentSpeed = STRAIGHT_SPEED;
      }
    }
  }

  // TOF always active
  handleTOF(stableYaw);
}
