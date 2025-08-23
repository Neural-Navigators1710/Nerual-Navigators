#include <Wire.h>
#include <VL53L0X.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// === Motor Driver Pins (L298N) ===
#define IN1 9
#define IN2 10
#define ENA 11
#define BUTTON_PIN 5

bool motorRunning = false;
bool lastButtonState = HIGH;
bool piStarted = false;

// === VL53L0X ===
VL53L0X sensor1;  // Left sensor
VL53L0X sensor2;  // Right sensor
#define XSHUT_1 7
#define XSHUT_2 8

// === MPU6050 ===
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

// === TOF Avoidance Flags ===
bool avoidingLeftObstacle = false;
bool avoidingRightObstacle = false;

// === LED + Pi Ready Signal ===
bool piReady = false;
unsigned long lastBlinkTime = 0;
bool ledState = false;

Servo myServo;

// ------ DS3240MG SERVO (270°) SETTINGS ------
static inline int deg270ToUs(float deg) {
  deg = constrain(deg, 0.0f, 270.0f);
  return (int)(500.0f + deg * (2000.0f / 270.0f));
}

// Convenience positions
const float SERVO_LEFT_DEG     = 100.0f;
const float SERVO_STRAIGHT_DEG = 135.0f;
const float SERVO_RIGHT_DEG    = 170.0f;

// === Smooth turning function ===
void smoothTurn(float targetDeg) {
  int currentUs = myServo.readMicroseconds();
  int targetUs = deg270ToUs(targetDeg);
  int step = (targetUs > currentUs) ? 5 : -5;

  for (int us = currentUs; us != targetUs; us += step) {
    myServo.writeMicroseconds(us);
    delay(5);
    if ((step > 0 && us >= targetUs) || (step < 0 && us <= targetUs)) break;
  }

  myServo.writeMicroseconds(targetUs);
}

void setup() {
  myServo.attach(3, 500, 2500);
  smoothTurn(SERVO_STRAIGHT_DEG);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.begin(9600);
  Wire.begin();

  pinMode(13, OUTPUT);

  // === VL53L0X Init ===
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(100);

  digitalWrite(XSHUT_1, HIGH);
  delay(50);
  if (sensor1.init()) sensor1.setAddress(0x30);
  else { Serial.println("Sensor 1 failed!"); while (1); }

  digitalWrite(XSHUT_2, HIGH);
  delay(50);
  if (sensor2.init()) sensor2.setAddress(0x31);
  else { Serial.println("Sensor 2 failed!"); while (1); }

  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);
  sensor1.startContinuous();
  sensor2.startContinuous();
  Serial.println("✅ TOF sensors ready");

  // === MPU6050 Init ===
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("✅ DMP Initialized");
  } else {
    Serial.println("❌ DMP init failed");
    while (1);
  }

  digitalWrite(13, LOW);
  Serial.println("===============================");
}

float toPositiveDegrees(float rawYaw) {
  float yawDeg = rawYaw * 180 / M_PI;
  if (yawDeg < 0) yawDeg += 360;
  return fmod(yawDeg, 360);
}

void loop() {
  // === Listen for Raspberry Pi Commands ===
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (!turning) {
      if (cmd == "LEFT") {
        Serial.println("📥 Pi Command: LEFT → Start Turn");
        turnStartYaw = stableYaw;
        currentTurn = LEFT_TURN;
        turning = true;
        smoothTurn(SERVO_LEFT_DEG);
      } else if (cmd == "RIGHT") {
        Serial.println("📥 Pi Command: RIGHT → Start Turn");
        turnStartYaw = stableYaw;
        currentTurn = RIGHT_TURN;
        turning = true;
        smoothTurn(SERVO_RIGHT_DEG);
      } else if (cmd == "STRAIGHT") {
        Serial.println("📥 Pi Command: STRAIGHT (No turn)");
        smoothTurn(SERVO_STRAIGHT_DEG);
        // do nothing, keep going straight
      }
    }
  }

  // === LED blink while waiting for gyro stabilization ===
  if (piReady && !yawSaved) {
    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState;
      digitalWrite(13, ledState);
      lastBlinkTime = millis();
    }
  }

  // === LED stays ON when yaw is stable ===
  if (yawSaved) {
    digitalWrite(13, HIGH);
  }

  // === Button Press ===
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH) {
    motorRunning = true;
    Serial.println("🟢 Button pressed → Motor STARTED");
  }
  lastButtonState = buttonState;

  if (motorRunning) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 110);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);
  }

  float currentYaw = 0.0;

  // === MPU6050 Yaw Reading ===
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    Quaternion q;
    VectorFloat gravity;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentYaw = toPositiveDegrees(ypr[0]);
    stableYaw = currentYaw; // continuously update stableYaw

    static float lastYawCheck = 0;
    static unsigned long calibrationStart = 0;
    static bool calibrating = false;

    if (!yawSaved) {
      Serial.print("🌀 Calibrating Yaw: ");
      Serial.println(currentYaw);

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
          Serial.print("✅ Stable Yaw Saved: ");
          Serial.println(stableYaw);
        }
      } else {
        calibrationStart = millis();
        lastYawCheck = currentYaw;
      }
    } else {
      Serial.print("🎯 Yaw: ");
      Serial.println(currentYaw);
    }

    // === Turn Completion Logic ===
    if (turning) {
      float targetYaw = turnStartYaw;
      if (currentTurn == RIGHT_TURN) { targetYaw += 90; if (targetYaw >= 360) targetYaw -= 360; }
      else if (currentTurn == LEFT_TURN) { targetYaw -= 90; if (targetYaw < 0) targetYaw += 360; }

      float yawError = fabs(currentYaw - targetYaw);
      if (yawError > 180) yawError = 360 - yawError;

      Serial.print("🔁 Target: "); Serial.print(targetYaw);
      Serial.print(" | Error: "); Serial.println(yawError);

      if (yawError <= 5) {
        Serial.println("✅ TURN COMPLETE → STRAIGHT");
        Serial.println("STRAIGHT");
        smoothTurn(SERVO_STRAIGHT_DEG);
        turning = false;
        currentTurn = NONE;
      }
    }
  }

  // === TOF Sensor Obstacle Logic (only if not in Pi-triggered turn) ===
  if (!turning) {
    uint16_t leftDist = sensor1.readRangeContinuousMillimeters();
    uint16_t rightDist = sensor2.readRangeContinuousMillimeters();

    Serial.print("Left (0x30): ");
    Serial.print(sensor1.timeoutOccurred() ? "TIMEOUT" : String(leftDist) + " mm");
    Serial.print("\tRight (0x31): ");
    Serial.println(sensor2.timeoutOccurred() ? "TIMEOUT" : String(rightDist) + " mm");

    if (!avoidingLeftObstacle && rightDist < 50) {
      Serial.println("🟠 TOF: OBSTACLE ON RIGHT → TURN LEFT");
      Serial.println("LEFT");
      smoothTurn(SERVO_LEFT_DEG);
      avoidingRightObstacle = true;
    } else if (avoidingRightObstacle && rightDist >= 50) {
      Serial.println("🟢 RIGHT CLEAR → STRAIGHT");
      Serial.println("STRAIGHT");
      smoothTurn(SERVO_STRAIGHT_DEG);
      avoidingRightObstacle = false;
    }

    if (!avoidingRightObstacle && leftDist < 50) {
      Serial.println("🔵 TOF: OBSTACLE ON LEFT → TURN RIGHT");
      Serial.println("RIGHT");
      smoothTurn(SERVO_RIGHT_DEG);
      avoidingLeftObstacle = true;
    } else if (avoidingLeftObstacle && leftDist >= 50) {
      Serial.println("🟢 LEFT CLEAR → STRAIGHT");
      Serial.println("STRAIGHT");
      smoothTurn(SERVO_STRAIGHT_DEG);
      avoidingLeftObstacle = false;
    }
  }
}
