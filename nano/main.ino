#include <Wire.h>
#include <VL53L0X.h>
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

// ===================================================
// === Motor Driver (L298N) Pin Definitions ===
// ===================================================
#define IN1 10
#define IN2 9
#define ENA 11
#define BUTTON_PIN 5   // Start button

bool motorRunning = false;      // Tracks motor state (on/off)
bool lastButtonState = HIGH;    // For button debounce

// ===================================================
// === LED Status Pins (Indicators) ===
// ===================================================
#define LED1 2   // ON → Sensors initialized
#define LED2 4   // ON → Raspberry Pi Ready
#define LED3 6   // ON → Stable Yaw Saved (MPU6050)

// ===================================================
// === VL53L0X TOF Sensors ===
// ===================================================
VL53L0X sensor1;  // Left sensor
VL53L0X sensor2;  // Right sensor
#define XSHUT_1 7 // Shutdown pin for sensor1
#define XSHUT_2 8 // Shutdown pin for sensor2

// ===================================================
// === MPU6050 IMU (Gyroscope + Accelerometer) ===
// ===================================================
MPU6050 mpu;
bool dmpReady = false;       // Flag: DMP ready or not
uint8_t fifoBuffer[64];      // Buffer to store DMP data
float ypr[3];                // Yaw, Pitch, Roll
float stableYaw = 0.0;       // Saved stable yaw reference
bool yawSaved = false;       // Has yaw been calibrated
float turnStartYaw = 0.0;    // Yaw at start of turn
bool turning = false;        // Flag: robot is in a turn

// Turn types used by MPU + Pi
enum TurnType { NONE, LEFT_TURN, RIGHT_TURN };
TurnType currentTurn = NONE;

// ===================================================
// === TOF Obstacle Avoidance Flags ===
// ===================================================
bool avoidingLeftObstacle = false;
bool avoidingRightObstacle = false;

// ===================================================
// === Raspberry Pi Ready Flag ===
// ===================================================
bool piReady = false;

// ===================================================
// === Servo Control (Steering) ===
// ===================================================
Servo myServo;

// Conversion helper: map 0–270° → 500–2500 µs
static inline int deg270ToUs(float deg) {
  deg = constrain(deg, 0.0f, 270.0f);
  return (int)(500.0f + deg * (2000.0f / 270.0f));
}

// Steering positions (tuned manually for your servo geometry)
const float SERVO_LEFT_DEG      = 130.0f;
const float SERVO_STRAIGHT_DEG  = 94.5f;
const float SERVO_STRAIGHT1_DEG = 100.0f;
const float SERVO_RIGHT_DEG     = 50.0f;

// ===================================================
// === Smooth turning (gradual servo movement) ===
// ===================================================
void smoothTurn(float targetDeg) {
  int currentUs = myServo.readMicroseconds();
  int targetUs = deg270ToUs(targetDeg);
  int step = (targetUs > currentUs) ? 5 : -5; // Move in small steps

  for (int us = currentUs; us != targetUs; us += step) {
    myServo.writeMicroseconds(us);
    delay(5);  // Smooth movement
    if ((step > 0 && us >= targetUs) || (step < 0 && us <= targetUs)) break;
  }

  myServo.writeMicroseconds(targetUs); // Final position
}

// ===================================================
// === F1 Lights Start Sequence Variables ===
// ===================================================
bool f1SequenceActive = false;
unsigned long f1StartTime = 0;

// ===================================================
// === STOP Sequence Variables ===
// ===================================================
bool stopActive = false;
unsigned long stopStartTime = 0;
int stopStep = 0;
bool blinkState = false;
unsigned long lastBlinkTime = 0;

// ===================================================
// === SETUP ===
// ===================================================
void setup() {
  // Servo steering setup
  myServo.attach(3, 500, 2500);
  smoothTurn(SERVO_STRAIGHT_DEG); // Start straight

  // Motor driver setup
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // LEDs setup
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);

  Serial.begin(9600);
  Wire.begin();

  // -----------------------
  // TOF (VL53L0X) Init
  // -----------------------
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);

  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(100);

  // Sensor1 (Left)
  digitalWrite(XSHUT_1, HIGH);
  delay(50);
  if (sensor1.init()) sensor1.setAddress(0x30);
  else { Serial.println("Sensor 1 failed!"); while (1); }

  // Sensor2 (Right)
  digitalWrite(XSHUT_2, HIGH);
  delay(50);
  if (sensor2.init()) sensor2.setAddress(0x31);
  else { Serial.println("Sensor 2 failed!"); while (1); }

  sensor1.setMeasurementTimingBudget(200000);
  sensor2.setMeasurementTimingBudget(200000);

  sensor1.startContinuous();
  sensor2.startContinuous();
  Serial.println("✅ TOF sensors ready");

  // -----------------------
  // MPU6050 Init
  // -----------------------
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    dmpReady = true;
    Serial.println("✅ DMP Initialized");
  } else {
    Serial.println("❌ DMP init failed");
    while (1); // Halt if failed
  }

  Serial.println("===============================");
  digitalWrite(LED1, HIGH); // ✅ All sensors initialized
}

// Convert raw yaw radians → positive degrees (0–360)
float toPositiveDegrees(float rawYaw) {
  float yawDeg = rawYaw * 180 / M_PI;
  if (yawDeg < 0) yawDeg += 360;
  return fmod(yawDeg, 360);
}

// ===================================================
// === STOP Handler (sequential LED + motor off) ===
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

    // Stop motors once all LEDs are ON
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    lastBlinkTime = millis();
  }

  // After LEDs ON → blink all together
  if (stopStep >= 3) {
    if (millis() - lastBlinkTime >= 500) {
      blinkState = !blinkState;
      digitalWrite(LED1, blinkState);
      digitalWrite(LED2, blinkState);
      digitalWrite(LED3, blinkState);
      lastBlinkTime = millis();
    }
  }
}

// ===================================================
// === MAIN LOOP ===
// ===================================================
void loop() {
  // -----------------------
  // Raspberry Pi Commands
  // -----------------------
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    // Pi ready handshake
    if (cmd == "READY") {
      piReady = true;
      digitalWrite(LED2, HIGH);
      Serial.println("📥 Pi READY received");
    }

    // STOP command → triggers stop sequence
    if (cmd == "STOP"){
      Serial.println("3 laps Completed, stopping now...");
      stopActive = true;
      stopStartTime = millis();
      stopStep = 0;
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
    }

    // Steering commands (ignored if already turning or stopping)
    if (!turning && !stopActive) {
      if (cmd == "LEFT") {
        Serial.println("📥 Pi Command: LEFT → Start Turn");
        turnStartYaw = stableYaw;
        currentTurn = LEFT_TURN;
        turning = true;
        smoothTurn(SERVO_LEFT_DEG);
      } 
      else if (cmd == "RIGHT") {
        Serial.println("📥 Pi Command: RIGHT → Start Turn");
        turnStartYaw = stableYaw;
        currentTurn = RIGHT_TURN;
        turning = true;
        smoothTurn(SERVO_RIGHT_DEG);
      } 
      else if (cmd == "STRAIGHT") {
        Serial.println("📥 Pi Command: STRAIGHT (No turn)");
        smoothTurn(SERVO_STRAIGHT_DEG);
      }
    }
  }

  // -----------------------
  // STOP Sequence Handler
  // -----------------------
  if (stopActive) {
    handleStop();
    return; // 🔴 Everything else frozen during STOP
  }

  // -----------------------
  // Button Start → F1 lights
  // -----------------------
  bool buttonState = digitalRead(BUTTON_PIN);
  if (buttonState == LOW && lastButtonState == HIGH && !f1SequenceActive && !motorRunning) {
    Serial.println("🏎️ Button pressed → Starting F1 Lights Sequence");
    f1SequenceActive = true;
    f1StartTime = millis();
    digitalWrite(LED1, LOW);
    digitalWrite(LED2, LOW);
    digitalWrite(LED3, LOW);
    delay(500); // Small delay before lights sequence
  }
  lastButtonState = buttonState;

  // -----------------------
  // F1 Lights Non-blocking Sequence
  // (LED1 → LED2 → LED3 → Lights Out → GO)
  // -----------------------
  if (f1SequenceActive) {
    unsigned long elapsed = millis() - f1StartTime;
    if (elapsed < 666) {
      digitalWrite(LED1, HIGH);
    } else if (elapsed < 1332) {
      digitalWrite(LED2, HIGH);
    } else if (elapsed < 1998) {
      digitalWrite(LED3, HIGH);
    } else {
      // Lights Out → Start motors
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
      digitalWrite(LED3, LOW);
      motorRunning = true;
      f1SequenceActive = false;
      Serial.println("🟢 Lights Out → Motor STARTED");
    }
  }

  // -----------------------
  // Motor Control (Forward)
  // -----------------------
  if (motorRunning) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 140);   // Motor speed
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(ENA, 0);     // Stop motor
  }

  float currentYaw = 0.0;

  // -----------------------
  // MPU6050 Yaw Reading + Calibration
  // -----------------------
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    Quaternion q;
    VectorFloat gravity;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentYaw = toPositiveDegrees(ypr[0]);
    stableYaw = currentYaw;

    // Initial yaw calibration: wait until stable for 1.5s
    static float lastYawCheck = 0;
    static unsigned long calibrationStart = 0;
    static bool calibrating = false;

    if (!yawSaved) {
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
          digitalWrite(LED3, HIGH);
        }
      } else {
        calibrationStart = millis();
        lastYawCheck = currentYaw;
      }
    }

    // -----------------------
    // Turn Completion Logic
    // -----------------------
    if (turning) {
      float targetYaw = turnStartYaw;
      if (currentTurn == RIGHT_TURN) { targetYaw += 80; if (targetYaw >= 360) targetYaw -= 360; }
      else if (currentTurn == LEFT_TURN) { targetYaw -= 85; if (targetYaw < 0) targetYaw += 360; }

      float yawError = fabs(currentYaw - targetYaw);
      if (yawError > 180) yawError = 360 - yawError;

      // If target yaw reached → straighten
      if (yawError <= 5 && currentTurn == RIGHT_TURN) {
        Serial.println("✅ TURN COMPLETE → STRAIGHT");
        Serial.println("STRAIGHT");
        smoothTurn(SERVO_STRAIGHT1_DEG);
        turning = false;
        currentTurn = NONE;
      }
      else if (yawError <= 5 && currentTurn == LEFT_TURN) {
        Serial.println("✅ TURN COMPLETE → STRAIGHT");
        Serial.println("STRAIGHT");
        smoothTurn(SERVO_STRAIGHT_DEG);
        turning = false;
        currentTurn = NONE;
      }
    }
  }

  // -----------------------
  // TOF Obstacle Avoidance
  // -----------------------
  if (!turning) { // Only if not already turning by Pi
    uint16_t leftDist = sensor1.readRangeContinuousMillimeters();
    uint16_t rightDist = sensor2.readRangeContinuousMillimeters();

    // Right obstacle → steer right
    if (!avoidingLeftObstacle && rightDist < 150) {
      Serial.println("🟠 TOF: OBSTACLE ON RIGHT → TURN LEFT");
      Serial.println("RIGHT");
      smoothTurn(SERVO_RIGHT_DEG);
      avoidingRightObstacle = true;
    } else if (avoidingRightObstacle && rightDist >= 150) {
      Serial.println("🟢 RIGHT CLEAR → STRAIGHT");
      Serial.println("STRAIGHT");
      smoothTurn(SERVO_STRAIGHT1_DEG);
      avoidingRightObstacle = false;
    }

    // Left obstacle → steer left
    if (!avoidingRightObstacle && leftDist < 150) {
      Serial.println("🔵 TOF: OBSTACLE ON LEFT → TURN RIGHT");
      Serial.println("LEFT");
      smoothTurn(SERVO_LEFT_DEG);
      avoidingLeftObstacle = true;
    } else if (avoidingLeftObstacle && leftDist >= 150) {
      Serial.println("🟢 LEFT CLEAR → STRAIGHT");
      Serial.println("STRAIGHT");
      smoothTurn(SERVO_STRAIGHT_DEG);
      avoidingLeftObstacle = false;
    }
  }
}
