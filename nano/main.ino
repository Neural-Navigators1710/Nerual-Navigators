#include <Wire.h>
#include "Adafruit_TCS34725.h"
#include <ESP32Servo.h>  // Use this instead of Servo.h

Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

Servo myServo;  // ESP32Servo
const int servoPin = 32;

// Ultrasonic sensor pins
const int TRIG_LEFT = 2;
const int ECHO_LEFT = 18;
const int TRIG_RIGHT = 5;
const int ECHO_RIGHT = 19;  

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); // ESP32 I2C

  if (!tcs.begin()) {
    Serial.println("TCS34725 not found!");
    while (1);
  }

  myServo.setPeriodHertz(50);    // Standard 50 Hz servo
  myServo.attach(servoPin, 500, 2400); // Min/max pulse width (us)
  myServo.write(120);  // Start centered

  // Ultrasonic pins setup
  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);
}
bool isBlack(float r, float g, float b, int c) {
  return (c < 500 &&
          r > 0.35 && r < 0.40 &&
          g > 0.28 && g < 0.31 &&
          b > 0.37 && b < 0.39);
}

bool isWhite(float r, float g, float b, int c) {
  return (c > 2400 &&
          r > 0.20 && r < 0.25 &&
          g > 0.30 && g < 0.35 &&
          b > 0.42 && b < 0.46);
}

float readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // timeout after 20ms
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

void loop() {
  // Read ultrasonic sensors
  float distLeft = readDistanceCM(TRIG_LEFT, ECHO_LEFT);
  float distRight = readDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  Serial.print("Left Distance: "); Serial.print(distLeft); Serial.print(" cm, ");
  Serial.print("Right Distance: "); Serial.print(distRight); Serial.println(" cm");

  // Obstacle detection has higher priority
  if (distLeft > 0 && distLeft < 10) {
    Serial.println("🚧 Obstacle LEFT – Turn RIGHT");
    myServo.write(70);  // Turn right
  } else if (distRight > 0 && distRight < 10) {
    Serial.println("🚧 Obstacle RIGHT – Turn LEFT");
    myServo.write(180);  // Turn left
  } 
  else {
    // Fallback to color sensor
    uint16_t r_raw, g_raw, b_raw, c;
    tcs.getRawData(&r_raw, &g_raw, &b_raw, &c);

    float r = (float)r_raw / c;
    float g = (float)g_raw / c;
    float b = (float)b_raw / c;

    Serial.print("R: "); Serial.print(r, 2);
    Serial.print(" G: "); Serial.print(g, 2);
    Serial.print(" B: "); Serial.print(b, 2);
    Serial.print(" Clear: "); Serial.println(c);

    if (isBlack(r, g, b, c)) {
      Serial.println("⚫ BLACK detected – Turn RIGHT");
      myServo.write(0); // Right
    } else if (isWhite(r, g, b, c)) {
      Serial.println("⚪ WHITE detected – Turn LEFT");
      myServo.write(180); // Left
    } else {
      Serial.println("❓ Unknown – Centered");
      myServo.write(90); // Center
    }
  }

  delay(500);
}

