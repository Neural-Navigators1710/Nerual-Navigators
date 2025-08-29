//How to trigger STOP
//Inside loop() or your serial command handler:

if (cmd == "STOP") {
  Serial.println("3 laps Completed, stopping now...");
  stopActive = true;
  stopStartTime = millis();
  stopStep = 0;
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
}

//And In loop
if (stopActive) {
  handleStop();
  return; // freeze everything else once STOP begins
}

// === STOP Sequence Variables ===
bool stopActive = false;
unsigned long stopStartTime = 0;
int stopStep = 0;
bool blinkState = false;
unsigned long lastBlinkTime = 0;

// === Motor Driver Pins (example, adjust if needed) ===
#define IN1 10
#define IN2 9
#define ENA 11

// === LED Pins ===
#define LED1 2
#define LED2 4
#define LED3 6

// === STOP Handler ===
void handleStop() {
  unsigned long elapsed = millis() - stopStartTime;

  if (stopStep == 0 && elapsed >= 1000) {
    digitalWrite(LED1, HIGH);
    stopStep = 1;
  } 
  else if (stopStep == 1 && elapsed >= 2000) {
    digitalWrite(LED2, HIGH);
    stopStep = 2;
  } 
  else if (stopStep == 2 && elapsed >= 3000) {
    digitalWrite(LED3, HIGH);
    stopStep = 3;

    // Stop motors once all LEDs are ON
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    lastBlinkTime = millis();
  }

  // After all LEDs ON → continuous blink
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
