**Week:** July 28 – August 4, 2025  

---

## ✅ Overview

This week focused on integrating and debugging the following systems:
- Color-based turning logic using TCS34725
- Obstacle avoidance using dual VL53L0X TOF sensors
- Yaw tracking with MPU6050 (DMP)
- Serial communication and logic separation between Nano and Raspberry Pi
- Hardware-level fixes (switching, power conflicts, LED interference)

---

## ⚙️ Hardware Setup

- **Controller:** Arduino Nano (powered by LiPo)
- **Raspberry Pi:** Connected via USB (powered by power bank)
- **Color Sensor:** TCS34725 via I2C  
- **Distance Sensors:** 2x VL53L0X (addressed 0x30, 0x31) using XSHUT pins  
- **IMU:** MPU6050 (Yaw tracking with DMP enabled)  
- **Motor Driver:** L298N (IN1, IN2, ENA connected)  
- **User Inputs:** 
  - Push button on D5 to start motor
  - Power switch for Nano-LiPo connection
- **LEDs:**
  - D2: Status LED
  - D3: Yaw calibration complete
  - D4: Pi signal received (deprecated)
- **Resistor:** 330Ω in series with LEDs

---

## 🧠 Features Implemented

- **Color Detection (TCS34725):**
  - Normalized RGB thresholding for orange and blue
  - Blue → LEFT turn, Orange → RIGHT turn
  - “UNKNOWN COLOR” fallback with raw data print

- **Obstacle Avoidance (VL53L0X):**
  - Continuous distance tracking
  - <50mm triggers LEFT/RIGHT turn
  - Clearance detection to resume STRAIGHT

- **Turning (MPU6050):**
  - Calibrated yaw detection
  - 90° turn logic with deltaYaw
  - Only one turn at a time (`turning = true/false`)

- **Motor Control:**
  - Button press starts motor
  - Controlled by push button and `motorRunning` flag

- **Raspberry Pi Serial Signal:**
  - Initial “START” signal support (now removed from code)
  - Nano now works independently

---

## 🛠️ Issues Faced & Fixes

| Issue | Fix |
|------|-----|
| Nano powering on when Pi USB is connected | Cut VCC (red) wire in USB cable to disable Nano power from Pi |
| LED connected to D2 affected TCS sensor power | Moved LED to unused digital pin and checked resistor |
| TCS34725 not detected | Added power pin logic: `digitalWrite(TCS_POWER_PIN, HIGH)` |
| Serial monitor not printing | Ensured `Serial.begin(9600)` is called early and USB cable is data-compatible |
| MPU yaw was unstable | Added logic to store yaw only after stabilizing for 1.5s |
| Pi START logic was blocking rest of setup | Removed Pi signal dependency from logic |

---

## 🧪 Calibration Code

- Created a standalone calibration sketch for the color sensor
- Printed normalized RGB values live to tune threshold ranges
- Final thresholds (working reliably):

```cpp
// Orange
r: 0.36–0.46  
g: 0.28–0.32  
b: 0.30–0.39  

// Blue
r: 0.35–0.39  
g: 0.29–0.34  
b: 0.40–0.45  

🔄 Final Code Behavior (As of Aug 4)
	•	Code runs fully without waiting for Raspberry Pi
	•	All sensors initialize on startup
	•	Color detection + obstacle avoidance + gyro turn all work together
	•	Motor starts on push button press
	•	LED feedback integrated

📅 Planned Next Week
	•	Begin testing with arena floor
	•	Tune thresholds under different lighting
	•	Enable Pi to send commands again (optional)

