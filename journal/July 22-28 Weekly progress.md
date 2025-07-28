# 🚀 WRO Future Engineers – Weekly Progress Journal  
**Week:** July 22–28, 2025  
---

## ✅ Summary of Progress

### 🛠️ Hardware Integration  
- Successfully connected **dual VL53L0X TOF sensors** via I2C by using `XSHUT` pins to assign new addresses (`0x30`, `0x31`).  
- Set up **TCS34725 color sensor** with controlled LED lighting via GPIO.  
- Integrated **MPU6050 DMP** yaw tracking for smooth 90° turns.

### 💡 Issue Solving: Multiple I2C Devices  
**Problem:**  
Multiple I2C devices (TOF sensors + TCS + MPU6050) clashed due to shared default I2C address.  
**Solution:**  
- Used `XSHUT` pins on Nano to disable one sensor at a time.
- Initialized each TOF sensor with unique address using `setAddress()`.  
- Confirmed functionality with continuous readings from both sensors.

### 🎯 Logic Improvements  
- **TOF obstacle avoidance** logic made completely independent of color and gyro logic.  
- If obstacle is detected:
  - Left TOF < 5 cm → Send `RIGHT` to RPi → Servo turns right until clear.
  - Right TOF < 5 cm → Send `LEFT` to RPi → Servo turns left until clear.  

### 🤖 Motor Control  
- Replaced standard BO motor with **BO Dual-Shaft Metal Gear Motor (110 RPM)** for higher torque.
- Controlled via **L298N motor driver** (mistakenly referred to as LM2596N before).

---

## 🧠 Codebase Updates

### 📝 `nano_open.ino` (Arduino Nano)
- Cleaned up logic to make TOF decisions independent of color & gyro.
- Modularized yaw logic to trigger only for color-based turns.
- Final messages sent over serial: `LEFT`, `RIGHT`, `STRAIGHT`.

### 🐍 `main_open.py` (Raspberry Pi)
- Servo turning jitter completely eliminated.
- Smoothed servo control using `set_servo_angle()` abstraction.
- Adjusted duty cycles:
  - `7.5` = Straight (90°)
  - `3.5` = 60° Left
  - `11.5` = 120° Right

---

## 📁 Files Modified This Week
- `nano_open.ino`  
- `main_open.py`

---

## 🔍 Testing Status
| Feature | Status | Comments |
|--------|--------|----------|
| Dual TOF Distance Detection | ✅ | Independent and responsive |
| Color Detection & Turn | ✅ | Detects blue/orange reliably |
| Gyro-based Turn Completion | ✅ | 90° completion using delta yaw |
| Servo Control via RPi | ✅ | No jitter, smooth turns |
| Communication (Serial) | ✅ | All messages received as expected |

---

## 📸 Media & Attachments (Not Included)
- Video of robot avoiding wall while turning right via TOF input.
- Screenshot of dual-shaft motor selected.

---

## 📌 Next Steps
- Mount new high-torque motors with 2-wheel shaft system.
- Begin full maze navigation test.
- Tune servo angles further if needed.
- Push repo to GitHub with docs & wiring diagram.

---

🧠 **Team Reflection:**  
This week, we learned a lot about modularizing logic between sensors and separating responsibilities. Handling multiple I2C devices with dynamic address assignment was a huge win and crucial for expanding sensor coverage in future designs!
