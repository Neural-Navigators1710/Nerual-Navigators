# 📘 WRO Future Engineers Weekly Journal (Aug 4 – Aug 23)

This journal documents all the updates and progress made on the robot project during this period.  
Both **hardware** and **software** milestones are included, along with design decisions and challenges.

---

## 🛠️ Hardware Updates
- ✅ Switched from **TCS34725 color sensor** to **Raspberry Pi Camera** for detecting **orange** and **blue** lines in the Open Challenge.  
  - Reason: Too many I²C connections + slower detection speed on Nano.  
  - PiCam + OpenCV is now used for color segmentation, giving faster results.
- ✅ Power switching finalized: using a **DPST switch** to safely toggle both Nano + Pi power lines without cross-over issues.  
- ✅ Integration of **TOF sensors** and **MPU6050 gyro** completed — hardware wiring and testing confirmed.  
- 🛠️ Next step: Mount all sensors + components on chassis and perform final field tests.

---

## 💻 Software Updates
- ✅ Raspberry Pi now handles **line detection** (orange/blue) via OpenCV and sends LEFT/RIGHT signals to Arduino Nano over serial.  
- ✅ Arduino Nano controls the **servo turning mechanism**:
  - When Pi sends `LEFT` or `RIGHT`, Nano turns servo accordingly.  
  - Works for both **Obstacle Challenge (TOF-based)** and **Open Challenge (color-based)** turns.  
- ✅ TOF sensors are always active:  
  - If distance < 5 cm → Nano sends `LEFT` or `RIGHT` command to Pi → Pi forwards turn signal back to Nano for servo action.  
  - Independent from color detection or gyro stabilization.  
- ✅ Gyro logic stable: yaw delta tracked for 90° turns, with reset to straight after turn.  

---

## 🔄 Communication Flow (Pi ↔️ Nano)
1. **Color/TOF decision made** (Pi for colors, Nano for TOF).  
2. Pi sends `"LEFT"` or `"RIGHT"` via serial to Nano.  
3. Nano executes servo turn + motor control.  
4. After completing turn (gyro stabilized), Nano resets servo to `STRAIGHT`.  

---

## 📊 Progress Table

| Date       | Update                                                                 |
|------------|-------------------------------------------------------------------------|
| Aug 4–7    | Finalized Arduino Nano code (TOF, gyro, color) ✅                       |
| Aug 8–12   | Journal updates & refactor for Pi ↔️ Nano communication 🔄              |
| Aug 13–16  | PiCam + OpenCV color detection (orange/blue) replaced TCS34725 🎥       |
| Aug 17–20  | DPST switch finalized for safe dual-power handling 🔌                   |
| Aug 21–23  | TOF + gyro + PiCam integration tested in software — ready for mounting 🏗️ |

---

## ✅ Current Status
- All **software modules ready and tested** (PiCam detection, TOF obstacle handling, gyro turns, Pi↔️Nano serial).  
- Power switching system confirmed safe with DPST.  
- Servo + motor control stable after turns.  
- Only **final mounting and field test** pending.

---

## 🎯 Next Steps
- Mount sensors and camera on chassis.  
- Perform **full run testing** for both **Obstacle Challenge** and **Open Challenge**.  
- Fine-tune thresholds (color + TOF distance) on actual field conditions.  
