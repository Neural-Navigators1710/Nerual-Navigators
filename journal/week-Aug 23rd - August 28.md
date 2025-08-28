# 📘 WRO Future Engineers Weekly Journal (Aug 23 – Aug 27)

This journal documents all the updates and progress made on the robot project during this period.  
Both hardware and software milestones are included, along with design decisions and challenges.

---

## 🛠️ Hardware Updates
- ✅ Completed wiring of **left + right VL53L0X TOF sensors** and validated readings via serial.  
- ✅ Integrated **servo smooth turning** (microsecond stepping) for more precise and consistent turns.  
- ✅ Confirmed hardware response during **test runs for Open Challenge** (color-based turns + gyro stabilization + TOF avoidance).  
- ⚠️ Issue discovered: TOF sensors stop responding when **motors are powered ON** → traced to **power noise from L298N + motors**.  

---

## 💻 Software Updates
- ✅ Implemented **non-blocking servo turn logic** on Nano to prevent abrupt movement.  
- ✅ Added **Pi-triggered turn handling** (`LEFT`, `RIGHT`, `STRAIGHT`) alongside Nano’s TOF-based obstacle avoidance.  
- ✅ Integrated `STRAIGHT` command handling to re-center servo after both Pi-based and TOF-based turns.  
- ✅ Debugged camera freeze issue on Raspberry Pi caused by blocking `sleep()` → identified need for non-blocking turn sequences.  

---

## 🔄 Communication Flow (Pi ↔️ Nano)
1. Pi detects **orange/blue** lines via camera → sends `LEFT` or `RIGHT` over serial.  
2. Nano executes turn (servo + gyro yaw tracking).  
3. After 90° turn complete, Nano re-centers servo and sends `STRAIGHT`.  
4. Independent TOF avoidance remains active → Nano directly sends turn commands to Pi when obstacle detected.  

---

## 📊 Progress Table

| Date      | Update |
|-----------|--------|
| Aug 23    | Refined MPU6050 yaw-based turning, added serial handling for `LEFT`/`RIGHT`/`STRAIGHT`. |
| Aug 24    | Improved Nano code for dual VL53L0X initialization, validated consistent sensor readings. |
| Aug 25    | Implemented obstacle logic (TOF left/right avoidance), tested with motors OFF, debugged Pi USB issue. |
| Aug 26    | Added smooth servo control, integrated Pi-triggered turns + TOF avoidance, test runs for Open Challenge. |
| Aug 27    | Found TOF sensors fail when motors ON → diagnosed as power noise issue, documented hardware fixes. |

---

## ✅ Current Status
- All core modules functional: **PiCam detection**, **TOF obstacle avoidance**, **gyro-based turns**, **Pi↔️Nano communication**.  
- Successfully conducted **Open Challenge test runs** (color → turn → re-center with gyro).  
- **Hardware interference issue remains**: TOF sensors unreliable when motors are active.  

---

## 🎯 Next Steps
- Add **capacitors and improved wiring layout** to stabilize TOF readings with motors active.  
- Optimize **motor PWM duty cycle** to reduce noise spikes.  
- Implement **non-blocking motor/servo control** on Pi to keep camera feed smooth.  
- Continue **field test runs for Open Challenge** and start logging performance metrics (turn accuracy, obstacle success rate, completion time).  
