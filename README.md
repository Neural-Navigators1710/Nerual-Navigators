
# 🤖 WRO Future Engineers 2025 -- Self-Driving Robot

An intelligent, self-driving robot designed for the **WRO Future Engineers 2025** category.
This robot is not just a machine -- it is a research project, a platform for experimentation, and a demonstration of how **affordable robotics** can achieve real-time autonomy in structured racing challenges.

Our design focuses on **reliability over luck**, **clarity over complexity**, and **team learning over shortcuts**.

> 💡 *Built with precision. Driven by code. Guided by engineering.*

---

## 📑 Table of Contents

- [🚀 Overview](#-overview)
- [🚗 Features](#-features)
- [🏁 Challenge Entry Points](#-challenge-entry-points)
- [📂 Project Structure](#-project-structure)
- [🔧 Setup Instructions](#-setup-instructions)
- [🧠 Architecture](#-architecture)
- [🧩 Design Rationale](#-design-rationale)
- [⚙️ Hardware Design](#️-hardware-design)
- [🧾 Bill of Materials (BOM)](#-bill-of-materials-bom)
- [🔌 Wiring & Interfaces](#-wiring--interfaces)
- [🖥️ Software & Algorithms](#️-software--algorithms)
- [🧪 Testing & Calibration](#-testing--calibration)
- [📊 Performance Metrics](#-performance-metrics)
- [🏆 Scoring Analysis & Strategy](#-scoring-analysis--strategy)
- [🧰 Troubleshooting Guide](#-troubleshooting-guide)
- [🧭 Event-Day Checklists](#-eventday-checklists)
- [🧱 Risk & Mitigations](#-risk--mitigations)
- [📈 Future Work](#-future-work)
- [📔 Extended Engineering Notes (Snapshot)](#-extended-engineering-notes-snapshot)
- [📸 Media](#-media)
- [👨‍💻 Contributors](#-contributors)
- [📜 License](#-license)
- [❓ FAQ & Glossary](#-faq--glossary)

---

## 🚀 Overview

This repository documents the complete design, development, and testing of our self-driving car for **WRO Future Engineers 2025**. It contains:

- Source code for both **Open Challenge** and **Obstacle Challenge**
- Arduino Nano firmware for **motor, sensor, and servo control**
- Vision algorithms for **obstacle and wall detection**
- CAD files, circuit diagrams, and setup notes
- Full engineering journal with testing logs, photos, and team notes

We believe in **open-source robotics**. Everything here can be reused and improved by future teams.

### Motivation

The motivation behind building this robot was not just to compete, but to **explore how accessible hardware and software can be pushed to their limits**. Robotics can feel inaccessible; platforms like Raspberry Pi and Arduino have changed that. Our goal is to demonstrate that with **tight engineering, clear state machines, and careful calibration**, a modestly priced platform can deliver robust autonomy.

The **Future Engineers** category is special because it forces teams to combine **vision, control, embedded systems, and systems engineering**. The robot must interpret a physical world (lighting shifts, surface texture, occlusions) and make **real-time decisions** under competition constraints. Our design is therefore not a collection of hacks; it's a **cohesive architecture** where each component (hardware + software) plays a defined role with clear interfaces.

### Design Principles

1. **Determinism beats luck** -- We favor predictable state machines, bounded latencies, and explicit transitions.
2. **Modularity** -- Pi (planning & vision) and Nano (actuation & I/O) are decoupled to simplify debugging and recovery.
3. **Graceful degradation** -- If vision is noisy, TOF sensors keep minimum-distance safety. If serial drops, Nano fails safe.
4. **Field-ready calibration** -- HSV ranges, steering trims, PID gains, and yaw filters are **fast to re-tune on-site**.
5. **Rule compliance upfront** -- Start/stop procedure, single main switch, push-button start, no remote control, etc.

---

## 🚗 Features

### 🎯 Lane & Wall Detection (Vision)
- **OpenCV** pipeline on **PiCam 4** using **HSV color space** for robustness against brightness variations.
- Region of interest + morphological operations → stable lane/wall masks even with glare and shadows.
- Produces a **steering target** (centerline error) that feeds the steering controller.

**Why HSV?** Hue separates color from illumination. RGB shifts with brightness; HSV lets us lock in color ranges while tolerating exposure changes.

---

### 🧭 Lap Counting & Zone Recognition
- The arena is logically divided into **8 sections**.
- We use **signature cues** (geometry + color features + heading) to detect transitions.
- A **finite state machine (FSM)** validates a full cycle before incrementing laps; spurious detections don't count.
- After **exactly 3 laps**, the robot transitions to a **controlled stop** in the **original start section**.

---

### ⛔ Obstacle Detection & Avoidance
- **VL53L0X TOF** sensors provide reliable short-range distance independent of texture/color.
- Side-angled mounting detects approach and alignment during bypass.
- Hard safety stop if an object is detected below a configurable threshold (e.g., **≤ 50 mm**).

---

### 🚦 Traffic Sign Handling (Obstacle Challenge)
- **Red pillar** → pass on **left**. **Green pillar** → pass on **right**.
- HSV color classifier + contour centroid location → side decision.
- **Self-compliance**: if logic believes the wrong side is unavoidable, it aborts and stops to respect rules.

---

### 🛑 Accurate Stop After 3 Laps
- Lap FSM triggers **deceleration profile** (smooth braking), then **servo neutral**, then **final halt**.
- Stop only in **start section**; if stop section mismatch is detected, the robot does **one corrective loop** (if time allows) or halts safely.

---

### 🔁 Seamless Mode Switching
- Two clean entry points:
  - `main/main_open.py` → **Open Challenge**
  - `main/obstacle_challenge.py` → **Obstacle Challenge**
- Shared libraries for sensors, control, and utilities to avoid code duplication.

---

### 🧩 Modular Compute Split
- **Raspberry Pi 4** → vision, high-level FSM, path decisions.
- **Arduino Nano** → motor PWM, steering servo, TOF reads, debounce for button/LED.
- Serial protocol is **human-readable** for easy debugging (see [Wiring & Interfaces](#-wiring--interfaces)).

---

### 🧭 Yaw Correction (MPU6050)
- Complementary filter stabilizes yaw for consistent turns and section changes.
- **90° turn detection** used to catch misalignment during obstacle bypass.

---

### 🔌 Push-Button Start & LED (Rule Compliance)
- Single main power switch.
- LED shows **IDLE → READY → RUN**.
- Program starts only on **button press**; no external control after start, per competition rules.

---

## 🏁 Challenge Entry Points

Each challenge has a dedicated entry script:

```bash
# Open Challenge
python3 main/main_open.py

# Obstacle Challenge
python3 main/obstacle_challenge.py
```



**📂 Project Structure**
------------------------

```
WRO2025-SelfDrivingRobot/
│
├── LICENSE                   # Open-source license
├── README.md                 # Engineering Journal (this file)
│
├── main/                     # Entry points for challenges
│   └── obstacle_challenge.py # Obstacle Challenge logic
│
├── vision/                   # Computer vision algorithms
│   └── obstacle_detection.py # Obstacle recognition & HSV filtering
│
├── logic/                    # Core decision-making
│   ├── lap_counter.ino        # Lap tracking FSM
│   └── stop_handler.ino      # Arduino stop handler
│
├── nano/                     # Nano firmware
│   ├── main_open.ino        # Controls motor, servo, sensors
│   └── stop_obstacle.ino
├── setup_and_tests/          # Calibration utilities
│   ├── callibration.py
│
├── utils/                    # Helper documents
│   └── Requirements.md       # Dependencies and setup notes
│
├── assets/                   # Photos and media
│   ├── team_photos/          # Team photos
│   └── vehicle_photos/       # Robot photos
│
└── journal/                  # Engineering Journal notes
```

* * * * *

**🔧 Setup Instructions**
-------------------------

1.  **Install dependencies** (on Raspberry Pi):

```
pip install -r utils/Requirements.md
```

1.

2.  **Connect hardware**

    -   Attach **PiCam 4** to Raspberry Pi

    -   Connect **Arduino Nano** to servo, motors, and sensors

    -   Wire **push button + LED** to GPIO via Nano or Pi (we use Nano)

    -   Use **Serial (UART)** between Pi ↔ Nano (115200 baud)

3.  **Run a challenge**

```
# Open Challenge
python3 main/main_open.py

# Obstacle Challenge
python3 main/obstacle_challenge.py
```

1.

2.  **Start procedure (per WRO rules)**

    -   Switch robot **OFF → ON** via the **single main switch**

    -   Robot enters **IDLE** (LED = red)

    -   Press the **start button**

    -   On **"Go!"**, LED = green, robot begins the run

* * * * *

## 🧠 Architecture

| **Component**        | **Role**                                        |
|-----------------------|-------------------------------------------------|
| Raspberry Pi 4        | Vision (OpenCV), high-level FSM, decisions      |
| PiCam 3               | Image capture; HSV pipeline for color & lanes   |
| Arduino Nano          | Motor PWM, steering servo, TOF polling, LED/button |
| L298N Driver          | H-bridge motor driver                          |
| Servo(MG996R)           | Steering actuator                              |
| DC Motors             | Propulsion                                     |
| MPU6050               | Yaw stabilization (IMU)                        |
| VL53L0X TOF           | Short-range obstacle sensing                   |
| TCS34725 Color Sensor | Optional color cross-check (blue/orange lanes) |
| LEDs + Button         | Start/stop signaling & status                  |
| DPST Switch + Pull Up Switch         | Powering on the Circuit and Running the motors                 |
| XL6009         | Steup Up Buck meter for Powering Raspberry pi                  |
| Dart PCB (Prototype Board)        | To Solder Components                 |
| Resistor       | For LEDs and Push Buttons                  |
| Headers        | For Connecting Jumpers                 |
| SD Card        | Raspberry Pi Memory                 |
| LiPo Battery       | Power and suffecient Current                  |






### **Communication Flow**

-   **Transport:** UART (9600 baud) over USB or GPIO serial.

-   **Pi → Nano Commands (ASCII):**

    SET:SPD:<0-255>, SET:STR:<-100..100>, CMD:STOP, CMD:STRAIGHT, CMD:LEFT, CMD:RIGHT

-   **Nano → Pi Feedback (ASCII):**

    ACK:OK, SENS:TOF:<mm>, SENS:YAW:<deg>, EVENT:TURN_DONE

This **text-based protocol** is easy to log and debug in the field.

The TOF is used for turning corners in both obstacle challenge and open challenge by detecting the change in measured values on either side and turns to the direction with more distance. Raspberry Pi cam is used for Obstacle challenge


* * * * *

**🧩 Design Rationale**
-----------------------

-   **Dual-Controller Split (Pi + Nano):**

    PWM and servo timing are sensitive to jitter; moving real-time actuation to Nano keeps control signals stable while Pi handles heavy vision tasks.

-   **TOF vs Ultrasonic:**

    TOF (VL53L0X) has **narrow FOV and accurate mm-range** readings regardless of surface color/texture at short range. Ultrasonic can be flaky with angled or soft surfaces.

-   **Complementary Filter for Yaw:**

    Gyro integrates quickly but drifts; accelerometer corrects slower but anchors long-term. The complementary filter provides a **low-latency yet stable** yaw.

-   **FSM Everywhere:**

    Laps, start/stop, obstacle bypass, and parking are all modeled as **finite state machines** to keep behavior deterministic and testable.

* * * * *

**⚙️ Hardware Design**
----------------------

-   **Chassis:** Custom 3D printed + acrylic plates for rigidity and easy mounting

-   **Wheels:** 4-wheel, front steering (Ackermann), rear drive

-   **Drive System:**

    -   1 DC motor pair → rear axle

    -   1 servo → steering linkage

-   **Power:** 3S Li-Po (11.1V, 2200 mAh) with power distribution

-   **Safety:** Fuse + cutoff, strain relief on cables, insulated motor leads

-   **Mounting:** Camera shock-isolated, sensors angled for FOV coverage

* * * * *

## 🧾 Bill of Materials (BOM)

| **Item**                  | **Qty** | **Notes**                          |
|----------------------------|---------|-------------------------------------|
| Raspberry Pi 4 (2–4 GB)    | 1       | Main compute                        |
| PiCam 4                    | 1       | 30 FPS capture                      |
| Arduino Nano               | 1       | Real-time actuation                 |
| L298N Motor Driver         | 1       | H-bridge for DC motors              |
| DC Gear Motors             | 2       | Rear drive                          |
| Metal-gear Servo           | 1       | Steering actuator                   |
| VL53L0X TOF Sensor         | 1–2     | Short-range obstacle detection      |
| MPU6050 IMU                | 1       | Yaw stabilization                   |
| TCS34725 Color (optional)  | 1       | Cross-check lane colors             |
| Push Button + LED          | 1       | Start/Status                        |
| 3S Li-Po Battery 11.1V     | 1       | Power                               |
| Buck Converter (5V/3A)     | 1       | Regulated logic power               |
| 3D Printed Mounts/Acrylic  | —       | Structural                          |
* * * * *

**🔌 Wiring & Interfaces**
--------------------------

### **Power**

-   **Battery → L298N (VMOT)**

-   **Battery → Powerbank (5V 3A) → Pi + Nano**

-   **Common GND** between Pi, Nano, sensors, and driver

### **Control**

-   **Nano PWM → Servo signal**

-   **Nano PWM → L298N ENA/ENB**

-   **Nano GPIO → Button, LED**

-   **I2C (SDA/SCL)** → MPU6050, VL53L0X (address pin if multiple TOFs)

### **Serial**

-   **Pi UART** ↔ **Nano UART** (9600 baud)

> Keep motor power lines twisted and away from camera ribbon to reduce noise.

* * * * *

**🖥️ Software & Algorithms**
-----------------------------

### **Pipeline Overview**

```
Frame Capture → Preprocess (resize, blur) → HSV Thresholds → Morphological Ops
→ Contours / Edges → Lane/Wall Mask → Centerline Error (cte)
→ Steering Controller → Pi→Nano Commands
→ Nano PWM → Motors + Servo → Feedback loop (TOF, Yaw)
```

### **1) Vision: HSV-based Detection**

**Key Steps:**

-   Convert BGR → HSV

-   Threshold for lane colors (or wall brightness / edges)

-   Morph open/close to reduce noise

-   Find largest contours or Hough lines

-   Compute **cte** (centerline error) and **heading hint**

**Pseudocode:**

```
frame = capture()
hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
mask_lane = inrange(hsv, lane_hsv_min, lane_hsv_max)
mask_clean = morph_open_close(mask_lane)
contours = find_contours(mask_clean)
center = estimate_centerline(contours)
cte = desired_center_x - center.x
```

* * * * *

### **2) Steering & Speed Control**

We use a simple **PI** controller for steering on cte and a **feed-forward speed** with safety clamping from TOF.

```
steer = kp_steer * cte + ki_steer * integrate(cte)
speed = base_speed * section_gain
if tof_mm < 120: speed = min(speed, slow_speed)
```

* * * * *

### **3) Lap FSM**

States: IDLE → RUNNING → LAP_DETECTED → LAPS_1/2/3 → STOP_READY → STOP_DONE

Transitions require **geometric + color + yaw** agreement to count a section crossing. False positives are suppressed with **debounce windows**.

* * * * *

### **4) Obstacle Bypass & Sign Compliance**

-   Detect **red/green pillar** with HSV thresholds.

-   Compute pillar centroid to determine **left/right** of track center.

-   If **red** → ensure path **Right**; **green** → path **Left**.

-   If geometry forces wrong side, **abort** and stop (rule-compliant).

* * * * *

### **5) Yaw Complementary Filter**

```
yaw = alpha*(yaw + gyro_z*dt) + (1-alpha)*accel_yaw
```

-   alpha ~ 0.98 for quick response

-   Used to detect **consistent 90° turns** and to stabilize steering when visual cues dip

* * * * *

### **6) Nano Firmware (Actuation)**

-   Parses ASCII commands from Pi

-   Applies PWM to motors, sets servo angle

-   Polls TOF and IMU; sends back telemetry

-   Implements **failsafe**: if comms lost for > T ms → stop

* * * * *

**🧪 Testing & Calibration**
----------------------------

### **HSV Calibration**

-   Collect frames under **indoor LED**, **daylight**, and **arena lighting**

-   Use a small GUI slider (in setup_and_tests/) to tune H/S/V ranges

-   Save presets per lighting profile; quick switch on event day

### **Steering Trim**

-   With robot lifted, command STRAIGHT and adjust servo neutral until wheels are visually centered

-   Record SERVO_CENTER in Nano firmware

### **Speed Tuning**

-   Gradually increase base_speed while watching for oscillation

-   Use TOF clamp near narrow passages

### **Yaw Stability**

-   Log IMU yaw while robot is still; verify drift < 2--3°/min

-   During turns, confirm 90° ± tolerance, adjust filter alpha

### **Lap Counter Validation**

-   Tape a small mock loop; verify 3 lap increments with no double counts

-   Randomize start sections to ensure **stop occurs in origin section**

### **Parking Tests**

-   After lap 3, verify stop position repeatability; adjust **deceleration ramp** and **servo neutral**

-   Target ≤ **2 cm** alignment error

* * * * *

## 📊 Performance Metrics

| **Metric**                   | **Typical Value**                  |
|-------------------------------|-------------------------------------|
| Vision FPS (640×480)          | 25–30 FPS on Pi 4                   |
| End-to-end control latency    | ~40–60 ms                           |
| Lap counting false positives  | < 1 per 20 runs after debounce      |
| Obstacle detection threshold  | 50–120 mm (configurable)            |
| Parking lateral error         | ≤ 20 mm (typical)                   |

* * * * *

**🏆 Scoring Analysis & Strategy**
----------------------------------

### **Open Challenge (Max 30 pts)**

-   **Sections:** 24 pts (3 pts per completed set of sections)

-   **Finish:** 3 pts (correct stop in original section after 3 laps)

-   **Laps:** 3 pts (three laps completed)

**Strategy**

-   Keep speed **consistent**, avoid penalties.

-   Prioritize **clean lap counting**; a missed section costs more than a small speed gain.

* * * * *

### **Obstacle Challenge (Max 62 pts)**

-   **Sections:** 24 pts

-   **Finish:** 3 pts

-   **Laps:** 3 pts

-   **Signs moved:** +10 pts if **none moved**, +8 pts if moved **after all 3 laps**, +4 pts if moved but **still did laps**, +2 pts if moved and **did not** do laps

-   **Start in parking lot:** +7 pts

-   **Parking quality:** up to **+15 pts** for perfect stop/alignment

**Strategy**

-   Mount TOF sensors to **avoid touching signs**.

-   If minimal contact is unavoidable, ensure it does **not** impair laps; salvage the **+4/+8** tiers rather than risking a DNF.

-   Always **start in the lot** and **tune parking** carefully; these are **high-value, deterministic** points.


* * * * *

**🧰 Troubleshooting Guide**
----------------------------

### **Vision looks noisy / misses lanes**

-   Re-load HSV profile for current lighting

-   Reduce exposure / increase blur slightly

-   Narrow ROI to the lower half of frame

### **Robot veers left/right on straights**

-   Re-trim servo neutral (SERVO_CENTER)

-   Check wheel alignment and tire friction mismatch

### **Sudden braking in open track**

-   TOF false trip: verify sensor alignment and shield from floor reflections

-   Raise obstacle threshold slightly

### **Lap counter double counts**

-   Increase debounce distance/time

-   Require yaw change + section signature for transitions

### **Serial drops**

-   Shield USB cable, separate from motor leads

-   Add heartbeat timeout and automatic STOP in Nano

* * * * *

**🧭 Event-Day Checklists**
---------------------------

### **Pre-Run**

-   Battery ≥ 90%

-   Servo centered (wheels straight)

-   Load correct HSV preset for venue lighting

-   Baud rate 115200 confirmed

-   Push-button & LED verified

-   Wheels tightened, no wobble

### **On the Track**

-   Camera view free of occlusion

-   Quick vision sanity check (mask preview)

-   TOF reads ~500--2000 mm in open space

### **Post-Run**

-   Save logs (if enabled)

-   Inspect signs/arena for contact

-   Battery health check

* * * * *

## 🧱 Risk & Mitigations

| **Risk**                  | **Impact**        | **Mitigation**                                |
|----------------------------|-------------------|-----------------------------------------------|
| Lighting change mid-run    | Missed lanes      | HSV presets + adaptive thresholds             |
| Serial interference        | Loss of control   | Shielded cable, timeout → STOP                |
| Servo drift                | Steering bias     | Pre-run trim, temperature warm-up             |
| Battery sag                | Brownouts         | Healthy pack, buck converters, capacitors     |
| Floor reflections (TOF)    | False stop        | Angle sensor, raise threshold, add shield     |
* * * * *

**📈 Future Work**
------------------

-   **SLAM-lite**: add low-overhead visual odometry for drift correction

-   **Better parking**: dual TOF triangulation + local search

-   **Closed-loop speed**: wheel encoders + PID for stable lap times

-   **Model-based control**: identify steering kinematics for predictive turns

-   **Auto-calibration**: quick HSV + servo trim wizard at startup

* * * * *

**📔 Extended Engineering Notes (Snapshot)**
--------------------------------------------

> Full weekly logs are in journal/ --- this is a short snapshot.

### **Week 1 --- Bring-up**

-   Pi OS + OpenCV install, Nano serial echo

-   Baseline frame capture @ 30 FPS

### **Week 2 --- HSV & Masks**

-   Built live sliders for HSV thresholds

-   Created mask visualizer overlays

### **Week 3 --- Motors & Servo**

-   L298N mapping + PWM ramp for smoothness

-   Servo center calibration utility

### **Week 4 --- IMU/TOF**

-   Complementary filter tuning

-   TOF integration + distance clamp on speed

### **Week 5 --- Lap FSM**

-   Section signatures + debounce logic

-   3-lap cycle with randomized start section

### **Week 6 --- Integration**

-   Pi decisions ↔ Nano actuation closed loop

-   Safety timeouts, failsafe STOP verified

### **Week 7 --- Obstacle Challenge**

-   Red/green logic with centroid-based side checks

-   Abort-on-wrong-side compliance

### **Week 8 --- Parking**

-   Deceleration profile + heading hold

-   ≤ 2 cm lateral alignment achieved repeatedly

* * * * *

**📸 Media**
------------

-   **Team Photos:** [assets/team_photos/](assets/team_photos)

-   **Robot Photos:** [assets/vehicle_photos/](assets/vehicle_photos)

-   **Demo Video:** [YouTube Link Here](https://youtu.be/C2lZSMPMFqI)

* * * * *

**👨‍💻 Contributors**
----------------------

Built by **Nitish Krishna, Ritvik Raghav / Neural Navigators**

For **WRO Future Engineers 2025**

* * * * *

**📜 License**
--------------

This project is open-source under the [MIT License](LICENSE).

* * * * *

**❓ FAQ & Glossary**
--------------------

### **FAQ**

**Q: Why not use only the Pi for PWM?**

A: Software PWM under Linux can jitter under CPU load. The Nano guarantees deterministic PWM for motors/servo.

**Q: How do you guarantee 3 laps and stop in the original section?**

A: Section signatures + yaw and position gating; lap increments only on a full cycle, and a stop is armed only when the robot is back in the original section.

**Q: What if a sign is nudged accidentally?**

A: Our TOF/vision combination is tuned to keep clearance. If contact happens, logic continues the laps to retain the best available "moved but laps completed" points tier.

**Q: Can this run on Pi 3?**

A: With reduced resolution and tuned thresholds, yes, but we recommend Pi 4 for stable 25--30 FPS.

* * * * *

### **Glossary**

-   **HSV:** Hue, Saturation, Value --- a color space robust to lighting changes.

-   **TOF:** Time-of-Flight --- distance sensing by light travel time.

-   **FSM:** Finite State Machine --- discrete states with defined transitions.

-   **CTE:** Cross-Track Error --- lateral offset from desired path center.

-   **Complementary Filter:** Sensor fusion that blends fast gyro with stable accelerometer.

* * * * *

