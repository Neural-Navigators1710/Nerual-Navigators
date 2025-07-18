# 🤖 WRO Future Engineers 2025 – Self-Driving Robot

An intelligent, self-driving robot designed for the **WRO Future Engineers 2025** category. Built to think, adapt, and race through open and obstacle challenges — using real-time vision, autonomous decision-making, and multi-mode switching.

> 💡 Built with precision. Driven by code.

---

## 🚗 Features

- 🎯 Lane and wall detection using computer vision
- 🧭 Lap counting and zone recognition
- ⛔ Obstacle detection and bypass logic
- 🚦 Push-button start and ready LED indication
- 🛑 Accurate stop in original section after 3 laps
- 🔁 Seamless mode switching (Open ↔ Obstacle Challenge)
- 🔌 Modular ESP-based control for motors and sensors

---

## 📁 Project Structure
## 📁 Project Structure

- `main.py` – Main script to run the robot
- `requirements.txt` – Python dependencies
- `README.md` – This file

**Folders:**

- `vision/` – OpenCV modules
  - `lane_detection.py`
  - `obstacle_detection.py`
- `logic/` – Robot's decision-making brain
  - `lap_counter.py`
  - `stop_handler.py`
- `hardware/` – Gyro, buttons, LEDs
  - `bma250_gyro.py`
  - `button.py`
  - `led_ready.py`
- `esp/` – ESP code for motors and sensors
  - `main.ino`
  - `color_sensor.ino`
- `test/` – Test and calibration scripts
  - `test_lane_detect.py`
- `assets/` – Robot pics, screenshots, demo GIFs
  - `robot.jpg`
  - `field_demo.gif`
- `docs/` – (Optional) Diagrams, flowcharts, rule summaries
  - `system_flow.png`

## 🔧 Setup Instructions

1. **Install dependencies** (on Raspberry Pi):
    ```bash
    pip install -r requirements.txt
    ```

2. **Connect hardware**
   - Attach PiCam, push button, and LED
   - Connect ESP to motor, servo, and color sensor
   - Optional: use I2C or UART between Pi ↔ ESP

3. **Start the robot:**
    ```bash
    python3 main.py
    ```

4. **Press the push button** when the LED turns green to begin.

---

## 🔁 Mode Switching

The robot has two files for two challenges: 
- **Open Challenge Mode**: lane following + lap tracking
- **Obstacle Challenge Mode**: activates object detection, path diversion

Use internal logic (lap counter or button press) to toggle mode at runtime.

---

## 🧠 Architecture

| Component     | Role                        |
|---------------|-----------------------------|
| Raspberry Pi  | Brain (vision + control)    |
| PiCam         | Eyes (OpenCV processing)    |
| ESP (external) | Muscles (motor/sensor control) |
| Gyro (BMA250) | Orientation + lap detection |
| LEDs + Button | Status & control interface  |

---

## 📸 Demo Media

> Add your own robot demo files inside `/assets` and embed like this:

![Robot Demo](assets/field_demo.gif)

---

## ✨ Status

🧪 **In Progress** — currently testing lane stability, corner handling, and obstacle avoidance logic.

---

## 👨‍💻 Contributors

Built by [Your Name / Team Name]  
Mentored by [Your mentor or coach]  
For **WRO Future Engineers 2025**

---

## 📜 License

This project is open-source under the [MIT License](LICENSE).
