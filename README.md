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
- 🔌 Modular Nano-based control for motors and sensors  

---

## 🚦 Challenge Entry Points

> Each challenge mode has a separate `main.py` script.

- `main/main_open.py` – Start **Open Challenge** mode  
- `main/obstacle_challenge.py` – Start **Obstacle Challenge** mode  

---

## 📁 Project Structure

- [`LICENSE`](LICENSE) – Open-source license  
- [`README.md`](README.md) – Project overview, setup, architecture, and credits  

### 🔹 Main (Challenge entry points)
- [`main/main_open.py`](main/main_open.py) – Open Challenge logic  
- [`main/obstacle_challenge.py`](main/obstacle_challenge.py) – Obstacle Challenge logic  

### 🔹 Vision (Computer vision logic)
- [`vision/obstacle_detection.py`](vision/obstacle_detection.py) – Obstacle recognition & filtering  

### 🔹 Logic (Core state management)
- [`logic/lap_counter.py`](logic/lap_counter.py) – Tracks laps completed  
- [`logic/stop_handler.ino`](logic/stop_handler.ino) – Arduino stop control logic  

### 🔹 Nano (Microcontroller code)
- [`nano/main.ino`](nano/main.ino) – Controls motors/servo + serial communication  

### 🔹 Setup and Tests (Calibration scripts)
- [`setup_and_tests/callibration.py`](setup_and_tests/callibration.py)  
- [`setup_and_tests/callibration_for_line_distance.py`](setup_and_tests/callibration_for_line_distance.py)  

### 🔹 Utils
- [`utils/Requirements.md`](utils/Requirements.md) – Dependencies and setup notes  

### 🔹 Assets (Photos & visuals)
- [`assets/team_photos/`](assets/team_photos) – Team photos  
- [`assets/vehicle_photos/`](assets/vehicle_photos) – Robot photos (all sides + top/bottom)  

### 🔹 Journal
- [`journal/`](journal) – Engineering journal notes & progress  

## 🔧 Setup Instructions

1. **Install dependencies** (on Raspberry Pi):

    ```bash
    pip install -r utils/Requirements.md
    ```

2. **Connect hardware**
   - Attach PiCam, push button, and LED  
   - Connect Nano to motors, servo, and sensors  
   - Communication: Serial (UART) between Pi ↔ Nano  

3. **Run challenge mode**

    ```bash
    python3 main/main_open.py
    ```
    or
    ```bash
    python3 main/obstacle_challenge.py
    ```

4. **Press the push button** when the LED turns green to begin.

---

## 🧠 Architecture

| Component        | Role                          |
|------------------|-------------------------------|
| Raspberry Pi 4   | Brain (vision + control)      |
| PiCam 4          | Eyes (OpenCV processing)      |
| Arduino Nano     | Muscles (motor + sensor I/O)  |
| MPU6050 		   | Orientation + yaw correction  |
| VL53L0X TOF      | Obstacle avoidance            |
| LEDs + Button    | Status & control interface    |

---

## 📸 Media

- **Team Photos:** [`assets/team_photos/`](assets/team_photos)  
- **Robot Photos:** [`assets/vehicle_photos/`](assets/vehicle_photos)  
- **Demo Video:** [YouTube Link Here](https://youtu.be/C2lZSMPMFqI)  

---

## 👨‍💻 Contributors

Built by **Nitish Krishna, Ritvik Raghav / Neural Navigators**  
For **WRO Future Engineers 2025**

---

## 📜 License

This project is open-source under the MIT License.

