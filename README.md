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
- 🔌 Modular nano-based control for motors and sensors

---

## 🚦 Challenge Entry Points

> Each challenge mode has a separate `main.py` script.

- `main/main_open.py` – Start **Open Challenge** mode
- `main/main_obstacle.py` – Start **Obstacle Challenge** mode
- `main/main_auto.py` – (Optional) Unified logic: switches modes automatically

## 📁 Project Structure

```
Neural-Navigators/
│
├── LICENSE                   # Open-source license (MIT recommended)
├── README.md                 # Project overview, setup, architecture, and credits
├── requirements.txt          # Python dependencies for OpenCV and hardware
│
├── main/                     # Entry points to run different robot modes
│   ├── main_open.py          # Open Challenge logic (lane following, lap counting)
│   ├── main_obstacle.py      # Obstacle Challenge logic (object detection + path switching)
│   └── main_auto.py          # Unified mode: auto-switches between open and obstacle
│
├── vision/                   # Computer vision logic (OpenCV)
│   ├── lane_detection.py     # Detects lanes, center alignment, curves
│   └── obstacle_detection.py # Color filtering, shape recognition for obstacle detection
│
├── logic/                    # Core logic and state management
│   ├── lap_counter.py        # Tracks laps completed
│   └── stop_handler.py       # Logic to stop robot in original start section
│
├── hardware/                 # Interfaces for sensors and actuators
│   ├── bma250_gyro.py        # Reads rotation data from BMA250 gyro for lap/orientation
│   ├── button.py             # Detects push button press to start the robot
│   └── led_ready.py          # Controls LED indicator for system readiness
│
├── nano/                      # Microcontroller (nano) code in Arduino/C++
│   ├── main.ino              # Controls motors and servo based on commands
│   └── color_sensor.ino      # Reads data from color sensor (e.g., for obstacle shape)
│
├── test/                     # Calibration and debugging scripts
│   └── test_lane_detect.py   # Standalone script to test lane detection separately
│
├── assets/                   # Visuals for documentation/demo
│   ├── robot.jpg             # Picture of your robot
│   └── field_demo.gif        # GIF or video showing the robot in action
│
└── docs/                     # Optional diagrams, notes, or rule highlights
    └── system_flow.png       # System block diagram or flowchart of logic
```

## 🔧 Setup Instructions

1. **Install dependencies** (on Raspberry Pi):

    ```bash
    pip install -r requirements.txt
    ```

2. **Connect hardware**
   - Attach PiCam, push button, and LED
   - Connect nano to motor, servo, and color sensor
   - Optional: use I2C or UART between Pi ↔ nano

3. **Start the robot**

    ```bash
    python3 main/main_open.py
    ```

4. **Press the push button** when the LED turns green to begin.

🔁 Mode Switching

Your robot can run both challenges from the same repo.

	•	Open Challenge Mode: lane detection + lap tracking
	•	Obstacle Challenge Mode: object detection + path change
Modes are separated to keep things modular and readable.

## 🧠 Architecture

| Component      | Role                          |
|----------------|-------------------------------|
| Raspberry Pi   | Brain (vision + control)      |
| PiCam          | Eyes (OpenCV processing)      |
| nano (external) | Muscles (motor/sensor control)|
| Gyro (BMA250)  | Orientation + lap detection   |
| LEDs + Button  | Status & control interface    |

👨‍💻 Contributors

Built by [ Nitish Krishna, Ritvik Raghav / Neural Navigators]
For WRO Future Engineers 2025

📜 License

This project is open-source under the MIT License.



