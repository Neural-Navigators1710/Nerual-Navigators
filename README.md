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

## 🚦 Challenge Entry Points

> Each challenge mode has a separate `main.py` script.

- `main/main_open.py` – Start **Open Challenge** mode
- `main/main_obstacle.py` – Start **Obstacle Challenge** mode
- `main/main_auto.py` – (Optional) Unified logic: switches modes automatically

File Structures:

wro-future-2025/
│
├── README.md                # Project overview (this file)
├── LICENSE                  # Open source license
├── requirements.txt         # Python dependencies
│
├── main/                    # Entry points for both challenges
│   ├── main_open.py
│   ├── main_obstacle.py
│   └── main_auto.py
│
├── vision/                  # OpenCV modules
│   ├── lane_detection.py
│   └── obstacle_detection.py
│
├── logic/                   # Robot's decision-making brain
│   ├── lap_counter.py
│   └── stop_handler.py
│
├── hardware/                # Gyro, buttons, LEDs
│   ├── bma250_gyro.py
│   ├── button.py
│   └── led_ready.py
│
├── esp/                     # ESP code for motors and sensors
│   ├── main.ino
│   └── color_sensor.ino
│
├── test/                    # Test and calibration scripts
│   └── test_lane_detect.py
│
├── assets/                  # Robot pics, screenshots, demo GIFs
│   ├── robot.jpg
│   └── field_demo.gif
│
└── docs/                    # (Optional) diagrams, flowcharts, rule summaries
    └── system_flow.png

## 🔧 Setup Instructions

1. **Install dependencies** (on Raspberry Pi):

    ```bash
    pip install -r requirements.txt
    ```

2. **Connect hardware**
   - Attach PiCam, push button, and LED
   - Connect ESP to motor, servo, and color sensor
   - Optional: use I2C or UART between Pi ↔ ESP

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
| ESP (external) | Muscles (motor/sensor control)|
| Gyro (BMA250)  | Orientation + lap detection   |
| LEDs + Button  | Status & control interface    |

👨‍💻 Contributors

Built by [ Nitish Krishna, Ritvik Raghav / Neural Navigators]
For WRO Future Engineers 2025

📜 License

This project is open-source under the MIT License.



