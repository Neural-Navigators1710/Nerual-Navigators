# 📘 WRO Robot – Hardware & Software Requirements

This document lists **all the hardware parts** and **software dependencies** required for running the robot project with Raspberry Pi + Arduino Nano.

---

## 🛠️ Hardware Components
- **Raspberry Pi 4** (with RPi Camera Module v3)
- **Arduino Nano**
- **MPU6050** (Gyroscope & Accelerometer)
- **VL53L0X** (Time-of-Flight Distance Sensors ×2 or 3)
- **Servo Motor** DSS ERVO (High Torque)(PWM controlled)
- **L298N Motor Driver**
- **DC Motors**
- **12V LiPo Battery** (for motors & servo power)
- **5V Power Bank** (for Raspberry Pi)
- **Push Button** (for motor start/stop)
- **DPST Switches** (for circuit on/off)
- **Chasis for Robot** Link - [Chasis Link](https://probots.co.in/4w-robot-kit-steering-plastic.html)


---

## 💻 Software Requirements

### Raspberry Pi (Python Environment)
Install required packages:

```bash
sudo apt update && sudo apt upgrade -y

# Install pip if not installed
sudo apt install python3-pip -y

# Install OpenCV, Numpy, Serial, Picamera2, and other dependencies
pip3 install opencv-python numpy pyserial

# Install PiCamera2
sudo apt install -y python3-picamera2

# Install pigpio for PWM servo control
sudo apt install -y pigpio python3-pigpio

# Extra useful libraries
pip3 install sys os

# 🐷 pigpio Daemon Commands

### ✅ Start pigpio daemon
# Run this when you want to begin using `pigpio` for GPIO/servo control.  
sudo pigpiod

#Use this to verify that the daemon is active. If you see pigpiod in the list, it’s running.
ps aux | grep pigpiod

#This ensures pigpiod runs automatically whenever the Pi boots up.
sudo systemctl enable pigpiod
sudo systemctl start pigpiod

Run this to stop the daemon when you no longer need it, or before restarting it.
sudo killall pigpiod 
```



## Arduino Nano (Arduino IDE Libraries)

Install these libraries in Arduino IDE → Library Manager or manually add them:

- `Wire.h` (built-in)  
- `Servo.h` (built-in)  
- `I2Cdev.h` and `MPU6050_6Axis_MotionApps20.h` - https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050 
- `VL53L0X.h` - https://github.com/pololu/vl53l0x-arduino/blob/master/VL53L0X.h 

---

# 📚 Installing Arduino Libraries (I2Cdev, MPU6050 DMP, VL53L0X)

This guide shows two ways to install:
- **Library Manager / Add .ZIP** (inside Arduino IDE)
- **Manual copy** into your `libraries/` folder

Works on Windows, macOS, and Linux.

---

## ✅ Folder locations (for manual install)

- **Windows:** `Documents\Arduino\libraries`
- **macOS:** `~/Documents/Arduino/libraries`
- **Linux:** `~/Arduino/libraries`

> Create the `libraries` folder if it doesn’t exist.

---

## 1) I2Cdev.h & MPU6050_6Axis_MotionApps20.h (Jeff Rowberg’s I2Cdevlib)

These two come from the same repo. You need **both**:
- `I2Cdev` library (gives `I2Cdev.h`)
- `MPU6050` library (includes `MPU6050_6Axis_MotionApps20.h`)

### Method A — Add .ZIP in Arduino IDE (recommended)
1. Download the repo as ZIP:
   - Go to: `https://github.com/jrowberg/i2cdevlib`
   - Click **Code → Download ZIP**.
2. Extract the ZIP somewhere.
3. In the extracted folder, navigate to:
   - `Arduino/I2Cdev/`
   - `Arduino/MPU6050/`
4. **Zip each of those two folders** individually (so you have `I2Cdev.zip` and `MPU6050.zip`).
5. In Arduino IDE: **Sketch → Include Library → Add .ZIP Library…**
   - Add `I2Cdev.zip`
   - Add `MPU6050.zip`
6. **Restart** Arduino IDE.

### Method B — Manual copy
1. Download/extract the repo.
2. Copy these two folders into your `libraries/` directory:
   - `i2cdevlib/Arduino/I2Cdev`  →  `libraries/I2Cdev`
   - `i2cdevlib/Arduino/MPU6050` →  `libraries/MPU6050`
3. **Restart** Arduino IDE.

### Verify install
- In Arduino IDE: **File → Examples → MPU6050 →** (e.g., `MPU6050_DMP6`).
- Or compile this minimal include test:
  ```cpp
  #include <Wire.h>
  #include <I2Cdev.h>
  #include <MPU6050_6Axis_MotionApps20.h>
  void setup() {}
  void loop() {}

## 2) VL53L0X.h (Time-of-Flight distance sensor)

Use the **Pololu** library if your code includes `#include <VL53L0X.h>`.

---

### 📥 Method A — Library Manager (easiest)
1. Open Arduino IDE → **Sketch → Include Library → Manage Libraries…**
2. Search **“VL53L0X”**.
3. Install **“Pololu VL53L0X”**.
4. *(Optional)* If you previously installed other VL53L0X libs, remove them to avoid conflicts.

---

### 📥 Method B — Add .ZIP from GitHub
1. Download ZIP from: [Pololu VL53L0X Arduino](https://github.com/pololu/vl53l0x-arduino)  
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library…**  
3. Select the downloaded ZIP.  
4. **Restart Arduino IDE**.  

---

### ✅ Verify install
- Arduino IDE → **File → Examples → VL53L0X** (Pololu’s examples), open and compile one.  
- Or compile this minimal include test:  

```cpp
#include <Wire.h>
#include <VL53L0X.h>

void setup() {}
void loop() {}

# 🛠️ Built-in libraries (no install needed)
- `Wire.h` (I²C)  
- `Servo.h`  

```
---

# 🧰 Common install pitfalls & fixes
- **“No such file or directory”**: The library folder is nested too deep (e.g., `libraries/MPU6050-1.0.0/MPU6050/...`).  
  → Move the inner folder up so the path is `libraries/MPU6050/(.h/.cpp files here)`.  

- **Duplicate libraries**: Remove older/duplicate copies from `libraries/` to avoid conflicts.  

- **Didn’t restart IDE**: Always restart Arduino IDE after adding/removing libraries.  

---

# 🚀 Quick final compile test (all together)

```cpp
#include <Wire.h>
#include <Servo.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <VL53L0X.h>

void setup() {
  Wire.begin();
}

void loop() {}
```



## ⚡ Notes

- Always connect **common GND** between Raspberry Pi, Arduino Nano, servo, and motor driver.  
- Servo can be powered from the **5V battery line** as long as grounds are shared.  
- Run Raspberry Pi scripts with **`python3`** instead of `python`.  






