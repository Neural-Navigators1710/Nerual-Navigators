# 🛠️ Engineering Log – 2025-07-18 – Initial Setup & Project Planning

## 🧠 Goal
Kick off the development of the self-driving robot for WRO Future Engineers 2025. Set up the GitHub repo, choose components, and plan folder structure.

---

## ⚙️ What I Did Today
- Read the 2025 WRO Future Engineers rulebook carefully
- Understood changes like track layout randomization and stop-in-section rule
- Finalized that I’ll use only:
  - Raspberry Pi 4
  - PiCam v4 for vision
  - BMA250 gyro
  - ESP32 for basic motor and sensor control (no LIDAR or ultrasonic)
- Created a public GitHub repo named `Neural-Navigators`
- Wrote a detailed and attractive `README.md` with:
  - Architecture
  - Project structure
  - Features and setup instructions
- Created basic folders: `main/`, `vision/`, `esp/`, `hardware/`, `logic/`, `assets/`, `test/`, and `docs/`

---

## 🧪 Observations
- GitHub markdown looks best when code blocks and project trees are inside triple backticks
- README structure is super important — it’s the first thing judges see
- Choosing the right file structure early makes future updates easier

---


## 💡 Learnings
- Engineering documentation is as important as the robot itself in WRO
- Judges look for polish, planning, and professionalism in online evaluation
- Markdown formatting can make or break the presentation

---

## 🔄 Next Steps
- Start working on `lane_detection.py` using OpenCV
- Test HSV color filtering using sliders
- Write a second log for lane detection experiments
