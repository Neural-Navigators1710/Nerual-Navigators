#Use this Code for Open Challenge

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
from picamera2 import Picamera2

# ==============================================================
# 🔌 Serial Connection: Auto-detect Arduino Nano port
# ==============================================================

def find_nano_port():
    """Scan connected serial ports and return the Nano device path."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "USB Serial" in port.description:
            return port.device
    return None

def connect_to_nano():
    """
    Keep searching until Arduino Nano is found.
    Once connected, send 'READY' signal so Nano knows Pi is online.
    """
    while True:
        port = find_nano_port()
        if port:
            try:
                ser = serial.Serial(port, 9600, timeout=1)
                print(f"✅ Connected to Arduino Nano on {port}")
                time.sleep(2)             # Allow Nano to reboot after serial init
                ser.write(b"READY\n")      # Handshake → Nano sets PiReady = true
                return ser
            except serial.SerialException:
                print("⚠️ Found port but couldn't open it, retrying...")
        else:
            print("⌛ Waiting for Arduino Nano to connect...")
        time.sleep(1)

# ==============================================================
# 📷 Camera Init (Raspberry Pi Camera v2.1 / HQ Camera)
# ==============================================================

picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "XRGB8888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()
picam2.set_controls({"AfMode": 1})  # Enable auto-focus

# ==============================================================
# 🎨 HSV Color Ranges for Detection
# ==============================================================

# Tuned thresholds (in HSV) for lane/marker detection
orange_lower = np.array([0, 149, 75])
orange_upper = np.array([17, 255, 158])

blue_lower   = np.array([83, 95, 10])
blue_upper   = np.array([130, 255, 255])

# ==============================================================
# 🚦 State Variables
# ==============================================================

ser = connect_to_nano()   # Serial connection
last_command_time = 0     # Cooldown timer (avoid spamming Nano)
cooldown = 5              # Seconds between allowed commands
count = 0                 # Count left/right turns (used to stop race)

# Approximate contour area thresholds representing "object within 10cm"
MIN_AREA_10CM = 20000
MAX_AREA_10CM = 65000

# ==============================================================
# 🏎️ Main Loop
# ==============================================================

try:
    while True:
        # 1. Capture frame and convert to HSV
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 2. Create binary masks for orange & blue colors
        mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
        mask_blue   = cv2.inRange(hsv, blue_lower, blue_upper)

        detected_color = None
        bbox = None
        area = 0

        # ----------------------------------------------------------
        # 🔶 Detect ORANGE (usually means "turn RIGHT")
        # ----------------------------------------------------------
        contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_orange:
            largest_orange = max(contours_orange, key=cv2.contourArea)
            area = cv2.contourArea(largest_orange)
            if MIN_AREA_10CM < area < MAX_AREA_10CM:
                x, y, w, h = cv2.boundingRect(largest_orange)
                bbox = (x, y, w, h)
                detected_color = "ORANGE"

        # ----------------------------------------------------------
        # 🔷 Detect BLUE (usually means "turn LEFT")
        # ----------------------------------------------------------
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            largest_blue = max(contours_blue, key=cv2.contourArea)
            area = cv2.contourArea(largest_blue)
            if MIN_AREA_10CM < area < MAX_AREA_10CM:
                x, y, w, h = cv2.boundingRect(largest_blue)
                bbox = (x, y, w, h)
                detected_color = "BLUE"

        # ----------------------------------------------------------
        # 🖼️ Visualization: Draw bounding box if a color detected
        # ----------------------------------------------------------
        if bbox:
            x, y, w, h = bbox
            color = (0, 165, 255) if detected_color == "ORANGE" else (255, 0, 0)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, f"{detected_color} ({int(area)})",
                        (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # ----------------------------------------------------------
        # 📡 Command Handling → Send LEFT / RIGHT to Nano
        # ----------------------------------------------------------
        if detected_color and (time.time() - last_command_time > cooldown):
            try:
                if ser and ser.is_open:
                    if detected_color == "ORANGE":
                        ser.write(b"RIGHT\n")
                        print("➡️ Sent: RIGHT (within 10cm)")
                        count += 1
                    elif detected_color == "BLUE":
                        ser.write(b"LEFT\n")
                        print("⬅️ Sent: LEFT (within 10cm)")
                        count += 1

                    last_command_time = time.time()

                    # 🏁 After 12 valid signals, stop race
                    if count >= 12:
                        print("🏁 12 turns reached → Sending STOP to Nano")
                        ser.write(b"STOP\n")
                        break

            # Reconnect logic if Nano disconnects mid-race
            except serial.SerialException:
                print("⚠️ Lost connection to Arduino Nano! Reconnecting...")
                try:
                    ser.close()
                except:
                    pass
                time.sleep(0.5)
                ser = connect_to_nano()

        # ----------------------------------------------------------
        # 👀 Display live camera feed with overlays
        # ----------------------------------------------------------
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

# ==============================================================
# 🔚 Cleanup: Ensure safe shutdown
# ==============================================================

finally:
    picam2.stop()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
    print("✅ System safely shut down.")
