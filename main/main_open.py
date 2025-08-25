#The code for open challenge

import cv2
import numpy as np
import serial
import serial.tools.list_ports
import time
from picamera2 import Picamera2

# =========================================================
# ========== SERIAL COMMUNICATION WITH ARDUINO ============
# =========================================================

# Automatically search for Arduino Nano in available ports
def find_nano_port():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        # Match based on common identifiers for Arduino Nano
        if "Arduino" in port.description or "USB Serial" in port.description:
            return port.device
    return None

# Keep trying to connect to Nano until successful
def connect_to_nano():
    while True:
        port = find_nano_port()
        if port:
            try:
                ser = serial.Serial(port, 9600, timeout=1)  # 9600 baud rate
                print(f"✅ Connected to Arduino Nano on {port}")
                return ser
            except serial.SerialException:
                print("⚠️ Found port but couldn't open it, retrying...")
        else:
            print("⌛ Waiting for Arduino Nano to connect...")
        time.sleep(1)

# =========================================================
# ========== CAMERA INITIALIZATION ========================
# =========================================================

picam2 = Picamera2()
# Configure preview resolution & format
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

# Autofocus mode enabled (1 = continuous)
picam2.set_controls({"AfMode": 1})

# =========================================================
# ========== COLOR RANGES IN HSV ==========================
# =========================================================

# Orange block HSV range (tuned experimentally)
orange_lower = np.array([0, 149, 75])
orange_upper = np.array([17, 255, 158])

# Blue block HSV range (tuned experimentally)
blue_lower = np.array([83, 95, 10])
blue_upper = np.array([130, 255, 255])

# =========================================================
# ========== SERIAL & DETECTION SETTINGS ==================
# =========================================================

ser = connect_to_nano()  # Connect to Arduino Nano
last_command_time = 0    # Track last command sent (for cooldown)
cooldown = 5             # Seconds between commands (avoid spamming)

# Contour area thresholds for detecting blocks within ~10 cm
# These values MUST be tuned with actual test measurements
MIN_AREA_10CM = 20000    # Lower bound
MAX_AREA_10CM = 65000    # Upper bound

# =========================================================
# ========== MAIN LOOP ====================================
# =========================================================

try:
    while True:
        # --- Capture frame from PiCamera ---
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create color masks
        mask_orange = cv2.inRange(hsv, orange_lower, orange_upper)
        mask_blue = cv2.inRange(hsv, blue_lower, blue_upper)

        detected_color = None   # Track detected color
        bbox = None             # Bounding box (x,y,w,h)
        area = 0                # Contour area

        # ------------------ ORANGE DETECTION ------------------
        contours_orange, _ = cv2.findContours(mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_orange:
            largest_orange = max(contours_orange, key=cv2.contourArea)
            area = cv2.contourArea(largest_orange)
            if MIN_AREA_10CM < area < MAX_AREA_10CM:
                # Only accept if the detected orange block is within area threshold
                x, y, w, h = cv2.boundingRect(largest_orange)
                bbox = (x, y, w, h)
                detected_color = "ORANGE"

        # ------------------ BLUE DETECTION -------------------
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours_blue:
            largest_blue = max(contours_blue, key=cv2.contourArea)
            area = cv2.contourArea(largest_blue)
            if MIN_AREA_10CM < area < MAX_AREA_10CM:
                # Only accept if the detected blue block is within area threshold
                x, y, w, h = cv2.boundingRect(largest_blue)
                bbox = (x, y, w, h)
                detected_color = "BLUE"

        # ------------------ VISUALIZATION --------------------
        if bbox:
            x, y, w, h = bbox
            # Choose color for drawing based on detection
            color = (0, 165, 255) if detected_color == "ORANGE" else (255, 0, 0)
            # Draw rectangle
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            # Display label with area value
            cv2.putText(frame, f"{detected_color} ({int(area)})", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # ------------------ SERIAL COMMAND -------------------
        if detected_color and (time.time() - last_command_time > cooldown):
            try:
                if ser and ser.is_open:
                    # Send RIGHT if Orange detected
                    if detected_color == "ORANGE":
                        ser.write(b"RIGHT\n")
                        print("➡️ Sent: RIGHT (within 10cm)")
                    # Send LEFT if Blue detected
                    elif detected_color == "BLUE":
                        ser.write(b"LEFT\n")
                        print("⬅️ Sent: LEFT (within 10cm)")
                    last_command_time = time.time()
            except serial.SerialException:
                # If connection lost, attempt reconnect
                print("⚠️ Lost connection to Arduino Nano! Reconnecting...")
                try:
                    ser.close()
                except:
                    pass
                time.sleep(0.5)
                ser = connect_to_nano()

        # ------------------ DISPLAY --------------------------
        cv2.imshow("Frame", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):  # Press 'q' to quit
            break

# =========================================================
# ========== CLEANUP ======================================
# =========================================================
finally:
    picam2.stop()
    cv2.destroyAllWindows()
    if ser:
        ser.close()
    print("✅ System safely shut down.")
