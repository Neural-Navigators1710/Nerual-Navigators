import time
import sys
import os
import serial
import serial.tools.list_ports
import cv2

# === Import detection functions ===
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from vision.obstacle_detection import detect_obstacle_color_direction, get_last_frame

# ===== Serial Connection to Arduino Nano =====
def find_nano_port():
    """Scan available serial ports and return the Nano device path."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if "Arduino" in port.description or "USB Serial" in port.description:
            return port.device
    return None

def connect_to_nano():
    """Keep retrying until Arduino Nano is connected."""
    while True:
        port = find_nano_port()
        if port:
            try:
                ser = serial.Serial(port, 9600, timeout=1)
                print(f"✅ Connected to Arduino Nano on {port}")
                return ser
            except serial.SerialException:
                print("⚠️ Found port but couldn't open it, retrying...")
        else:
            print("⌛ Waiting for Arduino Nano to connect...")
        time.sleep(1)

# ===== Main Loop =====
if __name__ == "__main__":
    ser = connect_to_nano()
    last_action = None

    try:
        while True:
            # === Obstacle Detection
            direction = detect_obstacle_color_direction(show_debug=False)
            frame = get_last_frame()

            # === Display camera output
            if frame is not None:
                cv2.imshow("Live Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # === Act on detection from vision
            if direction and direction != last_action:
                print(f"[🖼️] Block Detected → {direction}")
                if direction == "R":
                    print("💪 Action: Turn RIGHT")
                    if ser:
                        ser.write(b"R\n")
                elif direction == "L":
                    print("👈 Action: Turn LEFT")
                    if ser:
                        ser.write(b"L\n")
                last_action = direction

            # === Check for serial commands from Arduino Nano
            if ser.is_open:
                try:
                    raw_line = ser.readline()
                    line = raw_line.decode(errors="ignore").strip()
                    if line:
                        print(f"[📡] Received from Nano: {line}")
                        # (Servo handling removed, only logging remains)
                except serial.SerialException:
                    print("⚠️ Lost connection to Arduino Nano! Reconnecting...")
                    try:
                        ser.close()
                    except:
                        pass
                    time.sleep(0.5)
                    ser = connect_to_nano()
            else:
                print("🔄 Reconnecting to Arduino Nano...")
                time.sleep(0.5)
                ser = connect_to_nano()

            time.sleep(0.05)  # Small delay for loop stability

    except KeyboardInterrupt:
        print("\n❌ Stopped by user. Cleaning up...")

    finally:
        if ser:
            ser.close()
        cv2.destroyAllWindows()
        print("✅ System safely shut down.")
