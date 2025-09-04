import time
import sys
import os
import serial
import serial.tools.list_ports
import cv2

# === Import detection functions ===
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from vision.obstacle_detection import detect_obstacle_color_direction, get_last_frame


# ==========================================================
# === Serial Communication Helpers
# ==========================================================
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


# ==========================================================
# === Main Program
# ==========================================================
def main():
    ser = connect_to_nano()
    last_action = None

    try:
        while True:
            # --- Run Obstacle Detection
            direction = detect_obstacle_color_direction(show_debug=False)
            frame = get_last_frame()

            # --- Display Camera Output
            if frame is not None:
                cv2.imshow("Live Detection", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

            # --- Send Command to Arduino if New Action Detected
            if direction and direction != last_action:
                print(f"[🖼️] Block Detected → {direction}")

                if ser and ser.is_open:
                    if direction == "R":
                        print("💪 Action: Turn RIGHT")
                        ser.write(b"R\n")
                    elif direction == "L":
                        print("👈 Action: Turn LEFT")
                        ser.write(b"L\n")
                    elif direction == "S":
                        print("⬆️ Action: Go STRAIGHT")
                        ser.write(b"S\n")

                last_action = direction

            # --- Read Messages from Arduino
            if ser and ser.is_open:
                try:
                    raw_line = ser.readline()
                    line = raw_line.decode(errors="ignore").strip()
                    if line:
                        print(f"[📡] Received from Nano: {line}")
                        # (Servo control removed, only logging)
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

            time.sleep(0.05)  # Loop stability

    except KeyboardInterrupt:
        print("\n❌ Stopped by user. Cleaning up...")

    finally:
        if ser:
            try:
                ser.close()
            except:
                pass
        cv2.destroyAllWindows()
        print("✅ System safely shut down.")


# ==========================================================
# === Entry Point
# ==========================================================
if __name__ == "__main__":
    main()
