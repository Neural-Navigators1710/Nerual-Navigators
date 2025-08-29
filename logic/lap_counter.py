import time
import serial

# ===== Serial connection placeholder =====
ser = None  # Replace with your actual connected Serial object

# ===== Lap counting setup =====
last_command_time = 0
cooldown = 5   # seconds between signals
count = 0      # counter for left/right turns
MAX_TURNS = 12

def handle_turn(detected_color, ser):
    """Handles counting and sending turn signals to Arduino Nano"""
    global last_command_time, count

    if time.time() - last_command_time > cooldown:
        if ser and ser.is_open:
            if detected_color == "ORANGE":
                ser.write(b"RIGHT\n")
                print("➡️ Sent: RIGHT")
                count += 1
            elif detected_color == "BLUE":
                ser.write(b"LEFT\n")
                print("⬅️ Sent: LEFT")
                count += 1

            last_command_time = time.time()

            # Stop condition
            if count >= MAX_TURNS:
                print("🏁 12 turns reached → Sending STOP to Nano")
                ser.write(b"STOP\n")
                return True  # signal that lap counting finished
    return False
