#This code will be our vision for obstacle detection challenge

import cv2
import numpy as np
from picamera2 import Picamera2
import time

# === CONFIGURATION CONSTANTS ===
KNOWN_WIDTH_CM = 5.0         # The real-world width of the block (in cm) - used for distance calculation
FOCAL_LENGTH = 615           # Camera focal length (calibrated beforehand with a known distance)
MAX_DISTANCE_CM = 25         # Ignore objects detected beyond this distance (background filtering)

# === HSV COLOR RANGES ===
# These define the color thresholds for detection in HSV color space
HSV_RED_LOWER = np.array([115, 169, 0])     # Lower bound for red
HSV_RED_UPPER = np.array([179, 255, 195])   # Upper bound for red

HSV_ORANGE_LOWER = np.array([22, 255, 0])   # Lower bound for orange
HSV_ORANGE_UPPER = np.array([179, 255, 203])# Upper bound for orange

# === CAMERA INITIALIZATION ===
picam2 = Picamera2()
# Use low resolution for faster processing (good enough for block detection)
picam2.preview_configuration.main.size = (320, 240)  
picam2.preview_configuration.main.format = "RGB888"  # RGB format output
picam2.configure("preview")

# Auto White Balance mode set to AUTO (0) so colors appear natural
picam2.set_controls({"AwbMode": 0})
picam2.start()
time.sleep(2)   # Give camera time to adjust before capturing frames

# === SHARED GLOBAL VARIABLE ===
last_output_frame = None  # Stores the most recent frame with drawn detections (for GUI/Debug)

# --------------------------------------------------------
# Distance Calculation using pinhole camera model
# Formula: Distance = (Real Width * Focal Length) / Perceived Width (in pixels)
# --------------------------------------------------------
def calculate_distance(perceived_width):
    if perceived_width == 0:   # Avoid division by zero
        return 0
    return (KNOWN_WIDTH_CM * FOCAL_LENGTH) / perceived_width

# --------------------------------------------------------
# Detect a block of a specific color within a frame
# Arguments:
#   hsv           : HSV converted frame
#   rgb_frame     : Original RGB frame
#   lower, upper  : HSV lower and upper bounds for color filtering
#   label         : Name of the color ("Red" / "Orange")
#   action_letter : Return code ("R" for Right, "L" for Left)
# --------------------------------------------------------
def detect_color_block(hsv, rgb_frame, lower, upper, label, action_letter):
    global last_output_frame

    # Create a binary mask for the given color
    mask = cv2.inRange(hsv, lower, upper)
    masked = cv2.bitwise_and(rgb_frame, rgb_frame, mask=mask)

    # Convert to grayscale and blur to reduce noise
    gray = cv2.cvtColor(masked, cv2.COLOR_RGB2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    # Find contours of the masked regions
    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Prepare frame for visualization
    output = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
    detected = False

    # Loop through all detected contours
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:   # Ignore small objects (noise)
            continue

        # Get bounding box around contour
        x, y, w, h = cv2.boundingRect(cnt)

        # Shape analysis
        aspect_ratio = w / float(h)   # Width / Height
        extent = area / (w * h)       # Area ratio (contour area vs bounding box area)
        distance = calculate_distance(w)  # Estimate distance using perceived width

        # Apply filters to eliminate false positives
        if not (0.5 <= aspect_ratio <= 0.8):   # Block must roughly match expected shape
            continue
        if not (0.56 <= extent <= 0.95):       # Filled-ness filter (rejects hollow shapes)
            continue
        if distance > MAX_DISTANCE_CM:         # Too far → ignore
            continue

        # If passed all filters → Valid block detected
        detected = True

        # Draw rectangle around block
        cv2.rectangle(output, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Show label and distance on frame
        cv2.putText(output, f"{label}: {distance:.1f} cm", (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # Show shape stats (Aspect Ratio & Extent)
        cv2.putText(output, f"AR={aspect_ratio:.2f} EXT={extent:.2f}",
                    (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # Print detection in terminal
        print(f"[{label}] ✅ AR={aspect_ratio:.2f} | EXT={extent:.2f} | DIST={distance:.1f}cm")

        # Save output frame for GUI/debug usage
        last_output_frame = output

        # Return the action code immediately ("R" or "L")
        return action_letter  

    # If no valid block detected
    if not detected:
        print(f"[{label}] ❌ No valid block in range")

    last_output_frame = output
    return None

# --------------------------------------------------------
# Detect obstacles (Red → Right, Orange → Left)
# If show_debug=True, live annotated video will be displayed
# --------------------------------------------------------
def detect_obstacle_color_direction(show_debug=False):
    global last_output_frame

    # Capture frame from camera
    frame_rgb = picam2.capture_array()
    hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

    # Run detection for both colors
    result_red = detect_color_block(hsv, frame_rgb, HSV_RED_LOWER, HSV_RED_UPPER, "Red", "R")
    result_orange = detect_color_block(hsv, frame_rgb, HSV_ORANGE_LOWER, HSV_ORANGE_UPPER, "Orange", "L")

    # Optional debug visualization
    if show_debug:
        cv2.imshow("Detection Live", last_output_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit debug view
            cv2.destroyAllWindows()

    # Return whichever was detected first (priority order: Red → Orange)
    return result_red or result_orange

# --------------------------------------------------------
# Returns the most recent annotated frame
# Useful for GUI display in the main program
# --------------------------------------------------------
def get_last_frame():
    global last_output_frame
    return last_output_frame
