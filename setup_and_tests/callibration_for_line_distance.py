#Use this code to callibrate the distance at which the lines should be detected

import cv2
import numpy as np
from picamera2 import Picamera2

# --- Initialize Camera ---
# Create PiCamera2 object and set resolution to 640x480 for decent performance.
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

# --- HSV Ranges (tuned for your robot environment) ---
# These ranges filter specific colors in HSV space.
# Orange detection (adjusted for block at ~10 cm distance)
lower_orange = np.array([5, 100, 100])
upper_orange = np.array([20, 255, 255])

# Blue detection (same, tuned values)
lower_blue   = np.array([100, 150, 50])
upper_blue   = np.array([140, 255, 255])

# Kernel for morphological operations (noise reduction)
kernel = np.ones((5, 5), np.uint8)

def largest_line_short_side(mask, frame, draw_color):
    """
    Finds the largest contour in a binary mask and calculates
    the 'short side' length of its minimum bounding rectangle.

    mask: binary mask from HSV threshold
    frame: original camera frame (for drawing)
    draw_color: color used for drawing rectangle/annotations
    returns: length of the shortest side (pixels) or None if nothing found
    """

    # Step 1: Clean mask to remove noise
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)   # removes small blobs
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # closes small gaps

    # Step 2: Find contours in the cleaned mask
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None  # no contours found

    # Step 3: Pick the largest contour (most likely the target block)
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 500:  
        # Ignore very small areas (noise / irrelevant spots)
        return None

    # Step 4: Get minimum area rectangle (rotated bounding box)
    rect = cv2.minAreaRect(c)   # returns (center, (w, h), angle)
    (cx, cy), (w, h), angle = rect
    short_side = min(w, h)      # pick the shorter dimension (thickness of block)

    # Step 5: Draw rectangle + text for debugging
    box = np.intp(cv2.boxPoints(rect))  # corner points of rectangle
    cv2.drawContours(frame, [box], 0, draw_color, 2)
    cv2.putText(frame, f"short={short_side:.1f}px", (int(cx)-60, int(cy)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)

    return short_side

print("👉 Calibrate at EXACTLY 10 cm. Press 'q' when you’ve noted the value.")

while True:
    # Capture frame from camera
    frame = picam2.capture_array()

    # Convert to HSV color space for color filtering
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Create masks for orange and blue
    mask_orange = cv2.inRange(hsv, lower_orange, upper_orange)
    mask_blue   = cv2.inRange(hsv, lower_blue, upper_blue)

    # Measure short side of largest detected block (if present)
    short_orange = largest_line_short_side(mask_orange, frame, (0, 140, 255))  # orange box
    short_blue   = largest_line_short_side(mask_blue, frame, (255, 0, 0))      # blue box

    # Show live feed
    cv2.imshow("Calibration", frame)

    # Quit when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
cv2.destroyAllWindows()
picam2.stop()
