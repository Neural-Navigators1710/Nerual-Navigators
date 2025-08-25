#Use this code to callibrate your HSV values in your lighting for both lines and blocks

import cv2
import numpy as np
from picamera2 import Picamera2

# --- Initialize PiCamera2 ---
picam2 = Picamera2()
# Configure preview: format XRGB8888, resolution 640x480
config = picam2.create_preview_configuration(main={"format": "XRGB8888", "size": (640, 480)})
picam2.configure(config)
picam2.start()

# --- Dummy callback function for trackbars ---
# OpenCV requires a callback function for trackbars,
# but we don't need to do anything here.
def nothing(x):
    pass

# --- Create a window to hold the trackbars ---
cv2.namedWindow("Trackbars")

# --- Create HSV range trackbars ---
# LH = Lower Hue, LS = Lower Saturation, LV = Lower Value
# UH = Upper Hue, US = Upper Saturation, UV = Upper Value
# Hue ranges: 0–179, Saturation: 0–255, Value: 0–255
cv2.createTrackbar("LH", "Trackbars", 0,   179, nothing)
cv2.createTrackbar("LS", "Trackbars", 0,   255, nothing)
cv2.createTrackbar("LV", "Trackbars", 0,   255, nothing)
cv2.createTrackbar("UH", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("US", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("UV", "Trackbars", 255, 255, nothing)

# --- Main Loop ---
while True:
    # Capture frame from PiCamera2
    frame = picam2.capture_array()

    # Convert BGR frame (default) to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # --- Read positions of trackbars (current HSV values) ---
    lh = cv2.getTrackbarPos("LH", "Trackbars")
    ls = cv2.getTrackbarPos("LS", "Trackbars")
    lv = cv2.getTrackbarPos("LV", "Trackbars")
    uh = cv2.getTrackbarPos("UH", "Trackbars")
    us = cv2.getTrackbarPos("US", "Trackbars")
    uv = cv2.getTrackbarPos("UV", "Trackbars")

    # Define lower and upper HSV ranges based on trackbar values
    lower = np.array([lh, ls, lv])
    upper = np.array([uh, us, uv])

    # --- Create mask ---
    # Keeps only pixels within the HSV range (white = detected color)
    mask = cv2.inRange(hsv, lower, upper)

    # --- Apply mask to original frame ---
    # Shows the detected color in original frame, rest = black
    result = cv2.bitwise_and(frame, frame, mask=mask)

    # --- Show all windows ---
    cv2.imshow("Frame", frame)   # Original camera feed
    cv2.imshow("Mask", mask)     # Binary mask (white = detected color)
    cv2.imshow("Result", result) # Final result with color isolated

    # Press 'q' to quit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
picam2.stop()
cv2.destroyAllWindows()   # <-- this is a function, needs parentheses when used
