import numpy as np
import cv2
from picamzero import Camera

def findfish(camera):
    image = camera.capture_array()

    # Convert image to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define HSV range for detecting blue colour
    lower_hsv = np.array([110, 50, 50])
    upper_hsv = np.array([130, 255, 255])
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Apply magic to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=1)
    mask = cv2.dilate(mask, kernel, iterations=1)

    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    min_size = 1000  # Increase the minimum size to filter out smaller contours
    blue_coordinates = []

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < min_size:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        blue_coordinates.append((x, y, w, h))

    # Determine position of detected fish
    positions = []
    img_width = image.shape[1]
    for (x, y, w, h) in blue_coordinates:
        center_x = x + w // 2
        normalized_position = (center_x - img_width // 2) / (img_width // 2) * 10
        integer_position = int(round(normalized_position))
        positions.append(integer_position)

    return positions