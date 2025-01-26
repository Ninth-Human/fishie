import numpy as np
import cv2
from picamzero import Camera

blue_hsv_range = (np.array([110, 100, 100]), np.array([130, 255, 255]))
red_hsv_range1 = (np.array([0, 150, 150]), np.array([10, 255, 255]))
red_hsv_range2 = (np.array([170, 150, 150]), np.array([180, 255, 255]))

def preprocess_mask(mask, kernel_size = 5, iterations = 1):
    #Clean up a mask using erosion and dilation
    kernel = np.ones((kernel_size, kernel_size), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=iterations)
    mask = cv2.dilate(mask, kernel, iterations=iterations)
    return mask

def find_contours(mask, min_area = 1000):
    #Find contours in a mask and filter them based on a minimum area
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    coordinates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_area:
            x, y, w, h = cv2.boundingRect(contour)
            coordinates.append((x, y, w, h))
    return coordinates

def calculate_positions(coordinates, image_width):
    #Calculate normalised positions of detected objects
    positions = []
    for (x, y, w, h) in coordinates:
        centre_x = x + w // 2
        normalised_position = (centre_x - image_width // 2) / (image_width // 2) * 10
        position = int(round(normalised_position))
        positions.append(position)
    return positions

def findfish(camera, min_area = 1000, kernel_size = 5):
    #Detect blue and red fish in an image captured by the camera.
    try:
        #Capture image from the camera
        image = camera.capture_array()
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_width = image.shape[1]

        #Detect fish
        lower_blue_hsv, upper_blue_hsv = blue_hsv_range
        blue_mask = cv2.inRange(hsv, lower_blue_hsv, upper_blue_hsv)
        blue_mask = preprocess_mask(blue_mask, kernel_size)
        blue_coordinates = find_contours(blue_mask, min_area)
        blue_positions = calculate_positions(blue_coordinates, img_width)

        #Detect obstacle
        lower_red_hsv1, upper_red_hsv1 = red_hsv_range1
        lower_red_hsv2, upper_red_hsv2 = red_hsv_range2
        red_mask1 = cv2.inRange(hsv, lower_red_hsv1, upper_red_hsv1)
        red_mask2 = cv2.inRange(hsv, lower_red_hsv2, upper_red_hsv2)
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = preprocess_mask(red_mask, kernel_size)
        red_coordinates = find_contours(red_mask, min_area)
        red_positions = calculate_positions(red_coordinates, img_width)

        return blue_positions, red_positions

    except:
        print("An error occurred")
        return [], []
