import numpy as np
import RPi.GPIO as GPIO
import time
from picamzero import Camera
import fish_detection as fd

# Constants
max_speed = 4  # Maximum speed
max_force = 2  # Turning force
view_angle = np.radians(150)  # 150 degrees field of view

# GPIO pin setup
motor_pin1 = 10  # GPIO pin connected to L293D input 1
motor_pin2 = 12  # GPIO pin connected to L293D input 2
enable_pin = 8  # GPIO pin connected to L293D enable pin
water_pin = 22  # GPIO pin connected to the water sensor
led_pin = 36  # GPIO pin for the LED

# Initialise GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)
GPIO.setup(enable_pin, GPIO.OUT)
GPIO.setup(led_pin, GPIO.OUT)
GPIO.setup(water_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Set GPIO_PIN as input
GPIO.output(enable_pin, GPIO.LOW)  # Disable the motor driver initially
GPIO.output(led_pin, GPIO.HIGH)  # Turn on LED/ replace by logic

# Boid class
class Boid:
    def __init__(self, position, velocity):
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.acceleration = np.zeros(2)

    def limit(self, vector, max_val):
        mag = np.linalg.norm(vector)
        if mag > max_val:
            vector = vector / mag * max_val
        return vector

    def apply_force(self, force):
        self.acceleration += force

    def flock(self, fishes):
        if fishes:
            avg_position = np.mean(fishes)
            if avg_position < -3:
                self.velocity = np.array([max_speed, max_force])
            elif avg_position > 3:
                self.velocity = np.array([max_speed, -max_force])
            else:
                self.velocity = np.array([max_speed, 0])
        else:
            self.velocity = np.array([0, 0])

    def update(self):
        self.velocity = self.limit(self.velocity, max_speed)
        self.position += self.velocity
        self.acceleration = np.zeros(2)

    def control_tail(self, in_water):
        if in_water:
            # Calculate the desired direction
            desired_direction = np.arctan2(self.velocity[1], self.velocity[0])
            current_direction = np.arctan2(self.position[1], self.position[0])
            direction_difference = desired_direction - current_direction
            # Enable the motor driver
            GPIO.output(enable_pin, GPIO.HIGH)
            # Move the tail to turn the fish
            if np.abs(direction_difference) > 0.1:  # Threshold to determine if turning is needed
                if direction_difference > 0:  # Turn right
                    print('Turning right')
                    GPIO.output(motor_pin1, GPIO.HIGH)
                    GPIO.output(motor_pin2, GPIO.LOW)
                else:  # Turn left
                    print('Turning left')
                    GPIO.output(motor_pin1, GPIO.LOW)
                    GPIO.output(motor_pin2, GPIO.HIGH)
            else:  # Move forward
                self.move_tail()
        else:
            print('Stopping motor')
            # If not in water or not moving forward disable the motor driver
            GPIO.output(enable_pin, GPIO.LOW)
            GPIO.output(motor_pin1, GPIO.LOW)
            GPIO.output(motor_pin2, GPIO.LOW)

    def move_tail(self):
        print('Swimming forwards')
        GPIO.output(enable_pin, GPIO.HIGH)
        # Alternating tail movement
        GPIO.output(motor_pin1, GPIO.HIGH)
        GPIO.output(motor_pin2, GPIO.LOW)
        time.sleep(0.8)  # Sleep time determines speed
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.HIGH)
        time.sleep(0.8)

        # Disable the motor driver after wagging the tail to save power
        GPIO.output(enable_pin, GPIO.LOW)

def main():
    camera = Camera()  # Initialise camera in main script to save time of it turning on every cycle
    camera.resolution = (320, 240)
    time.sleep(1)  # Give camera time to turn on
    central_boid = Boid([0, 0], [0, 0])
    try:
        while True:
            try:
                fishes = fd.findfish(camera)
                print(fishes)
                central_boid.flock(fishes)
                central_boid.update()
                in_water = not GPIO.input(water_pin)
                central_boid.control_tail(in_water)
                time.sleep(0.1)
            except Exception as e:
                print(f"Error: {e}")
                break
    finally:
        GPIO.cleanup()
        camera.close()  # Ensure camera resources are released

if __name__ == "__main__":
    main()