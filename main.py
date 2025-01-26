import numpy as np
import RPi.GPIO as GPIO
import time
from picamzero import Camera
import fish_detection as fd

# Constants
MAX_SPEED = 4  # Maximum speed
TURNING_FORCE = 2  # Turning force
VIEW_ANGLE = np.radians(150)  # 150 degrees field of view
MOTOR_DELAY = 0.8  # Delay for motor movement (tail wagging)

# GPIO pin setup
MOTOR_PIN1 = 10  # GPIO pin connected to L293D input 1
MOTOR_PIN2 = 12  # GPIO pin connected to L293D input 2
ENABLE_PIN = 8  # GPIO pin connected to L293D enable pin
WATER_PIN = 22  # GPIO pin connected to the water sensor
LED_PIN = 36  # GPIO pin for the LED

#Initialise GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup(MOTOR_PIN1, GPIO.OUT)
GPIO.setup(MOTOR_PIN2, GPIO.OUT)
GPIO.setup(ENABLE_PIN, GPIO.OUT)
GPIO.setup(LED_PIN, GPIO.OUT)
GPIO.setup(WATER_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)  #Set GPIO_PIN as input
GPIO.output(ENABLE_PIN, GPIO.LOW)  #Disable the motor driver initially
GPIO.output(LED_PIN, GPIO.HIGH)  #Turn on LED/ replace by logic

#Boid class
class Boid:
    def __init__(self, position, velocity):
        self.position = np.array(position, dtype=np.float64)
        self.velocity = np.array(velocity, dtype=np.float64)
        self.acceleration = np.zeros(2)

    def limit(self, vector, max_val):
        #Limit vector to maximum magnitude
        magnitude = np.linalg.norm(vector)
        if magnitude > max_val:
            vector = vector / magnitude * max_val
        return vector

    def flock(self, fishes, obstacles):
        if fishes:
            avg_fish_direction = np.mean(fishes) #Find the average direction of all fishes
        else:
            avg_fish_direction = 0

        if obstacles:
            direction = avg_fish_direction - np.mean(obstacles) #Calculate the desired direction by avoiding the obstacles
        else:
            direction = avg_fish_direction

        if direction > 3:
            self.velocity = np.array([MAX_SPEED, -TURNING_FORCE])
        elif direction < -3:
            self.velocity = np.array([MAX_SPEED, TURNING_FORCE])
        else:
            self.velocity = np.array([MAX_SPEED, 0])

    def update(self):
        #Update boid position and reset acc
        self.velocity = self.limit(self.velocity, MAX_SPEED)
        self.position += self.velocity
        self.acceleration = np.zeros(2)

    def control_tail(self, in_water):
        if in_water:
            #Calculate the desired direction
            desired_direction = np.arctan2(self.velocity[1], self.velocity[0])
            current_direction = np.arctan2(self.position[1], self.position[0])
            direction_difference = desired_direction - current_direction
            
            #Move the tail to turn the fish
            if np.abs(direction_difference) > 0.1:  #Threshold to determine if turning is needed
                if direction_difference > 0:  #Turn right
                    print('Turning right')
                    self._move_motor(GPIO.HIGH, GPIO.LOW)
                else:  #Turn left
                    print('Turning left')
                    self._move_motor(GPIO.LOW, GPIO.HIGH)
            else:  #Move forward
                self.move_tail()
        else:
            print('Stopping motor')
            #If not in water or not moving forward disable the motor driver
            GPIO.output(ENABLE_PIN, GPIO.LOW)
            GPIO.output(MOTOR_PIN1, GPIO.LOW)
            GPIO.output(MOTOR_PIN2, GPIO.LOW)

    def move_tail(self):
        print('Swimming forwards')
        #Alternating tail movement to swimmmmmmmmmm
        self._move_motor(GPIO.HIGH, GPIO.LOW)
        time.sleep(MOTOR_DELAY)
        self._move_motor(GPIO.LOW, GPIO.HIGH)
        time.sleep(MOTOR_DELAY)

        self._stop_motor()

    def _move_motor(self, pin1_state, pin2_state):
        #Control the motor driver.
        GPIO.output(ENABLE_PIN, GPIO.HIGH)
        GPIO.output(MOTOR_PIN1, pin1_state)
        GPIO.output(MOTOR_PIN2, pin2_state)

    def _stop_motor(self):
        #Stop the motor and disable the motor driver.
        GPIO.output(ENABLE_PIN, GPIO.LOW)
        GPIO.output(MOTOR_PIN1, GPIO.LOW)
        GPIO.output(MOTOR_PIN2, GPIO.LOW)

def main():
    camera = Camera()  #Initialise camera in main script to save time of it turning on every cycle
    camera.resolution = (320, 240)
    time.sleep(1)  #Give camera time to turn on

    central_boid = Boid([0, 0], [0, 0])
    
    try:
        while True:
            try:
                fishes, obstacles = fd.findfish(camera)
                print(f'Fish positions: {fishes}')
                print(f'Obstacle positions: {obstacles}')

                central_boid.flock(fishes, obstacles)
                central_boid.update()

                in_water = not GPIO.input(WATER_PIN)
                central_boid.control_tail(in_water)

                time.sleep(0.1)
            except:
                print("Error occured")
                break
    finally:
        GPIO.cleanup()
        camera.close()  #Ensure camera resources are released
        print("GPIO cleaned up and camera closed.")

if __name__ == "__main__":
    main()