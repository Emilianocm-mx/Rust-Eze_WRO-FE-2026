import serial
import time
from pyrplidar import PyRPlidar

# --- CONFIGURATION ---
LIDAR_PORT   = '/dev/ttyUSB0'
ESP32_PORT   = '/dev/ttyUSB1'  
BAUD_ESP32   = 9600
TRACK_WIDTH  = 1000            # mm
TARGET_CENTER = TRACK_WIDTH / 2 

# --- SETUP ---
lidar = PyRPlidar()
lidar.connect(port=LIDAR_PORT, baudrate=115200, timeout=3)
lidar.set_motor_pwm(660)
time.sleep(2)

esp32 = serial.Serial(ESP32_PORT, BAUD_ESP32, timeout=1)
time.sleep(2)

# --- HELPER FUNCTION ---
def calculate_steering_angle(scan):
    """
    From a full scan, extract left and right wall distances
    and return a steering angle between 0 and 180.
    90 = straight ahead.
    """
    
    # collect distances at key angles
    # LIDAR 0 degrees = forward on your robot (you may need to adjust this)
    left_distances  = []
    right_distances = []
    
    for measurement in scan:
        angle    = measurement.angle     # 0-360 degrees
        distance = measurement.distance  # millimetres

        if distance == 0:       # ignore invalid readings
            continue

        # left wall: readings roughly 80-100 degrees
        if 80 <= angle <= 100:
            left_distances.append(distance)

        # right wall: readings roughly 260-280 degrees
        if 260 <= angle <= 280:
            right_distances.append(distance)
    
    if not left_distances or not right_distances:
        return 90  # default: go straight if no data
    
    left_avg  = sum(left_distances)  / len(left_distances)
    right_avg = sum(right_distances) / len(right_distances)


