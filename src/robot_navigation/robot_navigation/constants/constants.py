import numpy as np

# To turn the robot left or right
MARKER_DISTANCE_THRESHOLD = 5.2

# Robot halt distance
ROBOT_HALTING_POINT_DISTANCE = 2.3

# deceleration factor to stop the robot
DECELERATION_RATE = 0.5

MAX_LINEAR_SPEED = 0.4
MIN_LINEAR_SPEED = 0.15


# To control robot speed at curves
ANGULAR_SPEED_SCALING_FACTOR = 0.8

# Controller gains
KP = 0.001  # Proportional gain
KI = 0.0  # Integral gain
KD = 0.0  # Derivative gain

CALIB_FILE_DIRECTORY = "robot_vision"
CALIB_FILE_NAME = "camera_calibration.npz"
MARKER_SIZE = 0.05  # 5 cm marker

# HSV_LOWER_BOUND = np.array([100, 150, 50])
# HSV_UPPER_PBOUND = np.array([130, 255, 200])

HSV_LOWER_BOUND = np.array([105, 60, 35])
HSV_UPPER_PBOUND = np.array([138, 255, 255])
# constants to drive the robot

MAX_LINEAR = 0.12  # 30 RPM
MIN_LINEAR = 0.09  # 22.6 RPM 
MIN_WHEEL_SPEED = 0.0  # 15 RPM 

# angular speed scaling factor
KP = 0.4
# wheel seperation
WHEEL_SEPERATION = 0.179
