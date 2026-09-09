import numpy as np
# HSV_LOWER_BOUND = np.array([100, 150, 50])
# HSV_UPPER_PBOUND = np.array([130, 255, 200])
HSV_LOWER_BOUND = np.array([105, 60, 35])
HSV_UPPER_PBOUND = np.array([138, 255, 255])
# constants to drive the robot

MAX_LINEAR = 0.104  # 30 RPM
MIN_LINEAR = 0.078  # 22.6 RPM
MIN_WHEEL_SPEED = 0.052  # 15 RPM

# angular speed scaling factor
KP = 0.5
# wheel seperation
WHEEL_SEPERATION = 0.179
