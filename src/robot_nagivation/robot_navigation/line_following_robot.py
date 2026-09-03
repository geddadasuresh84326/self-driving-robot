import rclpy
import cv2
import os
import numpy as np
from rclpy import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from constants.constants import MAX_LINEAR_SPEED,MIN_LINEAR_SPEED,      MARKER_DISTANCE_THRESHOLD,ROBOT_HALTING_POINT_DISTANCE,DECELERATION_RATE,ANGULAR_SPEED_SCALING_FACTOR,CALIB_FILE_DIRECTORY,CALIB_FILE_NAME,MARKER_SIZE,HSV_LOWER_BOUND,HSV_UPPER_PBOUND

from ament_index_python.packages import get_package_share_directory

class LineFollower(Node):
    def __int__(self):
        super().__init__("line_following_robot")
        self.pub_ = self.create_publisher(TwistStamped,"robot_diff_drive_controller/cmd_vel",10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.pub_
        self.camera_matrix = None
        self.dist_coeffs = None
        self.marker_size = MARKER_SIZE
        self.calib_file_directory = CALIB_FILE_DIRECTORY
        self.calib_file_name = CALIB_FILE_NAME
        self.cv_bridge = CvBridge()
        self.hsv_lower_bound = HSV_LOWER_BOUND
        self.hsv_upper_bound = HSV_UPPER_PBOUND
        self.gstreamer_pipeline = (
                    "libcamerasrc ! "
                    "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
                    "videoconvert ! video/x-raw, format=BGR ! "
                    "appsink drop=true sync=false"
                )
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

    def get_image_from_stream(self):
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.IsOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")
        ret,frame = self.cap.read()
        if not ret:
            self.get_logger().info("no frame captured..")
            return
        frame = np.ascontiguousarray(frame)
        return frame
    def load_calibration_info(self):
        package_share_directory = get_package_share_directory(self.calib_file_directory)
        calib_file = os.path.join(package_share_directory, self.calib_file_name)
        try:
            with np.load(calib_file) as X:
                self.camera_matrix = np.ascontiguousarray(X['matrix'], dtype=np.float64)
                self.dist_coeffs = np.ascontiguousarray(X['dist'], dtype=np.float64)
            self.get_logger().info("Loaded camera calibration successfully.")
        except IOError:
            self.get_logger().error(f"Calibration file not found at {calib_file}! Make sure it's built.")
            self.camera_matrix = np.eye(3, dtype=np.float64)
            self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

    def aruco_detection(self,cv_image):
        gray = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

        corners,ids,rejected = cv2.aruco.detectMarkers(
            gray,
            self.dictionary,
            parameters = self.parameters
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image,corners,ids)
            rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(
                corners,
                self.marker_size,
                self.camera_matrix,
                self.dist_coeffs
            )
            aruco_distance = None
            marker_id = None 
            for rvec,tvec,id in zip(rvecs,tvecs,ids):
                cv2.drawFrameAxes(
                                    cv_image, 
                                    self.camera_matrix, 
                                    self.dist_coeffs, 
                                    rvec, 
                                    tvec, 
                                    self.MARKER_SIZE * 0.5
                                )
                self.get_logger().info(
                    f"Marker {id}: Position x : {tvec[0][0]} y : {tvec[0][1]} z : {tvec[0][2]}, Rotation :  {rvec}")
                aruco_distance = tvec[0][2]
                marker_id = id
            return aruco_distance,marker_id

    def line_segmentation(self,cv_image):
        # converting bgr to hsv
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # create a binary mask
        mask = cv2.inRange(hsv_img, self.hsv_lower_bound, self.hsv_upper_bound)

        # apply the mask to the original image
        blue_roi_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        return blue_roi_img, mask

    def timer_callback():
        