import rclpy
import cv2
import os
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from constants.constants import (
    MAX_LINEAR_SPEED,
    MIN_LINEAR_SPEED,
    MARKER_DISTANCE_THRESHOLD,
    ROBOT_HALTING_POINT_DISTANCE,
    DECELERATION_RATE,
    ANGULAR_SPEED_SCALING_FACTOR,
    CALIB_FILE_DIRECTORY,
    CALIB_FILE_NAME,
    MARKER_SIZE,
    HSV_LOWER_BOUND,
    HSV_UPPER_PBOUND,
    MIN_WHEEL_SPEED,
    MAX_LINEAR,
    MIN_LINEAR,
    KP,
    WHEEL_SEPERATION,
)

from ament_index_python.packages import get_package_share_directory


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_following_robot")
        self.pub_ = self.create_publisher(
            TwistStamped, "robot_diff_drive_controller/cmd_vel", 10
        )
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
        self.aruco_distance = None
        self.marker_id = None
        self.state = "FOLLOWING"
        self.last_linear_speed = 0.10
        self.stop_counter = 0

        self.gstreamer_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.IsOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")

        # loading camera calibration info
        self.load_calibration_info()

    def get_image_from_stream(self):
        ret, frame = self.cap.read()
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
                self.camera_matrix = np.ascontiguousarray(X["matrix"], dtype=np.float64)
                self.dist_coeffs = np.ascontiguousarray(X["dist"], dtype=np.float64)
            self.get_logger().info("Loaded camera calibration successfully.")
        except IOError:
            self.get_logger().error(
                f"Calibration file not found at {calib_file}! Make sure it's built."
            )
            self.camera_matrix = np.eye(3, dtype=np.float64)
            self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

    def aruco_detection(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            aruco_distance = None
            marker_id = None
            for rvec, tvec, id in zip(rvecs, tvecs, ids):
                cv2.drawFrameAxes(
                    cv_image,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size * 0.5,
                )
                self.get_logger().info(
                    f"Marker {id}: Position x : {tvec[0][0]} y : {tvec[0][1]} z : {tvec[0][2]}, Rotation :  {rvec}"
                )
                aruco_distance = tvec[0][2]
                marker_id = int(id[0])
            return aruco_distance, marker_id

    def line_segmentation(self, cv_image):
        # converting bgr to hsv
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # create a binary mask
        mask = cv2.inRange(hsv_img, self.hsv_lower_bound, self.hsv_upper_bound)

        # apply the mask to the original image
        blue_roi_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        return blue_roi_img, mask

    def get_contours(self, mask):
        """Returns the centroid of the largest contour in the binary image (mask)"""
        MIN_AREA = 200

        # get list of contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > MIN_AREA:
                self.get_logger().info(f"M : {M}")
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])

                return {"x": x, "y": y}

    def timer_callback(self):
        frame = self.get_image_from_stream()
        # img = cv2.resize(frame,(300,200))
        # detectino aruco markers on the way
        res = self.aruco_detection(cv_image=frame)
        if res:
            self.aruco_distance, self.marker_id = res

        # finding height and width for image to crop the right and left half of the image
        h, w = frame.shape[:2]
        # getting image right part to turn right

        if self.marker_id == 1 and self.aruco_distance < MARKER_DISTANCE_THRESHOLD:
            frame[:, : w // 2] = 0
        # getting image left part to turn left
        elif self.marker_id == 0 and self.aruco_distance < MARKER_DISTANCE_THRESHOLD:
            frame[:, w // 2 :] = 0
        elif self.marker_id == 2 and self.aruco_distance < ROBOT_HALTING_POINT_DISTANCE:
            if self.state == "FOLLOWING":
                self.state = "DECELERATING"
            self.get_logger().info("Stop Marker detected. Initiating smooth stop.")

            # cmd_msg = TwistStamped()
            # cmd_msg.twist.linear.x = 0.0
            # cmd_msg.twist.angular.z = 0.0
            # self.pub_.publish(cmd_msg)
            # return
        if self.state == "DECELERATING":
            self.last_linear_speed *= DECELERATION_RATE
            if self.last_linear_speed <= 0.052:
                self.last_linear_speed = 0.0
                self.state = "STOPPED"
            cmd = TwistStamped()
            cmd.twist.linear.x = float(self.last_linear_speed)
            cmd.twist.angular.z = 0.0
            self.pub_.publish(cmd)
            return
        if self.state == "STOPPED":
            cmd = TwistStamped()
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub_.publish(cmd)
            self.stop_counter += 1

            if self.stop_counter >= 10:
                self.get_logger().info("Stopped - closing ROS")
                self.destroy_node()
                rclpy.shutdown()
                return

        # line segmentation
        blue_roi_img, mask = self.line_segmentation(cv_image=frame)
        # getting line contour
        line = self.get_contours(mask)

        if line:
            x = line["x"]
            # finding the error from line centroid to center of the image
            error_px = x - w // 2
            # normalized error between -1 to +1
            error_norm = error_px / (w // 2)
            # linear speed of the robot

            cv2.circle(blue_roi_img,(line['x'],line['y']),5,(0,0,255),7)

            linear_x = MAX_LINEAR - abs(error_norm) * (MAX_LINEAR - MIN_LINEAR)
            self.last_linear_speed = float(linear_x)
            # angular speed based on error
            angular_z = -KP * error_norm

            # clipping angular speed to set both wheels rotate with min rpm
            max_w = (linear_x - MIN_WHEEL_SPEED) * 2.0 / WHEEL_SEPERATION
            angular_z = float(np.clip(angular_z, -max_w, max_w))

            # publish velocities
            cmd = TwistStamped()
            cmd.twist.linear.x = float(linear_x)
            cmd.twist.angular.z = float(angular_z)
            self.pub_.publish(cmd)

            # Debug - what wheels will get
            v_r = linear_x + angular_z * WHEEL_SEPERATION / 2
            v_l = linear_x - angular_z * WHEEL_SEPERATION / 2
            rpm_r = (v_r / 0.033) * 9.549
            rpm_l = (v_l / 0.033) * 9.549

            self.get_logger().info(
                f"error_px:{error_px} norm:{error_norm:.2f} -> v:{linear_x:.3f} w:{angular_z:.2f} -> RPM r:{rpm_r:.1f} l:{rpm_l:.1f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
