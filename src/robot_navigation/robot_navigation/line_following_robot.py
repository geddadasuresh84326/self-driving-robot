#!/usr/bin/env python3

import os
import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image,CompressedImage
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    qos_profile_sensor_data,
)

from robot_navigation.constants.constants import (
    MARKER_DISTANCE_THRESHOLD,
    ROBOT_HALTING_POINT_DISTANCE,
    DECELERATION_RATE,
    CALIB_FILE_DIRECTORY,
    CALIB_FILE_NAME,
    MARKER_SIZE,
    HSV_LOWER_BOUND,
    HSV_UPPER_PBOUND,
    MIN_WHEEL_SPEED,
    MAX_LINEAR,
    MIN_LINEAR,
    KP,
    KD,
    WHEEL_SEPERATION,
)


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_following_robot")

        # Publishers
        self.pub_ = self.create_publisher(
            TwistStamped, "robot_diff_drive_controller/cmd_vel", 10
        )
        self.line_pub_ = self.create_publisher(
            CompressedImage, "line_detection/stream/compressed", qos_profile_sensor_data
        )
        self.aruco_pub_ = self.create_publisher(
                    CompressedImage, "aruco_detection/stream/compressed", qos_profile_sensor_data
                )

        # Vision & Calibration Config
        self.cv_bridge = CvBridge()
        self.hsv_lower_bound = HSV_LOWER_BOUND
        self.hsv_upper_bound = HSV_UPPER_PBOUND
        self.marker_size = MARKER_SIZE
        self.calib_file_directory = CALIB_FILE_DIRECTORY
        self.calib_file_name = CALIB_FILE_NAME
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration_info()

        # ArUco Setup
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()
        # Robot State
        self.state = "FOLLOWING"
        self.last_linear_speed = 0.10
        self.stop_counter = 0
        self.prev_error_norm = 0.0
        self.last_time = self.get_clock().now()

        # GStreamer pipeline with max-buffers=1
        self.gstreamer_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=360, height=240, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")
        else:
            self.get_logger().info("camera is working")
        # Non-blocking threaded capture
        self.latest_frame = None
        self.running = True
        self.lock = threading.Lock()
        self.capture_thread = threading.Thread(
            target=self._update_frame_thread, daemon=True
        )
        self.capture_thread.start()

        # Timer callback running at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

    def _update_frame_thread(self):
        """Background thread for grabbing camera frames."""
        while self.running and rclpy.ok():
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    with self.lock:
                        self.latest_frame = np.ascontiguousarray(frame)

    def get_image_from_stream(self):
        """Thread-safe getter for the latest frame."""
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def load_calibration_info(self):
        try:
            package_share_directory = get_package_share_directory(
                self.calib_file_directory
            )
            calib_file = os.path.join(package_share_directory, self.calib_file_name)
            with np.load(calib_file) as X:
                self.camera_matrix = np.ascontiguousarray(X["matrix"], dtype=np.float64)
                self.dist_coeffs = np.ascontiguousarray(X["dist"], dtype=np.float64)
            self.get_logger().info("Loaded camera calibration successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration file: {e}")
            self.camera_matrix = np.eye(3, dtype=np.float64)
            self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

    def aruco_detection(self, cv_image):
        annotated_img = cv_image.copy()
        gray = cv2.cvtColor(annotated_img, cv2.COLOR_BGR2GRAY)
        try:

            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,self.dictionary, parameters=self.parameters
            )
        except Exception as e:
            self.get_logger().error(f"Aruco error: {e}", throttle_duration_sec=1.0)
            return None, None,annotated_img
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(annotated_img, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            for rvec, tvec, id in zip(rvecs, tvecs, ids):
                cv2.drawFrameAxes(
                    annotated_img,
                    self.camera_matrix,
                    self.dist_coeffs,
                    rvec,
                    tvec,
                    self.marker_size * 0.5,
                )
                return tvec[0][2], int(id[0]),annotated_img
        return None, None,annotated_img

    def line_segmentation(self, cv_image):
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, self.hsv_lower_bound, self.hsv_upper_bound)
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        blue_roi_img = cv2.bitwise_and(cv_image, cv_image, mask=mask)
        return blue_roi_img, mask

    def get_contours(self, mask):
        MIN_AREA = 200
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest_contour)
            if M["m00"] > MIN_AREA:
                x = int(M["m10"] / M["m00"])
                y = int(M["m01"] / M["m00"])
                return {"x": x, "y": y}
        return None

    def timer_callback(self):
        try:

            frame = self.get_image_from_stream()
            if frame is None:
                self.get_logger().info(
                    "Waiting for camera frame...", throttle_duration_sec=2.0
                )
                return

            h, w = frame.shape[:2]

            # 1. Detect ArUco markers for the current frame
            aruco_distance, marker_id, aruco_img = self.aruco_detection(
                cv_image=frame
            )

            # 2. Check turn or stop logic based on marker detection
            if marker_id is not None and aruco_distance is not None:
                if marker_id == 1:
                    frame[:, : w // 2] = 0  # Force turn right
                elif marker_id == 0:
                    frame[:, w // 2 :] = 0  # Force turn left
                elif marker_id == 2:
                    if self.state == "FOLLOWING":
                        self.state = "DECELERATING"
                    self.get_logger().info(
                        "Stop Marker detected. Initiating deceleration."
                    )

            # 3. Handle state machine for stopping/decelerating
            if self.state == "DECELERATING":
                self.last_linear_speed *= DECELERATION_RATE
                if self.last_linear_speed <= 0.052:
                    self.last_linear_speed = 0.0
                    self.state = "STOPPED"

                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "base_footprint"
                cmd.twist.linear.x = float(self.last_linear_speed)
                cmd.twist.angular.z = 0.0
                self.pub_.publish(cmd)
                return

            if self.state == "STOPPED":
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "base_footprint"
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.pub_.publish(cmd)

                self.stop_counter += 1
                if self.stop_counter >= 10:
                    self.get_logger().info("Robot fully stopped. Shutting down node.")
                    self.destroy_node()
                    rclpy.shutdown()
                    return

            # 4. Line segmentation and motion control
            blue_roi_img, mask = self.line_segmentation(cv_image=frame)
            line = self.get_contours(mask)

            debug_img = blue_roi_img.copy()
            now = self.get_clock().now()
            dt = (now-self.last_time).nanoseconds/1e9 #time in seconds
            self.last_time = now
            if line and dt>0:
                x = line["x"]
                y = line["y"]
                error_px = x - w // 2
                error_norm = error_px / (w // 2)

                cv2.circle(debug_img, (x, y), 6, (0, 0, 255), -1)

                derivative = (error_norm - self.prev_error_norm)/dt
                self.prev_error_norm = error_norm
                linear_x = MAX_LINEAR - abs(error_norm) * (MAX_LINEAR - MIN_LINEAR)
                self.last_linear_speed = float(linear_x)
                angular_z = -(KP * error_norm + KD * derivative)

                max_w = (linear_x - MIN_WHEEL_SPEED) * 2.0 / WHEEL_SEPERATION
                angular_z = float(np.clip(angular_z, -max_w, max_w))

                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "base_footprint"
                cmd.twist.linear.x = float(linear_x)
                cmd.twist.angular.z = float(angular_z)
                self.pub_.publish(cmd)
                self.get_logger().info(
                                f"error:{error_px} norm:{error_norm:.2f} v:{linear_x:.3f} w:{angular_z:.2f} marker id : {marker_id}"
                            )
            else:
                # Fallback stop if line is lost during FOLLOWING state
                cmd = TwistStamped()
                cmd.header.stamp = self.get_clock().now().to_msg()
                cmd.header.frame_id = "base_footprint"
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0
                self.pub_.publish(cmd)

            # Publish debug visualization image
            # ros_img = self.cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
            ret,buffer = cv2.imencode(".jpg",debug_img,[int(cv2.IMWRITE_JPEG_QUALITY),75])
            if not ret:
                self.get_logger().error("Error occurred when publishing stream")
                return
            msg = CompressedImage()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            try:
                self.line_pub_.publish(msg)
            except rclpy._rclpy_pybind11.RCLError:
                pass
            # self.line_pub_.publish(ros_img)

            # publishing aruco detection stream
            ret, buffer = cv2.imencode(
                ".jpg", aruco_img, [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            )
            if not ret:
                self.get_logger().error("Error occurred when publishing stream")
                return
            aruco_msg = CompressedImage()
            aruco_msg.header.stamp = self.get_clock().now().to_msg()
            aruco_msg.header.frame_id = "camera_link"
            aruco_msg.format = "jpeg"
            aruco_msg.data = buffer.tobytes()
            try:
                self.aruco_pub_.publish(aruco_msg)
            except rclpy._rclpy_pybind11.RCLError:
                pass
        except Exception as e:
            self.get_logger().error(f"timer crash: {e}")
            import traceback

            traceback.print_exc()

    def destroy_node(self):
        self.running = False
        if hasattr(self, "capture_thread") and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
