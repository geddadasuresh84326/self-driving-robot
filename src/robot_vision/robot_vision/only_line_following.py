import threading
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from constants.constants import (
    HSV_LOWER_BOUND,
    HSV_UPPER_PBOUND,
    MIN_WHEEL_SPEED,
    MAX_LINEAR,
    MIN_LINEAR,
    KP,
    WHEEL_SEPERATION,
)


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_following_robot")

        # Publishers
        self.pub_ = self.create_publisher(
            TwistStamped, "robot_diff_drive_controller/cmd_vel", 10
        )
        self.image_pub_ = self.create_publisher(
            Image, "line_detection/debug_image", 10
        )

        self.cv_bridge = CvBridge()
        self.hsv_lower_bound = HSV_LOWER_BOUND
        self.hsv_upper_bound = HSV_UPPER_PBOUND

        # GStreamer pipeline with max-buffers=1 to eliminate video frame latency
        self.gstreamer_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")

        # Threading setup for non-blocking frame capture
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
        """Background thread that continuously grabs camera frames without blocking rclpy.spin()."""
        while self.running and rclpy.ok():
            if self.cap.isOpened():
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    with self.lock:
                        self.latest_frame = np.ascontiguousarray(frame)

    def get_image_from_stream(self):
        """Thread-safe getter for the latest captured frame."""
        with self.lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy()

    def line_segmentation(self, cv_image):
        hsv_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv_img, self.hsv_lower_bound, self.hsv_upper_bound)
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
        frame = self.get_image_from_stream()
        if frame is None:
            self.get_logger().info(
                "Waiting for initial camera frame...", throttle_duration_sec=2.0
            )
            return

        h, w = frame.shape[:2]

        # Line segmentation
        blue_roi_img, mask = self.line_segmentation(cv_image=frame)
        line = self.get_contours(mask)

        # Image for debug topic
        debug_img = blue_roi_img.copy()

        if line:
            x = line["x"]
            y = line["y"]
            error_px = x - w // 2
            error_norm = error_px / (w // 2)

            # Visual overlay
            cv2.circle(debug_img, (x, y), 8, (0, 0, 255), -1)
            cv2.line(debug_img, (w // 2, 0), (w // 2, h), (255, 0, 0), 1)
            cv2.line(debug_img, (x, 0), (x, h), (0, 255, 0), 1)

            linear_x = MAX_LINEAR - abs(error_norm) * (MAX_LINEAR - MIN_LINEAR)
            angular_z = -KP * error_norm

            max_w = (linear_x - MIN_WHEEL_SPEED) * 2.0 / WHEEL_SEPERATION
            angular_z = float(np.clip(angular_z, -max_w, max_w))

            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_footprint"
            cmd.twist.linear.x = float(linear_x)
            cmd.twist.angular.z = float(angular_z)
            self.pub_.publish(cmd)

            self.get_logger().info(
                f"error:{error_px} norm:{error_norm:.2f} v:{linear_x:.3f} w:{angular_z:.2f}"
            )
        else:
            # Stop when no line is detected
            self.get_logger().info(f"no line detected stopping")
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "base_footprint"
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0
            self.pub_.publish(cmd)

        # Publish debug image
        ros_img = self.cv_bridge.cv2_to_imgmsg(debug_img, encoding="bgr8")
        self.image_pub_.publish(ros_img)

    def destroy_node(self):
        """Safely stops background threads and releases video hardware on node destroy."""
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