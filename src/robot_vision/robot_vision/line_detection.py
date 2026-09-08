import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from robot_vision.constants.constants import (
    HSV_LOWER_BOUND,
    HSV_UPPER_PBOUND,
)


class LineDetector(Node):
    def __init__(self):
        super().__init__("line_detector")
        self.image_pub_ = self.create_publisher(Image, "line_detection/debug_image", 10)
        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)
        self.cv_bridge = CvBridge()
        self.hsv_lower_bound = HSV_LOWER_BOUND
        self.hsv_upper_bound = HSV_UPPER_PBOUND

        self.gstreamer_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(self.gstreamer_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")

    def get_image_from_stream(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None
        return np.ascontiguousarray(frame)

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
            return

        blue_roi_img, mask = self.line_segmentation(frame)
        line = self.get_contours(mask)

        # draw line if found
        if line:
            cv2.circle(blue_roi_img, (line["x"], line["y"]), 8, (0, 0, 255), -1)

        # publish
        ros_img = self.cv_bridge.cv2_to_imgmsg(blue_roi_img, encoding="bgr8")
        self.image_pub_.publish(ros_img)


def main(args=None):
    rclpy.init(args=args)
    node = LineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
