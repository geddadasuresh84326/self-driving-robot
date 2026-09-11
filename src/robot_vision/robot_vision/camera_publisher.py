import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data


class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.publisher_ = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", qos_profile_sensor_data
        )

        self.bridge = CvBridge()

        # Pi 5 (PiSP) compatible GStreamer pipeline string
        gst_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1 sync=false"
        )

        self.get_logger().info("Attempting to open Pi 5 libcamera pipeline...")

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")
            return

        self.get_logger().info(
            "Camera opened successfully! Publishing to /camera/image_raw/compressed"
        )
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        if not rclpy.ok():
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        ret, buffer = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
        if not ret:
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        try:
            self.publisher_.publish(msg)
        except rclpy._rclpy_pybind11.RCLError:
            pass

    def destroy_node(self):
        if hasattr(self, "timer"):
            self.timer.cancel()
        if hasattr(self, "cap") and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()

    def _handle_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _handle_sigterm)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
