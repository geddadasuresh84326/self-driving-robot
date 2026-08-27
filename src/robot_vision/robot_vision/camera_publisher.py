import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 5)
        self.bridge = CvBridge()
        
        # Pi 5 (PiSP) compatible GStreamer pipeline string
        gst_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        
        self.get_logger().info("Attempting to open Pi 5 libcamera pipeline...")

        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")
            return

        self.get_logger().info("Camera opened successfully! Publishing to /camera/image_raw")
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()