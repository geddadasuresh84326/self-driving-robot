import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import threading
import time

class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')

        # Use Best Effort QoS to prevent the stream from hanging/lagging in RViz2
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.bridge = CvBridge()

        self.save_dir = "capture_images"
        os.makedirs(self.save_dir, exist_ok=True)
        self.image_count = 0
        self.latest_frame = None

        # Pi 5 (PiSP) compatible GStreamer pipeline
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
            sys.exit(1)

        self.get_logger().info("Camera opened! Publishing to /camera/image_raw")
        
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)

        # Start a background thread to listen for terminal keystrokes
        self.input_thread = threading.Thread(target=self.terminal_listener, daemon=True)
        self.input_thread.start()

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # Save a copy of the latest frame in memory for the terminal thread to grab
            self.latest_frame = frame.copy()
            
            # Publish to ROS 2
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher_.publish(msg)

    def terminal_listener(self):
        time.sleep(1) # Let the camera start up cleanly
        print("\n" + "="*50)
        print("📷 HEADLESS CAPTURE MODE ACTIVATED")
        print("👉 Press [ENTER] in this terminal to save the current frame.")
        print("👉 Type 'q' and press [ENTER] to exit.")
        print("="*50 + "\n")

        while rclpy.ok():
            try:
                # Wait for user input in the SSH terminal
                cmd = input()
                if cmd.strip().lower() == 'q':
                    self.get_logger().info("Exit command received. Shutting down...")
                    rclpy.shutdown()
                    break
                else:
                    self.save_image()
            except EOFError:
                break

    def save_image(self):
        if self.latest_frame is not None:
            filename = os.path.join(self.save_dir, f"img_{self.image_count:02d}.png")
            cv2.imwrite(filename, self.latest_frame)
            self.get_logger().info(f"✅ Saved calibration image: {filename}")
            self.image_count += 1
        else:
            self.get_logger().warning("No frame available yet, try again in a moment!")

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node stopped cleanly.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()