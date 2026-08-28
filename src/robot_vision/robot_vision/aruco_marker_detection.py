import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from ament_index_python.packages import get_package_share_directory

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')
        
        self.publisher = self.create_publisher(Image, '/camera/aruco_detections', qos_profile_sensor_data)
        self.timer = self.create_timer(1.0 / 30.0, self.process_frame)
        self.bridge = CvBridge()
        self.frame_count = 0  # Used to throttle terminal logs
        
        package_share_directory = get_package_share_directory('robot_vision')
        calib_file = os.path.join(package_share_directory, 'camera_calibration.npz')
        
        try:
            with np.load(calib_file) as X:
                self.camera_matrix = np.ascontiguousarray(X['matrix'], dtype=np.float64)
                self.dist_coeffs = np.ascontiguousarray(X['dist'], dtype=np.float64)
            self.get_logger().info("Loaded camera calibration successfully.")
        except IOError:
            self.get_logger().error(f"Calibration file not found at {calib_file}! Make sure it's built.")
            self.camera_matrix = np.eye(3, dtype=np.float64)
            self.dist_coeffs = np.zeros((1, 5), dtype=np.float64)

        self.MARKER_SIZE = 0.05  # 5 cm marker

        gstreamer_pipeline = (
            "libcamerasrc ! "
            "video/x-raw, format=NV12, width=640, height=360, framerate=30/1 ! "
            "videoconvert ! video/x-raw, format=BGR ! "
            "appsink drop=true sync=false"
        )
        self.cap = cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera via GStreamer!")
        
        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters_create()

        self.get_logger().info("Detecting markers and publishing to /camera/aruco_detections...")

    def process_frame(self):
        if not hasattr(self, 'cap') or not self.cap.isOpened():
            return
            
        ret, frame = self.cap.read()
        if not ret:
            return

        self.frame_count += 1
        frame = np.ascontiguousarray(frame)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, 
            self.dictionary, 
            parameters=self.parameters
        )

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(
                corners, 
                self.MARKER_SIZE, 
                self.camera_matrix, 
                self.dist_coeffs
            )
            
            for i in range(len(ids)):
                # Draw 3D axes
                cv2.drawFrameAxes(
                    frame, 
                    self.camera_matrix, 
                    self.dist_coeffs, 
                    rvecs[i], 
                    tvecs[i], 
                    self.MARKER_SIZE * 0.5
                )
                
                # 1. Extract distances
                x = tvecs[i][0][0]
                y = tvecs[i][0][1]
                z = tvecs[i][0][2]

                # 2. Convert rotation vector to Euler Angles (Degrees)
                rmat, _ = cv2.Rodrigues(rvecs[i][0])
                euler_angles, _, _, _, _, _ = cv2.RQDecomp3x3(rmat)
                roll, pitch, yaw = euler_angles[0], euler_angles[1], euler_angles[2]

                # 3. Print to terminal (Throttle to ~2 times a second)
                if self.frame_count % 15 == 0:
                    self.get_logger().info(
                        f"ID: {ids[i][0]} | Dist(m): X={x:.2f}, Y={y:.2f}, Z={z:.2f} | "
                        f"Angle(deg): R={roll:.1f}, P={pitch:.1f}, Y={yaw:.1f}"
                    )
                
                # 4. Draw high-visibility text on the video feed
                text = f"ID: {ids[i][0]} Z: {z:.2f}m"
                text_pos = (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 15)
                
                # Thick black outline
                cv2.putText(frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
                # Bright yellow center
                cv2.putText(frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        try:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error publishing: {e}")

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()