import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import os
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

class CameraNode(Node):
    def __init__(self, camera_device_index):
        super().__init__(f'camera_node_{camera_device_index}')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, f'camera{camera_device_index}_image', 10)
        self.status_publisher = self.create_publisher(Bool, f'camera{camera_device_index}_status', 10)

        # Initialize conditions & values
        self.camera_device_index = camera_device_index
        self.cap = None
        self.done = False
        self.FPS = 30.0
        self.FRAME_WIDTH = 1280
        self.FRAME_HEIGHT = 720
        self.out = None
        self.video_path = None

        # Set initial status to off
        self.publish_status(False)

    def create_video(self, seconds):
        try:
            self.get_logger().debug("[DEBUG] Starting create_video()...")

            self.cap = cv2.VideoCapture(self.camera_device_index)
            self.seconds = seconds
            self.get_logger().debug(f"[DEBUG] VideoCapture object created with index {self.camera_device_index}.")

            if not self.cap.isOpened():
                self.get_logger().error(f"Camera {self.camera_device_index} could not be opened.")
                self.publish_status(False)
                self.done = True
                return

            self.get_logger().info(f"Camera {self.camera_device_index} opened.")
            self.publish_status(True)

            # Set resolution, FPS, and codec
            self.get_logger().debug(f"[DEBUG] Setting resolution to {self.FRAME_WIDTH}x{self.FRAME_HEIGHT} and FPS to {self.FPS}...")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.FRAME_HEIGHT)
            self.cap.set(cv2.CAP_PROP_FPS, self.FPS)
            fourcc = cv2.VideoWriter_fourcc(*'MJPG')
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

            # Confirm settings
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
            self.get_logger().debug(f"[DEBUG] Camera actual resolution: {actual_width}x{actual_height}, FPS: {actual_fps}")

            # Define output folder
            video_dir = f'/media/pi/ELLAB/videos/camera{self.camera_device_index}'

            self.get_logger().debug(f"[DEBUG] Creating video directory at {video_dir}...")
            try:
                os.makedirs(video_dir, exist_ok=True)
            except Exception as e:
                self.get_logger().error(f"Error creating directory {video_dir}: {e}")
                self.publish_status(False)
                self.done = True
                return

            self.timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            self.video_path = os.path.join(video_dir, f'video_{self.timestamp}.avi')
            self.get_logger().debug(f"[DEBUG] Output video path: {self.video_path}")

            self.out = cv2.VideoWriter(self.video_path, fourcc, self.FPS, (self.FRAME_WIDTH, self.FRAME_HEIGHT))
            if not self.out.isOpened():
                self.get_logger().error(f"VideoWriter could not be opened for camera {self.camera_device_index}.")
                self.publish_status(False)
                self.done = True
                return

            self.get_logger().info("Starting video recording...")

            # Record for duration
            for frame_idx in range(seconds * int(self.FPS)):
                ret, frame = self.cap.read()
                if not ret:
                    self.get_logger().error(f"[DEBUG] Failed to read frame {frame_idx} from camera {self.camera_device_index}.")
                    break

                self.out.write(frame)
                ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.publisher.publish(ros_image)
                if frame_idx % 10 == 0:
                    self.get_logger().debug(f"[DEBUG] Recorded and published frame {frame_idx}")

        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")
            if self.cap:
                self.cap.release()
            if hasattr(self, 'out'):
                self.out.release()
            self.done = True
            self.publish_status(False)

        finally:
            self.get_logger().debug("[DEBUG] Releasing camera and video writer resources...")
            if self.cap and self.cap.isOpened():
                self.cap.release()
            if hasattr(self, 'out') and self.out.isOpened():
                self.out.release()

            if self.video_path:
                self.get_logger().info(f"Video for camera {self.camera_device_index} saved as {self.video_path}")
            else:
                self.get_logger().warn(f"Video path not available for camera {self.camera_device_index}.")

            self.publish_status(False)
            self.done = True
            self.get_logger().info(f"Shutting down camera {self.camera_device_index}...")


    def is_done(self):
        return self.done

    def publish_status(self, status):
        """Publish the camera's status (True for available, False for unavailable)."""
        self.get_logger().info(f"Publishing camera {self.camera_device_index} status: {status}")
        status_msg = Bool()
        status_msg.data = status
        self.status_publisher.publish(status_msg)
