from picamera2 import Picamera2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
from libcamera import controls

def initialize_cam( gain = 1, ExposureTime = 5000, lenspos = 8):
    
    # Initialize the Raspberry Pi Camera using Picamera2
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(main={"size": (1280, 720)}))
    picam2.start()
    return picam2


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        self.publisher_ = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()

        self.picam2 = initialize_cam( gain=1, ExposureTime=2000, lenspos=8)

        self.timer_period = 0.05  # ~20Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.last_time = time.time()
        self.get_logger().info("Camera Publisher Node Initialized")

    def timer_callback(self):
        try:
            frame = self.picam2.capture_array()
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(msg)

            now = time.time()
            fps = 1.0 / (now - self.last_time)
            self.last_time = now
            self.get_logger().info(f"Published image at {fps:.2f} FPS")

        except Exception as e:
            self.get_logger().error(f"Failed to capture/publish image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.picam2.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
