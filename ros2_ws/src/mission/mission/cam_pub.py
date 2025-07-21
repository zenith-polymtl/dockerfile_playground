#!/usr/bin/env python3
import cv2
import subprocess
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def configure_v4l2(device: str,
                   gain:   int,
                   exposure: int,
                   lenspos:  int,
                   fps:     float):
    """
    Use v4l2-ctl to disable auto-exposure/WB, set manual exposure, gain, focus, and frame rate.
    """
    # manual exposure (1 = manual, 3 = auto on many drivers)
    subprocess.run(['v4l2-ctl', '-d', device, '-c', 'exposure_auto=1'], check=False)
    # absolute exposure (units depend on driver; Pi’s driver uses ~µs steps)
    subprocess.run(['v4l2-ctl', '-d', device, '-c', f'exposure_absolute={exposure}'], check=False)
    # analog gain
    subprocess.run(['v4l2-ctl', '-d', device, '-c', f'gain={gain}'], check=False)
    # disable auto white balance
    subprocess.run(['v4l2-ctl', '-d', device, '-c', 'white_balance_temperature_auto=0'], check=False)
    # manual focus (if supported)
    subprocess.run(['v4l2-ctl', '-d', device, '-c', f'focus_absolute={lenspos}'], check=False)
    # set the capture frame rate
    subprocess.run(['v4l2-ctl', '-d', device, f'--set-parm={int(fps)}'], check=False)

class V4L2CameraPublisher(Node):
    def __init__(self,
                 device: str = '/dev/video0',
                 width:  int = 1280,
                 height: int =  720,
                 fps:   float =  60.0,
                 gain:   int =    1,
                 exposure: int = 5000,
                 lenspos:  int =    8):
        super().__init__('v4l2_camera_publisher')
        self.pub = self.create_publisher(Image, 'camera/image', 10)
        self.bridge = CvBridge()
        self.device = device

        # apply manual controls before opening the stream
        configure_v4l2(device, gain, exposure, lenspos, fps)

        # open V4L2 capture
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)   # width :contentReference[oaicite:1]{index=1}
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)  # height :contentReference[oaicite:2]{index=2}
        self.cap.set(cv2.CAP_PROP_FPS,         fps)     # frame rate :contentReference[oaicite:3]{index=3}
        self.cap.set(cv2.CAP_PROP_GAIN,       float(gain))      # gain :contentReference[oaicite:4]{index=4}
        self.cap.set(cv2.CAP_PROP_EXPOSURE,   float(exposure))  # exposure :contentReference[oaicite:5]{index=5}
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)         # manual exposure mode :contentReference[oaicite:6]{index=6}
        self.cap.set(cv2.CAP_PROP_AUTO_WB,        0)         # disable auto white balance
        self.cap.set(cv2.CAP_PROP_FOCUS,      float(lenspos))  # focus :contentReference[oaicite:7]{index=7}

        self.timer_period = 0.05  # publish at ~20 Hz
        self.timer = self.create_timer(self.timer_period,
                                       self.timer_callback)
        self.last_time = time.time()
        self.get_logger().info("V4L2 Camera Publisher Initialized")

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return

        # convert BGR→RGB so encoding matches downstream expectations
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        msg = self.bridge.cv2_to_imgmsg(rgb, encoding='rgb8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)

        now = time.time()
        fps = 1.0 / (now - self.last_time)
        self.last_time = now
        self.get_logger().info(f"Published image at {fps:.2f} FPS")

def main(args=None):
    rclpy.init(args=args)
    node = V4L2CameraPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.cap.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
