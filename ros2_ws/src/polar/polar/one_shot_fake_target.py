# polar/one_shot_fake_target.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class OneShotFakeTarget(Node):
    def __init__(self, delay_sec: float = 5.0):
        super().__init__('one_shot_fake_target')
        self.pub = self.create_publisher(PoseStamped, '/estimated_center_location', 10)
        # fire once after delay_sec
        self.create_timer(delay_sec, self._fire_once)

    def _fire_once(self):
        # build msg
        msg = PoseStamped()
        now = self.get_clock().now().to_msg()  # type: Time
        msg.header.stamp = now
        msg.header.frame_id = 'map'
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.5
        msg.pose.orientation.w = 1.0

        self.pub.publish(msg)
        self.get_logger().info('Published one PoseStamped to /estimated_center_location')
        # exit cleanly
        rclpy.shutdown()

def main():
    rclpy.init()
    node = OneShotFakeTarget(delay_sec=5.0)
    rclpy.spin(node)  # will return after shutdown() is called in _fire_once

if __name__ == '__main__':
    main()
