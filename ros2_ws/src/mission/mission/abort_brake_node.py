import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from zenmav.core import Zenmav
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AbortBrake(Node):
    def __init__(self):
        super().__init__('abort_brake')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        self.subscriber_ab = self.create_subscription(String, '/abort_brake', self.abort_brake_callback, qos_profile)
        self.publisher_ab_call = self.create_publisher(String, '/close', qos_profile)
        self.nav = Zenmav(ip = 'tcp:127.0.0.1:5763')
        #self.nav = Zenmav(ip = 'udp:127.0.0.1:14550')

    def abort_brake_callback(self, msg):
        if msg.data == "a.b.":
            self.get_logger().info(f'Brake incoming, {msg.data} received!')
            self.nav.set_mode('BRAKE')
            self.get_logger().info(f'Brake mode successfully enforced!')

            msg_close = String()
            msg_close.data = "close"
            self.publisher_ab_call.publish(msg_close)
            self.get_logger().info(f'Nodes approach, target and graph successfully closed!')
            time.sleep(3)

            self.nav.set_mode('GUIDED')
            self.get_logger().info(f'Guided mode successfully enforced!')
        
def main(args=None):
    rclpy.init(args=args)
    node = AbortBrake()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()