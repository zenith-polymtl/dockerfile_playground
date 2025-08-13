import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from zenmav.core import Zenmav
from geometry_msgs.msg import TwistStamped, PoseStamped
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class AbortBrake(Node):
    def __init__(self):
        super().__init__('abort_brake')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        qos_profile_BE = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        self.subscriber_ab = self.create_subscription(String, '/abort_brake', self.abort_brake_callback, qos_profile)
        self.publisher_ab_call = self.create_publisher(String, '/close', qos_profile)
        self.position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_BE)
        self.nav = Zenmav(ip = 'tcp:127.0.0.1:5762') # pour simu, parfois 5763 est dispo, mais pour brake, l'impression que 5762 est mieux
        #self.nav = Zenmav(ip = 'udp:127.0.0.1:14550') # pour test de vol

    def pose_callback(self,msg):
        self.curr_pos = msg.pose.position

    def abort_brake_callback(self, msg):
        if msg.data == "a.b.":
            msg_close = String()
            msg_close.data = "close"
            self.publisher_ab_call.publish(msg_close)
            self.get_logger().info(f'Nodes approach, target and graph successfully closed at {time.time():.3f} at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})!')

            self.get_logger().info(f'Brake incoming at {time.time():.3f} at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f}), {msg.data} received!')
            self.nav.set_mode('BRAKE')
            self.get_logger().info(f'Brake mode successfully enforced!')

            time.sleep(3)

            self.nav.set_mode('GUIDED')
            self.get_logger().info(f'Guided mode successfully enforced at {time.time():.3f}! at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})')

            AbortBrake().destroy_node()
            rclpy.shutdown()

        else:
            self.get_logger().info(f'Abort brake command ("a.b.") not recognized: {msg.data}')
        
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