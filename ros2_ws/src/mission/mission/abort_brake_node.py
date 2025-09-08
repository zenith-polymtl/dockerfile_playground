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
        
        self.get_logger().info(f"Zenmav main et ses ports splités vont s'initialiser")
        #self.nav = Zenmav(ip = 'tcp:127.0.0.1:5762', GCS = True, tcp_ports=[14553]) # pour simu : 5762 pour première instance Zenmav et 5763 pour Mavros
        self.nav = Zenmav(ip = 'tcp:127.0.0.1:5762', GCS = True, tcp_ports=[14553, 14554])
        #self.nav = Zenmav(ip = 'udp:127.0.0.1:14550', GCS = True, tcp_ports=[14553]) # pour test de vol
        self.get_logger().info(f'Zenmav main et ses ports splités sont connectés!')

        self.nav.guided_arm_takeoff(height = 10) # meters

    def pose_callback(self,msg):
        self.curr_pos = msg.pose.position

    def abort_brake_callback(self, msg):
        if msg.data == "a.b.":
            msg_close = String()
            msg_close.data = "close"
            self.publisher_ab_call.publish(msg_close)
            self.get_logger().info(f'Nodes approach/align, target/target_baselink and graph successfully closed at {time.time():.3f} at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})!')

            self.get_logger().info(f'Brake incoming at {time.time():.3f} at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f}), {msg.data} received!')
            self.nav.set_mode('BRAKE')
            self.get_logger().info(f'Brake mode successfully enforced!')

            time.sleep(3)

            self.get_logger().info(f'RTL mode successfully enforced at {time.time():.3f}! at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})')
            self.nav.RTL()
            self.get_logger().info(f'Land done at {time.time():.3f}! at ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})')

            time.sleep(1)

            self.destroy_node()
            time.sleep(1)
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