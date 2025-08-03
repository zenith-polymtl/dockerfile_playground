import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GoApproachPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        #self.manual_got_called = False # Control flag

        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.publisher_target = self.create_publisher(String, '/go_target', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, qos_profile)

        self.timer_period_between_target_switch = 8.0   # à modifier à la guide des distances entre targets
        self.target_1 = "0,0,15"
        self.target_2 = "11,6,18"
        self.target_3 = "-1,12,16"
        self.target_4 = "3,5,17"

        self.targets = [self.target_1, self.target_2, self.target_3, self.target_4]
        self.i = 0
        self.last_target = ""
    
        self.get_logger().info(f"Publisher initialized, will publish to '/go_target' every {self.timer_period} seconds when approach start")

    def timer_callback(self):
        msg = String()
        self.tar = (self.i) % len(self.targets)      # liste et index circulaire
        msg.data = self.targets[self.tar]
        self.publisher_target.publish(msg)

        # Section de contrôle de changement de target et de print terminaux de celle-ci
        current_time = time.time()
        elapsed_change_target = current_time - self.last_record_time_ct
        elapsed_get_logger_target = current_time - self.last_record_time_glt

        if elapsed_change_target > self.timer_period_between_target_switch:
            self.i += 1
            self.last_record_time_ct = current_time

        """if self.i == 3:        # à effacer si lignes hauts fonctionnent
            msg.data = self.target_1
            self.i = 0
        elif self.i == 2:
            msg.data = self.target_2
            self.i += 1
        elif self.i == 1:
            msg.data = self.target_3
            self.i += 1
        elif self.i == 0:
            msg.data = self.target_4
            self.i += 1"""

        if elapsed_get_logger_target > 0.5:
            self.get_logger().info(f'Published target at "{self.Hertz}": "{msg.data}"')
            self.last_record_time_glt = current_time

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message {msg.data} received to start target')
            self.Hertz = 20 # à changer si voulu
            self.timer = self.create_timer(1/self.Hertz, self.timer_callback)

        else:
            self.get_logger().info(f'Error in message received for target, GO! to start : {msg.data}')

    def close_callback(self, msg):
        if msg.data == "close":
            self.destroy_node()
            rclpy.shutdown()

        
def main(args=None):
    rclpy.init(args=args)
    node = GoApproachPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()