import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class GoApproachPublisher(Node):
    def __init__(self):
        super().__init__('go_approach_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.manual_got_called = False # Control flag

        self.subscriber_ = self.create_subscription(String, '/manual', self.manual_callback, qos_profile)
        self.publisher_ = self.create_publisher(String, '/go_approach', 10)

        # Set timer period at start (in seconds)
        self.timer_period = 7.0
        self.target_1 = "0,0,15"
        self.target_2 = "11,6,18"
        self.target_3 = "-1,12,16"
        self.target_4 = "3,5,17"
        #self.target_1 = "0,0,15"
        #self.target_2 = "100,100,15"
        #self.target_3 = "0,0,15"
        #self.target_4 = "100,100,15"
        self.targets = [self.target_1, self.target_2, self.target_3, self.target_4]
        self.i = -1
        self.last_target = ""
    
        self.get_logger().info(f"Publisher initialized, will publish to '/go_approach' every {self.timer_period} seconds when approach start")

    def timer_callback(self):
        msg = String()
        self.i = (self.i + 1) % len(self.targets)      # liste et index circulaire
        msg.data = self.targets[self.i]

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
        self.publisher_.publish(msg)
        self.last_target = msg.data #
        if not hasattr(self, 'republish_timer'):
            self.republish_timer = self.create_timer(0.03, self.republish_target)
        self.get_logger().info(f'Published: "{msg.data}"')

    def republish_target(self):
        if self.last_target:
            msg = String()
            msg.data = self.last_target
            self.publisher_.publish(msg)

    def manual_callback(self, msg):
        if msg.data == "AUTO":
            self.get_logger().info(f'message AUTO received for target')
            self.manual_got_called = True
            self.timer_callback()
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
        if msg.data == "MANUAL":
            self.get_logger().info(f'message MANUAL received to stop target')
            self.manual_got_called = False
            if not hasattr(self, 'republish_timer'):
                self.republish_timer.cancel()
            self.timer.cancel()   # à tester
        else:
            self.get_logger().info(f'no good message received for target: {msg.data}')
        
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