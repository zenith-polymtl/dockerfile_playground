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
        # Set timer period (in seconds)
        self.timer_period = 8.0
        self.target_1 = "0,0,15"
        self.target_2 = "11,6,18"
        self.target_3 = "-1,12,16"
        self.target_4 = "3,5,17"
        self.i = 0    
        self.get_logger().info(f"Publisher initialized, publishing to '/go_approach' every {self.timer_period} seconds")

    def timer_callback(self):
        msg = String()
        if self.i == 3:
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
            self.i += 1
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
    
    def manual_callback(self, msg):
        if msg == "AUTO":
            self.get_logger().info(f'NYESSSSSSSSSSSSSSSSSSSS AUTO')
            self.manual_got_called = True
            self.timer = self.create_timer(self.timer_period, self.timer_callback)
        else:
            self.get_logger().info(f'NOOOOOOOOOOOOOOOOOOOO')
        
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