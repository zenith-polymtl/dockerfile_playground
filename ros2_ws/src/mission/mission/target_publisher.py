import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GoApproachPublisher(Node):
    def __init__(self):
        super().__init__('go_approach_publisher')
        # Create a publisher on the /go_approach topic
        self.publisher_ = self.create_publisher(String, '/go_approach', 10)
        # Set timer period (in seconds)
        timer_period = 60.0  # publish every 1 second
        self.target_1 = "3,2,18"
        self.target_2 = "14,12,18"
        self.i = 0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"Publisher initialized, publishing to '/go_approach' every {timer_period} seconds")

    def timer_callback(self):
        msg = String()
        if self.i%2 == 0:
            msg.data = self.target_1
        else:
            msg.data = self.target_2
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.i += 1


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