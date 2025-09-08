import rclpy  
from rclpy.node import Node  
from mavros_msgs.msg import AttitudeTarget
from std_msgs.msg import Float32
import math  
from zenmav.core import Zenmav
class AbsoluteYawController(Node):  
    def __init__(self):  
        super().__init__('absolute_yaw_controller')  
          
        # Declare and set the thrust_scaling parameter  
        self.declare_parameter('thrust_scaling', 1.0)  
          
        self.attitude_pub = self.create_publisher(  
            AttitudeTarget,   
            '/mavros/setpoint_raw/attitude',   
            10  
        )  
        
        self.yaw_pub = self.create_publisher(Float32, '/yaw_target', self.publish_yaw_target, 10)
        self.timer = self.create_timer(0.1, self.publish_yaw_target)  


        self.drone = Zenmav('tcp:127.0.0.1:5762')
        self.get_logger().info("Yaw targets initialized.")
          
    def publish_yaw_target(self, msg):  
        self.drone.yaw_target(float(msg.data))  # Convert to float if necessary
  
def main(args=None):  
    rclpy.init(args=args)  
    controller = AbsoluteYawController()  
      
    try:  
        rclpy.spin(controller)  
    except KeyboardInterrupt:  
        pass  
    finally:  
        controller.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()