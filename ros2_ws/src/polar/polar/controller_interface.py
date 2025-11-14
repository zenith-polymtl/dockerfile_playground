#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from mavros_msgs.msg import RCIn  
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  
from std_msgs.msg import String
from custom_interfaces.msg import TargetPosePolar  
from mavros_msgs.srv import MessageInterval 

class RCChannelReader(Node):  
    def __init__(self):  
        super().__init__('rc_channel_reader')  
          
        # Use best effort QoS to match MAVROS sensor data  
        qos_profile = QoSProfile(  
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,  
            depth=10  
        )  

        qos_profile_RE = QoSProfile(  
            reliability=QoSReliabilityPolicy.RELIABLE,  
            history=QoSHistoryPolicy.KEEP_LAST,  
            depth=10  
        )  

        # Using MAVROS Python client  
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')  
          
        # Set up message intervals after a short delay  
        self.setup_timer = self.create_timer(1.0, self.setup_message_intervals)  

        self.declare_parameter('v_r_max', 1.5) 
        self.declare_parameter('v_theta_max', 2.0) 
        self.declare_parameter('v_z_max', 0.5)  
        self.declare_parameter('yaw_max', 3.14159/4)     # float
        self.declare_parameter('talk', True)   

        # --- Read parameter values ---
        self.v_r_max = self.get_parameter('v_r_max').get_parameter_value().double_value
        self.v_theta_max = self.get_parameter('v_theta_max').get_parameter_value().double_value
        self.v_z_max = self.get_parameter('v_z_max').get_parameter_value().double_value
        self.yaw_max = self.get_parameter('yaw_max').get_parameter_value().double_value
        self.talk = self.get_parameter('talk').get_parameter_value().bool_value
          
        # Subscribe to RC input channels  
        self.rc_sub = self.create_subscription(  
            RCIn,  
            '/mavros/rc/in',  
            self.rc_callback,  
            qos_profile  
        )

        self.start_sub = self.create_subscription(  
            String,  
            '/controller_activation',  
            self.start_callback,  
            qos_profile  
        )
        

        self.target_pub = self.create_publisher(  
            TargetPosePolar,  
            '/goal_pose_polar',  
            qos_profile_RE  
        )   

        self.activation_pub = self.create_publisher(
            String, '/approach_activation', qos_profile_RE
        ) 
          
        self.active = False
        self.last_active = False
        self.pitch, self.roll, self.yaw, self.throttle = None, None, None, None
        self.get_logger().info("RC Channel Reader started")  

    def setup_message_intervals(self):
        if True:
            """Set up message intervals after node initialization"""  
            if not self.msg_interval_client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn('Message interval service not available, aborting request...')  
                self.destroy_timer(self.setup_timer) 
                return  
            
            request = MessageInterval.Request()  
            request.message_id = 65  
            request.message_rate = 25.0  
            
            future = self.msg_interval_client.call_async(request)  
            future.add_done_callback(self.message_interval_callback)  
            
        # Destroy the timer since we only need to run this once  
        self.destroy_timer(self.setup_timer) 

    def message_interval_callback(self, future):  
        try:  
            response = future.result()  
            if response.success:  
                self.get_logger().info("Message interval set successfully")  
            else:  
                self.get_logger().error("Failed to set message interval")  
        except Exception as e:  
            self.get_logger().error(f"Service call failed: {e}") 

    def publish_target(self):
        msg = TargetPosePolar()
        msg.v_r = self.v_r_max * self.pitch
        msg.v_theta = -self.v_theta_max * self.roll
        msg.v_z = self.v_z_max * self.throttle
        msg.yaw_rate = self.yaw_max*self.yaw
        msg.relative = True
        self.target_pub.publish(msg)

    def start_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info("Controller Activated")
            self.active = True
        elif msg.data == "stop":
            self.get_logger().info("Controller Deactivated")
            self.active = False
      
    def rc_callback(self, msg):  
        """  
        Extract roll, pitch, throttle from RC channels.  
        Standard RC channel mapping (can vary by transmitter):  
        - Channel 1: Roll (aileron)  
        - Channel 2: Pitch (elevator)   
        - Channel 3: Throttle  
        - Channel 4: Yaw (rudder)  
        """
         
        if len(msg.channels) >= 4:  
            roll_raw = msg.channels[0]      # Channel 1  
            pitch_raw = msg.channels[1]     # Channel 2    
            throttle_raw = msg.channels[2]  # Channel 3  
            yaw_raw = msg.channels[3]       # Channel 4  
            activation = msg.channels[6]
            if activation > 1700:
                self.active = True
            elif activation < 1300:
                self.active = False
              
            # Convert PWM values (typically 1000-2000) to normalized values (-1 to 1 for roll/pitch/throttle)  
            self.roll = self.pwm_to_normalized(roll_raw)  
            self.pitch = self.pwm_to_normalized(pitch_raw)  
            self.throttle = self.pwm_to_normalized(throttle_raw)  
            self.yaw = self.pwm_to_normalized(yaw_raw)

            if self.talk:  
                self.get_logger().info(  
                    f"Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}, Throttle: {self.throttle:.3f} "  
                    f"(Raw: {roll_raw}, {pitch_raw}, {throttle_raw})"  
                    f"(Activation pwm : {activation}"  
                )

            if self.active and not self.last_active:
                self.start_position()
            elif not self.active and self.last_active:
                self.close_system()

            self.last_active = self.active 

            if not self.active:
                return

            self.publish_target()
        else:  
            self.get_logger().warn(f"Insufficient RC channels: {len(msg.channels)}")

    def close_system(self):
        msg = String()
        msg.data = 'stop'
        self.activation_pub.publish(msg)

    def start_position(self):
        msg = String()
        msg.data = 'start'
        self.activation_pub.publish(msg)
      
    def pwm_to_normalized(self, pwm_value, center=1500, deadband=50):  
        """Convert PWM value to normalized range [-1, 1] with center at 1500"""  
        if abs(pwm_value - center) < deadband:  
            return 0.0  
        return (pwm_value - center) / 500.0  
  
def main(args=None):  
    rclpy.init(args=args)  
    node = RCChannelReader()  
      
    try:  
        rclpy.spin(node)  
    except KeyboardInterrupt:  
        pass  
    finally:  
        node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()