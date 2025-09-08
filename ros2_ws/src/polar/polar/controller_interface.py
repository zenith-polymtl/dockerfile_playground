#!/usr/bin/env python3  
  
import rclpy  
from rclpy.node import Node  
from mavros_msgs.msg import RCIn  
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  
from std_msgs.msg import String
from custom_interfaces.msg import TargetPosePolar

class RCChannelReader(Node):  
    def __init__(self):  
        super().__init__('rc_channel_reader')  
          
        # Use best effort QoS to match MAVROS sensor data  
        qos_profile = QoSProfile(  
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  
            history=QoSHistoryPolicy.KEEP_LAST,  
            depth=10  
        )  
          
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

        self.target_pub = self.create_subscription(  
            TargetPosePolar,  
            '/goal_pose_polar',  
            qos_profile  
        )    
          
        self.get_logger().info("RC Channel Reader started")  
        self.active = False
        hz = 20
        self.pub_timer = self.create_timer(1/hz, self.publish_target)

    def publish_target(self):
        if not self.active:
            return
        msg = TargetPosePolar()
        v_r_max = 1.0
        msg.v_r = v_r_max * self.pitch
        max_v_theta = 2.0
        msg.v_theta = max_v_theta * self.roll
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
              
            # Convert PWM values (typically 1000-2000) to normalized values (-1 to 1 for roll/pitch, 0 to 1 for throttle)  
            self.roll = self.pwm_to_normalized(roll_raw)  
            self.pitch = self.pwm_to_normalized(pitch_raw)  
            self.throttle = self.pwm_to_throttle(throttle_raw)  
              
            self.get_logger().info(  
                f"Roll: {self.roll:.3f}, Pitch: {self.pitch:.3f}, Throttle: {self.throttle:.3f} "  
                f"(Raw: {roll_raw}, {pitch_raw}, {throttle_raw})"  
            )  
        else:  
            self.get_logger().warn(f"Insufficient RC channels: {len(msg.channels)}")  
      
    def pwm_to_normalized(self, pwm_value, center=1500, deadband=50):  
        """Convert PWM value to normalized range [-1, 1] with center at 1500"""  
        if abs(pwm_value - center) < deadband:  
            return 0.0  
        return (pwm_value - center) / 500.0  
      
    def pwm_to_throttle(self, pwm_value, min_pwm=1000, max_pwm=2000):  
        """Convert PWM value to throttle range [0, 1]"""  
        return max(0.0, min(1.0, (pwm_value - min_pwm) / (max_pwm - min_pwm)))  
  
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