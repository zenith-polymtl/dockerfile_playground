#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String, Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import time
from custom_interfaces.msg import TargetPosePolar
from mavros_msgs.msg import PositionTarget 
from mavros.cmd import CliClient
from mavros_msgs.srv import MessageInterval   

class PIDController():
    def __init__(self, kp, ki, kd, max_output = 3.0, max_i = 1):  # 3.0 m/s norm is max output for vel in xy directions vectorially
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0
        self.max_i = max_i

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        
        self.integral += error * dt
        # Anti-windup for integral term
        self.integral = max(min(self.integral, self.max_i), -self.max_i)
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Clamp output to max value
        return max(min(output, self.max_output), -self.max_output)



class ApproachNode(Node):
    def __init__(self):
        super().__init__("approach_node")

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

        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')  
          
        # Set up message intervals after a short delay  
        self.setup_timer = self.create_timer(1.0, self.setup_message_intervals)  

        self.publisher_raw = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', qos_profile)  

        self.drone_position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.drone_pose_callback, qos_profile_BE)
        self.drone_position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/velocity_local', self.drone_speed_callback, qos_profile_BE)
        self.pose_goal_sub = self.create_subscription(TargetPosePolar, '/goal_pose_polar', self.goal_pose_callback, qos_profile)
        self.estimated_target_sub = self.create_subscription(PoseStamped, '/estimated_target_location', self.estimation_callback, qos_profile)
        self.activation_sub = self.create_subscription(String, '/approach_activation', self.activation_callback, qos_profile)

        self.start_sub = self.create_subscription(  
            String,  
            '/controller_activation',  
            self.controller_callback,  
            qos_profile  
        )    

        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)

        # PD Controllers for XYZ pos control by try and retry sim analysis
        self.pid_r = PIDController(kp=1.5, ki=0.2, kd=1.5, max_i=1.0)
        self.pid_theta = PIDController(kp=0.6, ki=0, kd=0.24)
        self.pid_z = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_r_dot = PIDController(kp=3, ki=4, kd=1.0, max_i = 0.5)

        self.estimated_target_pose = None
        self.drone_pose = None
        self.target_pose = None
        self.last_time = None
        self.r_error = None
        self.z_error = None
        self.theta_error = None
        self.current_time = None
        self.last_time = None
        self.first = True
        self.r_ref = None
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0

        self.control_hz = 20
        #self.control_timer = self.create_timer(1.0/self.control_hz, self.send_commands)

        # ----- Radial deadband-hold state -----
        self.deadband_vr = 0.1  # [m/s]
        self.r_hold = None
        self.was_in_deadband = False
        self.prev_relative = None


        self.get_logger().info("Polar positioning node started")

    def setup_message_intervals(self):  
        """Set up message intervals after node initialization"""  
        if not self.msg_interval_client.wait_for_service(timeout_sec=1.0):  
            self.get_logger().warn('Message interval service not available, retrying...')  
            return  
          
        request = MessageInterval.Request()  
        request.message_id = 32  
        request.message_rate = 20.0  
          
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
    
    def controller_callback(self, msg):
        if msg.data == "stop":
            self.publish_zero()
            self.target_pose.v_r = 0.0
            self.target_pose.v_theta = 0.0
            self.target_pose.v_z = 0.0
            self.get_logger().info("Controller Deactivated, stopping targets follow")
    

    def activation_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info("Approach Activated")
            self.approach_active = True
        elif msg.data == "stop":
            self.get_logger().info("Approach Deactivated")
            self.publish_zero()
            self.approach_active = False
            # Reset controllers and state
            self.pid_r.integral = 0.0
            self.pid_r.prev_error = 0.0
            self.pid_theta.integral = 0.0
            self.pid_theta.prev_error = 0.0
            self.pid_z.integral = 0.0
            self.pid_z.prev_error = 0.0
            self.pid_r_dot.integral = 0.0
            self.pid_r_dot.prev_error = 0.0
            self.target_pose = None
            self.first = True
            self.r_hold = None
            self.was_in_deadband = False
            self.prev_relative = None
            self.estimated_target_pose = None
            self.estimated_target_pose = None
            self.last_time = None
            self.r_error = None
            self.z_error = None
            self.theta_error = None
            self.current_time = None
            self.last_time = None
            self.r_ref = None



    def compute_estimated_state(self):
        if not getattr(self, "approach_active", False):
            return

        if self.estimated_target_pose is None or self.drone_pose is None:
            return

        # --- Relative geometry (target → drone) ---
        delta_x = self.drone_pose.x - self.estimated_target_pose.x
        delta_y = self.drone_pose.y - self.estimated_target_pose.y
        distance_to_target = float(np.hypot(delta_x, delta_y))
        distance_safe = max(distance_to_target, 0.2)  # avoid divide-by-zero

        # --- Unit vectors in polar frame ---
        radial_unit_vector = np.array([delta_x / distance_safe,
                                    delta_y / distance_safe])  # points outward
        tangential_unit_vector = np.array([-radial_unit_vector[1],
                                            radial_unit_vector[0]])  # CCW tangent

        # --- Measured planar velocity (from /mavros/local_position/velocity_local) ---
        vel_x = getattr(self, "vel_x", 0.0)
        vel_y = getattr(self, "vel_y", 0.0)

        # --- Decompose velocity into radial/tangential components ---
        radial_speed_measured = vel_x * radial_unit_vector[0] + vel_y * radial_unit_vector[1]
        tangential_speed_measured = vel_x * tangential_unit_vector[0] + vel_y * tangential_unit_vector[1]

        # --- Pilot commands (make sure they use the same sign convention) ---
        radial_speed_command = getattr(self.target_speed, "v_r", 0.0)
        tangential_speed_command = getattr(self.target_speed, "v_theta", 0.0)

        # --- Tracking errors ---
        radial_speed_error = radial_speed_measured - radial_speed_command
        tangential_speed_error = tangential_speed_measured - tangential_speed_command

        # --- Save state for controller use ---
        self.distance_to_target = distance_to_target
        self.radial_unit_vector = radial_unit_vector
        self.tangential_unit_vector = tangential_unit_vector
        self.radial_speed_measured = radial_speed_measured
        self.tangential_speed_measured = tangential_speed_measured
        self.radial_speed_error = radial_speed_error
        self.tangential_speed_error = tangential_speed_error

        

    def compute_commands(self):
        pass
        
        

    def send_commands(self):  
        if self.estimated_target_pose is None or self.target_pose is None:
            return None
        # Create PositionTarget message for setpoint_raw  
        target = PositionTarget()  
        target.header.stamp = self.get_clock().now().to_msg()  
        target.header.frame_id = "map"  
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
          
        # Type mask to ignore position and acceleration, use only velocity and yaw  
        target.type_mask = (  
            PositionTarget.IGNORE_PX |  
            PositionTarget.IGNORE_PY |  
            PositionTarget.IGNORE_PZ |  
            PositionTarget.IGNORE_AFX |  
            PositionTarget.IGNORE_AFY |  
            PositionTarget.IGNORE_AFZ |  
            PositionTarget.IGNORE_YAW_RATE  # We want to control yaw angle, not yaw rate  
        )  
          
        # Set velocity components  
        target.velocity.x = self.vel_x  
        target.velocity.y = self.vel_y  
        target.velocity.z = self.vel_z  
          
        # Set yaw angle (convert from degrees to radians)  
        angle_towards_target_rad = np.arctan2(  
            self.estimated_target_pose.y - self.drone_pose.y,
            self.estimated_target_pose.x - self.drone_pose.x  
        )  
        target.yaw = angle_towards_target_rad  
          
        self.publisher_raw.publish(target)
        #self.get_logger().info(f"Published velocities: vx={self.vel_x:.2f}, vy={self.vel_y:.2f}, vz={self.vel_z:.2f}")

    def goal_pose_callback(self, msg):
        self.target_pose = msg
        self.target_pose.theta = (-msg.theta+90)/180*np.pi
        #self.get_logger().info(f"Received target pose: r={self.target_pose.r}, z={self.target_pose.z}, theta={self.target_pose.theta}, v_theta={self.target_pose.v_theta}, v_r={self.target_pose.v_r}, relative={self.target_pose.relative}")
        self.approach_active = True
        if self.prev_relative is None:
            self.prev_relative = self.target_pose.relative

    def estimation_callback(self, msg):
        self.estimated_target_pose = msg.pose.position

    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position
        if self.target_pose is not None and self.estimated_target_pose is not None:
            if self.last_time is None:
                self.last_time = time.time()
            else:
                self.compute_estimated_state()

    def drone_speed_callback(self, msg):
        self.drone_speed = msg.twist.linear

    def publish_zero(self):
        # One last zero-velocity setpoint
        target = PositionTarget()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        target.velocity.x = 0.0
        target.velocity.y = 0.0
        target.velocity.z = 0.0

        # Keep yaw stable if we can compute it; otherwise ignore yaw entirely.
        if self.estimated_target_pose is not None and self.drone_pose is not None:
            target.yaw = np.arctan2(
                self.estimated_target_pose.y - self.drone_pose.y,
                self.estimated_target_pose.x - self.drone_pose.x
            )
        else:
            target.type_mask |= PositionTarget.IGNORE_YAW

        self.publisher_raw.publish(target)
        self.get_logger().info("Published final ZERO velocity setpoint.")




def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
