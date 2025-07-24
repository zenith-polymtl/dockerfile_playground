#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Ilyes est trop cool
class PIDController:
    def __init__(self, kp, ki, kd, max_output=2.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, error, dt):
        if dt <= 0:
            return 0.0
        
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Clamp output to max value
        return max(min(output, self.max_output), -self.max_output)
    
class target():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z =z

class ApproachNode(Node):
    def __init__(self):
        super().__init__("approach_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        self.subscriber_ = self.create_subscription(String, 'go_approach', self.go_approach_callback, qos_profile)
        self.position_sub = self.create_subscription(PoseStamped, '/ap/pose/filtered', self.local_position_callback, qos_profile)

        self.get_logger().info("Approach node initialized")

        # PID Controllers for XYZ velocity control
        self.pid_x = PIDController(kp=0.7, ki=0.00, kd=0.06)
        self.pid_y = PIDController(kp=0.7, ki=0.00, kd=0.06)
        self.pid_z = PIDController(kp=0.7, ki=0.00, kd=0.06)

        self.curr_pos = None
        self.approach_active = False  # Control flag
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz loop

    """def go_approach_callback(self, msg):
        if msg.status == "Intermediate":
            if self.curr_pos: 
                self.approach_active = True
                self.target_pos = target(msg.x, msg.y, msg.z)
                self.get_logger().info("Approach PID activated. Holding position.")
            else:
                self.get_logger().warn("No position data received yet!")
        elif  msg.status == "Final": #Currently same exact logic is used for both status, as an example , change as needed
            if self.curr_pos: 
                self.approach_active = True
                self.target_pos = target(msg.x, msg.y, msg.z)
                self.get_logger().info("Approach PID activated. Holding position.")
            else:
                self.get_logger().warn("No position data received yet!")"""

    def go_approach_callback(self, msg):
        if self.curr_pos: 
            self.approach_active = True
            x, y, z = msg.data.split(",")
            self.target_pos = target(x, y, z)
            # Pour éviter control, juste recoit un message publié sur le topic qui part other_approach
            self.get_logger().info("Approach PID activated. Holding position.")
        else:
            self.get_logger().warn("No position data received yet!")

    def local_position_callback(self, msg):
        self.curr_pos = msg.pose.position
        self.get_logger().info(f"Current position : ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})")

    def control_loop(self):
        if not self.approach_active or self.curr_pos is None or self.target_pos is None:
            return
        
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = now

        error_x = self.target_pos.x - self.curr_pos.x
        error_y = self.target_pos.y - self.curr_pos.y
        error_z = self.target_pos.z - self.curr_pos.z

        vel_x = self.pid_x.compute(error_x, dt)
        vel_y = self.pid_y.compute(error_y, dt)
        vel_z = self.pid_z.compute(error_z, dt)

        twist = TwistStamped()
        twist.twist.linear.x = vel_x
        twist.twist.linear.y = vel_y
        twist.twist.linear.z = vel_z

        self.publisher_.publish(twist)
        self.get_logger().info(f"PID velocities - X: {vel_x}, Y: {vel_y}, Z: {vel_z}")

def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
