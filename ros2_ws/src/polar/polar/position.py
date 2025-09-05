#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from zenmav.core import Zenmav
import numpy as np
import time


class PIDController():
    def __init__(self, kp, ki, kd, max_output = 3.0):  # 3.0 m/s norm is max output for vel in xy directions vectorially
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
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        qos_profile_BE = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        self.publisher_vel = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)
        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.subscriber_gt = self.create_subscription(String, '/go_target', self.go_target_callback, qos_profile)
        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)
        self.position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_BE)

        # PD Controllers for XYZ pos control by try and retry sim analysis
        self.pid_x = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_y = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_z = PIDController(kp=0.73, ki=0, kd=0.3)

        # PID Controllers for XYZ pos control by Ziegler-Nichols Method       [pas fini]
        """self.pid_x = PIDController(kp=0.6, ki=0, kd=0.0)
        self.pid_y = PIDController(kp=0.6, ki=0, kd=0.0)
        self.pid_z = PIDController(kp=0.73, ki=0, kd=0.0)"""

        # NO PID for XYZ pos control
        """self.pid_x = PIDController(kp=1, ki=0, kd=0)
        self.pid_y = PIDController(kp=1, ki=0, kd=0)
        self.pid_z = PIDController(kp=1, ki=0, kd=0)"""

        # PID TEST DE VOL for XYZ pos control                [non modif encore en test]
        """self.pid_x = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_y = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_z = PIDController(kp=0.73, ki=0, kd=0.3)"""


    def pose_callback(self, msg):
        self.curr_pos = msg.pose.position
        current_time = time.time()
        temps = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(current_time))


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
        
        max_output = self.pid_x.max_output
        vel_x, vel_y = self.Failsafe_max_vel(vel_x,vel_y, max_output)

        twist = TwistStamped()
        twist.twist.linear.x = vel_x
        twist.twist.linear.y = vel_y
        twist.twist.linear.z = vel_z

        self.publisher_vel.publish(twist)

        current_time = time.time()
        if current_time - self.last_log_time_control >= 0.5:
            self.get_logger().info(f"PID velocities - X: {vel_x:.4f}, Y: {vel_y:.4f}, Z: {vel_z:.4f} at {current_time}")
            self.last_log_time_control = current_time



def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
