#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from zenmav.core import Zenmav
import numpy as np
import time
from custom_interfaces.msg import TargetPosePolar

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

        self.drone_position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.drone_pose_callback, qos_profile_BE)
        self.pose_goal_sub = self.create_subscription(TargetPosePolar, '/goal_pose_polar', self.goal_pose_callback, qos_profile)
        self.estimated_target_sub = self.create_subscription(PoseStamped, '/estimated_target_location', self.estimation_callback, qos_profile)

        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10) 

        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)

        # PD Controllers for XYZ pos control by try and retry sim analysis
        self.pid_r = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_theta = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_z = PIDController(kp=0.6, ki=0, kd=0.3)


        self.get_logger().info("Polar positionning node started")

    def compute_estimated_state(self):
        if self.estimated_target_pose is None:
            return None

        delta_x = self.estimated_target_pose.x - self.drone_pose.x
        delta_y = self.estimated_target_pose.y - self.drone_pose.y
        delta_z = self.estimated_target_pose.z - self.drone_pose.z


        self.z_error = delta_z - self.target_pose.z

        distance_from_target = np.sqrt(delta_x**2 + delta_y**2 + delta_z**2)
        if self.target_pose.relative:
            self.r_error = distance_from_target*(1 - self.target_pose.r_percent) #r+ is radial in
            self.v_theta = self.target_pose.v_theta
        else:
            self.r_error = distance_from_target - self.target_pose.r #r+ is radial in
            self.theta_error = np.arctan2(delta_y, delta_x) - self.target_pose.theta
            self.v_theta = None

        self.unit_vector_to_target = np.array([delta_x, delta_y]) / distance_from_target if distance_from_target != 0 else np.array([0.0, 0.0, 0.0])

        self.compute_commands()

    def compute_commands(self):
        if self.r_error is None or self.z_error is None:
            return None

        vel_r = self.pid_r.compute(self.r_error, self.dt)
        self.vel_z = self.pid_z.compute(self.z_error, self.dt)

        current_time = time.time()
        dt = self.last_time - self.current_time
        self.last_time = current_time

        if not self.target_pose.relative:

            theta_distance = self.theta_error*self.r_error
            self.vel_theta = self.pid_theta.compute(theta_distance, dt)

        self.vel_z = self.pid_z.compute(self.z_error, dt)
        self.vel_r = self.pid_r.compute(self.r_error, dt)

        self.vel_rx, self.vel_ry = vel_r*self.unit_vector_to_target
        self.ver_thetax, self.vel_thetay = self.vel_theta*np.array([-self.unit_vector_to_target[1], self.unit_vector_to_target[0]])

        self.vel_x = self.vel_rx + self.ver_thetax
        self.vel_y = self.vel_ry + self.ver_thetay
        self.send_commands()

    def send_commands(self):
        twist = TwistStamped()
        twist.twist.linear.x = self.vel_x
        twist.twist.linear.y = self.vel_y
        twist.twist.linear.z = self.vel_z

        self.publisher_vel.publish(twist)

    def goal_pose_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(f"Received target pose: r={self.target_pose.r}, z={self.target_pose.z}, theta={self.target_pose.theta}, v_theta={self.target_pose.v_theta}, r_percent={self.target_pose.r_percent}, relative={self.target_pose.relative}")
        self.approach_active = True

    def estimation_callback(self, msg):
        self.estimated_target_pose = msg.pose.position

    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position


    def control_loop(self):
        if not self.approach_active or self.drone_pose is None or self.target_pose is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = now

        error_x = self.target_pose.x - self.drone_pose.x
        error_y = self.target_pose.y - self.drone_pose.y
        error_z = self.target_pose.z - self.drone_pose.z

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
