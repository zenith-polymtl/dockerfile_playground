#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
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


        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)

        # PD Controllers for XYZ pos control by try and retry sim analysis
        self.pid_r = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_theta = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_z = PIDController(kp=0.6, ki=0, kd=0.3)

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


        

        self.get_logger().info("Polar positioning node started")

    def ref_timer_callback(self):
        self.r_ref *= self.target_pose.r_percent


    def compute_estimated_state(self):
        self.get_logger().info(f"Computing states")
        

        if self.estimated_target_pose is None:
            self.get_logger().info("No estimated target pose available")
            return None

        delta_x = self.estimated_target_pose.x - self.drone_pose.x
        delta_y = self.estimated_target_pose.y - self.drone_pose.y
        delta_z = self.estimated_target_pose.z - self.drone_pose.z


        self.z_error = delta_z + float(self.target_pose.z)

        distance_from_target = np.sqrt(delta_x**2 + delta_y**2)
        if self.first:
            self.first = False
            self.r_ref = distance_from_target
            self.ref_time = self.create_timer(0.5, self.ref_timer_callback)

        self.distance_from_target = distance_from_target

        
        if self.target_pose.relative:
            if self.r_ref is None:
                return
            self.r_error = distance_from_target - self.r_ref #r+ is radial in
            self.v_theta = self.target_pose.v_theta
        else:
            self.r_error = distance_from_target - self.target_pose.r #r+ is radial in
            self.theta_error = np.arctan2(-delta_y, -delta_x) - self.target_pose.theta
            self.v_theta = None

        self.unit_vector_to_target = np.array([delta_x, delta_y]) / distance_from_target if distance_from_target != 0 else np.array([0.0, 0.0])

        self.compute_commands()

    def compute_commands(self):

        self.get_logger().info(f"Compting commands")
        
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        self.vel_z = self.pid_z.compute(self.z_error, dt)
        self.vel_r = self.pid_r.compute(self.r_error, dt)

        #If not relative control, compute theta velocity using pid
        if not self.target_pose.relative:
            theta_distance = self.theta_error*self.r_error
            self.v_theta = self.pid_theta.compute(theta_distance, dt)



        self.vel_rx, self.vel_ry = self.vel_r*self.unit_vector_to_target
        self.vel_thetax, self.vel_thetay = self.v_theta*np.array([-self.unit_vector_to_target[1], self.unit_vector_to_target[0]])

        self.vel_x = self.vel_rx + self.vel_thetax
        self.vel_y = self.vel_ry + self.vel_thetay
        self.send_commands()

    def send_commands(self):
        twist = TwistStamped()
        twist.twist.linear.x = self.vel_x
        twist.twist.linear.y = self.vel_y
        twist.twist.linear.z = self.vel_z

        self.publisher_vel.publish(twist)
        self.get_logger().info(f"Published velocities: vx={self.vel_x:.2f}, vy={self.vel_y:.2f}, vz={self.vel_z:.2f}")

    def goal_pose_callback(self, msg):
        self.target_pose = msg
        self.get_logger().info(f"Received target pose: r={self.target_pose.r}, z={self.target_pose.z}, theta={self.target_pose.theta}, v_theta={self.target_pose.v_theta}, r_percent={self.target_pose.r_percent}, relative={self.target_pose.relative}")
        self.approach_active = True

    def estimation_callback(self, msg):
        self.estimated_target_pose = msg.pose.position
        self.get_logger().info(f"Received estimated target pose: x={self.estimated_target_pose.x}, y={self.estimated_target_pose.y}, z={self.estimated_target_pose.z}")

    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position
        if self.target_pose is not None and self.estimated_target_pose is not None:
            if self.last_time is None:
                self.last_time = time.time()
            else:
                self.compute_estimated_state()



def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
