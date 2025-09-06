#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from zenmav.core import Zenmav
import numpy as np
import time


class PIDController():
    def __init__(self, kp, ki, kd, max_output = 3.0):  # Une norme de 3.0 m/s est le max pour vitesses FL envoyées au drone
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
    
class target_baselink():
    def __init__(self, F, L, U, yaw_b):
        self.F = F
        self.L = L
        self.U = U
        self.yaw_b = yaw_b

class AlignNode(Node):
    def __init__(self):
        super().__init__("align_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=8
        )

        self.publisher_baselink_raw = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/local', qos_profile)
        self.subscriber_ab_call = self.create_subscription(String, '/close', self.close_callback, 10)
        self.subscriber_atg = self.create_subscription(String, '/approach_target_graph', self.atg_callback, qos_profile)
        self.subscriber_gt = self.create_subscription(String, '/go_target_baselink', self.go_target_baselink_callback, qos_profile)
        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)

        # PD Controllers for FLU pos control by try and retry sim analysis
        self.pid_F = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_L = PIDController(kp=0.6, ki=0, kd=0.3)
        self.pid_U = PIDController(kp=0.73, ki=0, kd=0.3)

        self.last_log_time_pose = 0.0
        self.last_log_time_control = 0.0
        self.atg_got_called = False
        self.align_active = False
        self.last_time = self.get_clock().now()
        self.der_target_recu = " "
        self.i = 0
        self.before_F, self.before_L, self.before_U, self.before_yaw_b = None, None, None, None

        self.Hertz_control = 30
        self.timer = self.create_timer(1/self.Hertz_control, self.control_loop)

    def go_target_baselink_callback(self, msg):
        if self.atg_got_called == True:
            self.align_active = True
            F, L, U, yaw_b = msg.data.split(",")

            self.der_target_time_recu = time.time()
            if not hasattr(self, 'timer_target'):
                Hertz = 15
                self.timer_target = self.create_timer(1/Hertz, self.Failsafe_target_acquired)

            self.target_baselink_pos = target_baselink(float(F), float(L), float(U), float(yaw_b))
            self.Failsafe_target_too_far()

            if not F == self.before_F or not L == self.before_L or not U == self.before_U or not yaw_b == self.before_yaw_b:
                self.i += 1
                self.before_F, self.before_L, self.before_U, self.before_yaw_b = F, L, U, yaw_b

            temps = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            self.get_logger().info(f"target {self.i} : {self.target_baselink_pos.F, self.target_baselink_pos.L, self.target_baselink_pos.U}, yaw_b : {(self.target_baselink_pos.yaw*180/np.pi):.3f} received at {temps}")
    
    def Failsafe_target_too_far(self):
        if self.der_target_recu != " ":
            diff_F = self.target_baselink_pos.F
            diff_L = self.target_baselink_pos.L
            diff_U = self.target_baselink_pos.U
            norme = (diff_F**2 + diff_L**2 + diff_U**2)**(1/2)
            if norme >= 12.0: # mètres d'écart vers target_baselink
                msg = String()
                msg.data = "a.b."
                self.get_logger().warn(f"Failsafe triggered: Target baselink received is way too far : {norme:.3f} mètres. Switching to BRAKE mode.")
                self.get_logger().warn(f"diff_F: {diff_F}")
                self.get_logger().warn(f"diff_L: {diff_L}")
                self.get_logger().warn(f"diff_U: {diff_U}")
                self.abort_state_pub.publish(msg)

        self.der_target_recu = self.target_baselink_pos

    def Failsafe_target_acquired(self):
        if hasattr(self, 'der_target_time_recu'):
            elapsed = time.time() - self.der_target_time_recu
            max_time_without_target = 3 # secondes # on vise entre 0.2 et 0.8?
            if elapsed >= max_time_without_target:
                self.get_logger().warn(f"Failsafe triggered: No target received in {max_time_without_target}s. Switching to BRAKE mode.")
                msg = String()
                msg.data = "a.b."
                self.abort_state_pub.publish(msg)

    def atg_callback(self, msg):
        if msg.data == "GO!":
            self.get_logger().info(f'message GO! received, align starting')
            self.atg_got_called = True

    def control_loop(self):
        if not self.align_active or self.target_baselink_pos is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9  # Convert nanoseconds to seconds
        self.last_time = now

        error_F = self.target_baselink_pos.F
        error_L = self.target_baselink_pos.L
        error_U = self.target_baselink_pos.U

        vel_F = self.pid_x.compute(error_F, dt)
        vel_L = self.pid_y.compute(error_L, dt)
        vel_U = self.pid_z.compute(error_U, dt)
        
        max_output = self.pid_F.max_output
        vel_F, vel_L = self.Failsafe_max_vel(vel_F,vel_L, max_output)

        PosTar = PositionTarget()
        PosTar.coordinate_frame = PositionTarget.FRAME_BODY_NED
        PosTar.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                PositionTarget.IGNORE_YAW_RATE
        #PositionTarget.IGNORE_YAW | 

        PosTar.velocity.F = vel_F
        PosTar.velocity.L = vel_L
        PosTar.velocity.U = vel_U
        PosTar.yaw = self.target_baselink_pos.yaw_b # radians

        self.publisher_baselink_raw.publish(PosTar)

        # Printing velocities and yaw
        current_time = time.time()
        if current_time - self.last_log_time_control >= 0.5:
            self.get_logger().info(f"PID velocities & yaw_b - F: {vel_F:.3f}, L: {vel_L:.3f}, U: {vel_U:.3f} with YAW_B: {(self.target_baselink_pos.yaw*180/np.pi):.3f}° at {current_time:.2f}")
            self.last_log_time_control = current_time

    def Failsafe_max_vel(self, vel_F,vel_L, max_output):
        eps = 0.001

        if (vel_F**2 + vel_L**2)**(1/2) >= max_output:
            hyp = (vel_F**2 + vel_L**2)**(1/2)
            theta = np.arccos(vel_F/hyp)
            vel_F = (max_output-eps) * np.cos(theta)
            vel_L = (max_output-eps) * np.sin(theta) * np.sign(vel_L)

        return vel_F, vel_L

    def close_callback(self, msg):
        if msg.data == "close":
            self.destroy_node()
            rclpy.shutdown()   

def main(args=None):
    rclpy.init(args=args)
    node = AlignNode()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()
