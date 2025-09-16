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
    def __init__(self, kp, ki, kd, max_output = 3.00):  # max_output est une norme du max de vitesse, 3.00 m/s par défaut
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
        self.F = F # m
        self.L = L # m
        self.U = U # m
        self.yaw_b = yaw_b # deg

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
        self.pid_yaw_b = PIDController(kp=-0.2, ki=0.0, kd=0.0, max_output= np.pi) # Limiting yaw_rate

        self.last_log_time_pose = 0.0
        self.atg_got_called, self.align_active = False, False
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
            yaw_b = np.deg2rad(float(yaw_b)) # yaw_b en radians! 

            self.der_target_time_recu = time.time()
            if not hasattr(self, 'timer_target'):
                Hertz = 15
                self.timer_target = self.create_timer(1/Hertz, self.Failsafe_target_acquired)

            self.target_baselink_pos = target_baselink(float(F), float(L), float(U), float(yaw_b))
            self.Failsafe_target_too_far()
            self.Failsafe_yaw_too_big()

            temps = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
            self.get_logger().info(f"target #{self.i} [FLU]: {self.target_baselink_pos.F:.3f}, {self.target_baselink_pos.L:.3f}, {self.target_baselink_pos.U:.3f} & yaw_b : {(self.target_baselink_pos.yaw_b*180/np.pi):.3f} deg ; received at {temps}")
            self.i += 1
    
    def Failsafe_yaw_too_big(self):
        if abs(self.target_baselink_pos.yaw_b) > 181: # degrés
            msg = String()
            msg.data = "a.b."
            self.get_logger().warn(f"Failsafe triggered: Target yaw baselink received is way too big : {self.target_baselink_pos.yaw_b:.3f} degrés. Switching to BRAKE mode.")
            self.abort_state_pub.publish(msg)

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
                self.get_logger().warn(f"diff_F: {diff_F} ; diff_L: {diff_L} ; diff_U: {diff_U}")
                self.abort_state_pub.publish(msg)

        self.der_target_recu = self.target_baselink_pos

    def Failsafe_target_acquired(self):
        if hasattr(self, 'der_target_time_recu'):
            elapsed = time.time() - self.der_target_time_recu
            max_time_without_target = 3 # secondes
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
        error_yaw_b = self.target_baselink_pos.yaw_b # déjà en radians

        vel_F = self.pid_F.compute(error_F, dt)
        vel_L = self.pid_L.compute(error_L, dt)
        vel_U = self.pid_U.compute(error_U, dt)
        vel_yaw_b = self.pid_yaw_b.compute(error_yaw_b, dt)
        
        max_output = self.pid_F.max_output
        vel_F, vel_L = self.Failsafe_max_vel(vel_F,vel_L, max_output)
        vel_F, vel_L, vel_U, vel_yaw_b = self.Failsafe_min_vel(vel_F, vel_L, vel_U, vel_yaw_b)

        PosTar = PositionTarget()
        PosTar.coordinate_frame = PositionTarget.FRAME_BODY_NED
        PosTar.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                PositionTarget.IGNORE_YAW
        # PositionTarget.IGNORE_YAW_RATE | PositionTarget.IGNORE_YAW

        PosTar.velocity.x = vel_F # Forward m/s
        PosTar.velocity.y = -vel_L # Right m/s
        PosTar.velocity.z = vel_U # m/s
        PosTar.yaw_rate = 0.00 #
        #PosTar.yaw_rate = vel_yaw_b # rad/s
        #PosTar.yaw = (self.target_baselink_pos.yaw_b)*np.pi/180 # radians

        self.publisher_baselink_raw.publish(PosTar)

        # Printing velocities
        temps = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time()))
        self.get_logger().info(f"PID velocities - F: {vel_F:.3f} m/s, L: {vel_L:.3f} m/s, U: {vel_U:.3f} m/s, YAW_rate_bl: {(vel_yaw_b*180/np.pi):.3f} deg/s at {temps}")

    def Failsafe_min_vel(self, vel_F, vel_L, vel_U, vel_yaw_b):
        eps = 0.01
        list = [vel_F, vel_L, vel_U, vel_yaw_b]

        for i in range(len(list)):
            if abs(list[i]) <= eps:
                list[i] = 0.0

        return tuple(list)

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
