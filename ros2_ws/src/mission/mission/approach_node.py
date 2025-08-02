#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mavros_msgs.srv import SetMode
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from zenmav.core import Zenmav
import numpy as np
import time


class PIDController:
    def __init__(self, kp, ki, kd, max_output=4.0):  # 4m/s max
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
        self.timer2 = None

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_profile)
        self.subscriber_man = self.create_subscription(String, '/manual', self.manual_callback, qos_profile)
        self.subscriber_go = self.create_subscription(String, '/go_approach', self.go_approach_callback, qos_profile)
        self.subscriber_abort = self.create_subscription(String, '/abort_state', self.abort_callback, qos_profile)
        self.position_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.local_position_callback, qos_profile)
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /mavros/set_mode service...')

        self.set_guided_mode()
        self.get_logger().info("Approach node initialized")

        ###############################################################################################

        # PD Controllers for XYZ pos control by try and retry
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

        # PID TEST DE VOL for XYZ pos control                [non set]
        """self.pid_x = PIDController(kp=0, ki=0, kd=0)
        self.pid_y = PIDController(kp=0, ki=0, kd=0)
        self.pid_z = PIDController(kp=0, ki=0, kd=0)"""

        ###############################################################################################

        self.curr_pos = None
        self.last_log_time = 0.0  # Initialisation du dernier temps de log
        self.manual_got_called = False # Control flag
        self.approach_active = False  # Control flag
        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz loop

        self.nav = Zenmav(ip = 'tcp:127.0.0.1:5763')

    def abort_callback(self, msg):
        self.nav.set_mode('BRAKE')
        self.destroy_node()
        rclpy.shutdown()


    def go_approach_callback(self, msg):
        if self.manual_got_called == True:
            if self.curr_pos: 
                self.approach_active = True
                x, y, z = msg.data.split(",")

                self.der_target_time_recu = time.time()
                if not hasattr(self, 'timer_target'):
                    self.timer_target = self.create_timer(0.04, self.Failsafe_target_acquired)

                self.target_pos = target(float(x), float(y), float(z))
                self.get_logger().info("Approach PID activated. Holding position.")
            else:
                self.get_logger().warn("No position data received yet!")
    
    def Failsafe_target_acquired(self):
        if hasattr(self, 'der_target_time_recu'):
            elapsed = time.time() - self.der_target_time_recu
            if elapsed >= 10.4:
                self.get_logger().warn("Failsafe triggered: No target received in 0.2s. Switching to BRAKE mode.")
                self.set_brake_mode()
                self.timer_target.cancel()  # Arrête le timer une fois le failsafe déclenché
                del self.timer_target  # Nettoie l’attribut pour permettre une relance plus tard

    def manual_callback(self, msg):
        if msg.data == "AUTO":
            self.get_logger().info(f'message AUTO received for approach')
            self.manual_got_called = True
        if msg.data == "MANUAL":
            self.get_logger().info(f'message MANUAL received to stop approach')
            self.manual_got_called = False

    def local_position_callback(self, msg):
        self.curr_pos = msg.pose.position
        current_time = time.time()
        if current_time - self.last_log_time >= 2:
            self.get_logger().info(f"Current position : ({self.curr_pos.x:.3f}, {self.curr_pos.y:.3f}, {self.curr_pos.z:.3f})")
            self.last_log_time = current_time

    def set_brake_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'BRAKE'
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.mode_response_callback)

    def set_guided_mode(self):
        req = SetMode.Request()
        req.custom_mode = 'GUIDED'
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.mode_response_callback)

    def mode_response_callback(self, future):
        try:
            response = future.result()
            if response.mode_sent:

                current_time = time.time()
                if current_time - self.last_log_time >= 1:
                    self.get_logger().info('Mode activated successfully.')
                    self.last_log_time = current_time
                    self.destroy_timer(self.timer2)
            else:
                self.get_logger().warn('Failed to activate mode.')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def control_loop(self):
        if not self.approach_active or self.curr_pos is None or self.target_pos is None:   # donne return quand target pas reçue encore en gros
            return
        
        if self.manual_got_called == False:   # en test, pour arrêter direct le drone (ABORT, HOVER FAST)
            if self.flag_arret == True:
                self.flag_arret = False

                self.get_logger().info("Arrêt de l'approche!")

                if not hasattr(self, 'timer2') or self.timer2 is None:
                    self.timer2 = self.create_timer(0.05, self.set_brake_mode)
                
                """Pistes de solution à considérer


                vel_x = float(0)
                vel_y = float(0)
                vel_z = float(0)

                twist = TwistStamped()
                twist.twist.linear.x = vel_x
                twist.twist.linear.y = vel_y
                twist.twist.linear.z = vel_z

                self.publisher_.publish(twist)
                self.get_logger().info(f"PID velocities zeros - X: {vel_x}, Y: {vel_y}, Z: {vel_z}")"""
                
            return

        """if self.timer2 is not None:
            self.timer2.cancel()
            self.set_guided_mode()
            self.timer2 = None"""

        self.flag_arret = True

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

        self.publisher_.publish(twist)
        current_time = time.time()
        if current_time - self.last_log_time >= 1.0:
            self.get_logger().info(f"PID velocities - X: {vel_x}, Y: {vel_y}, Z: {vel_z}")
            self.last_log_time = current_time

    def Failsafe_max_vel(self, vel_x,vel_y, max_output):
        cap = np.sqrt(2)*max_output
        eps = 0.001
        scap = cap - eps
        if (vel_x**2 + vel_y**2)**(1/2) >= max_output:
            if vel_x >= cap:
                if vel_y >= cap:
                    vx = 1*np.sign(vel_x)
                    vy = 1*np.sign(vel_y) 
                    vel_x = scap*vx
                    vel_y = scap*vy
                else:
                    vx = 1*np.sign(vel_x)
                    vel_x = scap*vx

            if vel_y >= cap:
                if vel_x >= cap:
                    vx = 1*np.sign(vel_x)
                    vy = 1*np.sign(vel_y) 
                    vel_x = scap*vx
                    vel_y = scap*vy
                else:
                    vy = 1*np.sign(vel_y)
                    vel_y = scap*vy
        return vel_x, vel_y
        

def main(args=None):
    rclpy.init(args=args)
    node = ApproachNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
