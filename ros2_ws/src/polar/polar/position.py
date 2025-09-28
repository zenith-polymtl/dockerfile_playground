#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import time
from custom_interfaces.msg import TargetPosePolar
from mavros_msgs.msg import PositionTarget 
from mavros_msgs.srv import MessageInterval 
import math
from zenmav.core import Zenmav
import csv

class SimpleCSV:
    """Ultra-light CSV logger.
       - fields: list of column names (t is auto-added as time since init)
       - log(**vals): provide values by field name
    """
    def __init__(self, path: str, fields):
        self.fields = ["t"] + list(fields)
        self._t0 = time.monotonic()
        self._fh = open(path, "w", newline="")
        self._w = csv.writer(self._fh)
        self._w.writerow(self.fields)  # header

    def log(self, **vals):
        t = vals.get("t", time.monotonic() - self._t0)
        row = [t] + [vals.get(k, "") for k in self.fields[1:]]
        self._w.writerow(row)
        self._fh.flush()  # simple & safe; remove if you want more speed

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass



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

def wrap_pi(a): return (a + np.pi) % (2*np.pi) - np.pi

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

        self._declare_params()

        # --- Services (uses param flags/rates in setup_message_intervals) ---
        self.msg_interval_client = self.create_client(MessageInterval, '/mavros/set_message_interval')
        # run once, 1s after startup
        self.setup_timer = self.create_timer(1.0, self.setup_message_intervals)

        # --- Publishers / Subscribers (all topic names from params) ---
        self.publisher_raw = self.create_publisher(
            PositionTarget, self.topic_raw_setpoint, qos_profile
        )

        self.drone_position_sub = self.create_subscription(
            PoseStamped, self.topic_pose, self.drone_pose_callback, qos_profile_BE
        )
        self.drone_speed_sub = self.create_subscription(
            TwistStamped, self.topic_vel, self.drone_speed_callback, qos_profile_BE
        )
        self.pose_goal_sub = self.create_subscription(
            TargetPosePolar, self.topic_goal_polar, self.goal_pose_callback, qos_profile
        )
        self.estimated_target_sub = self.create_subscription(
            PoseStamped, self.topic_estimated_target, self.estimation_callback, qos_profile
        )
        self.activation_sub = self.create_subscription(
            String, self.topic_activation, self.activation_callback, qos_profile
        )
        self.start_sub = self.create_subscription(
            String, self.topic_ctrl_activation, self.controller_callback, qos_profile
        )

        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)


        # --- Controllers (gains/limits from params) ---
        self.pid_r = PIDController(
            kp=self.pid_r_kp, ki=self.pid_r_ki, kd=self.pid_r_kd,
            max_i=self.pid_r_max_i, max_output=self.pid_r_max_out
        )
        self.pid_r_abs = PIDController(
            kp=self.pid_rabs_kp, ki=self.pid_rabs_ki, kd=self.pid_rabs_kd,
            max_i=self.pid_rabs_max_i, max_output=self.pid_rabs_max_out
        )
        self.pid_theta = PIDController(
            kp=self.pid_theta_kp, ki=self.pid_theta_ki, kd=self.pid_theta_kd,
            max_i=self.pid_theta_max_i, max_output=self.pid_theta_max_out
        )
        self.pid_z = PIDController(
            kp=self.pid_z_kp, ki=self.pid_z_ki, kd=self.pid_z_kd,
            max_i=self.pid_z_max_i, max_output=self.pid_z_max_out
        )
        self.pid_yaw = PIDController(
            kp=self.pid_yaw_kp, ki=self.pid_yaw_ki, kd=self.pid_yaw_kd,
            max_i=self.pid_yaw_max_i, max_output=self.pid_yaw_max_out
        )


        # --- State ---
        self.estimated_target_pose = None
        self.drone_pose = None
        self.drone_speed = None
        self.target_pose = None
        self.last_time = None
        self.r_error = None
        self.z_error = None
        self.theta_error = None
        self.first = True
        self.r_ref = None
        self.yaw = None
        self.r_hold = None
        self.filtered_v_r = None
        self.approach_active = False

        self.yaw_offset = 0.0
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0

        # --- Backends / Logging (from params) ---
        self.drone = Zenmav(self.zenmav_endpoint)

        self.csv = SimpleCSV(
            path=self.csv_path,
            fields=["r", "r_hold", "vel_r_measured", "v_r", "v_theta", "vel_theta", "v_z", "vel_z", "yaw", "yaw_target"]
        )

        # --- Periodic jobs ---
        hz = 20.0  # control smoothing timer; make a param later if you want
        self.smooth_timer = self.create_timer(1.0 / hz, self.filter_vr_callback)
        # alpha already set from params in _declare_params() -> self.alpha

        self.get_logger().info("Polar positioning node started")


        self.get_logger().info("Polar positioning node started")
    
    def _declare_params(self):
        # Topics / frame
        self.declare_parameter("topic_pose", "/mavros/local_position/pose")
        self.declare_parameter("topic_vel", "/mavros/local_position/velocity_local")
        self.declare_parameter("topic_goal_polar", "/goal_pose_polar")
        self.declare_parameter("topic_estimated_target", "/estimated_target_location")
        self.declare_parameter("topic_activation", "/approach_activation")
        self.declare_parameter("topic_ctrl_activation", "/controller_activation")
        self.declare_parameter("topic_raw_setpoint", "/mavros/setpoint_raw/local")
        self.declare_parameter("frame_id", "map")

        # Rates / filters
        self.declare_parameter("alpha", 0.25)          # smoothing for v_r

        # Limits
        self.declare_parameter("centripetal_limit", 2.0)  # [m/s^2]
        self.declare_parameter("minimal_margin", 2.0)

        # CSV log
        self.declare_parameter("csv_path", "approach_log_polar.csv")

        # Zenmav / MAVLink config
        self.declare_parameter("zenmav_endpoint", "tcp:127.0.0.1:5762")
        self.declare_parameter("set_msg_interval", True)
        self.declare_parameter("msg_interval_rate", 20.0)   # Hz

        # PID params (flat for simplicity)
        # r (relative mode stabilizer)
        self.declare_parameter("pid_r_kp", 2.0)
        self.declare_parameter("pid_r_ki", 1.0)
        self.declare_parameter("pid_r_kd", 0.5)
        self.declare_parameter("pid_r_max_i", 1.0)
        self.declare_parameter("pid_r_max_out", 3.0)

        # r_abs (absolute radius controller)
        self.declare_parameter("pid_rabs_kp", 2.0)
        self.declare_parameter("pid_rabs_ki", 0.6)
        self.declare_parameter("pid_rabs_kd", 1.2)
        self.declare_parameter("pid_rabs_max_i", 1.2)
        self.declare_parameter("pid_rabs_max_out", 5.0)

        # theta (angle * radius controller)
        self.declare_parameter("pid_theta_kp", 0.6)
        self.declare_parameter("pid_theta_ki", 0.0)
        self.declare_parameter("pid_theta_kd", 0.24)
        self.declare_parameter("pid_theta_max_i", 1.0)
        self.declare_parameter("pid_theta_max_out", 3.0)

        # z
        self.declare_parameter("pid_z_kp", 0.6)
        self.declare_parameter("pid_z_ki", 0.0)
        self.declare_parameter("pid_z_kd", 0.35)
        self.declare_parameter("pid_z_max_i", 1.0)
        self.declare_parameter("pid_z_max_out", 3.0)

        # yaw
        self.declare_parameter("pid_yaw_kp", 3.0)
        self.declare_parameter("pid_yaw_ki", 1.0)
        self.declare_parameter("pid_yaw_kd", 0.3)
        self.declare_parameter("pid_yaw_max_i", 0.5)
        self.declare_parameter("pid_yaw_max_out", 6.0)

        # extra bool (fixed typo)
        self.declare_parameter("talk", True)

        # ---------- variable attribution (cache values) ----------
        gp = self.get_parameter  # short alias

        # Topics / frame
        self.topic_pose            = gp("topic_pose").value
        self.topic_vel             = gp("topic_vel").value
        self.topic_goal_polar      = gp("topic_goal_polar").value
        self.topic_estimated_target= gp("topic_estimated_target").value
        self.topic_activation      = gp("topic_activation").value
        self.topic_ctrl_activation = gp("topic_ctrl_activation").value
        self.topic_raw_setpoint    = gp("topic_raw_setpoint").value
        self.frame_id              = gp("frame_id").value

        # Rates / filters / limits
        self.alpha             = float(gp("alpha").value)
        self.centripetal_limit = float(gp("centripetal_limit").value)
        self.minimal_margin = float(gp("minimal_margin").value)

        # CSV / comms
        self.csv_path         = gp("csv_path").value
        self.zenmav_endpoint  = gp("zenmav_endpoint").value
        self.set_msg_interval = bool(gp("set_msg_interval").value)
        self.msg_interval_rate= float(gp("msg_interval_rate").value)

        # PIDs
        self.pid_r_kp        = float(gp("pid_r_kp").value)
        self.pid_r_ki        = float(gp("pid_r_ki").value)
        self.pid_r_kd        = float(gp("pid_r_kd").value)
        self.pid_r_max_i     = float(gp("pid_r_max_i").value)
        self.pid_r_max_out   = float(gp("pid_r_max_out").value)

        self.pid_rabs_kp     = float(gp("pid_rabs_kp").value)
        self.pid_rabs_ki     = float(gp("pid_rabs_ki").value)
        self.pid_rabs_kd     = float(gp("pid_rabs_kd").value)
        self.pid_rabs_max_i  = float(gp("pid_rabs_max_i").value)
        self.pid_rabs_max_out= float(gp("pid_rabs_max_out").value)

        self.pid_theta_kp    = float(gp("pid_theta_kp").value)
        self.pid_theta_ki    = float(gp("pid_theta_ki").value)
        self.pid_theta_kd    = float(gp("pid_theta_kd").value)
        self.pid_theta_max_i = float(gp("pid_theta_max_i").value)
        self.pid_theta_max_out = float(gp("pid_theta_max_out").value)

        self.pid_z_kp        = float(gp("pid_z_kp").value)
        self.pid_z_ki        = float(gp("pid_z_ki").value)
        self.pid_z_kd        = float(gp("pid_z_kd").value)
        self.pid_z_max_i     = float(gp("pid_z_max_i").value)
        self.pid_z_max_out   = float(gp("pid_z_max_out").value)

        self.pid_yaw_kp      = float(gp("pid_yaw_kp").value)
        self.pid_yaw_ki      = float(gp("pid_yaw_ki").value)
        self.pid_yaw_kd      = float(gp("pid_yaw_kd").value)
        self.pid_yaw_max_i   = float(gp("pid_yaw_max_i").value)
        self.pid_yaw_max_out = float(gp("pid_yaw_max_out").value)

        self.talk            = bool(gp("talk").value)


    def setup_message_intervals(self):
        if True:
            """Set up message intervals after node initialization"""  
            if not self.msg_interval_client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn('Message interval service not available, aborting request...')  
                self.destroy_timer(self.setup_timer) 
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
            self.filtered_v_r = 0.0
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
            self.target_pose = None
            self.first = True
            self.r_hold = None

            self.last_time = None
            self.r_error = None
            self.z_error = None
            self.theta_error = None
            self.current_time = None
            self.last_time = None
            self.drone_speed = None
            self.filtered_v_r = None
            self.yaw_offset = 0.0
            


    def compute_estimated_state(self):
        if not self.approach_active and self.drone_speed is not None and self.drone_pose is not None:
            return None
        

        if self.estimated_target_pose is None:
            self.get_logger().info("No estimated target pose available")
            return None
        
        #Compute linear distances with target
        delta_x = self.estimated_target_pose.x - self.drone_pose.x
        delta_y = self.estimated_target_pose.y - self.drone_pose.y
        delta_z = self.estimated_target_pose.z - self.drone_pose.z

        #Compute distance with target
        self.distance_from_target = math.hypot(delta_x,delta_y)

        #Initialize r_hold if relative not active
        if self.first: 
            self.first = False
            self.r_hold = self.distance_from_target
            self.filter_vr_callback()

        # --- Unit vectors in polar frame ---
        self.unit_vector_to_target = np.array([delta_x, delta_y]) / self.distance_from_target if self.distance_from_target != 0 else np.array([0.0, 0.0])

        tangential_unit_vector = np.array([-self.unit_vector_to_target[1],
                                            self.unit_vector_to_target[0]])  # CCW tangent


        # --- Decompose velocity into radial/tangential components ---
        self.radial_speed_measured = self.drone_speed.x * self.unit_vector_to_target[0] + self.drone_speed.y * self.unit_vector_to_target[1]
        self.tangential_speed_measured = self.drone_speed.x * tangential_unit_vector[0] + self.drone_speed.y * tangential_unit_vector[1]

        if self.target_pose.relative:
            self.v_theta = self.target_pose.v_theta #Directly pass v_theta as target

            if abs(self.tangential_speed_measured) < 0.04:
                self.r_hold = self.distance_from_target
                self.pid_r.integral = 0
            self.r_hold = self.minimal_margin if self.r_hold < self.minimal_margin else self.r_hold
            self.r_error = self.distance_from_target - self.r_hold # Compute distance between goal radius

        else:
            self.r_error = self.distance_from_target - self.target_pose.r #r+ is radial in
            self.theta_error = wrap_pi(math.atan2(-delta_y, -delta_x) - self.target_pose.theta)
            self.theta_distance_error = self.theta_error*self.distance_from_target
            self.z_error = delta_z + float(self.target_pose.z)

            self.first = False #Reset the first flag to indicate no relative was going on

        # hdg_deg: 0 = North, +CW (aircraft heading)
        hdg_deg = self.drone.get_global_pos(heading=True).hdg
        self.yaw_enu = ((math.radians(90.0 - hdg_deg) + math.pi) % (2*math.pi)) - math.pi   # [-pi, pi]
          
        self.angle_towards_target_rad = np.arctan2(delta_y, delta_x)  

        
        self.error_yaw = wrap_pi(self.angle_towards_target_rad - self.yaw_enu)
        
        self.compute_commands()

    def compute_commands(self):
        
        #Compute delta-time since last command

        # in compute_commands()
        now = time.monotonic()
        self.dt = (now - self.last_time) if self.last_time is not None else 0.0
        self.last_time = now
        # clamp dt to kill spikes (and forbid negatives)
        self.dt = max(1e-3, min(self.dt, 0.10))

        if self.target_pose.relative:
            #Stabilisation PID and v_r feedforward
            self.vel_r = self.pid_r.compute(self.r_error, self.dt) - self.filtered_v_r



            #Direct vertical speed control
            self.vel_z = self.target_pose.v_z

            #Update r_hold if v_r is out of a small dead (prevents small instabilities)
            if abs(self.filtered_v_r) > 0.01:

                self.r_hold += self.filtered_v_r * self.dt
                
        else:
            # Compute speeds based of absolute error
            self.vel_r = self.pid_r_abs.compute(self.r_error, self.dt)
            self.vel_z = self.pid_z.compute(self.z_error, self.dt)
            self.v_theta = self.pid_theta.compute(self.theta_distance_error, self.dt)


        #Centrepedial acceleration limit
        r = max(self.distance_from_target, 1e-6)
        vtheta_max = math.sqrt(self.centripetal_limit * r)
        self.v_theta = float(np.clip(self.v_theta, -vtheta_max, vtheta_max))
        #Limit to 1m/s/s

        # Decompose velocities into x and y components
        #Radial speed
        self.yaw_feed_forward = -self.tangential_speed_measured/self.distance_from_target
        self.bend_correction()

        if self.talk:
            self.get_logger().info(f" Distance : {self.distance_from_target:.3f}, r_hold: {self.r_hold:.3f}, self.vel_r: {self.vel_r:.3f}")


        self.vel_rx, self.vel_ry = self.vel_r*self.unit_vector_to_target
        self.vel_theta_x, self.vel_theta_y = self.v_theta*np.array([-self.unit_vector_to_target[1], self.unit_vector_to_target[0]])
        

        #Combination
        self.vel_x = self.vel_rx + self.vel_theta_x
        self.vel_y = self.vel_ry + self.vel_theta_y

        #Pilot controls yaw rate, which modifies the goal orientation setpoint
        self.yaw_offset += self.target_pose.yaw_rate*self.dt
        if self.yaw_offset > np.pi :
            self.yaw_offset -= 2*np.pi
        elif self.yaw_offset < -np.pi:
            self.yaw_offset += 2*np.pi


        total_yaw_err = wrap_pi(self.error_yaw + self.yaw_offset)

        if self.talk:
            self.get_logger().info(f"yaw offset : {self.yaw_offset:.3f} , total_yaw_err = {total_yaw_err:.3f}")


        self.yaw_rate = self.pid_yaw.compute(total_yaw_err, self.dt)

        #Feed forward a rate to keep same pose relative to trajectory
        
        self.yaw_rate += - self.yaw_feed_forward if self.tangential_speed_measured > 0 else self.yaw_feed_forward

        self.csv.log(
            r=getattr(self, "distance_from_target", float("nan")),
            r_hold=(self.r_hold if self.r_hold is not None else float("nan")),
            vel_r_measured=getattr(self, "radial_speed_measured", float("nan")),
            v_r=(self.filtered_v_r if (self.target_pose and self.filtered_v_r is not None) else 0.0),
            v_theta=getattr(self, "v_theta", float("nan")),
            vel_theta=getattr(self, "tangential_speed_measured", float("nan")),
            v_z=getattr(self, "vel_z", float("nan")),
            vel_z=(getattr(self.drone_speed, "z", float("nan")) if hasattr(self, "drone_speed") else float("nan")),
            yaw = self.yaw_enu,
            yaw_target = self.angle_towards_target_rad
        )
        
        self.send_commands()

    def bend_correction(self):
        r = self.distance_from_target

        arc = self.v_theta*self.dt
        theta = arc/r
        x = r*math.sin(theta)
        y = r*(1-math.cos(theta))
        phi = math.atan2(y,x)

        D = math.hypot(x,y)
        v2 = D/self.dt
        self.get_logger().info(f"arc = {arc:.3f}, phi = {phi*180/np.pi:.2f}, v2 = {v2:.3f}, rff = {v2*math.sin(phi)}")
        self.v_theta = v2*math.cos(phi)
        self.vel_r -= v2*math.sin(phi)


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
            PositionTarget.IGNORE_YAW 
        )  
          
        # Set velocity components  
        target.velocity.x = self.vel_x  
        target.velocity.y = self.vel_y  
        target.velocity.z = self.vel_z  
        
        target.yaw_rate = float(self.yaw_rate)
          
        self.publisher_raw.publish(target)

        if self.talk:
            self.get_logger().info(f"Temps de traitement : {time.monotonic() - self.start_time:.4f}")
    
    def drone_speed_callback(self, msg):
        self.drone_speed = msg.twist.linear

    def filter_vr_callback(self):
        if self.target_pose is None:
            return 
        
        if self.filtered_v_r is not None:
            max_rate = self.alpha   # m/s per iteration
            delta = self.target_pose.v_r - self.filtered_v_r
            delta = np.clip(delta, -max_rate, max_rate)
            self.filtered_v_r += delta
        else:
            self.filtered_v_r = 0

    def goal_pose_callback(self, msg):
        self.target_pose = msg
        self.target_pose.theta = (-msg.theta+90)/180*np.pi
        #self.get_logger().info(f"Received target pose: r={self.target_pose.r}, z={self.target_pose.z}, theta={self.target_pose.theta}, v_theta={self.target_pose.v_theta}, v_r={self.filtered_v_r}, relative={self.target_pose.relative}")

    def estimation_callback(self, msg):
        self.estimated_target_pose = msg.pose.position

    def drone_pose_callback(self, msg):
        self.start_time = time.monotonic()
        self.drone_pose = msg.pose.position
        if self.target_pose is not None and self.estimated_target_pose is not None:
            if self.last_time is None:
                self.last_time = time.monotonic()
            else:
                self.compute_estimated_state()
    
        

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
            target.type_mask |= PositionTarget.IGNORE_YAW_RATE

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
