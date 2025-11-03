#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import String, Float64
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import time
from custom_interfaces.msg import TargetPosePolar
from mavros_msgs.msg import PositionTarget 
from mavros_msgs.srv import MessageInterval 
import math
import csv

class SimpleCSV:
    """Ultra-light CSV logger.
       - fields: list of column names (t is auto-added as time since init)
       - log(**vals): provide values by field name
    """
    def __init__(self, path: str, fields):
        self.fields = ["t"] + list(fields)
        self._t0 = time.monotonic()
        self._fh = open(path, "w", newline="", buffering=1024*64)
        self._w = csv.writer(self._fh)
        self._w.writerow(self.fields)  # header

    def log(self, **vals):
        t = vals.get("t", time.monotonic() - self._t0)
        row = [t] + [vals.get(k, "") for k in self.fields[1:]]
        self._w.writerow(row)

    def close(self):
        try:
            self._fh.close()
        except Exception:
            pass

# --- PID with filtered derivative & optional d_meas ---
class PIDController():
    def __init__(self, kp, ki, kd, max_output=3.0, max_i=1.0,
                 deriv_tau=0.0, d_clip=None):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_output, self.max_i = max_output, max_i
        self.prev_error = 0.0
        self.prev_error2 = 0.0
        self.integral = 0.0
        self.deriv_tau = deriv_tau  # [s], 0.06–0.12 works well at 30 Hz
        self._d_state = 0.0
        self.d_clip = d_clip
        # expose last computed PID components for logging
        self.last_p = 0.0
        self.last_i = 0.0
        self.last_d = 0.0


    def compute(self, error, dt, d_meas=None):
        if dt <= 0: return 0.0
        # I
        self.integral += error * dt
        self.integral = max(-self.max_i, min(self.integral, self.max_i))
        # store I term (ki*integral) for logging
        self.last_i = self.ki * self.integral
        # P term for logging
        self.last_p = self.kp * error
        #Second order backward derivative. If d_meas is provided, it's already a rate.
        raw_d = d_meas if d_meas is not None else (3*error - 4*self.prev_error + self.prev_error2)/ (2*dt)
        # Low-pass filter
        a = self.deriv_tau / (self.deriv_tau + dt)  # 0<a<1
        self._d_state = a*self._d_state + (1.0 - a)*raw_d
        dterm = self.kd * self._d_state
        if self.d_clip is not None:
            dterm = max(-self.d_clip, min(dterm, self.d_clip))
        # store D term (post clipping) for logging
        self.last_d = dterm
        # Sum & clamp
        u = self.last_p + self.last_i + dterm
        self.prev_error2 = self.prev_error
        self.prev_error = error
        
        
        return max(-self.max_output, min(u, self.max_output))

    def reset(self):
        self.prev_error = 0.0
        self.prev_error2 = 0.0
        self.integral = 0.0
        self._d_state = 0.0

def wrap_pi(a): return (a + np.pi) % (2*np.pi) - np.pi

class soft_pos_hold:
    def __init__(self, pid,
                 on_vel=0.20, off_vel=0.30,  # hysteresis on measured |vel|
                 on_cmd=0.04, off_cmd=0.06,
                 wrap_fn=None): # hysteresis on |cmd_vel|
        self.pid = pid
        self.last_lock_activation = False
        self.lock_activation = False
        self.lock_position = 0.0
        self.on_vel, self.off_vel = on_vel, off_vel
        self.on_cmd, self.off_cmd = on_cmd, off_cmd
        self.wrap_fn = wrap_fn 

    def reset(self):
        self.last_lock_activation = False
        self.lock_activation = False
        self.lock_position = 0.0
        #reset pid values
        self.pid.prev_error = 0.0
        self.pid.prev_error2 = 0.0
        self.pid.integral = 0.0

    def compute(self, position, vel, cmd_vel, dt, deriv_pid_sign = 1):
    
        if cmd_vel is None or dt <= 0.0:
            # not enough info; treat as unlocked
            self.lock_activation = False
            self.last_lock_activation = False
            return 0.0

        # --- hysteresis logic ---
        if not self.lock_activation:
            want_lock = (abs(vel) <= self.on_vel) and (abs(cmd_vel) <= self.on_cmd)
        else:
            # stay locked until user command clearly leaves deadband
            want_lock =  (abs(cmd_vel) <= self.off_cmd)

        self.lock_activation = want_lock

        # edge: just entered lock → latch position once
        if self.lock_activation and not self.last_lock_activation:
            self.lock_position = float(position)
        elif not self.lock_activation and self.last_lock_activation:
            self.reset()

        # compute correction if locked
        out = 0.0
        if self.lock_activation:
            error = self.lock_position - float(position)
            error = error if self.wrap_fn is None else self.wrap_fn(error)
            out = self.pid.compute(error, dt, d_meas=float(vel)*deriv_pid_sign)


        self.last_lock_activation = self.lock_activation
        return out

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
        self.estimated_center_sub = self.create_subscription(
            PoseStamped, self.topic_estimated_center, self.estimated_center_callback, qos_profile
        )
        self.activation_sub = self.create_subscription(
            String, self.topic_activation, self.activation_callback, qos_profile
        )
        self.start_sub = self.create_subscription(
            String, self.topic_ctrl_activation, self.controller_callback, qos_profile
        )
        self.yaw_sub = self.create_subscription(
            Float64, '/mavros/global_position/compass_hdg', self.yaw_callback, qos_profile_BE
        )

        self.abort_state_pub = self.create_publisher(String, '/abort_brake', qos_profile)


        self._intialise_controllers()

        self.z_pos_hold = soft_pos_hold(self.pid_z_hold)
        self.r_pos_hold = soft_pos_hold(self.pid_r_hold)
        self.theta_pos_hold = soft_pos_hold(self.pid_theta_hold)


        # --- State ---
        self.control_rate = 25.0  # main control loop
        self.estimated_center = None
        self.drone_pose = None
        self.drone_speed = None
        self.target_pose = None
        self.last_time = None
        self.r_error = None
        self.z_error = None
        self.theta_error = None
        self.r_ref = None
        self.yaw = None
        self.filtered_v_r = None
        self.approach_active = False
        self.last_delta_t = None
        self.total_yaw_err = None
        self.delta_t = None
        self.hdg_deg = None
        self.control_timer = None
        self.smooth_timer  = None
        self.info_timer    = None
        self.log_timer     = None
        self.dt = 1/self.control_rate
        self.yaw_offset = 0.0
        self.vel_x, self.vel_y, self.vel_z = 0.0, 0.0, 0.0
        self.a_theta_cmd = 0.0
        self.a_r_cmd = 0.0

        # initialize acceleration/aux variables that were referenced later but not set
        self.acc_x = 0.0
        self.acc_y = 0.0
        self.acc_z = 0.0
        self.vel_r = 0.0
        self.v_cmd = 0.0
        self.v_theta = 0.0

        # --- Backends / Logging (from params) ---
        # Expanded CSV fields to capture setpoint vs command vs response (focus: relative mode)
        self.csv = SimpleCSV(
            path=self.csv_path,
            fields=[
                # radius
                "radius",
                # radial speeds & acc
                "set_speed_r", "set_speed_r_filtered", "meas_speed_r", "acc_cmd_r",
                # tangential speeds & acc
                "set_speed_theta", "meas_speed_theta", "acc_cmd_theta",
                # vertical speeds & acc
                "set_speed_z", "meas_speed_z", "acc_cmd_z",
                # commanded accelerations (components)
                "acc_x", "acc_y", "acc_z",
                # published velocities (what we publish)
                "pub_vel_x", "pub_vel_y", "pub_vel_z",
                # PID components (P/I/D) for main controllers
                "pid_r_P", "pid_r_I", "pid_r_D",
                "pid_vtheta_P", "pid_vtheta_I", "pid_vtheta_D",
                "pid_z_P", "pid_z_I", "pid_z_D",
                # yaw / heading
                "yaw_enu", "yaw_target", "error_yaw", "total_yaw_err", "yaw_rate",
                # position holds parameters
                "drone_z", "target_z",
                "r_lock_position", "r_lock_active", "r_pid_P", "r_pid_I", "r_pid_D",
                "z_lock_position", "z_lock_active", "z_pid_P", "z_pid_I", "z_pid_D",
                # theta hold: actual/target/lock & PID
                "theta_actual", "theta_target", "theta_lock_position", "theta_lock_active",
                "theta_pid_P", "theta_pid_I", "theta_pid_D",
                # timing
                "dt"
            ]
        )

        start_timers_freq = 2.0  # Hz
        self.start_timers_timer = self.create_timer(1.0 / start_timers_freq, self.launch_timers)

    def launch_timers(self):
        # Start only when we WANT to run and have the MINIMUM inputs
        if not self.approach_active:
            return

        ready = (
            self.target_pose is not None and
            self.drone_speed is not None and
            self.drone_pose  is not None and
            self.hdg_deg     is not None and
            self.estimated_center is not None
        )
        if not ready:
            return

        # Create each timer once
        if self.control_timer is None:
            self.control_timer = self.create_timer(1.0 / self.control_rate, self.compute_estimated_state)
        if self.smooth_timer is None:
            self.smooth_timer = self.create_timer(1.0 / self.control_rate, self.filter_vr_callback)

        if self.talk and self.info_timer is None:
            self.info_timer = self.create_timer(1/4.0, self.info_callback)

        if self.log and self.log_timer is None:
            self.log_timer = self.create_timer(1/10.0, self.log_callback)
            self.get_logger().info("Polar positioning node started")


    def destroy_timers(self):
        try:
            self.control_timer.destroy()
            self.smooth_timer.destroy()
            if self.talk:
                self.info_timer.destroy()
            if self.log:
                self.log_timer.destroy()

            self.control_timer = None
            self.smooth_timer  = None
            self.info_timer    = None
            self.log_timer     = None
        except Exception:
            self.get_logger().error("Error destroying timers")

        

    def _intialise_controllers(self):
        # --- Controllers (gains/limits from params) ---
        self.pid_r = PIDController(
            kp=self.pid_r_kp, ki=self.pid_r_ki, kd=self.pid_r_kd,
            max_i=self.pid_r_max_i, deriv_tau=self.pid_r_deriv_tau, max_output=self.pid_r_max_out,
            d_clip=self.pid_r_d_clip if getattr(self, "pid_r_d_clip", 0.0) > 0.0 else None
        )
        self.pid_r_abs = PIDController(
            kp=self.pid_r_abs_kp, ki=self.pid_r_abs_ki, kd=self.pid_r_abs_kd,
            max_i=self.pid_r_abs_max_i, deriv_tau=self.pid_r_abs_deriv_tau, max_output=self.pid_r_abs_max_out,
            d_clip=self.pid_r_abs_d_clip if getattr(self, "pid_r_abs_d_clip", 0.0) > 0.0 else None
        )
        self.pid_r_hold = PIDController(
            kp=self.pid_r_hold_kp, ki=self.pid_r_hold_ki, kd=self.pid_r_hold_kd,
            max_i=self.pid_r_hold_max_i, deriv_tau=self.pid_r_hold_deriv_tau, max_output=self.pid_r_hold_max_out,
            d_clip=self.pid_r_hold_d_clip if getattr(self, "pid_r_hold_d_clip", 0.0) > 0.0 else None
        )

        self.pid_theta_abs = PIDController(
            kp=self.pid_theta_abs_kp, ki=self.pid_theta_abs_ki, kd=self.pid_theta_abs_kd,
            max_i=self.pid_theta_abs_max_i, deriv_tau=self.pid_theta_abs_deriv_tau, max_output=self.pid_theta_abs_max_out,
            d_clip=self.pid_theta_abs_d_clip if getattr(self, "pid_theta_abs_d_clip", 0.0) > 0.0 else None
        )
        self.pid_theta_hold = PIDController(
            kp=self.pid_theta_hold_kp, ki=self.pid_theta_hold_ki, kd=self.pid_theta_hold_kd,
            max_i=self.pid_theta_hold_max_i, deriv_tau=self.pid_theta_hold_deriv_tau, max_output=self.pid_theta_hold_max_out,
            d_clip=self.pid_theta_hold_d_clip if getattr(self, "pid_theta_hold_d_clip", 0.0) > 0.0 else None
        )

        self.pid_v_theta = PIDController(
            kp=self.pid_v_theta_kp, ki=self.pid_v_theta_ki, kd=self.pid_v_theta_kd,
            max_i=self.pid_v_theta_max_i, deriv_tau=self.pid_v_theta_deriv_tau, max_output=self.pid_v_theta_max_out,
            d_clip=self.pid_v_theta_d_clip if getattr(self, "pid_v_theta_d_clip", 0.0) > 0.0 else None
        )

        self.pid_z = PIDController(
            kp=self.pid_z_kp, ki=self.pid_z_ki, kd=self.pid_z_kd,
            max_i=self.pid_z_max_i, deriv_tau=self.pid_z_deriv_tau, max_output=self.pid_z_max_out,
            d_clip=self.pid_z_d_clip if getattr(self, "pid_z_d_clip", 0.0) > 0.0 else None
        )
        self.pid_z_abs = PIDController(
            kp=self.pid_z_abs_kp, ki=self.pid_z_abs_ki, kd=self.pid_z_abs_kd,
            max_i=self.pid_z_abs_max_i, deriv_tau=self.pid_z_abs_deriv_tau, max_output=self.pid_z_abs_max_out,
            d_clip=self.pid_z_abs_d_clip if getattr(self, "pid_z_abs_d_clip", 0.0) > 0.0 else None
        )
        self.pid_z_hold = PIDController(
            kp=self.pid_z_hold_kp, ki=self.pid_z_hold_ki, kd=self.pid_z_hold_kd,
            max_i=self.pid_z_hold_max_i, deriv_tau=self.pid_z_hold_deriv_tau, max_output=self.pid_z_hold_max_out,
            d_clip=self.pid_z_hold_d_clip if getattr(self, "pid_z_hold_d_clip", 0.0) > 0.0 else None
        )

        self.pid_yaw = PIDController(
            kp=self.pid_yaw_kp, ki=self.pid_yaw_ki, kd=self.pid_yaw_kd,
            max_i=self.pid_yaw_max_i, deriv_tau=self.pid_yaw_deriv_tau, max_output=self.pid_yaw_max_out,
            d_clip=self.pid_yaw_d_clip if getattr(self, "pid_yaw_d_clip", 0.0) > 0.0 else None
        )

    def _declare_params(self):
        # Topics / frame
        self.declare_parameter("topic_pose", "/mavros/local_position/pose")
        self.declare_parameter("topic_vel", "/mavros/local_position/velocity_local")
        self.declare_parameter("topic_goal_polar", "/goal_pose_polar")
        self.declare_parameter("topic_estimated_center", "/estimated_center_location")
        self.declare_parameter("topic_activation", "/approach_activation")
        self.declare_parameter("topic_ctrl_activation", "/controller_activation")
        self.declare_parameter("topic_raw_setpoint", "/mavros/setpoint_raw/local")
        self.declare_parameter("frame_id", "map")

        # Rates / filters
        self.declare_parameter("alpha", 0.1)

        # Limits
        self.declare_parameter("centripetal_limit", 1.5)
        self.declare_parameter("minimal_margin", 2.0)
        self.declare_parameter("soft_repulsion_initial_radius", 5.0)

        # CSV log
        self.declare_parameter("csv_path", "approach_log_polar.csv")

        # MAVLink config
        self.declare_parameter("set_msg_interval", True)
        self.declare_parameter("msg_interval_rate", 25.0)

        # extra bools
        self.declare_parameter("talk", True)
        self.declare_parameter("log", True)

        # --- Unified PID params for all controllers (kp, ki, kd, max_i, max_out, deriv_tau, d_clip) ---
        for pname, defaults in [
            ("pid_r",       (3.75, 1.0, 0.0, 1.0, 7.0, 0.0, 0.0)),
            ("pid_r_abs",   (0.2,  0.1, 0.1, 0.3, 2.0, 0.1, 0.0)),
            ("pid_r_hold",  (0.3,  0.1, 0.15,0.5, 3.0, 0.1, 0.0)),
            ("pid_theta_abs",(0.3, 0.1, 0.15,0.5, 3.0, 0.1, 0.0)),
            ("pid_theta_hold",(0.6,0.2, 0.3, 0.5, 3.0, 0.1, 0.0)),
            ("pid_v_theta", (2.5,  1.0, 0.0, 0.2, 2.25, 0.0, 0.3)),
            ("pid_z",       (0.6,  0.0, 0.3, 1.0, 3.0, 0.075, 0.0)),
            ("pid_z_abs",   (0.3,  0.0, 0.25,0.5, 3.0, 0.1, 0.8)),
            ("pid_z_hold",  (0.3,  0.1, 0.15,0.5, 3.0, 0.1, 0.0)),
            ("pid_yaw",     (2.0,  1.0, 0.3, 0.5, 6.0, 0.0, 0.0)),
        ]:
            kp, ki, kd, max_i, max_out, deriv_tau, d_clip = defaults
            self.declare_parameter(f"{pname}_kp", kp)
            self.declare_parameter(f"{pname}_ki", ki)
            self.declare_parameter(f"{pname}_kd", kd)
            self.declare_parameter(f"{pname}_max_i", max_i)
            self.declare_parameter(f"{pname}_max_out", max_out)
            self.declare_parameter(f"{pname}_deriv_tau", deriv_tau)
            self.declare_parameter(f"{pname}_d_clip", d_clip)

        # ---------- variable attribution (cache values) ----------
        gp = self.get_parameter  # short alias

        # Topics / frame
        self.topic_pose            = gp("topic_pose").value
        self.topic_vel             = gp("topic_vel").value
        self.topic_goal_polar      = gp("topic_goal_polar").value
        self.topic_estimated_center= gp("topic_estimated_center").value
        self.topic_activation      = gp("topic_activation").value
        self.topic_ctrl_activation = gp("topic_ctrl_activation").value
        self.topic_raw_setpoint    = gp("topic_raw_setpoint").value
        self.frame_id              = gp("frame_id").value

        # Rates / filters / limits
        self.alpha             = float(gp("alpha").value)
        self.alpha = None if self.alpha <= 0.0 else self.alpha
        self.centripetal_limit = float(gp("centripetal_limit").value)
        self.minimal_margin = float(gp("minimal_margin").value)
        self.soft_repulsion_initial_radius = float(gp("soft_repulsion_initial_radius").value)

        # CSV / comms
        self.csv_path         = gp("csv_path").value
        self.set_msg_interval = bool(gp("set_msg_interval").value)
        self.msg_interval_rate= float(gp("msg_interval_rate").value)

        # cache all unified PID params
        for pname in ["pid_r","pid_r_abs","pid_r_hold","pid_theta_abs","pid_theta_hold",
                      "pid_v_theta","pid_z","pid_z_abs","pid_z_hold","pid_yaw"]:
            setattr(self, f"{pname}_kp", float(gp(f"{pname}_kp").value))
            setattr(self, f"{pname}_ki", float(gp(f"{pname}_ki").value))
            setattr(self, f"{pname}_kd", float(gp(f"{pname}_kd").value))
            setattr(self, f"{pname}_max_i", float(gp(f"{pname}_max_i").value))
            setattr(self, f"{pname}_max_out", float(gp(f"{pname}_max_out").value))
            setattr(self, f"{pname}_deriv_tau", float(gp(f"{pname}_deriv_tau").value))
            setattr(self, f"{pname}_d_clip", float(gp(f"{pname}_d_clip").value))

        self.talk            = bool(gp("talk").value)
        self.log             = bool(gp("log").value)

    def setup_message_intervals(self):
        if True:
            """Set up message intervals after node initialization"""  
            if not self.msg_interval_client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn('Message interval service not available, aborting request...')  
                self.destroy_timer(self.setup_timer) 
                return  
            
            request = MessageInterval.Request()  
            request.message_id = 32  
            request.message_rate = self.control_rate 

            request2 = MessageInterval.Request()  
            request2.message_id = 33  
            request2.message_rate = self.control_rate

            future = self.msg_interval_client.call_async(request2)  
            future.add_done_callback(self.message_interval_callback) 

            future2 = self.msg_interval_client.call_async(request)  
            future2.add_done_callback(self.message_interval_callback)
            
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
            self.target_pose = None  # reset target
            self.launch_timers()
        elif msg.data == "stop":
            self.get_logger().info("Approach Deactivated")
            self.publish_zero()
            self.approach_active = False
            # Reset controllers and state
            self.pid_r.reset()
            self.pid_z.reset()
            self.pid_theta_abs.reset()
            self.pid_yaw.reset()
            self.pid_v_theta.reset()
            self.pid_r_abs.reset()
            self.pid_z_hold.reset()
            self.pid_r_hold.reset()
            self.target_pose = None

            self.last_time = None
            self.r_error = None
            self.z_error = None
            self.theta_error = None
            self.current_time = None
            self.last_time = None
            self.drone_speed = None
            self.filtered_v_r = None
            self.yaw_offset = 0.0
            self.destroy_timers()
            
    def info_callback(self):
        self.get_logger().info(f" Distance : {self.distance_from_target:.3f}, self.vel_r: {self.vel_r:.3f}")
        self.get_logger().info(f"yaw offset : {self.yaw_offset:.3f} , total_yaw_err = {self.total_yaw_err:.3f}")
        self.get_logger().info(f"Temps de traitement : {self.delta_t:.5f}")

    def safe_radius_correction(self):
        self.initial_no_go_radius = self.soft_repulsion_initial_radius # start of soft zone
        r      = self.distance_from_target
        r_safe = self.minimal_margin          # hard keep-out
        r0     = self.initial_no_go_radius          # start of soft zone
        v_in   = self.filtered_v_r

        if r <= r_safe:
            # Inside the keep-out: forbid inward (negative) velocity; allow outward
            v_r = min(v_in, 0.0) - 0.05  # small outward bias to escape

        elif (r < r0):
            if (v_in > 0.0):
                if v_in > (r0 - r_safe):
                    v_in = (r - r_safe)  # cap max inward speed to not overshoot soft zone
                # In the soft zone: scale inward velocity smoothly toward 0
                denom = max(r0 - r_safe, 1e-6)
                alpha = (r - r_safe) / denom      # in (0,1)
                alpha = max(0.0, min(1.0, alpha)) # clip for safety
                v_r = (alpha * alpha) * v_in      # quadratic taper
            else:
                # In the soft zone but moving outward
                v_r = v_in
                v_r += self.dt*0.5*v_r #Damping outward speed in soft zone
        else:
            # Outside the soft zone or moving outward
            v_r = v_in

        return v_r

    def compute_estimated_state(self):
        
        self.start_time = time.monotonic()
        #time.sleep(0.01)  # simulate small processing delay if needed
        #Compute linear distances with target
        delta_x = self.estimated_center.x - self.drone_pose.x
        delta_y = self.estimated_center.y - self.drone_pose.y
        delta_z = self.estimated_center.z - self.drone_pose.z

        #Compute distance with target
        self.distance_from_target = math.hypot(delta_x,delta_y)
        
        # --- Filter radial speed command ---
            
        self.filter_vr_callback()

        # --- Unit vectors in polar frame ---
        self.unit_vector_to_target = delta_x / self.distance_from_target, delta_y / self.distance_from_target

        tangential_unit_vector = -self.unit_vector_to_target[1], self.unit_vector_to_target[0]  # CCW tangent

        # --- Decompose velocity into radial/tangential components ---
        self.radial_speed_measured = self.drone_speed.x * self.unit_vector_to_target[0] + self.drone_speed.y * self.unit_vector_to_target[1]
        self.tangential_speed_measured = self.drone_speed.x * tangential_unit_vector[0] + self.drone_speed.y * tangential_unit_vector[1]
        self.vertical_speed_measured = self.drone_speed.z

        angle = math.atan2(delta_y, delta_x)

        z_pos_hold_speed_command = self.z_pos_hold.compute(self.drone_pose.z, self.drone_speed.z, self.target_pose.v_z, self.dt)
        r_pos_hold_speed_command = self.r_pos_hold.compute(-self.distance_from_target, self.radial_speed_measured, self.target_pose.v_r, self.dt, deriv_pid_sign=-1)
        theta_pos_hold_speed_command = self.theta_pos_hold.compute(-angle, self.tangential_speed_measured/self.distance_from_target, self.target_pose.v_theta/self.distance_from_target, self.dt, deriv_pid_sign=-1)

        v_r = self.safe_radius_correction()
        if self.target_pose.relative:
            if self.target_pose.v_theta**2/max(self.distance_from_target, 1e-6) > self.centripetal_limit:
                self.target_pose.v_theta = math.copysign(math.sqrt(self.centripetal_limit * max(self.distance_from_target, 1e-6)), self.target_pose.v_theta)
        
            self.theta_speed_error = self.target_pose.v_theta + theta_pos_hold_speed_command - self.tangential_speed_measured
            self.r_speed_error = v_r + r_pos_hold_speed_command - self.radial_speed_measured
            self.z_speed_error = self.target_pose.v_z + z_pos_hold_speed_command - self.vertical_speed_measured

        else:
            self.r_error = self.distance_from_target - self.target_pose.r #r+ is radial in
            self.theta_error = wrap_pi(math.atan2(-delta_y, -delta_x) - self.target_pose.theta)
            self.theta_distance_error = self.theta_error*self.distance_from_target
            self.z_error = delta_z + float(self.target_pose.z)


        # hdg_deg: 0 = North, +CW (aircraft heading)
        self.yaw_enu = ((math.radians(90.0 - self.hdg_deg) + math.pi) % (2*math.pi)) - math.pi   # [-pi, pi]
          
        self.angle_towards_target_rad = np.arctan2(delta_y, delta_x)  
        
        self.error_yaw = wrap_pi(self.angle_towards_target_rad - self.yaw_enu)
        
        self.compute_commands()

    def compute_commands(self):
        
        #Compute delta-time since last command
        now = time.monotonic()
        self.dt = (now - self.last_time) if self.last_time is not None else 0.0
        self.last_time = now
        # clamp dt to kill spikes (and forbid negatives)
        self.dt = max(1e-3, min(self.dt, 0.10))

        if self.target_pose.relative:
            #Computed required accelerations based on speed errors directly
            a_r_des = self.pid_r.compute(self.r_speed_error, self.dt)
            a_z_des = self.pid_z.compute(self.z_speed_error, self.dt)
            a_theta_des = self.pid_v_theta.compute(self.theta_speed_error, self.dt)
     
        else:
            # Compute speeds based of absolute position error -> Speed -> Acceleration
            self.vel_r = self.pid_r_abs.compute(self.r_error, self.dt, -self.radial_speed_measured)
            self.vel_z = self.pid_z_abs.compute(self.z_error, self.dt, self.drone_speed.z)
            self.v_theta = self.pid_theta_abs.compute(self.theta_distance_error, self.dt, -self.tangential_speed_measured)
            
            a_r_des = self.pid_r.compute(self.vel_r - self.radial_speed_measured, self.dt)
            a_z_des = self.pid_z.compute(self.vel_z - self.drone_speed.z, self.dt)
            a_theta_des = self.pid_v_theta.compute(self.v_theta - self.tangential_speed_measured, self.dt)
        

        centripetal = self.tangential_speed_measured**2 / max(self.distance_from_target, 1e-6)
        coriolis = -self.tangential_speed_measured*self.radial_speed_measured / max(self.distance_from_target, 1e-6) if getattr(self, "distance_from_target", None) else 0.0
        
        #Centrepedial acceleration limit
        self.a_r_cmd = a_r_des + centripetal
        self.a_theta_cmd = a_theta_des + coriolis
        self.a_z_cmd = a_z_des
        
        acc_max = 6.0
        # Clamp commands to reasonable values
        self.a_r_cmd = max(min(self.a_r_cmd, acc_max), -acc_max)
        self.a_theta_cmd = max(min(self.a_theta_cmd, acc_max), -acc_max)
        self.a_z_cmd = max(min(self.a_z_cmd, acc_max), -acc_max)

        # Decompose velocities into x and y components
        self.acc_rx, self.acc_ry = self.a_r_cmd*self.unit_vector_to_target[0], self.a_r_cmd*self.unit_vector_to_target[1]
        self.acc_theta_x, self.acc_theta_y = self.a_theta_cmd*-self.unit_vector_to_target[1], self.a_theta_cmd*self.unit_vector_to_target[0]
        
        #Combination
        self.acc_x = self.acc_rx + self.acc_theta_x
        self.acc_y = self.acc_ry + self.acc_theta_y
        self.acc_z = self.a_z_cmd

        #Radial speed
        self.yaw_feed_forward = -self.tangential_speed_measured/self.distance_from_target if getattr(self, "distance_from_target", None) else 0.0

        #Pilot controls yaw rate, which modifies the goal orientation setpoint
        self.yaw_offset += self.target_pose.yaw_rate*self.dt
        if self.yaw_offset > math.pi :
            self.yaw_offset -= 2*math.pi
        elif self.yaw_offset < -math.pi:
            self.yaw_offset += 2*math.pi


        total_yaw_err = wrap_pi(self.error_yaw + self.yaw_offset)
            
        self.total_yaw_err = total_yaw_err

        self.yaw_rate = self.pid_yaw.compute(total_yaw_err, self.dt)

        #Feed forward a rate to keep same pose relative to trajectory
        
        self.yaw_rate += - self.yaw_feed_forward if self.tangential_speed_measured > 0 else self.yaw_feed_forward

        self.send_commands()

    def send_commands(self):  
        if self.estimated_center is None or self.target_pose is None:
            return None
        # Create PositionTarget message for setpoint_raw  
        target = PositionTarget()  
        target.header.stamp = self.get_clock().now().to_msg()  
        target.header.frame_id = "map"  
        target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
          
        # Type mask: IGNORE positions and velocities, DO use accelerations and yaw_rate
        target.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_YAW   # keep ignoring absolute yaw if you only command yaw_rate
        )  
        
        # Set acceleration components (these are used by MAVLink when velocity bits are ignored)
        target.acceleration_or_force.x = float(self.acc_x)  
        target.acceleration_or_force.y = float(self.acc_y)  
        target.acceleration_or_force.z = float(self.acc_z) 
        
        # If you want to command yaw rate, keep this; otherwise set yaw and clear IGNORE_YAW
        target.yaw_rate = float(self.yaw_rate)
          
        self.publisher_raw.publish(target)

        if self.talk:
            dt = time.monotonic() - self.start_time
            if self.last_delta_t is None :
                self.last_delta_t = delta_t =  dt
            else:
                alpha = 0.005
                delta_t = (dt)*alpha + self.last_delta_t*(1-alpha)

            self.delta_t = delta_t

            self.last_delta_t = delta_t

    def filter_vr_callback(self):
        if self.target_pose is None:
            return 
        
        if self.alpha is not None:
            max_rate = self.alpha   # m/s per iteration
            delta = self.target_pose.v_r - self.filtered_v_r
            delta = np.clip(delta, -max_rate, max_rate)
            self.filtered_v_r += delta
        else:
            self.filtered_v_r = self.target_pose.v_r


    def log_callback(self):
        # Comprehensive CSV logging: setpoint vs command vs measured response (relative mode)
        if not self.approach_active:
            return
        try:
            # safe accessors
            drone_z = getattr(self.drone_pose, "z", float("nan")) if getattr(self, "drone_pose", None) else float("nan")
            target_z = getattr(self.target_pose, "z", float("nan")) if getattr(self, "target_pose", None) else float("nan")

            # radial pos-hold (soft_pos_hold) info
            r_lock_pos = getattr(self, "r_pos_hold", None)
            if r_lock_pos is not None:
                r_lock_position = -getattr(self.r_pos_hold, "lock_position", float("nan"))
                r_lock_active = int(bool(getattr(self.r_pos_hold, "lock_activation", False)))
                r_pid = getattr(self.r_pos_hold, "pid", None)
                r_pid_P = getattr(r_pid, "last_p", float("nan")) if r_pid is not None else float("nan")
                r_pid_I = getattr(r_pid, "last_i", float("nan")) if r_pid is not None else float("nan")
                r_pid_D = getattr(r_pid, "last_d", float("nan")) if r_pid is not None else float("nan")
            else:
                r_lock_position = float("nan"); r_lock_active = 0
                r_pid_P = r_pid_I = r_pid_D = float("nan")

            # vertical pos-hold (soft_pos_hold) info
            z_lock_pos = getattr(self, "z_pos_hold", None)
            if z_lock_pos is not None:
                z_lock_position = getattr(self.z_pos_hold, "lock_position", float("nan"))
                z_lock_active = int(bool(getattr(self.z_pos_hold, "lock_activation", False)))
                z_pid = getattr(self.z_pos_hold, "pid", None)
                z_pid_P = getattr(z_pid, "last_p", float("nan")) if z_pid is not None else float("nan")
                z_pid_I = getattr(z_pid, "last_i", float("nan")) if z_pid is not None else float("nan")
                z_pid_D = getattr(z_pid, "last_d", float("nan")) if z_pid is not None else float("nan")
            else:
                z_lock_position = float("nan"); z_lock_active = 0
                z_pid_P = z_pid_I = z_pid_D = float("nan")

            # theta pos-hold info (actual angle, target, locked latched value and PID terms)
            theta_lock_pos = getattr(self, "theta_pos_hold", None)
            if theta_lock_pos is not None:
                # theta_pos_hold stored lock as -angle in compute_estimated_state calls, invert to get actual angle
                theta_lock_position = -getattr(self.theta_pos_hold, "lock_position", float("nan"))
                theta_lock_active = int(bool(getattr(self.theta_pos_hold, "lock_activation", False)))
                theta_pid = getattr(self.theta_pos_hold, "pid", None)
                theta_pid_P = getattr(theta_pid, "last_p", float("nan")) if theta_pid is not None else float("nan")
                theta_pid_I = getattr(theta_pid, "last_i", float("nan")) if theta_pid is not None else float("nan")
                theta_pid_D = getattr(theta_pid, "last_d", float("nan")) if theta_pid is not None else float("nan")
            else:
                theta_lock_position = float("nan"); theta_lock_active = 0
                theta_pid_P = theta_pid_I = theta_pid_D = float("nan")

            theta_actual = getattr(self, "angle_towards_target_rad", float("nan"))
            theta_target = getattr(self.target_pose, "theta", float("nan")) if getattr(self, "target_pose", None) else float("nan")

            self.csv.log(
                radius=getattr(self, "distance_from_target", float("nan")),
                # radial
                set_speed_r=(getattr(self.target_pose, "v_r", float("nan")) if self.target_pose is not None else float("nan")),
                set_speed_r_filtered=(self.filtered_v_r if self.filtered_v_r is not None else float("nan")),
                meas_speed_r=getattr(self, "radial_speed_measured", float("nan")),
                acc_cmd_r=self.a_r_cmd,
                # tangential (theta)
                set_speed_theta=(getattr(self.target_pose, "v_theta", float("nan")) if self.target_pose is not None else float("nan")),
                meas_speed_theta=getattr(self, "tangential_speed_measured", float("nan")),
                acc_cmd_theta=self.a_theta_cmd,
                # vertical
                set_speed_z=(getattr(self.target_pose, "v_z", float("nan")) if self.target_pose is not None else float("nan")),
                meas_speed_z=getattr(self, "vertical_speed_measured", float("nan")),
                acc_cmd_z=self.a_z_cmd,
                # components & published velocities
                acc_x=self.acc_x,
                acc_y=self.acc_y,
                acc_z=self.acc_z,
                pub_vel_x=getattr(self, "vel_x", float("nan")),
                pub_vel_y=getattr(self, "vel_y", float("nan")),
                pub_vel_z=getattr(self, "vel_z", float("nan")),
                # PID P/I/D terms (main controllers)
                pid_r_P=(getattr(self.pid_r, "last_p", float("nan")) if getattr(self, "pid_r", None) else float("nan")),
                pid_r_I=(getattr(self.pid_r, "last_i", float("nan")) if getattr(self, "pid_r", None) else float("nan")),
                pid_r_D=(getattr(self.pid_r, "last_d", float("nan")) if getattr(self, "pid_r", None) else float("nan")),
                pid_vtheta_P=(getattr(self.pid_v_theta, "last_p", float("nan")) if getattr(self, "pid_v_theta", None) else float("nan")),
                pid_vtheta_I=(getattr(self.pid_v_theta, "last_i", float("nan")) if getattr(self, "pid_v_theta", None) else float("nan")),
                pid_vtheta_D=(getattr(self.pid_v_theta, "last_d", float("nan")) if getattr(self, "pid_v_theta", None) else float("nan")),
                pid_z_P=(getattr(self.pid_z, "last_p", float("nan")) if getattr(self, "pid_z", None) else float("nan")),
                pid_z_I=(getattr(self.pid_z, "last_i", float("nan")) if getattr(self, "pid_z", None) else float("nan")),
                pid_z_D=(getattr(self.pid_z, "last_d", float("nan")) if getattr(self, "pid_z", None) else float("nan")),
                # yaw / heading
                yaw_enu=getattr(self, "yaw_enu", float("nan")),
                yaw_target=getattr(self, "angle_towards_target_rad", float("nan")),
                error_yaw=getattr(self, "error_yaw", float("nan")),
                total_yaw_err=getattr(self, "total_yaw_err", float("nan")),
                yaw_rate=getattr(self, "yaw_rate", float("nan")),
                # --- new soft_pos_hold & position fields ---
                drone_z=drone_z,
                target_z=target_z,
                r_lock_position=r_lock_position,
                r_lock_active=r_lock_active,
                r_pid_P=r_pid_P, r_pid_I=r_pid_I, r_pid_D=r_pid_D,
                z_lock_position=z_lock_position,
                z_lock_active=z_lock_active,
                z_pid_P=z_pid_P, z_pid_I=z_pid_I, z_pid_D=z_pid_D,
                # theta hold fields
                theta_actual=theta_actual,
                theta_target=theta_target,
                theta_lock_position=theta_lock_position,
                theta_lock_active=theta_lock_active,
                theta_pid_P=theta_pid_P, theta_pid_I=theta_pid_I, theta_pid_D=theta_pid_D,
                dt=self.dt
            )
        except Exception as e:
            # Keep operation robust if logging fails
            self.get_logger().error(f"CSV logging failed: {e}")

    def drone_pose_callback(self, msg):
        self.drone_pose = msg.pose.position

    def yaw_callback(self, msg):
        self.hdg_deg = msg.data

    def drone_speed_callback(self, msg):
        self.drone_speed = msg.twist.linear

    def goal_pose_callback(self, msg):
        self.target_pose = msg
        self.target_pose.theta = (-msg.theta+90)/180*np.pi
        

    def estimated_center_callback(self, msg):
        self.estimated_center = msg.pose.position

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
        if self.estimated_center is not None and self.drone_pose is not None:
            target.yaw = np.arctan2(
                self.estimated_center.y - self.drone_pose.y,
                self.estimated_center.x - self.drone_pose.x
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
