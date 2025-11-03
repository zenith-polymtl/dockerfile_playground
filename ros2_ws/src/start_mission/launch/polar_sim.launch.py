from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess

def generate_launch_description():
    ld = LaunchDescription()

    position_node = Node(
            package="polar",
            executable="position",
            name="position",
            parameters=[{
                # Topics / frame
                'topic_pose': "/mavros/local_position/pose",
                'topic_vel': "/mavros/local_position/velocity_local",
                'topic_goal_polar': "/goal_pose_polar",
                'topic_estimated_center': "/estimated_center_location",
                'topic_activation': "/approach_activation",
                'topic_ctrl_activation': "/controller_activation",
                'topic_raw_setpoint': "/mavros/setpoint_raw/local",
                'frame_id': "map",

                # Rates / filters
                'alpha': 0.0, #Higher means more rate v_r rate allowed

                # Limits
                'centripetal_limit': 1.5,
                'minimal_margin': 2.0,
                'soft_repulsion_initial_radius': 5.0,

                # CSV log
                'csv_path': "approach_log_polar.csv",

                # MAVLink config
                'set_msg_interval': True,
                'msg_interval_rate': 25.0,

                # verbose
                'talk': True,
                'log': True,

                # Unified PID params (kp, ki, kd, max_i, max_out, deriv_tau, d_clip)
                # pid_r
                'pid_r_kp': 2.2, 'pid_r_ki': 1.0, 'pid_r_kd': 0.2, 'pid_r_max_i': 0.2, 'pid_r_max_out': 7.0, 'pid_r_deriv_tau': 0.0, 'pid_r_d_clip': 1.0,
                # pid_r_abs
                'pid_r_abs_kp': 0.2, 'pid_r_abs_ki': 0.1, 'pid_r_abs_kd': 0.1, 'pid_r_abs_max_i': 0.3, 'pid_r_abs_max_out': 2.0, 'pid_r_abs_deriv_tau': 0.1, 'pid_r_abs_d_clip': 0.0,
                # pid_r_hold
                'pid_r_hold_kp': 0.2, 'pid_r_hold_ki': 0.1, 'pid_r_hold_kd': 0.12, 'pid_r_hold_max_i': 0.5, 'pid_r_hold_max_out': 3.0, 'pid_r_hold_deriv_tau': 0.1, 'pid_r_hold_d_clip': 0.0,
                # pid_theta_abs
                'pid_theta_abs_kp': 0.3, 'pid_theta_abs_ki': 0.1, 'pid_theta_abs_kd': 0.15, 'pid_theta_abs_max_i': 0.5, 'pid_theta_abs_max_out': 3.0, 'pid_theta_abs_deriv_tau': 0.1, 'pid_theta_abs_d_clip': 0.0,
                # pid_theta_hold
                'pid_theta_hold_kp': 0.6, 'pid_theta_hold_ki': 0.2, 'pid_theta_hold_kd': 0.3, 'pid_theta_hold_max_i': 0.5, 'pid_theta_hold_max_out': 3.0, 'pid_theta_hold_deriv_tau': 0.1, 'pid_theta_hold_d_clip': 0.0,
                # pid_v_theta
                'pid_v_theta_kp': 2.2, 'pid_v_theta_ki': 1.0, 'pid_v_theta_kd': 0.0, 'pid_v_theta_max_i': 0.2, 'pid_v_theta_max_out': 2.25, 'pid_v_theta_deriv_tau': 0.0, 'pid_v_theta_d_clip': 0.3,
                # pid_z
                'pid_z_kp': 0.6, 'pid_z_ki': 0.0, 'pid_z_kd': 0.3, 'pid_z_max_i': 1.0, 'pid_z_max_out': 3.0, 'pid_z_deriv_tau': 0.075, 'pid_z_d_clip': 0.0,
                # pid_z_abs
                'pid_z_abs_kp': 0.3, 'pid_z_abs_ki': 0.0, 'pid_z_abs_kd': 0.25, 'pid_z_abs_max_i': 0.5, 'pid_z_abs_max_out': 3.0, 'pid_z_abs_deriv_tau': 0.1, 'pid_z_abs_d_clip': 0.8,
                # pid_z_hold
                'pid_z_hold_kp': 0.3, 'pid_z_hold_ki': 0.1, 'pid_z_hold_kd': 0.15, 'pid_z_hold_max_i': 0.5, 'pid_z_hold_max_out': 3.0, 'pid_z_hold_deriv_tau': 0.1, 'pid_z_hold_d_clip': 0.0,
                # pid_yaw
                'pid_yaw_kp': 1.5, 'pid_yaw_ki': 1.0, 'pid_yaw_kd': 0.24, 'pid_yaw_max_i': 0.4, 'pid_yaw_max_out': 6.0, 'pid_yaw_deriv_tau': 0.0, 'pid_yaw_d_clip': 0.0,
            }]
        )
    
    rc = Node(
            package="polar",
            executable="controller_interface",
            name="controller_interface",
            parameters=[{
                'v_r_max': 1.0,
                'v_thetha_max': 2.0,
                'v_z_max': 0.5,
                'talk' : False,
            }]
        )

    one_shot_fake = Node(
        package = 'polar',
        executable = 'one_shot_fake_target',
        name = 'one_shot'
    )

    control = Node(
        package = 'mission',
        executable = 'control',
        name = 'control'
    )
    
    
    ld.add_action(position_node)
    ld.add_action(one_shot_fake)
    ld.add_action(control)

    launch_rc = False
    if launch_rc:
        ld.add_action(rc)
    

    return ld