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
                'alpha': 0.04, #High alpha meaning high v_r change rate allowed, value should be 0.01-0.1
                'centripetal_limit': 1.5,
                'msg_interval_rate': 25.0,
                'talk' : True,
                'pid_r_kd' : 0.0,
                'pid_r_kp' : 3.75,
                'pid_r_ki' : 1.0,
                'pid_z_kd' : 0.0,
                'pid_z_kp' : 1.0,
                'pid_z_ki' : 1.0,
                'pid_v_theta_kd' : 0.0,
                'pid_v_theta_kp' : 2.5,
                'pid_v_theta_ki' : 1.0,
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