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
                'centripetal_limit': 2.0,
                'msg_interval_rate': 25.0,
                'talk' : True,
                'pid_r_kd' : 2.0,
                'pid_r_kp' : 2.0,
                'pid_yaw_kp' : 1.5,
            }]
        )
    
    rc = Node(
            package="polar",
            executable="controller_interface",
            name="controller_interface",
            parameters=[{
                'v_r_max': 0.75,
                'v_thetha_max': 1.5,
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
    #ld.add_action(control)

    launch_rc = True
    if launch_rc:
        ld.add_action(rc)
    

    return ld