from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    web_manual_control_node = Node(
            package="mission",
            executable="control",
            name="web_manual_control_node"
        )
    
    align_node_raw = Node(  
        package="mission",
        executable="align_raw",
        name="align_node_raw"
    )

    target_baselink_publisher_node = Node(
        package="mission",
        executable="targets_baselink",
        name="target_baselink_publisher_node"
    )

    abort_brake_node = Node(
            package="mission",
            executable="abort",
            name="abort_brake_node"
        )
    
    ld.add_action(target_baselink_publisher_node)
    ld.add_action(align_node_raw)
    ld.add_action(web_manual_control_node)
    ld.add_action(abort_brake_node)
    return ld