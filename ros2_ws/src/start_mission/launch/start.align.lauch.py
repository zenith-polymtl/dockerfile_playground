from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    align_node_raw = Node(  
        package="mission",
        executable="align_raw",
        name="align_node_raw"
    )
    
    # Web manual control node à comment si test de vol réel
    web_manual_control_node = Node(
            package="mission",
            executable="control",
            name="web_manual_control_node"
        )

    graph_baselink_node = Node(
            package="mission",
            executable="graph_baselink",
            name="graph_baselink_node"
        )
    
    abort_brake_node = Node(
            package="mission",
            executable="abort",
            name="abort_brake_node"
        )
    
    target_baselink_publisher_node = Node(
        package="mission",
        executable="targets_baselink",
        name="target_baselink_publisher_node"
    )
    
    ld.add_action(target_baselink_publisher_node)
    ld.add_action(align_node_raw)
    ld.add_action(graph_baselink_node)
    ld.add_action(web_manual_control_node) # À commenter si test de vol réel
    ld.add_action(abort_brake_node)
    return ld