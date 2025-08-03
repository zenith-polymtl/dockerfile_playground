from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    approach_node = Node(
        package="mission",
        executable="approach",
        name="approach_node"
    )
    
    """web_manual_control_node = Node(
            package="mission",
            executable="control",
            name="web_manual_control_node"
        )"""

    graph_node = Node(
            package="mission",
            executable="graph",
            name="graph_node"
        )
    
    abort_brake_node = Node(
            package="mission",
            executable="abort",
            name="abort_brake_node"
        )
    
    target_publisher_node = Node(
        package="mission",
        executable="targets",
        name="target_publisher_node"
    )
    
    ld.add_action(target_publisher_node)
    ld.add_action(approach_node)
    ld.add_action(graph_node)
    #ld.add_action(web_manual_control_node)
    ld.add_action(abort_brake_node)
    return ld