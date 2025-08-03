from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    target_publisher_node = Node(
        package="mission",
        executable="target_publisher_node",
        name="target_publisher_node"
    )

    approach_node = Node(
        package="mission",
        executable="approach_node",
        name="approach_node"
    )
    
    web_manual_control_node = Node(
            package="mission",
            executable="web_manual_control_node",
            name="web_manual_control_node"
        )

    graph_node = Node(
            package="mission",
            executable="graph_node",
            name="graph_node"
        )
    
    abort_brake_node = Node(
            package="mission",
            executable="abort_brake_node",
            name="abort_brake_node"
        )
    

    ld.add_action(target_publisher_node)
    ld.add_action(approach_node)
    ld.add_action(graph_node)
    ld.add_action(web_manual_control_node)
    ld.add_action(abort_brake_node)
    return ld