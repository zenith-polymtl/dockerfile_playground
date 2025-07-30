from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node_targets = Node(
        package="mission",
        executable="targets",
        name="targets"
    )

    approach_node = Node(
        package="mission",
        executable="approach",
        name="approach"
    )
    
    manual_control_node = Node(
            package="mission",
            executable="control",
            name="control"
        )

    graph_node = Node(
            package="mission",
            executable="graph",
            name="graph"
        )

    ld.add_action(publisher_node_targets)
    ld.add_action(approach_node)
    ld.add_action(graph_node)
    ld.add_action(manual_control_node)
    return ld