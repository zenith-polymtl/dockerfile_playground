from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    """approach_node = Node(  
        package="mission",
        executable="approach",
        name="approach_node"
    )"""
    
    approach_node_raw = Node(  
        package="mission",
        executable="approach_raw",
        name="approach_node_raw"
    )
    
    # Web manual control node à comment si test de vol réel
    control1 = Node(
            package="drone_interfaces",
            executable="DroneControlInterface",
            name="control1"
        )
    
    control2 = Node(
        package="drone_interfaces",
        executable="PolarInterface",
        name="control2"
    )
    
    
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
    #ld.add_action(approach_node) # Soit lui...
    ld.add_action(approach_node_raw) # ... soit lui!
    ld.add_action(graph_node)

    #control interfaces
    ld.add_action(control1) # aussi à commenter si test de vol réel
    ld.add_action(control2) # aussi à commenter si test de vol réel

    ld.add_action(abort_brake_node)

    return ld