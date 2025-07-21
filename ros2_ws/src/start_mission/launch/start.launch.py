#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    publisher_node = Node(
        package="mission",
        executable="state",
        name="state"
    )

    vision_node = Node(
        package="mission",
        executable="vision",
        name="vision"
    )

    approach_node = Node(
        package="mission",
        executable="approach",
        name="approach"
    )

    winch_node = Node(
        package="mission",
        executable="winch",
        name="winch"
    )

    valve_source_node = Node(
            package="mission",
            executable="source_valve",
            name="source_valve"
        )

    valve_bucket_node = Node(
            package="mission",
            executable="bucket_valve",
            name="bucket_valve"
        )
    
    manual_control_node = Node(
            package="mission",
            executable="control",
            name="control"
        )


    ld.add_action(publisher_node)
    ld.add_action(vision_node)
    ld.add_action(approach_node)
    ld.add_action(valve_bucket_node)
    ld.add_action(valve_source_node)
    ld.add_action(winch_node)
    #ld.add_action(manual_control_node)
    return ld