from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="elevator_sim_pkg",
            executable="elevator_sim_node",
            name="elevator_sim_node",
            output="screen",
            parameters=[{
                "initial_floor": "F1",
                "floor_travel_time_sec": 5.0,
                "door_open_time_sec": 3.0,
            }],
        )
    ])
