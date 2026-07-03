"""Launch the simulation-only Gazebo world swap node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("method", default_value="model_swap"),
        DeclareLaunchArgument("initial_floor", default_value="F1"),
        DeclareLaunchArgument("world_dir", default_value=""),
        DeclareLaunchArgument("status_topic", default_value="/floor_orchestrator/status"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="gazebo_world_swap_pkg",
            executable="world_swap_node",
            name="gazebo_world_swap_node",
            output="screen",
            parameters=[{
                "method": LaunchConfiguration("method"),
                "initial_floor": LaunchConfiguration("initial_floor"),
                "world_dir": LaunchConfiguration("world_dir"),
                "status_topic": LaunchConfiguration("status_topic"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
