"""Launch the automatic floor orchestrator.

Dry-run (no Nav2 needed):
  ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py

Live, with Nav2 running:
  ros2 launch auto_floor_orchestrator_pkg auto_floor_orchestrator.launch.py \
      dry_run_map_load:=false target_floor:=F2 use_sim_time:=true

Do NOT launch the legacy manual floor_orchestrator_node at the same time:
this node intentionally claims the same node name and services.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("current_floor", default_value="F1"),
        DeclareLaunchArgument("target_floor", default_value="F2"),
        DeclareLaunchArgument("spawn_point_id", default_value="elevator_inside"),
        DeclareLaunchArgument(
            "dry_run_map_load", default_value="true",
            description="true: no Nav2 calls, just the state machine + initialpose"),
        DeclareLaunchArgument(
            "floor_maps_yaml", default_value="",
            description="override path to floor_maps.yaml (empty = source-tree default)"),
        DeclareLaunchArgument("load_map_service", default_value="/map_server/load_map"),
        DeclareLaunchArgument("publish_initialpose", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="auto_floor_orchestrator_pkg",
            executable="auto_floor_orchestrator_node",
            # node name is fixed to floor_orchestrator_node inside the class:
            # legacy SwitchFloor hardcodes /floor_orchestrator_node/set_parameters
            output="screen",
            parameters=[{
                "current_floor": LaunchConfiguration("current_floor"),
                "target_floor": LaunchConfiguration("target_floor"),
                "spawn_point_id": LaunchConfiguration("spawn_point_id"),
                "dry_run_map_load": LaunchConfiguration("dry_run_map_load"),
                "floor_maps_yaml": LaunchConfiguration("floor_maps_yaml"),
                "load_map_service": LaunchConfiguration("load_map_service"),
                "publish_initialpose": LaunchConfiguration("publish_initialpose"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        ),
    ])
