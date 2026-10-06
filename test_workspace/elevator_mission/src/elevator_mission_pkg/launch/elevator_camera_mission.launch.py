"""Real camera mission: no elevator simulator and one ROS serial owner for the arm."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument("serial_port", default_value=""),
        DeclareLaunchArgument("camera_pose_config", default_value=PathJoinSubstitution([
            FindPackageShare("robot_arm_pkg"), "config", "camera_views.json"])),
        DeclareLaunchArgument("reader_url", default_value="http://127.0.0.1:8765/api/state"),
        DeclareLaunchArgument("door_state_topic", default_value="/elevator/door_state"),
        DeclareLaunchArgument("require_door_confirmation", default_value="false"),
        DeclareLaunchArgument("mission_id", default_value="parcel_to_208"),
        DeclareLaunchArgument("mission_workspace", default_value=""),
        DeclareLaunchArgument("floor_maps_yaml", default_value=""),
        DeclareLaunchArgument("call_press_cycle", default_value="1"),
        DeclareLaunchArgument("destination_press_cycle", default_value="2"),
    ]
    return LaunchDescription(args + [
        Node(package="robot_arm_pkg", executable="arm_sequence", output="screen", parameters=[{
            "serial_port": ParameterValue(LaunchConfiguration("serial_port"), value_type=str),
            "camera_pose_config": LaunchConfiguration("camera_pose_config"),
            "simulation_mode": False, "enable_floor_trigger": False, "home_on_start": False,
        }]),
        Node(package="elevator_mission_pkg", executable="floor_reader_bridge", output="screen",
             parameters=[{"reader_url": LaunchConfiguration("reader_url")}]),
        Node(package="auto_floor_orchestrator_pkg", executable="auto_floor_orchestrator_node",
             output="screen", parameters=[{
                 "elevator_state_topic": "/elevator/camera_state",
                 "arrival_door_state": "camera_confirmed",
                 "dry_run_map_load": False,
                 "floor_maps_yaml": ParameterValue(LaunchConfiguration("floor_maps_yaml"), value_type=str),
             }]),
        Node(package="elevator_mission_pkg", executable="delivery_mission_node", output="screen",
             parameters=[{
                 "camera_elevator_mode": True, "dry_run_nav2": False, "tick_period_sec": .1,
                 "mission_id": LaunchConfiguration("mission_id"),
                 "workspace_root": ParameterValue(LaunchConfiguration("mission_workspace"), value_type=str),
                 "door_state_topic": LaunchConfiguration("door_state_topic"),
                 "require_door_confirmation": ParameterValue(
                     LaunchConfiguration("require_door_confirmation"), value_type=bool),
                 "call_press_cycle": ParameterValue(LaunchConfiguration("call_press_cycle"), value_type=int),
                 "destination_press_cycle": ParameterValue(LaunchConfiguration("destination_press_cycle"), value_type=int),
             }]),
    ])
