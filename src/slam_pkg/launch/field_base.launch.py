"""Persistent physical base stack: TF + LiDAR + guarded drive, without SLAM/Nav2/arm."""

import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false, got {value!r}")
    return value == "true"


def _launch_setup(context, *args, **kwargs):
    if _boolean(context, "use_sim_time"):
        raise RuntimeError("field_base.launch.py is physical-only; use_sim_time must be false")

    pkg_common = get_package_share_directory("common_pkg")
    pkg_drive = get_package_share_directory("drive_pkg")
    urdf_file = os.path.join(pkg_common, "urdf", "delivery_robot.urdf.xacro")
    mappings = {
        key: LaunchConfiguration(key).perform(context)
        for key in ("laser_x", "laser_y", "laser_z", "laser_roll", "laser_pitch", "laser_yaw")
    }
    robot_desc = xacro.process_file(urdf_file, mappings=mappings).toxml()

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        )
    ]
    if _boolean(context, "enable_lidar"):
        actions.append(
            Node(
                package="rplidar_ros",
                executable="rplidar_composition",
                name="rplidar",
                output="screen",
                parameters=[{
                    "serial_port": LaunchConfiguration("lidar_port"),
                    "serial_baudrate": 115200,
                    "frame_id": "laser",
                    "angle_compensate": True,
                    "scan_mode": "Standard",
                    "use_sim_time": False,
                }],
            )
        )
    if _boolean(context, "enable_drive"):
        actions.extend([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_drive, "launch", "drive_bringup.launch.py")
                ),
                launch_arguments={
                    "serial_port": LaunchConfiguration("opencr_port"),
                    "cmd_vel_topic": "/cmd_vel_safe",
                }.items(),
            ),
            Node(
                package="drive_pkg",
                executable="nav_safety_gate",
                name="nav_safety_gate",
                output="screen",
                parameters=[
                    os.path.join(pkg_drive, "config", "nav_safety.yaml"),
                    {"input_cmd_topic": LaunchConfiguration("cmd_vel_input_topic")},
                ],
            ),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enable_lidar", default_value="true"),
        DeclareLaunchArgument(
            "enable_drive",
            default_value="false",
            description="현장 승인·바퀴 들림 H01 전에는 false 유지",
        ),
        DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
        DeclareLaunchArgument("opencr_port", default_value="/dev/opencr"),
        DeclareLaunchArgument("cmd_vel_input_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("laser_x", default_value="-0.1015"),
        DeclareLaunchArgument("laser_y", default_value="0.0"),
        DeclareLaunchArgument("laser_z", default_value="0.750"),
        DeclareLaunchArgument("laser_roll", default_value="0.0"),
        DeclareLaunchArgument("laser_pitch", default_value="0.0"),
        DeclareLaunchArgument("laser_yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
