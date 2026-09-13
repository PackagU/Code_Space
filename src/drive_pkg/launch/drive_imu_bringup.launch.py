"""Opt-in wheel+IMU odometry profile; never selected by the default field path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drive_pkg")
    calib = os.path.join(pkg_share, "config", "drive_calib.yaml")
    ekf = os.path.join(pkg_share, "config", "drive_ekf.yaml")

    serial_port = LaunchConfiguration("serial_port")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    imu_x = LaunchConfiguration("imu_x")
    imu_y = LaunchConfiguration("imu_y")
    imu_z = LaunchConfiguration("imu_z")
    imu_roll = LaunchConfiguration("imu_roll")
    imu_pitch = LaunchConfiguration("imu_pitch")
    imu_yaw = LaunchConfiguration("imu_yaw")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/opencr"),
        DeclareLaunchArgument("cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("imu_x"),
        DeclareLaunchArgument("imu_y"),
        DeclareLaunchArgument("imu_z"),
        DeclareLaunchArgument("imu_roll"),
        DeclareLaunchArgument("imu_pitch"),
        DeclareLaunchArgument("imu_yaw"),
        Node(
            package="drive_pkg",
            executable="opencr_bridge",
            name="packagu_opencr_bridge",
            output="screen",
            parameters=[calib, {
                "serial_port": serial_port,
                "cmd_vel_topic": cmd_vel_topic,
                "imu_frame": "imu_link",
                "publish_tf": False,
            }],
            remappings=[("/odom", "/wheel/odom")],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="imu_static_tf",
            output="screen",
            arguments=[
                "--x", imu_x, "--y", imu_y, "--z", imu_z,
                "--roll", imu_roll, "--pitch", imu_pitch, "--yaw", imu_yaw,
                "--frame-id", "base_link", "--child-frame-id", "imu_link",
            ],
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf],
            remappings=[("/odometry/filtered", "/odom")],
        ),
    ])

