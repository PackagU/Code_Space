"""OpenCR 브리지 bringup. 시리얼 포트는 launch 인자 (이식성 R3)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("drive_pkg")
    calib = os.path.join(pkg_share, "config", "drive_calib.yaml")
    serial_port = LaunchConfiguration("serial_port")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/opencr",
            description="OpenCR 시리얼 포트 (udev 별칭 권장)",
        ),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/cmd_vel",
            description="브리지가 구독할 속도 명령 토픽",
        ),
        Node(
            package="drive_pkg",
            executable="opencr_bridge",
            name="packagu_opencr_bridge",
            output="screen",
            parameters=[calib, {
                "serial_port": serial_port,
                "cmd_vel_topic": cmd_vel_topic,
            }],
        ),
    ])
