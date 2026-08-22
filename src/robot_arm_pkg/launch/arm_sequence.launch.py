"""팔 시퀀스 노드 launch — 장치 경로는 인자로만 지정 (이식성 정책 R2)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="", description="예: /dev/arm_servo (빈 값=토픽 전용)"),
            DeclareLaunchArgument("serial_baud", default_value="115200"),
            DeclareLaunchArgument("rate_hz", default_value="50.0"),
            DeclareLaunchArgument("self_test", default_value="false"),
            DeclareLaunchArgument("press_cycle", default_value="1", description="1~3 (servo_protocol.PRESS_CYCLES)"),
            Node(
                package="robot_arm_pkg",
                executable="arm_sequence",
                name="packagu_arm_sequence",
                output="screen",
                parameters=[
                    {
                        "serial_port": LaunchConfiguration("serial_port"),
                        "serial_baud": LaunchConfiguration("serial_baud"),
                        "rate_hz": LaunchConfiguration("rate_hz"),
                        "self_test": LaunchConfiguration("self_test"),
                        "press_cycle": LaunchConfiguration("press_cycle"),
                    }
                ],
            ),
        ]
    )
