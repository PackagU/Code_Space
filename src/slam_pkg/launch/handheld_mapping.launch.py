"""핸드헬드 매핑 launch — 구동부 없이 라이다만 들고 걸어다니며 지도를 만든다.

2026-09-10 작성. HW(OpenCR/바퀴) 미장착 상태에서 라이다 계통을 실물 검증하고
신공학관 본 매핑을 리허설하기 위한 임시 경로다.

원본 slam_toolbox.launch.py 와의 차이:
  1) drive_pkg(OpenCR 브리지)를 아예 include 하지 않는다 -> /odom 이 없다
  2) 그 빈자리를 static_transform_publisher(odom -> base_footprint, 항등)로 메운다
  3) slam_toolbox 파라미터로 slam_toolbox_handheld_params.yaml 을 쓴다
     (minimum_travel_* = 0.0. 이유는 그 파일 주석 참조)

  실기 매핑에는 이 파일을 쓰지 말 것. 원본 slam_toolbox.launch.py 를 쓴다.

사용 (컨테이너 안):
  ros2 launch slam_pkg handheld_mapping.launch.py

  포트 바꾸기:   serial_port:=/dev/ttyUSB0
  라이다 없이:   enable_lidar:=false      (노드 기동만 확인)
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_common = get_package_share_directory("common_pkg")
    pkg_slam = get_package_share_directory("slam_pkg")
    urdf_file = os.path.join(pkg_common, "urdf", "delivery_robot.urdf.xacro")
    robot_desc = xacro.process_file(urdf_file).toxml()

    handheld_params = os.path.join(
        pkg_slam, "config", "slam_toolbox_handheld_params.yaml"
    )

    serial_port = LaunchConfiguration("serial_port")
    enable_lidar = LaunchConfiguration("enable_lidar")

    return LaunchDescription([
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLiDAR 포트 (udev 별칭 권장)"),
        DeclareLaunchArgument("enable_lidar", default_value="true",
                              description="라이다 드라이버 기동 여부"),

        # base_footprint -> base_link -> ... -> laser
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_desc, "use_sim_time": False}],
        ),

        # [핸드헬드 전용] odom -> base_footprint 항등 변환.
        # 실기에서는 opencr_bridge_node 가 이 변환을 바퀴 엔코더로 발행한다.
        # 여기서는 바퀴가 없으므로 고정해두고, 이동량 추정을 전부
        # slam_toolbox 의 스캔 매칭에 맡긴다.
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="handheld_fake_odom",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "odom", "--child-frame-id", "base_footprint",
            ],
        ),

        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar",
            condition=IfCondition(enable_lidar),
            parameters=[{
                "serial_port": serial_port,
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "use_sim_time": False,
            }],
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[handheld_params, {"use_sim_time": False}],
        ),
    ])
