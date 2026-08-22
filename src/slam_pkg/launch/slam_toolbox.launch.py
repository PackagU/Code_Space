"""SLAM Toolbox launch (시뮬레이션 + 실제 하드웨어 공용).

시뮬 (kku_simulation.launch.py가 include — RSP/LiDAR는 Gazebo 쪽이 담당):
  ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true rviz:=true

실기 매핑 (Jetson — RViz 없음, RSP+LiDAR+드라이브 포함):
  ros2 launch slam_pkg slam_toolbox.launch.py \
    use_sim_time:=false enable_drive:=true serial_port:=/dev/rplidar

실기 dry-run (센서/구동 없이 노드 기동만 — Jetson B-1 게이트):
  ros2 launch slam_pkg slam_toolbox.launch.py \
    use_sim_time:=false enable_lidar:=false enable_drive:=false
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("slam_pkg")
    pkg_common = get_package_share_directory("common_pkg")
    pkg_drive = get_package_share_directory("drive_pkg")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    urdf_file = os.path.join(pkg_common, "urdf", "delivery_robot.urdf.xacro")
    robot_desc = xacro.process_file(urdf_file).toxml()

    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    rviz = LaunchConfiguration("rviz")
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_drive = LaunchConfiguration("enable_drive")

    real_lidar = PythonExpression(
        ["'", use_sim_time, "' == 'false' and '", enable_lidar, "' == 'true'"]
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false",
                              description="Gazebo: true, 실기: false"),
        DeclareLaunchArgument("serial_port", default_value="/dev/rplidar",
                              description="RPLiDAR 포트 (udev 별칭 권장)"),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="RViz 실행 여부 (Jetson 이미지에는 rviz2 없음)"),
        DeclareLaunchArgument("enable_lidar", default_value="true",
                              description="실기 LiDAR 드라이버 기동 여부 (dry-run: false)"),
        DeclareLaunchArgument("enable_drive", default_value="false",
                              description="OpenCR 브리지 bringup 포함 여부"),

        # 실기 TF 체인: base_footprint -> ... -> laser (시뮬은 Gazebo 쪽 RSP가 담당)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            condition=UnlessCondition(use_sim_time),
            parameters=[{"robot_description": robot_desc,
                         "use_sim_time": use_sim_time}],
        ),

        # RPLiDAR A1m8 드라이버 — 실기 + enable_lidar 시에만
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar",
            condition=IfCondition(real_lidar),
            parameters=[{
                "serial_port": serial_port,
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "use_sim_time": use_sim_time,
            }],
        ),

        # OpenCR 브리지 (/cmd_vel -> 바퀴, /odom + TF + /imu)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_drive, "launch", "drive_bringup.launch.py")
            ),
            condition=IfCondition(enable_drive),
        ),

        # SLAM Toolbox (비동기 맵핑 모드)
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),

        # RViz2 시각화 — rviz:=true 시에만 (Jetson 이미지에는 rviz2 미포함)
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(rviz),
            arguments=["-d", os.path.join(pkg_share, "config", "slam_view.rviz")]
            if os.path.exists(os.path.join(pkg_share, "config", "slam_view.rviz"))
            else [],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
