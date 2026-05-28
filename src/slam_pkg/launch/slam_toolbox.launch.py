"""
SLAM Toolbox launch file (시뮬레이션 + 실제 하드웨어 공용)

시뮬레이션 실행:
  ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=true

실제 하드웨어 실행:
  ros2 launch slam_pkg slam_toolbox.launch.py use_sim_time:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("slam_pkg")
    slam_params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="시뮬레이션 시간 사용 여부 (Gazebo: true, 실제 HW: false)",
        ),

        # RPLiDAR A1m8 드라이버 — 실제 하드웨어 시에만 실행
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            name="rplidar",
            condition=UnlessCondition(use_sim_time),
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "angle_compensate": True,
                "scan_mode": "Standard",
                "use_sim_time": use_sim_time,
            }],
        ),

        # SLAM Toolbox (비동기 맵핑 모드)
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params,
                {"use_sim_time": use_sim_time},
            ],
        ),

        # RViz2 시각화
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", os.path.join(pkg_share, "config", "slam_view.rviz")]
            if os.path.exists(os.path.join(pkg_share, "config", "slam_view.rviz"))
            else [],
            parameters=[{"use_sim_time": use_sim_time}],
        ),
    ])
