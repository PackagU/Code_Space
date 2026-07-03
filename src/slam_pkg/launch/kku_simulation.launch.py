"""KKU pre-simulation: Gazebo world + robot + SLAM Toolbox + RViz2 in one launch.

Usage:
  ros2 launch slam_pkg kku_simulation.launch.py            # default floor=F1
  ros2 launch slam_pkg kku_simulation.launch.py floor:=F2
  ros2 launch slam_pkg kku_simulation.launch.py floor:=F3

Map saving (call from a second terminal once the map is good):
  ros2 run nav2_map_server map_saver_cli \\
    -f $(ros2 pkg prefix slam_pkg)/share/slam_pkg/maps/kku_virtual/f1/kku_f1

Or, if your maps/ directory is host-side and writable:
  ros2 run nav2_map_server map_saver_cli \\
    -f src/slam_pkg/maps/kku_virtual/f1/kku_f1
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_common = get_package_share_directory("common_pkg")
    pkg_slam = get_package_share_directory("slam_pkg")

    floor = LaunchConfiguration("floor")

    return LaunchDescription([
        DeclareLaunchArgument(
            "floor",
            default_value="F1",
            description="Which floor world to load: F1 | F2 | F3",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_common, "launch", "gazebo.launch.py")
            ),
            launch_arguments={
                "floor": floor,
                "use_sim_time": "true",
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_slam, "launch", "slam_toolbox.launch.py")
            ),
            launch_arguments={
                "use_sim_time": "true",
            }.items(),
        ),
    ])
