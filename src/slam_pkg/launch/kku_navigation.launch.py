"""KKU saved-map navigation launch.

Usage:
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F1
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F2
  ros2 launch slam_pkg kku_navigation.launch.py floor:=F3

Run Gazebo separately first:
  ros2 launch common_pkg gazebo.launch.py floor:=F2 use_sim_time:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MAPS = {
    "F1": ("f1", "kku_f1.yaml"),
    "F2": ("f2", "kku_f2.yaml"),
    "F3": ("f3", "kku_f3.yaml"),
}


def _launch_setup(context, *args, **kwargs):
    floor = LaunchConfiguration("floor").perform(context).upper()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context).lower()

    if floor not in MAPS:
        raise RuntimeError(f"floor must be one of {list(MAPS)}, got '{floor}'")

    pkg_slam = get_package_share_directory("slam_pkg")
    pkg_nav2 = get_package_share_directory("nav2_bringup")

    map_dir, map_file = MAPS[floor]
    map_yaml = os.path.join(pkg_slam, "maps", "kku_virtual", map_dir, map_file)
    params_file = os.path.join(pkg_slam, "config", "nav2_params.yaml")

    if not os.path.exists(map_yaml):
        raise RuntimeError(f"map yaml not found: {map_yaml}")
    if not os.path.exists(params_file):
        raise RuntimeError(f"nav2 params not found: {params_file}")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "False",
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": "true",
            }.items(),
        ),
    ]

    if rviz in ("true", "1", "yes", "on"):
        rviz_config = os.path.join(pkg_nav2, "rviz", "nav2_default_view.rviz")
        rviz_args = ["-d", rviz_config] if os.path.exists(rviz_config) else []
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav2",
                output="screen",
                arguments=rviz_args,
                parameters=[{"use_sim_time": use_sim_time.lower() == "true"}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "floor",
            default_value="F1",
            description="Which saved map to load: F1 | F2 | F3",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo clock.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Open RViz2 with Nav2 view.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
