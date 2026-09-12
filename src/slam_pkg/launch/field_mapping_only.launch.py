"""Physical SLAM process only; field_base.launch.py must remain running separately."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("slam_pkg")
    params = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")
    rviz_config = os.path.join(pkg_share, "config", "slam_view.rviz")
    rviz = LaunchConfiguration("rviz")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="false"),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[params, {"use_sim_time": False}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_mapping",
            output="screen",
            condition=IfCondition(rviz),
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": False}],
        ),
    ])
