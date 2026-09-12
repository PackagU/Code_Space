"""Saved-map Nav2 only. Physical sensors/drive stay in field_base.launch.py."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from slam_pkg.map_contract import MapContractError, validate_map_yaml


SIM_MAPS = {
    "F1": ("f1", "kku_f1.yaml"),
    "F2": ("f2", "kku_f2.yaml"),
    "F3": ("f3", "kku_f3.yaml"),
}


def _boolean(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in ("true", "false"):
        raise RuntimeError(f"{name} must be true or false, got {value!r}")
    return value == "true"


def _launch_setup(context, *args, **kwargs):
    use_sim_time = _boolean(context, "use_sim_time")
    open_rviz = _boolean(context, "rviz")
    floor = LaunchConfiguration("floor").perform(context).upper()
    map_yaml = LaunchConfiguration("map").perform(context).strip()

    pkg_slam = get_package_share_directory("slam_pkg")
    pkg_nav2 = get_package_share_directory("nav2_bringup")
    if not map_yaml:
        if not use_sim_time:
            raise RuntimeError("physical Nav2 requires map:=/absolute/path/to/map.yaml")
        if floor not in SIM_MAPS:
            raise RuntimeError(f"floor must be one of {list(SIM_MAPS)}, got {floor!r}")
        map_dir, map_file = SIM_MAPS[floor]
        map_yaml = os.path.join(pkg_slam, "maps", "kku_virtual", map_dir, map_file)
    map_yaml = os.path.abspath(os.path.expanduser(map_yaml))

    try:
        validate_map_yaml(map_yaml)
    except MapContractError as exc:
        raise RuntimeError(f"saved map rejected: {exc}") from exc

    params_file = LaunchConfiguration("params_file").perform(context).strip()
    if not params_file:
        params_file = os.path.join(pkg_slam, "config", "nav2_params.yaml")
    params_file = os.path.abspath(os.path.expanduser(params_file))
    if not os.path.isfile(params_file):
        raise RuntimeError(f"Nav2 params not found: {params_file}")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_nav2, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "slam": "False",
                "map": map_yaml,
                "use_sim_time": str(use_sim_time).lower(),
                "params_file": params_file,
                "autostart": LaunchConfiguration("autostart"),
                "use_composition": "False",
            }.items(),
        )
    ]
    if open_rviz:
        rviz_config = os.path.join(pkg_nav2, "rviz", "nav2_default_view.rviz")
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_nav2",
                output="screen",
                arguments=["-d", rviz_config] if os.path.isfile(rviz_config) else [],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )
    return actions


def generate_launch_description():
    pkg_slam = get_package_share_directory("slam_pkg")
    return LaunchDescription([
        DeclareLaunchArgument("floor", default_value="F1"),
        DeclareLaunchArgument(
            "map",
            default_value="",
            description="실차에서는 반드시 저장 지도 YAML 절대경로를 지정",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(pkg_slam, "config", "nav2_params.yaml"),
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
