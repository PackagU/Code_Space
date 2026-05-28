"""KKU pre-simulation Gazebo launch.

Usage:
  ros2 launch common_pkg gazebo.launch.py            # default floor=F1
  ros2 launch common_pkg gazebo.launch.py floor:=F2
  ros2 launch common_pkg gazebo.launch.py floor:=F3 use_sim_time:=true

`floor` selects which kku_f{1,2,3}.world to load and where to spawn the robot
(elevator exit_pose from kku_pre_simulation_map.yaml).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# Spawn pose per floor: elevator exit_pose from YAML (right corridor, just outside elevator door).
SPAWN_POSES = {
    "F1": (1.6, 0.0, 0.0),
    "F2": (1.6, 0.0, 0.0),
    "F3": (1.6, 0.0, 0.0),
}
SPAWN_Z = 0.05


def _launch_setup(context, *args, **kwargs):
    floor = LaunchConfiguration("floor").perform(context).upper()
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

    if floor not in SPAWN_POSES:
        raise RuntimeError(f"floor must be one of {list(SPAWN_POSES)}, got '{floor}'")

    pkg_common = get_package_share_directory("common_pkg")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    urdf_file = os.path.join(pkg_common, "urdf", "robot.urdf.xacro")
    robot_desc = xacro.process_file(urdf_file).toxml()

    world_file = os.path.join(pkg_common, "worlds", f"kku_{floor.lower()}.world")
    if not os.path.exists(world_file):
        raise RuntimeError(f"world file not found: {world_file}")

    sx, sy, syaw = SPAWN_POSES[floor]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
            ),
            launch_arguments={"world": world_file}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_desc,
                "use_sim_time": use_sim_time.lower() == "true",
            }],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_robot",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-entity", f"elevator_robot_{floor.lower()}",
                "-x", str(sx),
                "-y", str(sy),
                "-z", str(SPAWN_Z),
                "-Y", str(syaw),
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "floor",
            default_value="F1",
            description="Which floor world to load: F1 | F2 | F3",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo clock (true for simulation).",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
