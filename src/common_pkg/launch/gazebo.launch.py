"""KKU pre-simulation Gazebo launch.

Usage:
  ros2 launch common_pkg gazebo.launch.py            # default floor=F1
  ros2 launch common_pkg gazebo.launch.py floor:=F2
  ros2 launch common_pkg gazebo.launch.py floor:=F1 spawn_point:=charge_station
  ros2 launch common_pkg gazebo.launch.py floor:=F2 spawn_point:=elevator_inside
  ros2 launch common_pkg gazebo.launch.py floor:=F3 use_sim_time:=true

`floor` selects which kku_f{1,2,3}.world to load and where to spawn the robot
(`spawn_point` defaults to the elevator exit for existing workflows).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


# Spawn poses per floor. ``charge_station`` is the idle/home pose in front of
# the elevator and intentionally aliases ``elevator_exit`` for now.
# World-swap restarts can use elevator_inside so the robot matches the
# orchestrator initialpose.
SPAWN_POINTS = {
    "F1": {
        "elevator_inside": (0.0, 0.0, 0.0),
        "elevator_exit": (1.6, 0.0, 0.0),
        "charge_station": (1.6, 0.0, 0.0),
    },
    "F2": {
        "elevator_inside": (0.0, 0.0, 0.0),
        "elevator_exit": (1.6, 0.0, 0.0),
        "charge_station": (1.6, 0.0, 0.0),
    },
    "F3": {
        "elevator_inside": (0.0, 0.0, 0.0),
        "elevator_exit": (1.6, 0.0, 0.0),
        "charge_station": (1.6, 0.0, 0.0),
    },
}
SPAWN_Z = 0.05


def resolve_spawn_pose(floor, spawn_point):
    floor = str(floor).upper()
    spawn_point = str(spawn_point)
    if floor not in SPAWN_POINTS:
        raise RuntimeError(f"floor must be one of {list(SPAWN_POINTS)}, got '{floor}'")
    poses = SPAWN_POINTS[floor]
    if spawn_point not in poses:
        raise RuntimeError(
            f"spawn_point must be one of {list(poses)}, got '{spawn_point}'"
        )
    return poses[spawn_point]


def _launch_setup(context, *args, **kwargs):
    floor = LaunchConfiguration("floor").perform(context).upper()
    spawn_point = LaunchConfiguration("spawn_point").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    gui = LaunchConfiguration("gui").perform(context)

    pkg_common = get_package_share_directory("common_pkg")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    urdf_name = robot_model if robot_model.endswith(".xacro") else f"{robot_model}.urdf.xacro"
    urdf_file = os.path.join(pkg_common, "urdf", urdf_name)
    if not os.path.exists(urdf_file):
        raise RuntimeError(f"robot urdf not found: {urdf_file}")
    robot_desc = xacro.process_file(urdf_file).toxml()

    world_file = os.path.join(pkg_common, "worlds", f"kku_{floor.lower()}.world")
    if not os.path.exists(world_file):
        raise RuntimeError(f"world file not found: {world_file}")

    sx, sy, syaw = resolve_spawn_pose(floor, spawn_point)

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
            ),
            launch_arguments={"world": world_file, "gui": gui}.items(),
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
            "spawn_point",
            default_value="elevator_exit",
            description=(
                "Robot spawn point: elevator_exit | elevator_inside | "
                "charge_station"
            ),
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo clock (true for simulation).",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch gzclient GUI (false = gzserver only, headless).",
        ),
        DeclareLaunchArgument(
            "robot_model",
            default_value="delivery_robot",
            description=(
                "Robot URDF to spawn (file stem under common_pkg/urdf). "
                "delivery_robot = HW팀 실측 모델(기본) | robot = 기존 모델"
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
