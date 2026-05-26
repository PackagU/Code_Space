import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_common = get_package_share_directory("common_pkg")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    urdf_file = os.path.join(pkg_common, "urdf", "robot.urdf.xacro")
    robot_desc = xacro.process_file(urdf_file).toxml()

    world_file = os.path.join(pkg_common, "worlds", "walls.world")

    return LaunchDescription([
        # Gazebo 월드 실행 (5x5m 방 + 기둥 2개)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
            ),
            launch_arguments={"world": world_file}.items(),
        ),

        # robot_state_publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_desc,
                "use_sim_time": True,
            }],
        ),

        # Gazebo에 로봇 스폰
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_robot",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-entity", "elevator_robot",
                "-x", "0.0",
                "-y", "0.0",
                "-z", "0.05",
            ],
        ),
    ])
