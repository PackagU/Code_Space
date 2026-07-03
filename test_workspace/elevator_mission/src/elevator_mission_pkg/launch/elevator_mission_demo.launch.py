from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="elevator_sim_pkg",
            executable="elevator_sim_node",
            name="elevator_sim_node",
            output="screen",
        ),
        # 층 전환은 자동 오케스트레이터가 담당한다 (노드명은 내부에서
        # floor_orchestrator_node로 고정 — SwitchFloor 하드코딩 호환).
        # 수동 2-Phase 버전은 legacy/floor_orchestrator_pkg 에 보관.
        Node(
            package="auto_floor_orchestrator_pkg",
            executable="auto_floor_orchestrator_node",
            output="screen",
        ),
        Node(
            package="elevator_mission_pkg",
            executable="delivery_mission_node",
            name="delivery_mission_node",
            output="screen",
            parameters=[{
                "mission_id": "parcel_to_208",
                "dry_run_nav2": False,
            }],
        ),
    ])
