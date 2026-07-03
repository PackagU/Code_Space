#!/usr/bin/env python3
from pathlib import Path
import yaml


ROOT = Path(__file__).resolve().parents[3]
PARAMS = ROOT / "src" / "slam_pkg" / "config" / "nav2_params.yaml"


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    params = yaml.safe_load(PARAMS.read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]
    goal_checker = controller["general_goal_checker"]
    follow_path = controller["FollowPath"]

    require(goal_checker["xy_goal_tolerance"] <= 0.12, "xy goal tolerance is too loose for corner waypoints")
    require(goal_checker["yaw_goal_tolerance"] <= 0.10, "yaw goal tolerance is too loose for 90-degree turns")
    # DWB -> Regulated Pure Pursuit 교체(ad96746) 이후의 코너 충실도 계약:
    require(
        "RegulatedPurePursuitController" in follow_path["plugin"],
        "FollowPath controller should stay Regulated Pure Pursuit",
    )
    require(
        follow_path["use_rotate_to_heading"] is True,
        "rotate_to_heading keeps 90-degree corners sharp on diff drive",
    )
    require(
        follow_path["use_regulated_linear_velocity_scaling"] is True,
        "curvature-based slowdown keeps tight corners from being cut",
    )
    require(
        follow_path["min_approach_linear_velocity"] <= 0.05,
        "approach velocity should force clean stops at route waypoints",
    )

    print("PASS nav2 orthogonal tuning")


if __name__ == "__main__":
    main()
