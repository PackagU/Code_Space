#!/usr/bin/env python3
"""Synthetic-bag contract for scripts/analyze_nav_bag.py (writes a temp bag; no node, no publisher)."""

import math
import sys
import tempfile
from pathlib import Path

from rclpy.serialization import serialize_message  # first: ROS-less hosts SKIP on the rclpy import
import rosbag2_py
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from std_msgs.msg import Bool

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import analyze_nav_bag  # noqa: E402

T0 = 1_800_000_000.0


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def write_bag(uri):
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(uri), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    kinds = {
        "/rosout": "rcl_interfaces/msg/Log",
        "/amcl_pose": "geometry_msgs/msg/PoseWithCovarianceStamped",
        "/odom": "nav_msgs/msg/Odometry",
        "/cmd_vel_safe": "geometry_msgs/msg/Twist",
        "/navigate_to_pose/_action/status": "action_msgs/msg/GoalStatusArray",
        "/drive/ready": "std_msgs/msg/Bool",
    }
    for name, kind in kinds.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr"))

    def put(topic, msg, t):
        writer.write(topic, serialize_message(msg), int((T0 + t) * 1e9))

    amcl = PoseWithCovarianceStamped()
    amcl.pose.pose.position.x, amcl.pose.pose.position.y = 1.0, 2.0
    amcl.pose.pose.orientation.z, amcl.pose.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
    put("/amcl_pose", amcl, 0.0)
    for step in range(0, 81):
        t = step * 0.1
        moving = t <= 5.2
        x = 0.1 * min(t, 5.2)
        odom = Odometry()
        odom.pose.pose.position.x = x
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = 0.1 if moving else 0.0
        put("/odom", odom, t)
        cmd = Twist()
        if t < 5.0:
            cmd.linear.x = 0.1
        if abs(t - 2.0) < 1e-9:
            cmd.linear.x, cmd.angular.z = 0.2, 0.5
        if abs(t - 6.0) < 1e-9:
            cmd.angular.z = 0.1
        put("/cmd_vel_safe", cmd, t)
        ready = Bool()
        ready.data = not (2.95 < t < 3.25)
        put("/drive/ready", ready, t)
    log = Log()
    log.name, log.msg = "controller_server", "RegulatedPurePursuitController detected collision ahead!"
    put("/rosout", log, 3.0)
    status = GoalStatusArray()
    goal = GoalStatus()
    goal.goal_info.goal_id.uuid = list(range(16))
    for t, code in ((0.1, GoalStatus.STATUS_EXECUTING), (5.0, GoalStatus.STATUS_SUCCEEDED)):
        goal.status = code
        status.status_list = [goal]
        put("/navigate_to_pose/_action/status", status, t)
    del writer


def main():
    with tempfile.TemporaryDirectory() as raw:
        bag = Path(raw) / "bag"
        write_bag(bag)
        report, events = analyze_nav_bag.analyze(bag, 0.033, 0.4323, 48.0)
    require(report["required_missing"] == [], f"all required topics present: {report['required_missing']}")
    require(report["events"]["collision_ahead"] == 1, "one collision event")
    pose = report["collision_first"]["map_pose"]
    require(abs(pose[0] - 1.0) < 0.02 and abs(pose[1] - 2.3) < 0.02 and abs(pose[2] - math.pi / 2) < 0.02,
            f"collision map pose must compose AMCL with odom motion: {pose}")
    require(report["cmd_vel_safe"]["samples_over_rpm_limit"] == 1, "v0.2+w0.5 exceeds 48 rpm once")
    require(report["nonzero_cmd_after_goal_end"]["count"] == 1, "non-zero command after SUCCEEDED is reported")
    goal = next(iter(report["goals"].values()))
    require([name for _, name in goal] == ["EXECUTING", "SUCCEEDED"], f"goal timeline: {goal}")
    stop = report["stops_after_zero_command"][-1]
    require(0.0 < stop["stop_distance_m"] <= 0.03, f"stop distance from odom: {stop}")
    require([v for _, v in report["drive_ready_transitions"]] == [True, False, True], "drive ready drop captured")
    print("PASS analyze_nav_bag synthetic contract")


if __name__ == "__main__":
    main()
