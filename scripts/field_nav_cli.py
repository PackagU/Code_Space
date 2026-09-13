#!/usr/bin/env python3
"""Low-overhead terminal operations for field navigation.

This file deliberately keeps ROS imports inside command handlers so waypoint
registry validation can be tested without a ROS installation.  It never opens
a serial port and never sends a navigation goal from status/list commands.
"""

import argparse
import json
import math
import os
import re
import sys
import time
from pathlib import Path


DEFAULT_REGISTRY = Path("/ros2_ws/maps/field/waypoints.json")
GOAL_STATE = Path("/ros2_ws/logs/field_execution/.fieldctl_last_goal.json")
NAME_RE = re.compile(r"^[A-Za-z0-9가-힣][A-Za-z0-9가-힣_.-]{0,63}$")


def finite(value, label):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def validate_floor(value):
    floor = value.upper()
    if floor not in {"F1", "F2", "F3"}:
        raise ValueError("floor must be F1, F2, or F3")
    return floor


def validate_name(value):
    if not NAME_RE.fullmatch(value):
        raise ValueError("name must be 1-64 safe characters")
    return value


def reject_unverified_origin(x, y, yaw, allow_origin):
    if not allow_origin and abs(x) < 1e-12 and abs(y) < 1e-12 and abs(yaw) < 1e-12:
        raise ValueError("all-zero pose is refused; pass --allow-origin only for a measured map origin")


def load_registry(path):
    if not path.exists():
        return {"schema_version": 1, "waypoints": {}}
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1 or not isinstance(data.get("waypoints"), dict):
        raise ValueError(f"invalid waypoint registry: {path}")
    return data


def save_registry(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def cmd_waypoint_save(args):
    path = Path(args.registry)
    name = validate_name(args.name)
    floor = validate_floor(args.floor)
    x = finite(args.x, "x")
    y = finite(args.y, "y")
    yaw = finite(args.yaw, "yaw")
    reject_unverified_origin(x, y, yaw, args.allow_origin)
    data = load_registry(path)
    if name in data["waypoints"] and not args.replace:
        raise ValueError(f"waypoint already exists: {name}; use --replace explicitly")
    data["waypoints"][name] = {
        "floor": floor,
        "frame_id": "map",
        "x": x,
        "y": y,
        "yaw_rad": yaw,
        "source": "user-supplied field coordinate",
    }
    save_registry(path, data)
    print(f"WAYPOINT={name} floor={floor} x={x:.6f} y={y:.6f} yaw={yaw:.6f}")
    print(f"REGISTRY={path}")
    return 0


def cmd_waypoint_list(args):
    path = Path(args.registry)
    data = load_registry(path)
    if not data["waypoints"]:
        print("등록된 실측 waypoint가 없습니다 (TODO).")
        return 0
    for name, pose in sorted(data["waypoints"].items()):
        print(
            f"{name}: {pose['floor']} frame={pose['frame_id']} "
            f"x={pose['x']:.6f} y={pose['y']:.6f} yaw={pose['yaw_rad']:.6f}"
        )
    return 0


def ros_imports():
    import rclpy
    from action_msgs.msg import GoalStatus
    from action_msgs.srv import CancelGoal
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    return locals()


def make_node(name):
    ros = ros_imports()
    ros["rclpy"].init()
    return ros, ros["Node"](name)


def shutdown(ros, node):
    node.destroy_node()
    ros["rclpy"].shutdown()


def transient_qos(ros):
    return ros["QoSProfile"](
        depth=1,
        reliability=ros["ReliabilityPolicy"].RELIABLE,
        durability=ros["DurabilityPolicy"].TRANSIENT_LOCAL,
    )


def stamp_age(node, msg):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is None or (stamp.sec == 0 and stamp.nanosec == 0):
        return None
    now = node.get_clock().now().nanoseconds / 1e9
    return max(0.0, now - (stamp.sec + stamp.nanosec / 1e9))


def collect_status(ros, node, timeout_sec):
    samples = {}

    def remember(key, value):
        samples[key] = {"value": value, "received": time.monotonic()}

    def sensor_callback(key):
        return lambda msg: remember(key, stamp_age(node, msg))

    node.create_subscription(ros["LaserScan"], "/scan", sensor_callback("scan"), 10)
    node.create_subscription(ros["Odometry"], "/odom", sensor_callback("odom"), 10)
    node.create_subscription(ros["Bool"], "/drive/ready", lambda msg: remember("drive", msg.data), 10)
    node.create_subscription(
        ros["Bool"], "/nav_safety/ready", lambda msg: remember("gate", msg.data), transient_qos(ros)
    )
    node.create_subscription(
        ros["Bool"], "/nav_safety/stopped", lambda msg: remember("stopped", msg.data), transient_qos(ros)
    )
    node.create_subscription(
        ros["String"], "/nav_safety/status", lambda msg: remember("reason", msg.data), transient_qos(ros)
    )
    deadline = time.monotonic() + timeout_sec
    required = {"scan", "odom", "drive", "gate", "stopped", "reason"}
    while time.monotonic() < deadline and not required.issubset(samples):
        ros["rclpy"].spin_once(node, timeout_sec=0.05)
    return samples


def format_sensor(sample):
    if sample is None:
        return "미수신"
    age = sample["value"]
    if age is None:
        return "수신됨(header 시각 미확인)"
    return f"header_age={age:.3f}s"


def cmd_status(args):
    ros, node = make_node("fieldctl_status")
    try:
        samples = collect_status(ros, node, args.timeout)
        print(f"scan: {format_sensor(samples.get('scan'))}")
        print(f"odom: {format_sensor(samples.get('odom'))}")
        for key, label in (("drive", "drive ready"), ("gate", "gate ready"), ("stopped", "software stop")):
            sample = samples.get(key)
            print(f"{label}: {'미수신' if sample is None else str(sample['value']).lower()}")
        reason = samples.get("reason")
        reasons_ko = {
            "software stop asserted": "software stop이 걸려 있음",
            "scan stale": "scan 미수신/시간 초과",
            "odom stale": "odom 미수신/시간 초과",
            "drive ready stale": "drive ready 미수신/시간 초과",
            "drive not ready": "drive가 준비되지 않음",
            "command stale": "새 속도 명령 없음(정상 0 출력)",
            "non-finite command": "NaN/Inf 속도 명령 차단",
            "command exceeds field limit": "현장 속도 상한 초과 명령 차단",
            "command forwarded": "명령 전달 중",
        }
        if reason is None:
            reason_text = "미수신"
        else:
            raw_reason = reason["value"]
            reason_text = f"{reasons_ko.get(raw_reason, '알 수 없는 사유')} [{raw_reason}]"
        print(f"gate reason: {reason_text}")
        missing = sorted({"scan", "odom", "drive", "gate", "stopped", "reason"} - set(samples))
        if missing:
            print("missing: " + ", ".join(missing))
            return 1
        return 0
    finally:
        shutdown(ros, node)


def publish_stop_and_verify(ros, node, desired, timeout_sec=2.0):
    observed = []
    node.create_subscription(
        ros["Bool"], "/nav_safety/stopped", lambda msg: observed.append(msg.data), transient_qos(ros)
    )
    publisher = node.create_publisher(ros["Bool"], "/nav_safety/stop", 10)
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline and publisher.get_subscription_count() < 1:
        ros["rclpy"].spin_once(node, timeout_sec=0.05)
    if publisher.get_subscription_count() < 1:
        raise RuntimeError("nav_safety_gate subscriber is absent; stop state was not acknowledged")
    message = ros["Bool"]()
    message.data = desired
    for _ in range(5):
        publisher.publish(message)
        ros["rclpy"].spin_once(node, timeout_sec=0.08)
        if observed and observed[-1] is desired:
            return
    raise RuntimeError("software stop state did not acknowledge the requested value")


def cmd_stop_state(args):
    desired = args.state == "assert"
    ros, node = make_node("fieldctl_stop_state")
    try:
        publish_stop_and_verify(ros, node, desired)
        print(f"software_stop={str(desired).lower()} (gate acknowledgement observed)")
        if not desired:
            print("기존 goal은 재개하지 않습니다. 새 goal은 별도로 명시해야 합니다.")
        return 0
    finally:
        shutdown(ros, node)


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def cmd_pose(args):
    x, y, yaw = finite(args.x, "x"), finite(args.y, "y"), finite(args.yaw, "yaw")
    reject_unverified_origin(x, y, yaw, args.allow_origin)
    ros, node = make_node("fieldctl_initial_pose")
    try:
        publisher = node.create_publisher(ros["PoseWithCovarianceStamped"], "/initialpose", transient_qos(ros))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and publisher.get_subscription_count() < 1:
            ros["rclpy"].spin_once(node, timeout_sec=0.05)
        if publisher.get_subscription_count() < 1:
            raise RuntimeError("/initialpose subscriber is absent; is AMCL running?")
        msg = ros["PoseWithCovarianceStamped"]()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = quaternion_from_yaw(yaw)
        # [제안값] Field operator uncertainty, not a measured localization error.
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = math.radians(15.0) ** 2
        # 2026-09-13: AMCL /initialpose 구독은 BEST_EFFORT다. 부하(load 6.6) 중 5회 발행이 전부 유실돼
        # AMCL이 이전 위치를 유지한 채 goal이 '도착'으로 끝난 사례가 있어, /amcl_pose 반영을 확인한다.
        latest = []
        node.create_subscription(
            ros["PoseWithCovarianceStamped"], "/amcl_pose", lambda m: latest.append(m), transient_qos(ros)
        )

        def accepted():
            for m in reversed(latest):
                p = m.pose.pose
                got_yaw = 2.0 * math.atan2(p.orientation.z, p.orientation.w)
                d_yaw = abs(math.atan2(math.sin(got_yaw - yaw), math.cos(got_yaw - yaw)))
                if math.hypot(p.position.x - x, p.position.y - y) <= 0.30 and d_yaw <= 0.35:
                    return p, got_yaw
            return None

        deadline = time.monotonic() + 8.0
        result = None
        while time.monotonic() < deadline and result is None:
            latest.clear()
            msg.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(msg)
            until = time.monotonic() + 0.5
            while time.monotonic() < until and result is None:
                ros["rclpy"].spin_once(node, timeout_sec=0.05)
                result = accepted()
        if result is None:
            raise RuntimeError("AMCL did not reflect /initialpose within 8 s; do not send a goal")
        p, got_yaw = result
        print(f"initialpose published: frame=map x={x:.6f} y={y:.6f} yaw={yaw:.6f}")
        print(f"AMCL accepted: x={p.position.x:.3f} y={p.position.y:.3f} yaw={got_yaw:.3f}")
        print("실제 위치 일치는 현장에서 확인해야 합니다.")
        return 0
    finally:
        shutdown(ros, node)


def write_goal_state(goal_id, name, floor):
    GOAL_STATE.parent.mkdir(parents=True, exist_ok=True)
    temporary = GOAL_STATE.with_suffix(".tmp")
    temporary.write_text(
        json.dumps({"goal_id": goal_id, "name": name, "floor": floor}, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.replace(temporary, GOAL_STATE)


def goal_id_hex(goal_handle):
    return bytes(goal_handle.goal_id.uuid).hex()


def cancel_specific(ros, node, goal_handle):
    future = goal_handle.cancel_goal_async()
    ros["rclpy"].spin_until_future_complete(node, future, timeout_sec=3.0)
    result = future.result()
    return result is not None and bool(result.goals_canceling)


def cmd_goal(args):
    data = load_registry(Path(args.registry))
    name = validate_name(args.name)
    if name not in data["waypoints"]:
        raise ValueError(f"unknown waypoint: {name}")
    pose = data["waypoints"][name]
    if args.floor and validate_floor(args.floor) != pose["floor"]:
        raise ValueError(f"waypoint floor is {pose['floor']}, not {args.floor.upper()}")
    ros, node = make_node("fieldctl_goal")
    try:
        readiness = collect_status(ros, node, 1.5)
        if readiness.get("gate", {}).get("value") is not True:
            reason = readiness.get("reason", {}).get("value", "unknown")
            raise RuntimeError(f"gate is not ready: {reason}")
        if readiness.get("stopped", {}).get("value") is not False:
            raise RuntimeError("software stop is asserted or unobserved; use resume explicitly")

        tf_buffer = ros["Buffer"]()
        listener = ros["TransformListener"](tf_buffer, node, spin_thread=False)
        deadline = time.monotonic() + 3.0
        transform_ok = False
        while time.monotonic() < deadline:
            ros["rclpy"].spin_once(node, timeout_sec=0.05)
            if tf_buffer.can_transform("map", "base_footprint", ros["Time"]()):
                transform_ok = True
                break
        if not transform_ok:
            raise RuntimeError("map->base_footprint transform is unavailable; localization is not ready")
        del listener

        client = ros["ActionClient"](node, ros["NavigateToPose"], "/navigate_to_pose")
        if not client.wait_for_server(timeout_sec=3.0):
            raise RuntimeError("NavigateToPose action server is unavailable")
        goal = ros["NavigateToPose"].Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose["x"])
        goal.pose.pose.position.y = float(pose["y"])
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = quaternion_from_yaw(float(pose["yaw_rad"]))
        last_feedback = [0.0]

        def feedback_callback(message):
            now = time.monotonic()
            if now - last_feedback[0] >= 1.0:
                print(f"feedback distance_remaining={message.feedback.distance_remaining:.3f}m", flush=True)
                last_feedback[0] = now

        send_future = client.send_goal_async(goal, feedback_callback=feedback_callback)
        ros["rclpy"].spin_until_future_complete(node, send_future, timeout_sec=5.0)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("NavigateToPose goal was rejected")
        identifier = goal_id_hex(handle)
        write_goal_state(identifier, name, pose["floor"])
        print(f"goal accepted: id={identifier} name={name} floor={pose['floor']}", flush=True)
        result_future = handle.get_result_async()
        try:
            while not result_future.done():
                ros["rclpy"].spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            print("Ctrl+C: gate stop을 먼저 요청하고 해당 goal을 취소합니다.", file=sys.stderr)
            publish_stop_and_verify(ros, node, True)
            cancelled = cancel_specific(ros, node, handle)
            print(f"goal cancel acknowledged={str(cancelled).lower()}", file=sys.stderr)
            return 130
        wrapped = result_future.result()
        status_names = {
            ros["GoalStatus"].STATUS_SUCCEEDED: "SUCCEEDED",
            ros["GoalStatus"].STATUS_CANCELED: "CANCELED",
            ros["GoalStatus"].STATUS_ABORTED: "ABORTED",
        }
        status = status_names.get(wrapped.status, f"STATUS_{wrapped.status}")
        print(f"goal result: id={identifier} status={status}")
        print("action 결과는 물리 도착·정지의 현장 관찰을 대신하지 않습니다.")
        if GOAL_STATE.exists():
            GOAL_STATE.unlink()
        return 0 if wrapped.status == ros["GoalStatus"].STATUS_SUCCEEDED else 1
    finally:
        shutdown(ros, node)


def cmd_cancel(args):
    if not GOAL_STATE.exists() and not args.all:
        raise RuntimeError("saved active goal id is absent; refusing broad cancel (use --all explicitly)")
    identifier = "0" * 32 if args.all else json.loads(GOAL_STATE.read_text(encoding="utf-8"))["goal_id"]
    if len(identifier) != 32 or not re.fullmatch(r"[0-9a-fA-F]+", identifier):
        raise RuntimeError("saved goal id is invalid")
    ros, node = make_node("fieldctl_cancel")
    try:
        client = node.create_client(ros["CancelGoal"], "/navigate_to_pose/_action/cancel_goal")
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("NavigateToPose cancel service is unavailable")
        request = ros["CancelGoal"].Request()
        request.goal_info.goal_id.uuid = list(bytes.fromhex(identifier))
        future = client.call_async(request)
        ros["rclpy"].spin_until_future_complete(node, future, timeout_sec=3.0)
        response = future.result()
        if response is None:
            raise RuntimeError("cancel service did not answer")
        accepted = bool(response.goals_canceling)
        print(f"cancel return_code={response.return_code} goals_canceling={len(response.goals_canceling)}")
        if accepted and GOAL_STATE.exists():
            GOAL_STATE.unlink()
        return 0 if accepted else 1
    finally:
        shutdown(ros, node)


def build_parser():
    parser = argparse.ArgumentParser(description="Packagu field navigation terminal helper")
    sub = parser.add_subparsers(dest="command", required=True)

    status = sub.add_parser("status")
    status.add_argument("--timeout", type=float, default=1.5)
    status.set_defaults(handler=cmd_status)

    stop = sub.add_parser("stop-state")
    stop.add_argument("state", choices=("assert", "release"))
    stop.set_defaults(handler=cmd_stop_state)

    pose = sub.add_parser("pose")
    pose.add_argument("x")
    pose.add_argument("y")
    pose.add_argument("yaw")
    pose.add_argument("--allow-origin", action="store_true")
    pose.set_defaults(handler=cmd_pose)

    waypoint = sub.add_parser("waypoint-save")
    waypoint.add_argument("name")
    waypoint.add_argument("floor")
    waypoint.add_argument("x")
    waypoint.add_argument("y")
    waypoint.add_argument("yaw")
    waypoint.add_argument("--allow-origin", action="store_true")
    waypoint.add_argument("--replace", action="store_true")
    waypoint.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    waypoint.set_defaults(handler=cmd_waypoint_save)

    listing = sub.add_parser("waypoint-list")
    listing.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    listing.set_defaults(handler=cmd_waypoint_list)

    goal = sub.add_parser("goal")
    goal.add_argument("name")
    goal.add_argument("--floor")
    goal.add_argument("--registry", default=str(DEFAULT_REGISTRY))
    goal.set_defaults(handler=cmd_goal)

    cancel = sub.add_parser("cancel")
    cancel.add_argument("--all", action="store_true")
    cancel.set_defaults(handler=cmd_cancel)
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        return args.handler(args)
    except (ValueError, RuntimeError, OSError, json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
