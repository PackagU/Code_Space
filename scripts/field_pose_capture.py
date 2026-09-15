#!/usr/bin/env python3
"""Capture a stable field pose for a new waypoint (2026-09-15, F1 re-registration).

Subscribe-only: /amcl_pose, TF map->base_footprint, /global_costmap/costmap and
/local_costmap/costmap for N seconds. It never publishes /initialpose, a goal or
cmd_vel. The robot must already be placed on the taped floor mark and AMCL must
already be aligned (web UI or remote RViz 2D Pose Estimate).

PASS requires ([제안값] thresholds, see STABILITY): enough samples, small spread,
small AMCL covariance, AMCL and TF agreeing, and the footprint free of lethal or
unknown cells in both costmaps. --save writes a *new* registry name only after PASS.
"""

import argparse
import datetime as dt
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import field_nav_cli as navcli  # noqa: E402

KST = dt.timezone(dt.timedelta(hours=9))
RECORD_DIR = Path("/ros2_ws/logs/field_execution/pose_capture")
FOOTPRINT = [(0.033, 0.219), (0.033, -0.219), (-0.327, -0.219), (-0.327, 0.219)]
STABILITY = {  # [제안값] 2026-09-15; not field-validated
    "min_samples": 5,
    "max_std_xy_m": 0.02,
    "max_std_yaw_deg": 1.0,
    "max_cov_xy_m2": 0.04,
    "max_cov_yaw_rad2": math.radians(5.0) ** 2,
    "max_amcl_tf_delta_m": 0.05,
    "max_amcl_tf_delta_deg": 3.0,
}
LETHAL = 253  # nav2 INSCRIBED_INFLATED_OBSTACLE and above
UNKNOWN = 255


def circular_mean(angles):
    return math.atan2(sum(math.sin(a) for a in angles) / len(angles), sum(math.cos(a) for a in angles) / len(angles))


def summarize(samples):
    """samples: list of (x, y, yaw). Returns mean and population std (yaw wrapped)."""
    n = len(samples)
    if n == 0:
        return None
    mx = sum(s[0] for s in samples) / n
    my = sum(s[1] for s in samples) / n
    myaw = circular_mean([s[2] for s in samples])
    sx = math.sqrt(sum((s[0] - mx) ** 2 for s in samples) / n)
    sy = math.sqrt(sum((s[1] - my) ** 2 for s in samples) / n)
    syaw = math.sqrt(sum(math.atan2(math.sin(s[2] - myaw), math.cos(s[2] - myaw)) ** 2 for s in samples) / n)
    return {"n": n, "x": mx, "y": my, "yaw": myaw, "std_x": sx, "std_y": sy, "std_xy": math.hypot(sx, sy), "std_yaw_deg": math.degrees(syaw)}


def footprint_cost(grid, x, y, yaw, footprint=FOOTPRINT):
    """grid: dict(width,height,resolution,origin_x,origin_y,origin_yaw,data,frame).
    Returns max cost and counts of lethal/unknown cells whose centres fall inside the footprint."""
    res = grid["resolution"]
    c, s = math.cos(yaw), math.sin(yaw)
    poly = [(x + px * c - py * s, y + px * s + py * c) for px, py in footprint]
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    oc, os_ = math.cos(-grid["origin_yaw"]), math.sin(-grid["origin_yaw"])

    def inside(px, py):
        hit = False
        for i in range(len(poly)):
            x1, y1 = poly[i]
            x2, y2 = poly[(i + 1) % len(poly)]
            if (y1 > py) != (y2 > py) and px < (x2 - x1) * (py - y1) / (y2 - y1) + x1:
                hit = not hit
        return hit

    result = {"cells": 0, "max_cost": 0, "lethal_cells": 0, "unknown_cells": 0, "outside_grid_cells": 0}
    step = res / 2.0
    seen = set()
    yy = min(ys)
    while yy <= max(ys):
        xx = min(xs)
        while xx <= max(xs):
            if inside(xx, yy):
                gx = (xx - grid["origin_x"]) * oc - (yy - grid["origin_y"]) * os_
                gy = (xx - grid["origin_x"]) * os_ + (yy - grid["origin_y"]) * oc
                col, row = int(math.floor(gx / res)), int(math.floor(gy / res))
                if (col, row) not in seen:
                    seen.add((col, row))
                    result["cells"] += 1
                    if not (0 <= col < grid["width"] and 0 <= row < grid["height"]):
                        result["outside_grid_cells"] += 1
                    else:
                        value = grid["data"][row * grid["width"] + col]
                        cost = UNKNOWN if value < 0 else int(value)
                        result["max_cost"] = max(result["max_cost"], cost)
                        result["lethal_cells"] += int(LETHAL <= cost < UNKNOWN)
                        result["unknown_cells"] += int(cost == UNKNOWN)
            xx += step
        yy += step
    return result


def evaluate(amcl_summary, tf_summary, last_cov, costs):
    checks = []

    def check(name, ok, value=None):
        checks.append({"name": name, "ok": bool(ok), "value": value})

    check("amcl_samples", amcl_summary is not None and amcl_summary["n"] >= STABILITY["min_samples"], amcl_summary and amcl_summary["n"])
    check("tf_samples", tf_summary is not None and tf_summary["n"] >= STABILITY["min_samples"], tf_summary and tf_summary["n"])
    if amcl_summary and tf_summary:
        check("tf_std_xy", tf_summary["std_xy"] <= STABILITY["max_std_xy_m"], round(tf_summary["std_xy"], 4))
        check("tf_std_yaw", tf_summary["std_yaw_deg"] <= STABILITY["max_std_yaw_deg"], round(tf_summary["std_yaw_deg"], 3))
        delta = math.hypot(amcl_summary["x"] - tf_summary["x"], amcl_summary["y"] - tf_summary["y"])
        dyaw = abs(math.degrees(math.atan2(math.sin(amcl_summary["yaw"] - tf_summary["yaw"]), math.cos(amcl_summary["yaw"] - tf_summary["yaw"]))))
        check("amcl_tf_delta_xy", delta <= STABILITY["max_amcl_tf_delta_m"], round(delta, 4))
        check("amcl_tf_delta_yaw", dyaw <= STABILITY["max_amcl_tf_delta_deg"], round(dyaw, 3))
    if last_cov is not None:
        check("amcl_cov_xx", last_cov[0] <= STABILITY["max_cov_xy_m2"], round(last_cov[0], 5))
        check("amcl_cov_yy", last_cov[7] <= STABILITY["max_cov_xy_m2"], round(last_cov[7], 5))
        check("amcl_cov_yaw", last_cov[35] <= STABILITY["max_cov_yaw_rad2"], round(last_cov[35], 6))
    else:
        check("amcl_cov_present", False)
    for name in ("global_costmap", "local_costmap"):
        cost = costs.get(name)
        check(f"{name}_received", cost is not None)
        if cost is not None:
            check(f"{name}_footprint_non_lethal", cost["lethal_cells"] == 0 and cost["unknown_cells"] == 0 and cost["outside_grid_cells"] == 0, cost)
    return checks


def grid_from_msg(msg):
    q = msg.info.origin.orientation
    return {
        "frame": msg.header.frame_id,
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin_x": msg.info.origin.position.x,
        "origin_y": msg.info.origin.position.y,
        "origin_yaw": math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
        "data": list(msg.data),
    }


def capture(duration):
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = Node("fieldctl_pose_capture")
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    amcl, tf_samples, grids, covariance = [], [], {}, [None]

    def on_amcl(msg):
        p = msg.pose.pose
        amcl.append((p.position.x, p.position.y, 2.0 * math.atan2(p.orientation.z, p.orientation.w)))
        covariance[0] = list(msg.pose.covariance)

    node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_amcl, latched)
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap", lambda m: grids.__setitem__("global_costmap", m), latched)
    node.create_subscription(OccupancyGrid, "/local_costmap/costmap", lambda m: grids.__setitem__("local_costmap", m), latched)
    buffer = Buffer()
    TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + duration
    next_tf = time.monotonic()
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() >= next_tf:
                next_tf += 0.2
                if buffer.can_transform("map", "base_footprint", Time()):
                    t = buffer.lookup_transform("map", "base_footprint", Time()).transform
                    tf_samples.append((t.translation.x, t.translation.y, 2.0 * math.atan2(t.rotation.z, t.rotation.w)))
        odom_tf = None
        if "local_costmap" in grids and buffer.can_transform(grids["local_costmap"].header.frame_id, "base_footprint", Time()):
            t = buffer.lookup_transform(grids["local_costmap"].header.frame_id, "base_footprint", Time()).transform
            odom_tf = (t.translation.x, t.translation.y, 2.0 * math.atan2(t.rotation.z, t.rotation.w))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return amcl, tf_samples, {k: grid_from_msg(v) for k, v in grids.items()}, covariance[0], odom_tf


def main(argv=None):
    parser = argparse.ArgumentParser(description="subscribe-only stable pose capture for a new waypoint")
    parser.add_argument("name")
    parser.add_argument("floor")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--save", action="store_true", help="write the new waypoint only when every check passes")
    parser.add_argument("--replace", action="store_true")
    parser.add_argument("--floor-mark", default="", help="tape mark id / description written into the record")
    parser.add_argument("--map-guard-record", default="")
    parser.add_argument("--registry", default=str(navcli.DEFAULT_REGISTRY))
    parser.add_argument("--record-dir", default=str(RECORD_DIR))
    args = parser.parse_args(argv)
    name = navcli.validate_name(args.name)
    floor = navcli.validate_floor(args.floor)
    if not 5.0 <= args.duration <= 60.0:
        raise SystemExit("error: duration must be 5..60 s")

    amcl, tf_samples, grids, cov, odom_tf = capture(args.duration)
    amcl_s, tf_s = summarize(amcl), summarize(tf_samples)
    costs = {}
    if tf_s and "global_costmap" in grids:
        costs["global_costmap"] = footprint_cost(grids["global_costmap"], tf_s["x"], tf_s["y"], tf_s["yaw"])
    if odom_tf and "local_costmap" in grids:
        costs["local_costmap"] = footprint_cost(grids["local_costmap"], *odom_tf)
    checks = evaluate(amcl_s, tf_s, cov, costs)
    ok = all(c["ok"] for c in checks)
    now = dt.datetime.now(KST)
    record = {
        "captured_at_kst": now.isoformat(timespec="seconds"), "name": name, "floor": floor, "duration_s": args.duration,
        "floor_mark": args.floor_mark, "map_guard_record": args.map_guard_record, "stability_thresholds": STABILITY,
        "amcl": amcl_s, "tf_map_base_footprint": tf_s, "amcl_covariance_last": cov, "footprint_costs": costs,
        "checks": checks, "result": "PASS" if ok else "FAIL", "saved": False,
    }
    if ok and args.save:
        data = navcli.load_registry(Path(args.registry))
        if name in data["waypoints"] and not args.replace:
            record["save_error"] = f"waypoint exists: {name}"
        else:
            data["waypoints"][name] = {
                "floor": floor, "frame_id": "map", "x": round(tf_s["x"], 4), "y": round(tf_s["y"], 4), "yaw_rad": round(tf_s["yaw"], 5),
                "source": "field AMCL/TF stable capture (field_pose_capture.py)", "validation_status": "field_pose_captured_route_not_verified",
                "captured_at_kst": record["captured_at_kst"], "floor_mark": args.floor_mark,
                "map_guard_record": args.map_guard_record,
            }
            navcli.save_registry(Path(args.registry), data)
            record["saved"] = True
    Path(args.record_dir).mkdir(parents=True, exist_ok=True)
    out = Path(args.record_dir) / f"{now.strftime('%Y%m%d_%H%M%S')}_KST_{name}.json"
    out.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for c in checks:
        print(f"{'PASS' if c['ok'] else 'FAIL'} {c['name']} {json.dumps(c['value'], ensure_ascii=False)}")
    if tf_s:
        print(f"TF mean: x={tf_s['x']:.4f} y={tf_s['y']:.4f} yaw={tf_s['yaw']:.5f}")
    print(f"POSE_CAPTURE={record['result']} saved={str(record['saved']).lower()} record={out}")
    if record.get("save_error"):
        print(f"error: {record['save_error']}", file=sys.stderr)
    if not ok:
        print("do not save this pose or send a goal from it; re-align AMCL and capture again", file=sys.stderr)
    return 0 if ok and (record["saved"] or not args.save) else 1


if __name__ == "__main__":
    raise SystemExit(main())
