#!/usr/bin/env python3
"""Offline analysis of a field navigation rosbag (2026-09-15).

Reads the bag only (rosbag2_py); never publishes, never starts a node.
Answers the questions the 2026-09-14 run could not:
  * map pose at every collision-ahead / lethal-start / no-path log line,
  * Nav2 goal status timeline versus physical stop (non-zero /cmd_vel_safe after the end),
  * commanded wheel RPM from /cmd_vel_safe and feedback-derived RPM from /odom,
  * drive-ready drops, safety-gate reasons, and stopping distance after command-to-zero.
Action `SUCCEEDED` is never treated as physical success; the report keeps them separate.
"""

import argparse
import bisect
import csv
import json
import math
import re
import sys
from pathlib import Path

EVENT_PATTERNS = {
    "collision_ahead": re.compile(r"collision ahead", re.I),
    "start_lethal": re.compile(r"Starting point in lethal space", re.I),
    "no_valid_path": re.compile(r"no valid path", re.I),
    "drive_not_ready": re.compile(r"drive ready=False", re.I),
    "rejected_feedback": re.compile(r"rejected feedback", re.I),
    "goal_succeeded": re.compile(r"Goal succeeded", re.I),
    "goal_canceled": re.compile(r"Goal canceled", re.I),
    "goal_failed": re.compile(r"Goal failed|aborted", re.I),
}
STATUS_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
TOPICS = (
    "/rosout", "/amcl_pose", "/odom", "/cmd_vel_safe", "/cmd_vel", "/drive/ready", "/nav_safety/status",
    "/nav_safety/stopped", "/navigate_to_pose/_action/status", "/initialpose",
)


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a, b):
    ax, ay, at = a
    bx, by, bt = b
    return ax + bx * math.cos(at) - by * math.sin(at), ay + bx * math.sin(at) + by * math.cos(at), wrap(at + bt)


def inverse(a):
    x, y, t = a
    return -x * math.cos(t) - y * math.sin(t), x * math.sin(t) - y * math.cos(t), -t


def wheel_rpm(v, w, radius, separation):
    k = 60.0 / (2.0 * math.pi) / radius
    return (v - w * separation / 2.0) * k, (v + w * separation / 2.0) * k


def read_bag(path, topics):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    wanted = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {t: get_message(types[t]) for t in wanted}
    data = {t: [] for t in wanted}
    while reader.has_next():
        topic, raw, stamp_ns = reader.read_next()
        data[topic].append((stamp_ns / 1e9, deserialize_message(raw, classes[topic])))
    return types, data


class PoseTrack:
    """map pose(t) = last AMCL pose composed with odom motion since that AMCL sample."""

    def __init__(self, amcl, odom):
        self.amcl = [(t, (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))) for t, m in amcl]
        self.odom = [(t, (m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation))) for t, m in odom]
        self.amcl_t = [t for t, _ in self.amcl]
        self.odom_t = [t for t, _ in self.odom]

    def _odom_at(self, t):
        if not self.odom:
            return None
        i = min(max(bisect.bisect_left(self.odom_t, t), 0), len(self.odom) - 1)
        if i > 0 and abs(self.odom_t[i - 1] - t) < abs(self.odom_t[i] - t):
            i -= 1
        return self.odom[i]

    def at(self, t):
        i = bisect.bisect_right(self.amcl_t, t) - 1
        if i < 0:
            return None, None
        ta, pa = self.amcl[i]
        o_a, o_t = self._odom_at(ta), self._odom_at(t)
        if o_a is None or o_t is None:
            return pa, t - ta
        return compose(pa, compose(inverse(o_a[1]), o_t[1])), t - ta


def analyze(bag, radius, separation, rpm_limit):
    types, data = read_bag(bag, TOPICS)
    report = {"bag": str(bag), "topics_present": sorted(types), "required_missing": [], "note": "action status is not physical success"}
    for need in ("/rosout", "/amcl_pose", "/odom", "/cmd_vel_safe", "/navigate_to_pose/_action/status"):
        if not data.get(need):
            report["required_missing"].append(need)
    track = PoseTrack(data.get("/amcl_pose", []), data.get("/odom", []))

    events = []
    for t, log in data.get("/rosout", []):
        for kind, pattern in EVENT_PATTERNS.items():
            if pattern.search(log.msg):
                pose, age = track.at(t)
                events.append({"t": round(t, 3), "kind": kind, "node": log.name, "msg": log.msg[:160],
                               "map_pose": [round(v, 3) for v in pose] if pose else None,
                               "amcl_age_s": round(age, 3) if age is not None else None})
    report["events"] = {kind: sum(1 for e in events if e["kind"] == kind) for kind in EVENT_PATTERNS}
    collisions = [e for e in events if e["kind"] == "collision_ahead"]
    if collisions:
        report["collision_first"] = collisions[0]
        report["collision_last"] = collisions[-1]
        report["collision_span_s"] = round(collisions[-1]["t"] - collisions[0]["t"], 3)

    goals = {}
    for t, msg in data.get("/navigate_to_pose/_action/status", []):
        for status in msg.status_list:
            gid = bytes(status.goal_info.goal_id.uuid).hex()
            name = STATUS_NAMES.get(status.status, str(status.status))
            timeline = goals.setdefault(gid, [])
            if not timeline or timeline[-1][1] != name:
                timeline.append((round(t, 3), name))
    report["goals"] = {gid: timeline for gid, timeline in goals.items()}
    terminal = [tl[-1][0] for tl in goals.values() if tl and tl[-1][1] in ("SUCCEEDED", "CANCELED", "ABORTED")]
    end_t = max(terminal) if terminal else None

    cmd = data.get("/cmd_vel_safe", [])
    cmd_rpm = [(t, wheel_rpm(m.linear.x, m.angular.z, radius, separation)) for t, m in cmd]
    report["cmd_vel_safe"] = {
        "samples": len(cmd),
        "max_abs_linear": round(max((abs(m.linear.x) for _, m in cmd), default=0.0), 4),
        "max_abs_angular": round(max((abs(m.angular.z) for _, m in cmd), default=0.0), 4),
        "max_abs_wheel_rpm": round(max((max(abs(l), abs(r)) for _, (l, r) in cmd_rpm), default=0.0), 2),
        "samples_over_rpm_limit": sum(1 for _, (l, r) in cmd_rpm if max(abs(l), abs(r)) > rpm_limit),
    }
    if end_t is not None:
        after = [(t, m) for t, m in cmd if t > end_t + 0.5 and (abs(m.linear.x) > 1e-6 or abs(m.angular.z) > 1e-6)]
        report["nonzero_cmd_after_goal_end"] = {"goal_end_t": end_t, "count": len(after), "last_t": round(after[-1][0], 3) if after else None}

    odom = data.get("/odom", [])
    fb_rpm = [(t, wheel_rpm(m.twist.twist.linear.x, m.twist.twist.angular.z, radius, separation)) for t, m in odom]
    report["odom_feedback_derived"] = {
        "samples": len(odom),
        "max_abs_wheel_rpm": round(max((max(abs(l), abs(r)) for _, (l, r) in fb_rpm), default=0.0), 2),
        "samples_over_rpm_limit": sum(1 for _, (l, r) in fb_rpm if max(abs(l), abs(r)) > rpm_limit),
    }

    stops = []
    odom_t = [t for t, _ in odom]
    for (t0, m0), (t1, m1) in zip(cmd, cmd[1:]):
        if abs(m0.linear.x) >= 0.05 and abs(m1.linear.x) < 1e-6:
            i = bisect.bisect_left(odom_t, t1)
            if i >= len(odom):
                continue
            x0, y0 = odom[i][1].pose.pose.position.x, odom[i][1].pose.pose.position.y
            j = i
            while j < len(odom) and abs(odom[j][1].twist.twist.linear.x) > 0.005 and odom[j][0] - t1 < 5.0:
                j += 1
            j = min(j, len(odom) - 1)
            dist = math.hypot(odom[j][1].pose.pose.position.x - x0, odom[j][1].pose.pose.position.y - y0)
            stops.append({"t": round(t1, 3), "cmd_before": round(m0.linear.x, 3), "stop_distance_m": round(dist, 3),
                          "stop_time_s": round(odom[j][0] - t1, 3)})
    report["stops_after_zero_command"] = stops

    def transitions(topic, value):
        out, last = [], object()
        for t, m in data.get(topic, []):
            v = value(m)
            if v != last:
                out.append((round(t, 3), v))
                last = v
        return out

    report["drive_ready_transitions"] = transitions("/drive/ready", lambda m: m.data)
    report["gate_status_transitions"] = transitions("/nav_safety/status", lambda m: m.data)
    report["software_stop_transitions"] = transitions("/nav_safety/stopped", lambda m: m.data)
    return report, events


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("--wheel-radius", type=float, default=0.033)
    parser.add_argument("--wheel-separation", type=float, default=0.4323)
    parser.add_argument("--rpm-limit", type=float, default=48.0)
    parser.add_argument("--out-dir", help="default: <bag>/analysis")
    args = parser.parse_args(argv)
    bag = Path(args.bag)
    report, events = analyze(bag, args.wheel_radius, args.wheel_separation, args.rpm_limit)
    out = Path(args.out_dir) if args.out_dir else bag / "analysis"
    out.mkdir(parents=True, exist_ok=True)
    (out / "nav_bag_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    with open(out / "events.csv", "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "kind", "node", "map_x", "map_y", "map_yaw", "amcl_age_s", "msg"])
        for e in events:
            pose = e["map_pose"] or [None, None, None]
            writer.writerow([e["t"], e["kind"], e["node"], *pose, e["amcl_age_s"], e["msg"]])
    summary = {k: report[k] for k in ("required_missing", "events", "cmd_vel_safe", "odom_feedback_derived") if k in report}
    summary.update({k: report[k] for k in ("collision_first", "nonzero_cmd_after_goal_end") if k in report})
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"REPORT={out / 'nav_bag_report.json'}")
    return 1 if report["required_missing"] else 0


if __name__ == "__main__":
    sys.exit(main())
