#!/usr/bin/env python3
"""Record and classify drive command-chain stops without publishing commands.

Humble nav2_bringup remaps, checked in the container on 2026-09-13:
  controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
  behavior_server, keyboard teleop and the field web UI also publish /cmd_vel.
  /cmd_vel -> nav_safety_gate -> /cmd_vel_safe -> packagu_opencr_bridge -> OpenCR.

`record` keeps one subscriber node and appends receive-time JSONL.
`analyze` is pure Python so the classification contract is testable offline.
Receive time is the probe callback time. A starved probe delays every topic
together, so the report also prints the probe's own CPU cost.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
import os
from pathlib import Path
import re
import sys
import time

SCHEMA_VERSION = 1
ZERO_EPS = 1e-4
PROBE_NODE_NAME = "packagu_cmd_chain_probe"
MOTION_TOPICS = ("/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe", "/nav_safety/stop")
TWIST_TOPICS = ("/cmd_vel_nav", "/cmd_vel", "/cmd_vel_safe")
BOOL_TOPICS = ("/drive/ready", "/nav_safety/ready", "/nav_safety/stopped")
ALL_LEVEL_LOG_NODES = ("packagu_opencr_bridge", "nav_safety_gate", "packagu_keyboard_teleop")
NAV_LOG_NODES = ("controller_server", "velocity_smoother", "bt_navigator", "behavior_server")
WARN_LEVEL = 30

DEFAULT_CONFIG = {
    # [설정값] drive_calib.yaml / nav_safety.yaml, 2026-09-13 Jetson 기준.
    "gate_command_timeout_sec": 0.3,
    "gate_sensor_timeout_sec": 0.5,
    "gate_drive_ready_timeout_sec": 0.5,
    "bridge_cmd_timeout_sec": 0.5,
    "wheel_radius": 0.033,
    "wheel_separation": 0.51324,
    "max_wheel_rpm": 30.0,
    # [제안값] 분석 휴리스틱이며 안전 파라미터가 아니다.
    "bracket_sec": 1.0,
    "stale_demand_sec": 2.0,
    "nav_demand_sec": 0.5,
    "zero_mix_neighbor_sec": 0.15,
    "near_limit_ratio": 0.97,
    "evidence_margin_sec": 0.3,
}


# ---------------------------------------------------------------- recording


class JsonlWriter:
    def __init__(self, path: Path, max_bytes: int):
        self.handle = open(path, "w", encoding="utf-8")
        self.max_bytes = max_bytes
        self.buffer: list[str] = []
        self.bytes = 0
        self.lines = 0
        self.truncated = False

    def add(self, record: dict, force: bool = False):
        line = json.dumps(record, separators=(",", ":"), ensure_ascii=False)
        if not force:
            if self.truncated:
                return
            if self.bytes + len(line) + 1 > self.max_bytes:
                self.truncated = True
                return
        self.bytes += len(line) + 1
        self.buffer.append(line)

    def flush(self):
        if self.buffer:
            self.handle.write("\n".join(self.buffer) + "\n")
            self.handle.flush()
            self.lines += len(self.buffer)
            self.buffer.clear()

    def close(self):
        self.flush()
        self.handle.close()


def _process_label(pid: str) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            argv = [part.decode("utf-8", "replace") for part in handle.read().split(b"\0") if part]
    except OSError:
        return "?"
    for part in argv:
        if part.startswith("__node:="):
            return part.split(":=", 1)[1]
    for part in argv[1:]:
        if part.endswith(".py") or "/lib/" in part:
            return os.path.basename(part)
    return os.path.basename(argv[0]) if argv else "?"


class ProcSampler:
    """Low-rate /proc snapshot of processes visible from this container."""

    def __init__(self, top_n: int = 8):
        self.top_n = top_n
        self.ncpu = os.cpu_count() or 1
        self.prev_ticks: dict[str, int] = {}
        self.prev_total = None

    @staticmethod
    def _total_jiffies() -> int:
        with open("/proc/stat", encoding="ascii") as handle:
            return sum(int(value) for value in handle.readline().split()[1:])

    @staticmethod
    def _read_first(path: str):
        try:
            with open(path, encoding="ascii") as handle:
                return handle.read().strip()
        except OSError:
            return None

    def sample(self) -> dict:
        total = self._total_jiffies()
        ticks: dict[str, tuple[int, int]] = {}
        for entry in os.scandir("/proc"):
            if not entry.name.isdigit():
                continue
            try:
                with open(f"/proc/{entry.name}/stat", encoding="utf-8", errors="replace") as handle:
                    raw = handle.read()
                fields = raw[raw.rfind(")") + 2:].split()
                ticks[entry.name] = (int(fields[11]) + int(fields[12]), int(fields[17]))
            except (OSError, ValueError, IndexError):
                continue
        top = []
        if self.prev_total is not None and total > self.prev_total:
            span = total - self.prev_total
            for pid, (value, threads) in ticks.items():
                delta = value - self.prev_ticks.get(pid, value)
                if delta > 0:
                    top.append((100.0 * delta * self.ncpu / span, pid, threads))
        self.prev_total = total
        self.prev_ticks = {pid: value for pid, (value, _threads) in ticks.items()}
        top.sort(reverse=True)
        record = {
            "top": [[_process_label(pid), round(pct, 1), threads] for pct, pid, threads in top[: self.top_n]],
        }
        loadavg = self._read_first("/proc/loadavg")
        if loadavg:
            record["load1"] = float(loadavg.split()[0])
        meminfo = self._read_first("/proc/meminfo") or ""
        for key, name in (("MemAvailable", "mem_avail_kb"), ("SwapTotal", "swap_total_kb"), ("SwapFree", "swap_free_kb")):
            match = re.search(rf"^{key}:\s+(\d+)", meminfo, re.MULTILINE)
            if match:
                record[name] = int(match.group(1))
        temps = {}
        for zone in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
            kind = self._read_first(str(zone / "type"))
            value = self._read_first(str(zone / "temp"))
            if kind and value and value.lstrip("-").isdigit():
                temps[kind] = int(value) / 1000.0
        if temps:
            record["temp_c"] = temps
        freq = self._read_first("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
        if freq and freq.isdigit():
            record["cpu0_khz"] = int(freq)
        return record


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def run_record(args) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from rcl_interfaces.msg import Log
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

    try:
        from action_msgs.msg import GoalStatusArray
    except ImportError:  # pragma: no cover - Humble ships action_msgs
        GoalStatusArray = None

    out_dir = Path(args.output_dir)
    if out_dir.exists() and any(out_dir.iterdir()):
        print(f"error: refusing to overwrite non-empty {out_dir}", file=sys.stderr)
        return 2
    out_dir.mkdir(parents=True, exist_ok=True)
    writer = JsonlWriter(out_dir / "events.jsonl", int(args.max_mb * 1024 * 1024))
    start_mono = time.monotonic_ns()
    cpu_start = time.process_time()
    writer.add({
        "k": "meta",
        "schema": SCHEMA_VERSION,
        "start_wall_ns": time.time_ns(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
        "duration_sec": args.duration_sec,
        "config": DEFAULT_CONFIG,
    }, force=True)

    def rel() -> int:
        return time.monotonic_ns() - start_mono

    rclpy.init()
    node = Node(PROBE_NODE_NAME, start_parameter_services=False)
    reliable = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def twist_cb(topic):
        def callback(msg):
            writer.add({"t": rel(), "k": "tw", "tp": topic, "v": round(msg.linear.x, 5), "w": round(msg.angular.z, 5)})
        return callback

    def bool_cb(topic):
        def callback(msg):
            writer.add({"t": rel(), "k": "b", "tp": topic, "d": bool(msg.data)})
        return callback

    for topic in TWIST_TOPICS:
        node.create_subscription(Twist, topic, twist_cb(topic), reliable)
    for topic in BOOL_TOPICS:
        node.create_subscription(Bool, topic, bool_cb(topic), reliable)
    node.create_subscription(
        String, "/nav_safety/status",
        lambda msg: writer.add({"t": rel(), "k": "st", "d": msg.data[:120]}), latched,
    )
    node.create_subscription(
        Odometry, "/odom",
        lambda msg: writer.add({
            "t": rel(), "k": "od", "h": _stamp_ns(msg.header.stamp),
            "v": round(msg.twist.twist.linear.x, 5), "w": round(msg.twist.twist.angular.z, 5),
        }),
        reliable,
    )
    node.create_subscription(
        LaserScan, "/scan",
        lambda msg: writer.add({"t": rel(), "k": "sc", "h": _stamp_ns(msg.header.stamp)}),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Imu, "/imu",
        lambda msg: writer.add({
            "t": rel(), "k": "im", "h": _stamp_ns(msg.header.stamp), "gz": round(msg.angular_velocity.z, 5),
        }),
        reliable,
    )
    if not args.no_tf:
        wanted = {("odom", "base_footprint"), ("map", "odom")}

        def tf_cb(msg):
            now = rel()
            for transform in msg.transforms:
                pair = (transform.header.frame_id.lstrip("/"), transform.child_frame_id.lstrip("/"))
                if pair in wanted:
                    writer.add({"t": now, "k": "tf", "p": pair[0], "c": pair[1], "h": _stamp_ns(transform.header.stamp)})

        node.create_subscription(TFMessage, "/tf", tf_cb, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))

    def log_cb(msg):
        if msg.name == PROBE_NODE_NAME:
            return
        if msg.level >= WARN_LEVEL or msg.name in ALL_LEVEL_LOG_NODES:
            writer.add({
                "t": rel(), "k": "log", "n": msg.name, "l": int(msg.level),
                "m": msg.msg[:300], "h": _stamp_ns(msg.stamp),
            })

    node.create_subscription(
        Log, "/rosout", log_cb,
        QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE),
    )
    if GoalStatusArray is not None:
        def goal_cb(msg):
            states = [[bytes(status.goal_info.goal_id.uuid).hex()[:8], int(status.status)] for status in msg.status_list[-5:]]
            writer.add({"t": rel(), "k": "gs", "s": states})

        node.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", goal_cb, latched)

    sampler = None if args.no_sys else ProcSampler()
    if sampler is not None:
        sampler.sample()  # prime CPU deltas
        node.create_timer(max(0.2, args.sys_period_sec), lambda: writer.add({"t": rel(), "k": "sys", **sampler.sample()}))
    node.create_timer(0.5, writer.flush)

    deadline = None if args.duration_sec <= 0 else time.monotonic() + args.duration_sec
    print(f"[probe] recording to {out_dir}/events.jsonl; publishes no command topics", flush=True)
    try:
        while rclpy.ok():
            if deadline is not None and time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as exc:  # noqa: BLE001 - external shutdown still writes the end record
        if type(exc).__name__ != "ExternalShutdownException":
            raise
    finally:
        publishers = sorted(name for name, _types in node.get_publisher_names_and_types_by_node(node.get_name(), node.get_namespace()))
        writer.add({
            "k": "end", "t": rel(), "probe_cpu_sec": round(time.process_time() - cpu_start, 3),
            "lines": writer.lines + len(writer.buffer), "truncated": writer.truncated, "publishers": publishers,
        }, force=True)
        writer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    motion = sorted(set(publishers) & set(MOTION_TOPICS))
    if motion:
        print(f"error: probe unexpectedly owns motion publishers: {motion}", file=sys.stderr)
        return 3
    print(f"[probe] done: {out_dir}/events.jsonl", flush=True)
    return 0


# ---------------------------------------------------------------- analysis


class Series:
    def __init__(self, records: list[dict]):
        self.records = records
        self.times = [record["t"] for record in records]

    def __len__(self):
        return len(self.records)

    def last_at(self, t: int):
        index = bisect.bisect_right(self.times, t) - 1
        return self.records[index] if index >= 0 else None

    def between(self, t0: int, t1: int) -> list[dict]:
        return self.records[bisect.bisect_left(self.times, t0):bisect.bisect_right(self.times, t1)]


def load_events(path: Path):
    meta, end, events = None, None, []
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                continue  # a killed recorder may leave one partial line
            kind = record.get("k")
            if kind == "meta":
                meta = record
            elif kind == "end":
                end = record
            elif isinstance(record.get("t"), int):
                events.append(record)
    events.sort(key=lambda record: record["t"])
    return meta, events, end


def split_series(events: list[dict]) -> dict[str, Series]:
    buckets: dict[str, list[dict]] = {}
    for record in events:
        kind = record["k"]
        if kind in ("tw", "b"):
            key = record["tp"]
        elif kind == "st":
            key = "/nav_safety/status"
        elif kind == "od":
            key = "/odom"
        elif kind == "sc":
            key = "/scan"
        elif kind == "im":
            key = "/imu"
        elif kind == "tf":
            key = f"tf:{record['p']}->{record['c']}"
        elif kind == "log":
            key = "/rosout"
        elif kind == "gs":
            key = "goal"
        elif kind == "sys":
            key = "sys"
        else:
            continue
        buckets.setdefault(key, []).append(record)
    return {key: Series(records) for key, records in buckets.items()}


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile / 100
    lower, upper = math.floor(index), math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def topic_stats(series: Series) -> dict:
    times = series.times
    gaps = [(right - left) / 1e6 for left, right in zip(times, times[1:])]
    span = (times[-1] - times[0]) / 1e9 if len(times) > 1 else 0.0
    stats = {
        "count": len(times),
        "rate_hz": round(len(gaps) / span, 2) if span > 0 else 0.0,
        "gap_ms_p50": _round(_percentile(gaps, 50)),
        "gap_ms_p95": _round(_percentile(gaps, 95)),
        "gap_ms_max": _round(max(gaps)) if gaps else None,
    }
    headers = [record.get("h") for record in series.records if record.get("h")]
    if headers:
        stats["header_regressions"] = sum(1 for left, right in zip(headers, headers[1:]) if right < left)
        stats["header_duplicates"] = sum(1 for left, right in zip(headers, headers[1:]) if right == left)
    return stats


def _round(value, digits=1):
    return None if value is None else round(value, digits)


def _is_zero(record) -> bool:
    return abs(record.get("v", 0.0)) < ZERO_EPS and abs(record.get("w", 0.0)) < ZERO_EPS


def _empty() -> Series:
    return Series([])


def _max_gap_ms(series: Series, t0: int, t1: int):
    """Largest arrival gap touching [t0, t1], including the gap into the window."""
    if not len(series):
        return None
    left = max(0, bisect.bisect_left(series.times, t0) - 1)
    right = min(len(series.times) - 1, bisect.bisect_right(series.times, t1))
    window = series.times[left:right + 1]
    gaps = [(b - a) / 1e6 for a, b in zip(window, window[1:])]
    return round(max(gaps), 1) if gaps else None


def _wheel_rpm(v, w, config):
    scale = 60.0 / (2.0 * math.pi * config["wheel_radius"])
    half = config["wheel_separation"] / 2.0
    return (v - w * half) * scale, (v + w * half) * scale


def _normalize_message(text: str) -> str:
    return re.sub(r"-?\d+(\.\d+)?", "#", text)[:120]


BRIDGE_READY_FALSE = re.compile(r"drive ready=False: (.+)$")


def _bridge_reason(logs: list[dict]):
    reasons = []
    for record in logs:
        if record.get("n") == "packagu_opencr_bridge":
            match = BRIDGE_READY_FALSE.search(record.get("m", ""))
            if match:
                reasons.append(match.group(1).strip())
    return reasons


def _zero_mix(inputs: Series, t0: int, t1: int, neighbor_ns: int) -> int:
    """Count isolated zero input samples surrounded by nonzero samples."""
    window = inputs.between(t0 - neighbor_ns, t1 + neighbor_ns)
    flips = 0
    for index in range(1, len(window) - 1):
        record = window[index]
        if not (t0 <= record["t"] <= t1) or not _is_zero(record):
            continue
        before, after = window[index - 1], window[index + 1]
        if (
            not _is_zero(before) and not _is_zero(after)
            and record["t"] - before["t"] <= neighbor_ns
            and after["t"] - record["t"] <= neighbor_ns
        ):
            flips += 1
    return flips


def analyze(meta, events: list[dict], end, config: dict | None = None) -> dict:
    cfg = dict(DEFAULT_CONFIG)
    if meta and isinstance(meta.get("config"), dict):
        cfg.update(meta["config"])
    if config:
        cfg.update(config)
    ns = 1_000_000_000
    series = split_series(events)
    get = lambda key: series.get(key, _empty())  # noqa: E731
    safe, inputs, nav = get("/cmd_vel_safe"), get("/cmd_vel"), get("/cmd_vel_nav")
    statuses, logs, goals = get("/nav_safety/status"), get("/rosout"), get("goal")
    margin = int(cfg["evidence_margin_sec"] * ns)
    neighbor = int(cfg["zero_mix_neighbor_sec"] * ns)

    safe_gaps = [(b - a) for a, b in zip(safe.times, safe.times[1:])]
    period = int(_percentile(safe_gaps, 50)) if safe_gaps else int(0.05 * ns)

    def input_demand(t):
        last = inputs.last_at(t)
        return last is not None and not _is_zero(last) and (t - last["t"]) <= cfg["stale_demand_sec"] * ns

    def nav_demand(t):
        last = nav.last_at(t)
        return last is not None and not _is_zero(last) and (t - last["t"]) <= cfg["nav_demand_sec"] * ns

    def goal_executing(t):
        last = goals.last_at(t)
        return bool(last) and any(state == 2 for _goal, state in last.get("s", []))

    stutters, normal_stops = [], 0
    index = 0
    records = safe.records
    while index < len(records):
        if not _is_zero(records[index]):
            index += 1
            continue
        stop = index
        while stop + 1 < len(records) and _is_zero(records[stop + 1]):
            stop += 1
        t_start = records[index]["t"]
        has_prev = index > 0
        has_next = stop + 1 < len(records)
        t_end = records[stop + 1]["t"] if has_next else records[stop]["t"] + period
        duration_ms = (t_end - t_start) / 1e6
        run_times = [record["t"] for record in records[index:stop + 1]]
        demand = any(input_demand(t) or nav_demand(t) for t in run_times)
        bracketed = has_prev and has_next and duration_ms <= cfg["bracket_sec"] * 1000.0
        index = stop + 1
        if not (demand or bracketed):
            normal_stops += 1
            continue
        stutters.append(_classify(
            cfg, t_start, t_end, duration_ms, demand, bracketed, series, inputs, nav, statuses, logs,
            margin, neighbor, input_demand, nav_demand, goal_executing,
        ))

    by_class: dict[str, dict] = {}
    for event in stutters:
        bucket = by_class.setdefault(event["class"], {"count": 0, "total_ms": 0.0, "max_ms": 0.0})
        bucket["count"] += 1
        bucket["total_ms"] = round(bucket["total_ms"] + event["duration_ms"], 1)
        bucket["max_ms"] = max(bucket["max_ms"], event["duration_ms"])

    ready = get("/drive/ready")
    false_runs, run_start = [], None
    for record in ready.records:
        if not record.get("d") and run_start is None:
            run_start = record["t"]
        elif record.get("d") and run_start is not None:
            false_runs.append((record["t"] - run_start) / 1e6)
            run_start = None
    bridge_reasons: dict[str, int] = {}
    for reason in _bridge_reason(logs.records):
        bridge_reasons[reason] = bridge_reasons.get(reason, 0) + 1

    gate_reasons: dict[str, int] = {}
    for record in statuses.records:
        gate_reasons[record["d"]] = gate_reasons.get(record["d"], 0) + 1

    warnings: dict[str, int] = {}
    for record in logs.records:
        if record.get("l", 0) >= WARN_LEVEL:
            key = f"{record.get('n')}: {_normalize_message(record.get('m', ''))}"
            warnings[key] = warnings.get(key, 0) + 1

    odom = get("/odom")
    peak_rpm, near_limit = 0.0, 0
    for record in odom.records:
        left, right = _wheel_rpm(record.get("v", 0.0), record.get("w", 0.0), cfg)
        peak = max(abs(left), abs(right))
        peak_rpm = max(peak_rpm, peak)
        if peak >= cfg["near_limit_ratio"] * cfg["max_wheel_rpm"]:
            near_limit += 1

    span_ns = (events[-1]["t"] - events[0]["t"]) if len(events) > 1 else 0
    wall_sec = (end or {}).get("t", span_ns) / 1e9
    probe_cpu = (end or {}).get("probe_cpu_sec")
    sys_records = get("sys").records
    loads = [record["load1"] for record in sys_records if "load1" in record]
    return {
        "schema": SCHEMA_VERSION,
        "wall_sec": round(wall_sec, 2),
        "probe_cpu_sec": probe_cpu,
        "probe_cpu_pct_of_one_core": round(100.0 * probe_cpu / wall_sec, 2) if probe_cpu and wall_sec > 0 else None,
        "truncated": bool((end or {}).get("truncated")),
        "probe_publishers": (end or {}).get("publishers"),
        "topics": {key: topic_stats(value) for key, value in sorted(series.items()) if key not in ("/rosout", "goal", "sys")},
        "stutter_count": len(stutters),
        "stutter_total_ms": round(sum(event["duration_ms"] for event in stutters), 1),
        "stutter_by_class": dict(sorted(by_class.items(), key=lambda item: -item[1]["total_ms"])),
        "stutters": stutters,
        "normal_stops": normal_stops,
        "cmd_vel_zero_mix_flips": _zero_mix(inputs, inputs.times[0], inputs.times[-1], neighbor) if len(inputs) else 0,
        "drive_ready_false": {
            "count": len(false_runs),
            "max_ms": _round(max(false_runs)) if false_runs else None,
            "bridge_reasons": bridge_reasons,
        },
        "gate_reason_transitions": gate_reasons,
        "rosout_warnings": dict(sorted(warnings.items(), key=lambda item: -item[1])[:15]),
        "odom_peak_wheel_rpm": round(peak_rpm, 2),
        "odom_frames_near_wheel_limit": near_limit,
        "load1_max": max(loads) if loads else None,
        "config": cfg,
    }


def _classify(cfg, t_start, t_end, duration_ms, demand, bracketed, series, inputs, nav, statuses, logs,
              margin, neighbor, input_demand, nav_demand, goal_executing):
    ns = 1_000_000_000
    get = lambda key: series.get(key, _empty())  # noqa: E731
    reasons = [record["d"] for record in statuses.between(t_start - int(0.15 * ns), t_end)]
    last_status = statuses.last_at(t_start)
    if last_status is not None and (not reasons or reasons[0] != last_status["d"]):
        reasons.insert(0, last_status["d"])
    window_logs = logs.between(t_start - margin, t_end + int(0.05 * ns))
    bridge = _bridge_reason(window_logs)
    teleop_deadman = any(
        record.get("n") == "packagu_keyboard_teleop" and "deadman" in record.get("m", "").lower()
        for record in window_logs
    )
    nav_warnings = [
        record for record in window_logs
        if record.get("n") in NAV_LOG_NODES and record.get("l", 0) >= WARN_LEVEL
    ]
    cpu_hint = any("missed" in record.get("m", "").lower() for record in nav_warnings)
    flips = _zero_mix(inputs, t_start - neighbor, t_end, neighbor)
    last_input = inputs.last_at(t_start)
    evidence = {
        "gate_reasons": reasons[:4],
        "bridge_reasons": bridge[:3],
        "cmd_vel_gap_ms": _max_gap_ms(inputs, t_start - margin, t_end),
        "cmd_vel_nav_gap_ms": _max_gap_ms(nav, t_start - margin, t_end),
        "odom_gap_ms": _max_gap_ms(get("/odom"), t_start - margin, t_end),
        "scan_gap_ms": _max_gap_ms(get("/scan"), t_start - margin, t_end),
        "zero_mix_flips": flips,
        "nav_warnings": [f"{record['n']}: {record['m'][:100]}" for record in nav_warnings[:3]],
    }
    before = get("/odom").between(t_start - margin, t_start)
    if before:
        evidence["odom_peak_rpm_before"] = round(max(
            max(abs(value) for value in _wheel_rpm(record.get("v", 0.0), record.get("w", 0.0), cfg))
            for record in before
        ), 2)
    sys_record = get("sys").last_at(t_end)
    if sys_record is not None and "load1" in sys_record:
        evidence["load1"] = sys_record["load1"]

    reason_text = " | ".join(reasons)
    if "software stop asserted" in reason_text:
        label = "software_stop"
    elif teleop_deadman:
        label = "teleop_deadman"
    elif flips > 0:
        label = "input_zero_mix"
    elif "command exceeds field limit" in reason_text:
        label = "gate_limit_reject"
    elif "scan stale" in reason_text:
        label = "sensor_stale:scan"
    elif "odom stale" in reason_text:
        label = "sensor_stale:odom"
    elif "drive not ready" in reason_text or "drive ready stale" in reason_text:
        detail = bridge[0] if bridge else ("drive ready stale" if "drive ready stale" in reason_text else "unknown")
        label = f"drive_not_ready:{detail}"
    elif "command stale" in reason_text:
        label = "command_stale"
    elif last_input is not None and _is_zero(last_input):
        if nav_demand(t_start):
            label = "smoother_or_behavior_zero"
        elif goal_executing(t_start) or len(nav.between(t_start - 2 * ns, t_start)):
            label = "nav2_zero"
        else:
            label = "input_zero"
    else:
        label = "unexplained"
    if cpu_hint:
        evidence["cpu_hint"] = "controller loop missed rate"
    return {
        "t_sec": round(t_start / 1e9, 3),
        "duration_ms": round(duration_ms, 1),
        "class": label,
        "demand": demand,
        "bracketed": bracketed,
        "evidence": evidence,
    }


def render_report(summary: dict) -> str:
    lines = [
        f"[명령 경로 계측] 기록 {summary['wall_sec']} s, probe CPU "
        f"{summary['probe_cpu_pct_of_one_core']} % (코어 1개 기준), 잘림={summary['truncated']}",
        "토픽 수신 간격(ms): count rate_hz p50 p95 max",
    ]
    for topic, stats in summary["topics"].items():
        extra = ""
        if "header_regressions" in stats:
            extra = f" header역행={stats['header_regressions']} 중복={stats['header_duplicates']}"
        lines.append(
            f"  {topic:<26} {stats['count']:>6} {stats['rate_hz']:>6} {stats['gap_ms_p50']} "
            f"{stats['gap_ms_p95']} {stats['gap_ms_max']}{extra}"
        )
    lines.append(
        f"끊김 후보(출력 0, 입력은 주행 요구 또는 1 s 이내 재출발): {summary['stutter_count']}건, "
        f"합계 {summary['stutter_total_ms']} ms / 정상 정지 {summary['normal_stops']}건"
    )
    for label, bucket in summary["stutter_by_class"].items():
        lines.append(f"  {label}: {bucket['count']}건, 합 {bucket['total_ms']} ms, 최대 {bucket['max_ms']} ms")
    for event in sorted(summary["stutters"], key=lambda item: -item["duration_ms"])[:8]:
        evidence = event["evidence"]
        lines.append(
            f"    +{event['t_sec']} s {event['duration_ms']} ms {event['class']} | gate={evidence['gate_reasons']} "
            f"bridge={evidence['bridge_reasons']} cmd_gap={evidence['cmd_vel_gap_ms']} "
            f"odom_gap={evidence['odom_gap_ms']} scan_gap={evidence['scan_gap_ms']}"
        )
    ready = summary["drive_ready_false"]
    lines.append(f"/drive/ready false 구간: {ready['count']}건, 최대 {ready['max_ms']} ms, bridge 사유={ready['bridge_reasons']}")
    lines.append(f"/cmd_vel 고립된 0 샘플(0 섞임 의심): {summary['cmd_vel_zero_mix_flips']}")
    lines.append(f"gate 사유 전이: {summary['gate_reason_transitions']}")
    lines.append(
        f"odom 기반 바퀴 RPM 최대 {summary['odom_peak_wheel_rpm']} [계산값], "
        f"상한 근접 프레임 {summary['odom_frames_near_wheel_limit']}"
    )
    lines.append(f"load1 최대 {summary['load1_max']}")
    for key, count in summary["rosout_warnings"].items():
        lines.append(f"  경고 {count}회 {key}")
    lines.append("한계: 수신 시각은 probe callback 기준이다. Twist에는 header가 없다. 분류는 휴리스틱이며 원출력으로 재확인한다.")
    return "\n".join(lines)


def run_analyze(args) -> int:
    base = Path(args.path)
    events_path = base / "events.jsonl" if base.is_dir() else base
    meta, events, end = load_events(events_path)
    if not events:
        print(f"error: no events in {events_path}", file=sys.stderr)
        return 2
    summary = analyze(meta, events, end)
    out_dir = events_path.parent
    (out_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    report = render_report(summary)
    (out_dir / "report.txt").write_text(report + "\n", encoding="utf-8")
    print(report)
    return 0


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    record = sub.add_parser("record", help="subscribe and write events.jsonl (never publishes commands)")
    record.add_argument("--output-dir", required=True)
    record.add_argument("--duration-sec", type=float, default=120.0, help="0 = until Ctrl+C")
    record.add_argument("--max-mb", type=float, default=64.0)
    record.add_argument("--sys-period-sec", type=float, default=1.0)
    record.add_argument("--no-sys", action="store_true")
    record.add_argument("--no-tf", action="store_true")
    analyze_parser = sub.add_parser("analyze", help="classify a recorded events.jsonl")
    analyze_parser.add_argument("path")
    args = parser.parse_args(argv)
    if args.command == "record":
        if not math.isfinite(args.duration_sec) or args.duration_sec < 0 or args.max_mb <= 0:
            parser.error("duration must be >= 0 and max-mb > 0")
        return run_record(args)
    return run_analyze(args)


if __name__ == "__main__":
    raise SystemExit(main())
