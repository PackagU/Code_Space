#!/usr/bin/env python3
"""Offline contract for field_cmd_chain_probe stop classification (no ROS)."""

import json
from pathlib import Path
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

import field_cmd_chain_probe as probe  # noqa: E402


def ms(value):
    return int(value * 1_000_000)


class Builder:
    def __init__(self):
        self.events = []

    def twist(self, topic, t, v, w=0.0):
        self.events.append({"t": ms(t), "k": "tw", "tp": topic, "v": v, "w": w})

    def ready(self, t, value):
        self.events.append({"t": ms(t), "k": "b", "tp": "/drive/ready", "d": value})

    def status(self, t, text):
        self.events.append({"t": ms(t), "k": "st", "d": text})

    def log(self, t, node, text, level=30):
        self.events.append({"t": ms(t), "k": "log", "n": node, "l": level, "m": text, "h": 0})

    def odom(self, t, v, w, header=None):
        self.events.append({"t": ms(t), "k": "od", "h": ms(t if header is None else header), "v": v, "w": w})

    def scan(self, t):
        self.events.append({"t": ms(t), "k": "sc", "h": ms(t)})

    def analyze(self):
        events = sorted(self.events, key=lambda record: record["t"])
        end = {"t": events[-1]["t"], "probe_cpu_sec": 0.1, "publishers": ["/parameter_events", "/rosout"]}
        return probe.analyze({"config": {}}, events, end)


def chain(builder, t0, t1, v=0.05, w=0.0, nav=True, safe_zero=(), ready_false=(), scan_gap=None):
    inside = lambda t, spans: any(a <= t <= b for a, b in spans)  # noqa: E731
    t = t0
    while t <= t1:
        if nav:
            builder.twist("/cmd_vel_nav", t, v, w)
        builder.twist("/cmd_vel", t + 2, v, w)
        zero = inside(t, safe_zero)
        builder.twist("/cmd_vel_safe", t + 5, 0.0 if zero else v, 0.0 if zero else w)
        if not inside(t + 6, ready_false):
            builder.ready(t + 6, True)
        t += 50
    for t in range(t0, t1 + 1, 20):
        builder.odom(t + 1, v, w)
    for t in range(t0, t1 + 1, 100):
        if scan_gap is None or not (scan_gap[0] <= t <= scan_gap[1]):
            builder.scan(t + 3)


def test_drive_not_ready_from_feedback_limit():
    b = Builder()
    chain(b, 0, 3000, v=0.05, w=0.20, safe_zero=[(1500, 1600)], ready_false=[(1480, 1650)])
    b.status(0, "command forwarded")
    b.ready(1480, False)
    b.log(1481, "packagu_opencr_bridge", "drive ready=False: feedback rpm over limit")
    b.status(1505, "drive not ready")
    b.status(1655, "command forwarded")
    summary = b.analyze()
    assert summary["stutter_count"] == 1, summary["stutters"]
    event = summary["stutters"][0]
    assert event["class"] == "drive_not_ready:feedback rpm over limit", event
    assert 140.0 <= event["duration_ms"] <= 160.0, event
    assert summary["drive_ready_false"]["count"] == 1
    assert summary["drive_ready_false"]["bridge_reasons"] == {"feedback rpm over limit": 1}
    assert summary["odom_peak_wheel_rpm"] > 29.0  # [계산값] 0.05 m/s + 0.20 rad/s ≈ 29.32 rpm
    assert summary["odom_frames_near_wheel_limit"] > 0


def test_zero_mixing_from_second_publisher():
    b = Builder()
    chain(b, 0, 2000, v=0.05)
    for t in range(25, 2001, 50):  # e.g. web UI idle zeros between smoother samples
        b.twist("/cmd_vel", t, 0.0)
        b.twist("/cmd_vel_safe", t + 5, 0.0)
    b.status(0, "command forwarded")
    summary = b.analyze()
    assert summary["cmd_vel_zero_mix_flips"] >= 30, summary["cmd_vel_zero_mix_flips"]
    assert list(summary["stutter_by_class"]) == ["input_zero_mix"], summary["stutter_by_class"]


def test_teleop_deadman_dip():
    b = Builder()
    for t in range(0, 3001, 50):
        zero = 1000 <= t <= 1300
        b.twist("/cmd_vel", t, 0.0 if zero else 0.10)
        b.twist("/cmd_vel_safe", t + 5, 0.0 if zero else 0.10)
        b.ready(t + 6, True)
    for t in range(0, 3001, 20):
        b.odom(t + 1, 0.10, 0.0)
    for t in range(0, 3001, 100):
        b.scan(t + 3)
    b.status(0, "command forwarded")
    b.log(1001, "packagu_keyboard_teleop", "deadman stop: no key for 0.62 s")
    summary = b.analyze()
    assert summary["stutter_count"] == 1, summary["stutters"]
    assert summary["stutters"][0]["class"] == "teleop_deadman", summary["stutters"][0]


def test_command_stale_with_controller_hint():
    b = Builder()
    for t in range(0, 3001, 50):
        if not 1400 <= t <= 1850:
            b.twist("/cmd_vel_nav", t, 0.05)
            b.twist("/cmd_vel", t + 2, 0.05)
        stale = 1700 <= t <= 1850
        b.twist("/cmd_vel_safe", t + 5, 0.0 if stale else 0.05)
        b.ready(t + 6, True)
    for t in range(0, 3001, 20):
        b.odom(t + 1, 0.05, 0.0)
    for t in range(0, 3001, 100):
        b.scan(t + 3)
    b.status(0, "command forwarded")
    b.status(1705, "command stale")
    b.status(1905, "command forwarded")
    b.log(1500, "controller_server", "Control loop missed its desired rate of 20.0000Hz")
    summary = b.analyze()
    assert summary["stutter_count"] == 1, summary["stutters"]
    event = summary["stutters"][0]
    assert event["class"] == "command_stale", event
    assert event["evidence"]["cpu_hint"] == "controller loop missed rate", event
    assert event["evidence"]["cmd_vel_gap_ms"] >= 500.0, event


def test_normal_stop_is_not_a_stutter():
    b = Builder()
    for t in range(0, 2001, 50):
        moving = t < 1000
        b.twist("/cmd_vel", t, 0.05 if moving else 0.0)
        b.twist("/cmd_vel_safe", t + 5, 0.05 if moving else 0.0)
        b.ready(t + 6, True)
    summary = b.analyze()
    assert summary["stutter_count"] == 0, summary["stutters"]
    assert summary["normal_stops"] == 1


def test_scan_stale_and_software_stop():
    b = Builder()
    chain(b, 0, 3000, safe_zero=[(1700, 1850)], scan_gap=(1200, 1900))
    b.status(0, "command forwarded")
    b.status(1705, "scan stale")
    b.status(1905, "command forwarded")
    summary = b.analyze()
    assert list(summary["stutter_by_class"]) == ["sensor_stale:scan"], summary["stutter_by_class"]
    assert summary["stutters"][0]["evidence"]["scan_gap_ms"] >= 700.0

    b = Builder()
    chain(b, 0, 2000, safe_zero=[(1000, 1200)])
    b.status(0, "command forwarded")
    b.status(1005, "software stop asserted")
    b.status(1255, "command forwarded")
    summary = b.analyze()
    assert list(summary["stutter_by_class"]) == ["software_stop"], summary["stutter_by_class"]


def test_header_regression_and_files():
    b = Builder()
    b.odom(0, 0.0, 0.0, header=100)
    b.odom(20, 0.0, 0.0, header=120)
    b.odom(40, 0.0, 0.0, header=110)  # duplicate TF owner symptom
    b.twist("/cmd_vel_safe", 0, 0.0)
    b.twist("/cmd_vel_safe", 50, 0.0)
    summary = b.analyze()
    assert summary["topics"]["/odom"]["header_regressions"] == 1

    with tempfile.TemporaryDirectory() as tmp:
        path = Path(tmp) / "events.jsonl"
        lines = [json.dumps({"k": "meta", "schema": 1, "config": {}})]
        lines += [json.dumps(record) for record in sorted(b.events, key=lambda record: record["t"])]
        lines.append(json.dumps({"k": "end", "t": ms(60), "probe_cpu_sec": 0.01, "publishers": ["/rosout"]}))
        path.write_text("\n".join(lines) + '\n{"t": 7, "k": "tw"', encoding="utf-8")  # partial tail line
        assert probe.main(["analyze", tmp]) == 0
        written = json.loads((Path(tmp) / "summary.json").read_text(encoding="utf-8"))
        assert written["topics"]["/odom"]["count"] == 3
        assert "끊김 후보" in (Path(tmp) / "report.txt").read_text(encoding="utf-8")


def test_probe_source_never_publishes_motion():
    source = (ROOT / "scripts" / "field_cmd_chain_probe.py").read_text(encoding="utf-8")
    assert "create_publisher" not in source, "probe must stay subscribe-only"
    assert "publish(" not in source.replace(".flush(", "")


def main():
    tests = [value for name, value in sorted(globals().items()) if name.startswith("test_") and callable(value)]
    for test in tests:
        test()
    print(f"field cmd chain probe contract passed ({len(tests)} tests)")


if __name__ == "__main__":
    main()
