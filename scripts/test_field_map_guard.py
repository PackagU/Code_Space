#!/usr/bin/env python3
"""Offline contract for scripts/field_map_guard.py (no ROS, temp files only)."""

import contextlib
import hashlib
import io
import json
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import field_map_guard as guard  # noqa: E402


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def build(tmp, pointer="v2", loaded="v2", pgm_bytes=b"P5 v2", servers=1):
    maps = tmp / "maps"
    (maps / "f1").mkdir(parents=True)
    for version, payload in (("v1", b"P5 v1"), ("v2", b"P5 v2")):
        (maps / "f1" / f"m_{version}.pgm").write_bytes(payload)
        (maps / "f1" / f"m_{version}.yaml").write_text(f"image: m_{version}.pgm\nresolution: 0.05\n", encoding="utf-8")
    (maps / "f1" / "latest_map.txt").write_text(f"/ros2_ws/maps/field/f1/m_{pointer}.yaml\n", encoding="utf-8")
    pins = {
        "schema_version": 1,
        "floors": {
            "F1": {
                "yaml": "/ros2_ws/maps/field/f1/m_v2.yaml",
                "yaml_sha256": hashlib.sha256((maps / "f1/m_v2.yaml").read_bytes()).hexdigest(),
                "image": "/ros2_ws/maps/field/f1/m_v2.pgm",
                "image_sha256": hashlib.sha256(pgm_bytes).hexdigest(),
            }
        },
    }
    (maps / "map_pins.json").write_text(json.dumps(pins), encoding="utf-8")
    (maps / "waypoints.json").write_text(json.dumps({"schema_version": 1, "waypoints": {
        "f1_idle": {"floor": "F1", "frame_id": "map", "x": 1.0, "y": 2.0, "yaw_rad": 0.1},
        "f2_x": {"floor": "F2", "frame_id": "map", "x": 1.0, "y": 2.0, "yaw_rad": 0.1},
    }}), encoding="utf-8")
    proc = tmp / "proc"
    logs = tmp / "log"
    logs.mkdir()
    for index in range(servers):
        pid = 4242 + index
        (proc / str(pid)).mkdir(parents=True)
        (proc / str(pid) / "cmdline").write_bytes(b"/opt/ros/humble/lib/nav2_map_server/map_server\0--ros-args\0")
        (logs / f"map_server_{pid}_1.log").write_text(
            f"[INFO] [1.0] [map_server]: Loading yaml file: /ros2_ws/maps/field/f1/m_{loaded}.yaml\n"
            f"[INFO] [1.1] [map_server]: Read map /ros2_ws/maps/field/f1/m_{loaded}.pgm: 512 X 423 map @ 0.05 m/cell\n",
            encoding="utf-8",
        )
    (proc / "99").mkdir(parents=True)
    (proc / "99" / "cmdline").write_bytes(b"bash\0-c\0grep nav2_map_server/map_server\0")
    return maps, proc, logs


def run(tmp, maps, proc, logs, *extra):
    argv = ["check", *extra, "--pins", str(maps / "map_pins.json"), "--registry", str(maps / "waypoints.json"),
            "--map-root", str(maps), "--record-dir", str(tmp / "records"), "--proc-root", str(proc), "--log-dir", str(logs)]
    with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
        rc = guard.main(argv)
    records = sorted((tmp / "records").glob("*.json"))
    return rc, json.loads(records[-1].read_text(encoding="utf-8"))


def case(**kwargs):
    extra = kwargs.pop("extra", ("--waypoint", "f1_idle"))
    with tempfile.TemporaryDirectory() as raw:
        tmp = Path(raw)
        maps, proc, logs = build(tmp, **kwargs)
        return run(tmp, maps, proc, logs, *extra)


def main():
    rc, record = case()
    require(rc == 0 and record["result"] == "PASS", f"all-matching v2 must pass: {record}")
    require(any(c["name"] == "map_server_loaded_yaml_matches_pin" for c in record["checks"]), "goal stage checks map_server log")

    rc, record = case(pointer="v1")
    require(rc == 1 and record["result"] == "FAIL", "latest_map pointing at v1 must fail")

    rc, record = case(loaded="v1")
    require(rc == 1, "running map_server that loaded v1 must fail even when pointer is v2")

    rc, record = case(pgm_bytes=b"different")
    require(rc == 1, "pinned PGM hash mismatch must fail")

    rc, record = case(servers=0)
    require(rc == 1, "goal stage without a live map_server must fail closed")

    rc, record = case(servers=2)
    require(rc == 1, "two live map_servers are ambiguous and must fail")

    rc, record = case(servers=0, extra=("--floor", "F1", "--stage", "pre-nav"))
    require(rc == 0, "pre-nav stage checks pointer and hashes only")

    rc, record = case(servers=0, extra=("--floor", "F1", "--stage", "pre-nav", "--map-yaml", "/ros2_ws/maps/field/f1/m_v1.yaml"))
    require(rc == 1, "explicit non-pinned map at nav start must fail")

    rc, record = case(extra=("--waypoint", "f2_x"))
    require(rc == 1 and "error" in record, "floor without a pin must fail closed")

    rc, record = case(extra=("--waypoint", "missing"))
    require(rc == 1, "unknown waypoint must fail")
    print("PASS field_map_guard contract (10 cases)")


if __name__ == "__main__":
    main()
