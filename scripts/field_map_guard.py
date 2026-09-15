#!/usr/bin/env python3
"""Fail-closed map identity guard for field navigation (2026-09-15).

Before Nav2 start and before every goal, the floor's pinned map must match all of:
  1. maps/field/<floor>/latest_map.txt content (the default pointer),
  2. SHA256 of the YAML and of the PGM that YAML names,
  3. (goal stage) the path the *running* map_server logged as loaded.
Any mismatch or missing evidence exits non-zero so no goal is sent.
Every check writes one JSON record. No ROS imports; no node is started.
"""

import argparse
import datetime as dt
import glob
import hashlib
import json
import os
import re
import sys
from pathlib import Path

MAP_ROOT = Path("/ros2_ws/maps/field")
PINS = MAP_ROOT / "map_pins.json"
REGISTRY = MAP_ROOT / "waypoints.json"
RECORD_DIR = Path("/ros2_ws/logs/field_execution/map_guard")
KST = dt.timezone(dt.timedelta(hours=9))
LOAD_RE = re.compile(r"Loading yaml file: (\S+)")
READ_RE = re.compile(r"Read map (\S+?): ")


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def yaml_image(yaml_path):
    for line in Path(yaml_path).read_text(encoding="utf-8").splitlines():
        if line.strip().startswith("image:"):
            image = line.split(":", 1)[1].strip().strip("'\"")
            return str(Path(image) if os.path.isabs(image) else Path(yaml_path).parent / image)
    raise ValueError(f"image key missing in {yaml_path}")


def floor_of_waypoint(registry, name):
    data = json.loads(Path(registry).read_text(encoding="utf-8"))
    try:
        return data["waypoints"][name]["floor"].upper(), data["waypoints"][name]
    except KeyError:
        raise ValueError(f"unknown waypoint: {name}")


def live_map_servers(proc_root):
    pids = []
    for cmdline in glob.glob(os.path.join(proc_root, "[0-9]*", "cmdline")):
        try:
            argv = Path(cmdline).read_bytes().split(b"\0")
        except OSError:
            continue
        if argv and argv[0].decode(errors="replace").endswith("/nav2_map_server/map_server"):
            pids.append(int(Path(cmdline).parent.name))
    return sorted(pids)


def map_server_loaded(pid, log_dir):
    logs = sorted(glob.glob(os.path.join(log_dir, f"map_server_{pid}_*.log")), key=os.path.getmtime)
    if not logs:
        return None, None, None
    loaded = read = None
    for line in Path(logs[-1]).read_text(encoding="utf-8", errors="replace").splitlines():
        match = LOAD_RE.search(line)
        if match:
            loaded, read = match.group(1), None
        match = READ_RE.search(line)
        if match:
            read = match.group(1)
    return logs[-1], loaded, read


def default_log_dir():
    if os.environ.get("ROS_LOG_DIR"):
        return os.environ["ROS_LOG_DIR"]
    home = os.environ.get("ROS_HOME") or os.path.join(os.path.expanduser("~"), ".ros")
    return os.path.join(home, "log")


def run_check(args):
    now = dt.datetime.now(KST)
    record = {
        "checked_at_kst": now.isoformat(timespec="seconds"),
        "stage": args.stage,
        "checks": [],
        "result": "FAIL",
    }

    def check(name, ok, **detail):
        record["checks"].append(dict(name=name, ok=bool(ok), **detail))
        return ok

    try:
        waypoint = None
        floor = args.floor.upper() if args.floor else None
        if args.waypoint:
            wp_floor, waypoint = floor_of_waypoint(args.registry, args.waypoint)
            record["waypoint"] = {"name": args.waypoint, "floor": wp_floor, "x": waypoint["x"], "y": waypoint["y"],
                                  "yaw_rad": waypoint["yaw_rad"], "map_yaml_metadata": waypoint.get("map_yaml")}
            if floor and floor != wp_floor:
                check("waypoint_floor_matches_request", False, requested=floor, waypoint=wp_floor)
                raise RuntimeError("waypoint floor mismatch")
            floor = wp_floor
        if floor not in {"F1", "F2", "F3"}:
            raise ValueError("floor must be F1, F2, or F3 (or give --waypoint)")
        record["floor"] = floor

        pins = json.loads(Path(args.pins).read_text(encoding="utf-8"))
        pin = pins.get("floors", {}).get(floor)
        if not check("pin_present", pin is not None, pins=str(args.pins)):
            raise RuntimeError(f"no pinned map for {floor}; add it to {args.pins} after verifying hashes")
        record["pin"] = pin

        pointer_path = Path(args.map_root) / floor.lower() / "latest_map.txt"
        pointer = pointer_path.read_text(encoding="utf-8").strip()
        check("latest_map_pointer_matches_pin", pointer == pin["yaml"], pointer=pointer, pointer_file=str(pointer_path))

        if args.map_yaml:
            check("requested_map_matches_pin", args.map_yaml == pin["yaml"], requested=args.map_yaml)

        yaml_local = Path(args.map_root) / Path(pin["yaml"]).relative_to("/ros2_ws/maps/field")
        yaml_hash = sha256(yaml_local)
        check("yaml_sha256_matches_pin", yaml_hash == pin["yaml_sha256"], actual=yaml_hash)
        image = yaml_image(yaml_local)
        image_hash = sha256(image)
        pin_image_local = Path(args.map_root) / Path(pin["image"]).relative_to("/ros2_ws/maps/field")
        check("yaml_image_path_matches_pin", Path(image) == pin_image_local, image=str(image))
        check("pgm_sha256_matches_pin", image_hash == pin["image_sha256"], actual=image_hash)

        if waypoint is not None and waypoint.get("map_yaml") and waypoint["map_yaml"] != pin["yaml"]:
            record.setdefault("warnings", []).append(
                f"waypoint metadata map_yaml={waypoint['map_yaml']} differs from pinned map; coordinates must be valid in the pinned frame"
            )

        if args.stage == "goal":
            pids = live_map_servers(args.proc_root)
            if check("exactly_one_live_map_server", len(pids) == 1, pids=pids):
                log, loaded, read = map_server_loaded(pids[0], args.log_dir)
                check("map_server_log_found", log is not None, log=log)
                check("map_server_loaded_yaml_matches_pin", loaded == pin["yaml"], loaded=loaded)
                check("map_server_read_image_matches_pin", read == pin["image"], read=read)
    except (OSError, ValueError, RuntimeError, KeyError, json.JSONDecodeError) as exc:
        record["error"] = str(exc)

    ok = "error" not in record and record["checks"] and all(item["ok"] for item in record["checks"])
    record["result"] = "PASS" if ok else "FAIL"
    out_dir = Path(args.record_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{now.strftime('%Y%m%d_%H%M%S')}_KST_{record.get('floor', 'unknown')}_{args.stage}"
    out = out_dir / f"{stem}.json"
    suffix = 1
    while out.exists():
        out = out_dir / f"{stem}_{suffix}.json"
        suffix += 1
    out.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    for item in record["checks"]:
        detail = {k: v for k, v in item.items() if k not in ("name", "ok")}
        print(f"{'PASS' if item['ok'] else 'FAIL'} {item['name']} {json.dumps(detail, ensure_ascii=False)}")
    for warning in record.get("warnings", []):
        print(f"WARN {warning}")
    if "error" in record:
        print(f"error: {record['error']}", file=sys.stderr)
    print(f"MAP_GUARD={record['result']} record={out}")
    if not ok:
        print("map guard failed: do NOT send a goal on this floor", file=sys.stderr)
    return 0 if ok else 1


def build_parser():
    parser = argparse.ArgumentParser(description="fail-closed field map identity guard")
    sub = parser.add_subparsers(dest="command", required=True)
    check = sub.add_parser("check")
    target = check.add_mutually_exclusive_group(required=True)
    target.add_argument("--floor")
    target.add_argument("--waypoint")
    check.add_argument("--stage", choices=("pre-nav", "goal"), default="goal")
    check.add_argument("--map-yaml", help="explicit map passed to nav start")
    check.add_argument("--pins", default=str(PINS))
    check.add_argument("--registry", default=str(REGISTRY))
    check.add_argument("--map-root", default=str(MAP_ROOT))
    check.add_argument("--record-dir", default=str(RECORD_DIR))
    check.add_argument("--proc-root", default="/proc")
    check.add_argument("--log-dir", default=default_log_dir())
    check.set_defaults(handler=run_check)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    return args.handler(args)


if __name__ == "__main__":
    raise SystemExit(main())
