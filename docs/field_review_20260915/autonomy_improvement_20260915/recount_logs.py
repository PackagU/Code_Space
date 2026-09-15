"""Recount 2026-09-14~15 field session metrics from raw ROS 2 logs (read-only)."""
import datetime as dt
import pathlib
import re
import sys

LOG = pathlib.Path(sys.argv[1])
KST = dt.timezone(dt.timedelta(hours=9))
LINE = re.compile(r"^\[(\w+)\] \[(\d+\.\d+)\] \[([^\]]+)\]: (.*)$")
START = dt.datetime(2026, 9, 14, 22, 30, tzinfo=KST).timestamp()


def kst(t):
    return dt.datetime.fromtimestamp(t, KST).strftime("%m-%d %H:%M:%S.%f")[:-3]


events = []
for path in sorted(LOG.glob("*.log")):
    if not re.match(r"(bt_navigator|controller_server|planner_server|map_server|amcl)_", path.name):
        continue
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        m = LINE.match(raw)
        if not m or float(m.group(2)) < START:
            continue
        events.append((float(m.group(2)), path.name.split("_")[0] if not path.name.startswith(("bt_", "map_", "controller_", "planner_")) else "_".join(path.name.split("_")[:2]), m.group(1), m.group(4)))
events.sort()

keys = {
    "map_load": re.compile(r"Loading yaml file|yaml_filename|Read map"),
    "goal_begin": re.compile(r"Begin navigating"),
    "succeeded": re.compile(r"Goal succeeded"),
    "canceled": re.compile(r"Goal canceled|cancel"),
    "aborted": re.compile(r"Goal failed|aborted", re.I),
    "collision": re.compile(r"collision ahead", re.I),
    "lethal": re.compile(r"lethal space", re.I),
    "no_valid_path": re.compile(r"no valid path", re.I),
}
for t, node, level, msg in events:
    for key, rx in keys.items():
        if rx.search(msg):
            if key in ("collision",):
                continue
            print(f"{kst(t)} {node:18s} {key:14s} {msg[:150]}")

# collision bursts grouped by goal windows
begins = [t for t, n, l, m in events if keys["goal_begin"].search(m)]
ends = [(t, m) for t, n, l, m in events if n.startswith("bt_navigator") and re.search(r"Goal (succeeded|canceled|failed)", m)]
print("\n== per goal")
for b in begins:
    end = next(((t, m) for t, m in ends if t > b), (None, "no end"))
    e = end[0] if end[0] else b + 3600
    col = [t for t, n, l, m in events if b <= t <= e and keys["collision"].search(m)]
    leth = [t for t, n, l, m in events if b <= t <= e + 10 and keys["lethal"].search(m)]
    nvp = [t for t, n, l, m in events if b <= t <= e and keys["no_valid_path"].search(m)]
    dur = f"{e - b:.3f}" if end[0] else "?"
    span = f"{col[0] and kst(col[0])}~{kst(col[-1])} ({col[-1]-col[0]:.3f}s)" if col else "-"
    print(f"begin {kst(b)} end {end[1][:40]!r} dur={dur}s collision={len(col)} {span} lethal={len(leth)} no_valid_path={len(nvp)}")
