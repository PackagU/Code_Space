#!/usr/bin/env python3
"""Offline contract for the pure parts of scripts/field_pose_capture.py (no ROS)."""

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import field_pose_capture as cap  # noqa: E402


def require(condition, message):
    if not condition:
        print(f"FAIL: {message}", file=sys.stderr)
        raise SystemExit(1)


def grid(width=100, height=100, res=0.05, origin=(-2.5, -2.5), fill=0):
    return {"frame": "map", "width": width, "height": height, "resolution": res, "origin_x": origin[0],
            "origin_y": origin[1], "origin_yaw": 0.0, "data": [fill] * (width * height)}


def put(g, x, y, value):
    col = int(math.floor((x - g["origin_x"]) / g["resolution"]))
    row = int(math.floor((y - g["origin_y"]) / g["resolution"]))
    g["data"][row * g["width"] + col] = value


def main():
    wrapped = cap.summarize([(1.0, 2.0, math.pi - 0.01), (1.0, 2.0, -math.pi + 0.01)])
    require(abs(abs(wrapped["yaw"]) - math.pi) < 1e-6 and wrapped["std_yaw_deg"] < 1.0, f"yaw mean must wrap at ±pi: {wrapped}")

    free = grid()
    cost = cap.footprint_cost(free, 0.0, 0.0, 0.0)
    require(60 <= cost["cells"] <= 100 and cost["lethal_cells"] == 0 and cost["max_cost"] == 0, f"free footprint: {cost}")

    behind = grid()
    put(behind, -0.30, 0.0, 254)  # inside footprint: base_footprint origin is near the front edge
    require(cap.footprint_cost(behind, 0.0, 0.0, 0.0)["lethal_cells"] == 1, "lethal cell behind origin is inside footprint")
    require(cap.footprint_cost(behind, 0.0, 0.0, math.pi)["lethal_cells"] == 0, "rotating 180 deg moves footprint off that cell")

    front = grid()
    put(front, 0.10, 0.0, 254)
    require(cap.footprint_cost(front, 0.0, 0.0, 0.0)["lethal_cells"] == 0, "cell 0.10 m ahead is outside footprint (front edge 0.033)")

    unknown = grid()
    put(unknown, -0.1, 0.1, -1)
    require(cap.footprint_cost(unknown, 0.0, 0.0, 0.0)["unknown_cells"] == 1, "unknown (-1) inside footprint is counted")
    require(cap.footprint_cost(grid(), 2.49, 0.0, 0.0)["outside_grid_cells"] > 0, "footprint leaving the grid is flagged")

    stable = [(1.0 + 0.001 * i, 2.0, 0.5) for i in range(10)]
    s = cap.summarize(stable)
    cov = [0.0] * 36
    cov[0] = cov[7] = 0.01
    cov[35] = 0.001
    ok_costs = {"global_costmap": cap.footprint_cost(free, 1.0, 2.0, 0.5), "local_costmap": cap.footprint_cost(free, 0.0, 0.0, 0.0)}
    checks = cap.evaluate(s, s, cov, ok_costs)
    require(all(c["ok"] for c in checks), f"stable, confident, free capture passes: {checks}")

    noisy = cap.summarize([(1.0 + 0.1 * (i % 2), 2.0, 0.5) for i in range(10)])
    require(not all(c["ok"] for c in cap.evaluate(noisy, noisy, cov, ok_costs)), "0.05 m std must fail")

    wide = list(cov)
    wide[0] = 0.25
    require(not all(c["ok"] for c in cap.evaluate(s, s, wide, ok_costs)), "operator-seed covariance 0.25 must fail")

    lethal_costs = dict(ok_costs, global_costmap=cap.footprint_cost(behind, 0.0, 0.0, 0.0))
    require(not all(c["ok"] for c in cap.evaluate(s, s, cov, lethal_costs)), "lethal footprint must fail")
    require(not all(c["ok"] for c in cap.evaluate(s, s, cov, {"global_costmap": ok_costs["global_costmap"]})), "missing local costmap must fail")
    shifted = cap.summarize([(1.2, 2.0, 0.5)] * 10)
    require(not all(c["ok"] for c in cap.evaluate(s, shifted, cov, ok_costs)), "AMCL/TF disagreement must fail")
    print("PASS field_pose_capture pure contract")


if __name__ == "__main__":
    main()
