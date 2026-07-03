#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from point_registry import PointRegistry


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    registry = PointRegistry.from_file(ROOT / "config" / "kku_nav_points.yaml")

    p = registry.get("F2", "208")
    require(p.floor == "F2", "208 should be on F2")
    require(abs(p.x - 2.35) < 1e-6, "208 x mismatch")
    require(abs(p.y - 12.0) < 1e-6, "208 y mismatch")
    require(p.yaw_deg == 0.0, "208 yaw mismatch")

    inside = registry.get_qualified("elevator_inside@F1")
    require(inside.floor == "F1", "qualified floor mismatch")
    require(inside.point_id == "elevator_inside", "qualified point mismatch")

    try:
        registry.get("F2", "999")
    except KeyError as exc:
        require("999" in str(exc), "missing point error should include id")
    else:
        raise AssertionError("missing point should raise KeyError")

    print("PASS point registry")


if __name__ == "__main__":
    main()
