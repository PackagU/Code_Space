#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from orthogonal_router import OrthogonalRouter
from point_registry import PointRegistry


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def coords(route):
    return [(round(p.x, 2), round(p.y, 2), round(p.yaw_deg, 1)) for p in route]


def ids(route):
    return [p.point_id for p in route]


def main():
    registry = PointRegistry.from_file(ROOT / "config" / "kku_nav_points.yaml")
    router = OrthogonalRouter(registry)

    room_route = router.route("F2", "elevator_exit", "208")
    require(
        ids(room_route) == ["route_1", "route_2", "208"],
        f"F2 route ids mismatch: {ids(room_route)}",
    )
    require(
        coords(room_route) == [(2.0, 0.0, 90.0), (2.0, 12.0, 0.0), (2.35, 12.0, 0.0)],
        f"F2 route coords mismatch: {coords(room_route)}",
    )

    parcel_route = router.route("F1", "elevator_exit", "parcel_pickup")
    require(
        ids(parcel_route) == ["route_1", "parcel_pickup"],
        f"F1 route ids mismatch: {ids(parcel_route)}",
    )
    require(
        coords(parcel_route) == [(5.0, 0.0, -90.0), (5.0, -1.4, -90.0)],
        f"F1 route coords mismatch: {coords(parcel_route)}",
    )

    direct_route = router.route("F1", "elevator_entry", "elevator_inside")
    require(
        ids(direct_route) == ["elevator_inside"],
        f"direct route should only include final point: {ids(direct_route)}",
    )

    print("PASS orthogonal router")


if __name__ == "__main__":
    main()
