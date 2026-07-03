#!/usr/bin/env python3
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "elevator_mission_pkg"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(ROOT / "scripts"))

from elevator_mission_pkg.behaviors import NavigateRoute, RUNNING, SUCCESS
from orthogonal_router import OrthogonalRouter
from point_registry import PointRegistry


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def get_logger(self):
        return self.logger


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    registry = PointRegistry.from_file(ROOT / "config" / "kku_nav_points.yaml")
    router = OrthogonalRouter(registry)
    route = router.route("F2", "elevator_exit", "208")
    node = FakeNode()

    behavior = NavigateRoute(
        node,
        route,
        route_name="F2:elevator_exit->208",
        dry_run=True,
    )

    require(behavior.name == "NavigateRoute:F2:elevator_exit->208", "route name mismatch")
    require(behavior.tick() == RUNNING, "first dry-run tick should be running")
    behavior._dry_start = time.monotonic() - 1.0
    require(behavior.tick() == SUCCESS, "dry-run route should complete")
    require(any("route_1 -> route_2 -> 208" in m for m in node.logger.messages), "route log missing")

    print("PASS navigate route behavior")


if __name__ == "__main__":
    main()
