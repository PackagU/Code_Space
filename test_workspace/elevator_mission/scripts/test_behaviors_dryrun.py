#!/usr/bin/env python3
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "elevator_mission_pkg"
sys.path.insert(0, str(PKG))
sys.path.insert(0, str(ROOT / "scripts"))

from elevator_mission_pkg.mission_model import load_mission
from elevator_mission_pkg.mission_tree import build_dryrun_steps


def main():
    mission = load_mission(ROOT / "config" / "delivery_missions.yaml", "parcel_to_208")
    steps = build_dryrun_steps(mission)
    expected = [
        "NavigateRoute:F1:elevator_exit->parcel_pickup",
        "WaitForAck:parcel_loaded",
        "NavigateRoute:F1:parcel_pickup->elevator_entry",
        "CallElevator:F1",
        "WaitElevatorArrived:F1:open",
        "NavigateRoute:F1:elevator_entry->elevator_inside",
        "CallElevator:F2",
        "SwitchFloor:F2:elevator_inside",
        "WaitElevatorArrived:F2:open",
        "Relocalize:F2:elevator_exit",
        "NavigateRoute:F2:elevator_exit",
        "NavigateRoute:F2:elevator_exit->208",
        "WaitForAck:delivered",
    ]
    if steps != expected:
        raise AssertionError(f"unexpected steps: {steps}")
    print("PASS behavior dry-run")


if __name__ == "__main__":
    main()
