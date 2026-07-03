#!/usr/bin/env python3
"""L0 offline test: floor_maps.yaml parsing and robust map path resolution.

Run from anywhere:
  python3 test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py
"""
import tempfile
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]          # .../elevator_auto_map_switch
PROJECT_ROOT = ROOT.parents[1]                       # .../2026_graduation_project
sys.path.insert(0, str(ROOT / "src" / "auto_floor_orchestrator_pkg"))

from auto_floor_orchestrator_pkg.floor_map_registry import FloorMapRegistry


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def main():
    registry = FloorMapRegistry.from_file(ROOT / "config" / "floor_maps.yaml")

    # --- basic schema ---
    require(registry.frame_id == "map", "frame_id should default to map")
    require(registry.default_spawn_point_id == "elevator_inside",
            "default spawn point should be elevator_inside (robot is inside the elevator when the map switches)")
    require(registry.floor_ids() == ["F1", "F2", "F3"], f"unexpected floors: {registry.floor_ids()}")

    f2 = registry.get("F2")
    require(f2.floor == "F2", "floor id mismatch")
    require(f2.map_yaml.endswith("kku_f2.yaml"), "F2 map yaml mismatch")

    # case-insensitive floor lookup (elevator/state may carry lowercase)
    require(registry.get("f2").floor == "F2", "floor lookup should be case-insensitive")

    # --- unknown floor must fail loudly ---
    try:
        registry.get("B1")
    except KeyError as exc:
        require("B1" in str(exc), "missing floor error should include floor id")
    else:
        raise AssertionError("unknown floor should raise KeyError")

    # --- pose points for /initialpose ---
    inside = registry.get_point("F2", "elevator_inside")
    require(inside.x == 0.0 and inside.y == 0.0, "elevator_inside seed pose should be (0, 0)")
    exit_pt = registry.get_point("F2", "elevator_exit")
    require(exit_pt.x == 1.6, "elevator_exit seed pose should match kku_nav_points.yaml")
    require(abs(inside.yaw_rad) < 1e-9, "yaw_rad conversion broken")

    try:
        registry.get_point("F2", "nope")
    except KeyError as exc:
        require("nope" in str(exc), "missing point error should include point id")
    else:
        raise AssertionError("unknown point should raise KeyError")

    # --- map path resolution: workspace-relative entry must resolve to a real file ---
    resolved = registry.resolve_map_yaml("F2")
    require(Path(resolved).is_absolute(), "resolved map path should be absolute")
    require(Path(resolved).exists(), f"resolved map path should exist: {resolved}")
    require(resolved == str(PROJECT_ROOT / "src/slam_pkg/maps/kku_virtual/f2/kku_f2.yaml"),
            f"workspace-relative path resolved wrong: {resolved}")

    # absolute path entries are used as-is
    abs_yaml = ROOT / "config" / "floor_maps.yaml"  # any existing file works for this check
    with tempfile.TemporaryDirectory() as tmp:
        cfg = Path(tmp) / "floor_maps.yaml"
        cfg.write_text(
            "schema_version: 1\n"
            "floors:\n"
            f"  F1: {{map_yaml: {abs_yaml}}}\n",
            encoding="utf-8",
        )
        tmp_registry = FloorMapRegistry.from_file(cfg, workspace_root=PROJECT_ROOT)
        require(tmp_registry.resolve_map_yaml("F1") == str(abs_yaml),
                "absolute existing path should be returned as-is")

        # missing map file must raise and list every candidate tried
        cfg.write_text(
            "schema_version: 1\n"
            "floors:\n"
            "  F9: {map_yaml: src/slam_pkg/maps/never/exists.yaml}\n",
            encoding="utf-8",
        )
        broken = FloorMapRegistry.from_file(cfg, workspace_root=PROJECT_ROOT)
        try:
            broken.resolve_map_yaml("F9")
        except FileNotFoundError as exc:
            require("exists.yaml" in str(exc), "error should mention the missing file")
            require("candidates" in str(exc), "error should list tried candidates")
        else:
            raise AssertionError("unresolvable map path should raise FileNotFoundError")

    print("PASS floor map registry")


if __name__ == "__main__":
    main()
