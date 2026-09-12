#!/usr/bin/env python3
"""Offline tests for saved occupancy-map validation."""

import importlib.util
from pathlib import Path
import tempfile

import yaml


ROOT = Path(__file__).resolve().parents[1]
MODULE = ROOT / "src/slam_pkg/slam_pkg/map_contract.py"


def load_module():
    spec = importlib.util.spec_from_file_location("map_contract", MODULE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_map(root, resolution=0.05, image="map.pgm"):
    (root / "map.pgm").write_bytes(b"P5\n# test\n2 3\n255\n" + bytes([0] * 6))
    content = {
        "image": image,
        "resolution": resolution,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    path = root / "map.yaml"
    path.write_text(yaml.safe_dump(content), encoding="utf-8")
    return path


def main():
    contract = load_module()
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        valid = write_map(root)
        result = contract.validate_map_yaml(valid)
        assert result["width"] == 2 and result["height"] == 3
        assert result["resolution"] == 0.05

        bad_resolution = write_map(root, resolution=0.0)
        try:
            contract.validate_map_yaml(bad_resolution)
        except contract.MapContractError as exc:
            assert "resolution" in str(exc)
        else:
            raise AssertionError("zero resolution accepted")

        missing = write_map(root, image="missing.pgm")
        try:
            contract.validate_map_yaml(missing)
        except contract.MapContractError as exc:
            assert "not found" in str(exc)
        else:
            raise AssertionError("missing image accepted")

    print("map contract tests passed")


if __name__ == "__main__":
    main()
