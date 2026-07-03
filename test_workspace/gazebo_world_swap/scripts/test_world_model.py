#!/usr/bin/env python3
"""Offline checks for extracting Gazebo building models from world files."""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path


def _add_source_path():
    repo_root = Path(__file__).resolve().parents[3]
    package_root = repo_root / "test_workspace" / "gazebo_world_swap" / "src" / "gazebo_world_swap_pkg"
    sys.path.insert(0, str(package_root))
    return repo_root


def main():
    repo_root = _add_source_path()

    from gazebo_world_swap_pkg.world_model import (
        building_model_name,
        extract_building_spawn_xml,
        extract_building_model_xml,
        world_path_for_floor,
    )

    world_dir = repo_root / "src" / "common_pkg" / "worlds"
    f2_world = world_path_for_floor("f2", world_dir)
    assert f2_world == world_dir / "kku_f2.world"

    model_name = building_model_name("F2")
    assert model_name == "kku_f2_building"

    xml_text = extract_building_model_xml(f2_world, model_name)
    model = ET.fromstring(xml_text)
    assert model.tag == "model"
    assert model.attrib["name"] == "kku_f2_building"
    assert model.findtext("static") == "true"
    assert model.find("world") is None
    assert model.find("link") is not None

    spawn_xml = extract_building_spawn_xml(f2_world, model_name)
    sdf = ET.fromstring(spawn_xml)
    assert sdf.tag == "sdf"
    assert sdf.attrib["version"] == "1.6"
    spawn_model = sdf.find("model")
    assert spawn_model is not None
    assert spawn_model.attrib["name"] == "kku_f2_building"

    try:
        building_model_name("B1")
    except ValueError as exc:
        assert "floor must be one of" in str(exc)
    else:
        raise AssertionError("unknown floor should fail")

    try:
        extract_building_model_xml(f2_world, "missing_model")
    except ValueError as exc:
        assert "model not found" in str(exc)
    else:
        raise AssertionError("unknown model should fail")

    print("PASS world model extraction")


if __name__ == "__main__":
    main()
