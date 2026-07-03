"""Helpers for locating KKU world files and extracting building model SDF."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


KNOWN_FLOORS = ("F1", "F2", "F3")


def normalize_floor(floor) -> str:
    floor_id = str(floor).upper()
    if floor_id not in KNOWN_FLOORS:
        raise ValueError(f"floor must be one of {list(KNOWN_FLOORS)}, got '{floor_id}'")
    return floor_id


def building_model_name(floor) -> str:
    floor_id = normalize_floor(floor)
    return f"kku_{floor_id.lower()}_building"


def world_path_for_floor(floor, world_dir) -> Path:
    floor_id = normalize_floor(floor)
    return Path(world_dir) / f"kku_{floor_id.lower()}.world"


def extract_building_model_xml(world_path, model_name) -> str:
    world_path = Path(world_path)
    tree = ET.parse(world_path)
    root = tree.getroot()
    for model in root.findall(".//model"):
        if model.attrib.get("name") == model_name:
            return ET.tostring(model, encoding="unicode")
    raise ValueError(f"model not found in {world_path}: {model_name}")


def extract_building_spawn_xml(world_path, model_name) -> str:
    model_xml = extract_building_model_xml(world_path, model_name)
    return f'<sdf version="1.6">{model_xml}</sdf>'
