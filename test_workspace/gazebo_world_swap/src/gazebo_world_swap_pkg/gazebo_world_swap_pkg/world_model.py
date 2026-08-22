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


def extract_world_model_names(world_path, exclude=("ground_plane",)) -> list[str]:
    """world 파일의 최상위 model 이름 전부 (건물 + 층 전용 소품).

    층 전환 시 건물만 지우면 F1 전용 소품(parcel_box, obstacle_*)이 다른 층에
    잔류한다(2026-07-03 재현). swap 은 이 목록 전체를 대상으로 해야 한다.
    """
    world_path = Path(world_path)
    root = ET.parse(world_path).getroot()
    names = []
    # 최상위(world 직속)만 — 중첩 model 은 부모 spawn 에 포함된다.
    for model in root.findall("./world/model"):
        name = model.attrib.get("name", "")
        if name and name not in exclude and name not in names:
            names.append(name)
    return names


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
