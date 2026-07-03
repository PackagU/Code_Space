"""Floor map registry: parses floor_maps.yaml and resolves map paths robustly.

Pure Python (yaml only, no ROS imports) so offline tests run on any host.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml


@dataclass(frozen=True)
class FloorPoint:
    point_id: str
    x: float
    y: float
    yaw_deg: float

    @property
    def yaw_rad(self) -> float:
        return math.radians(self.yaw_deg)


@dataclass(frozen=True)
class FloorMap:
    floor: str
    map_yaml: str
    points: dict


class FloorMapRegistry:
    def __init__(self, frame_id, default_spawn_point_id, floors, workspace_root):
        self.frame_id = frame_id
        self.default_spawn_point_id = default_spawn_point_id
        self._floors = floors
        self.workspace_root = Path(workspace_root)

    @classmethod
    def from_file(cls, path, workspace_root=None):
        path = Path(path).resolve()
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        if workspace_root is None:
            # <project>/test_workspace/elevator_auto_map_switch/config/floor_maps.yaml
            # parents: [0]=config [1]=elevator_auto_map_switch [2]=test_workspace [3]=project root
            workspace_root = path.parents[3]
        floors = {}
        for floor_id, floor_data in (data.get("floors") or {}).items():
            floor_key = str(floor_id).upper()
            points = {}
            for point_id, p in (floor_data.get("points") or {}).items():
                points[str(point_id)] = FloorPoint(
                    point_id=str(point_id),
                    x=float(p["x"]),
                    y=float(p["y"]),
                    yaw_deg=float(p.get("yaw_deg", 0.0)),
                )
            floors[floor_key] = FloorMap(
                floor=floor_key,
                map_yaml=str(floor_data["map_yaml"]),
                points=points,
            )
        return cls(
            frame_id=str(data.get("frame_id", "map")),
            default_spawn_point_id=str(data.get("default_spawn_point_id", "elevator_inside")),
            floors=floors,
            workspace_root=workspace_root,
        )

    def floor_ids(self):
        return sorted(self._floors)

    def get(self, floor) -> FloorMap:
        floor_key = str(floor).upper()
        if floor_key not in self._floors:
            raise KeyError(f"unknown floor: {floor_key} (known: {self.floor_ids()})")
        return self._floors[floor_key]

    def get_point(self, floor, point_id) -> FloorPoint:
        floor_map = self.get(floor)
        if point_id not in floor_map.points:
            raise KeyError(
                f"unknown point: {point_id} on {floor_map.floor} "
                f"(known: {sorted(floor_map.points)})"
            )
        return floor_map.points[point_id]

    def resolve_map_yaml(self, floor) -> str:
        """Return the first existing absolute path for the floor's map yaml.

        Candidates, in order:
        1. the entry itself when absolute
        2. workspace_root / entry          (source tree, host or /ros2_ws mount)
        3. ament share dir of the package in 'src/<pkg>/rest' shaped entries
           (covers /ros2_ws/install/<pkg>/share/<pkg>/rest after colcon build)
        """
        entry = Path(self.get(floor).map_yaml)
        candidates = []
        if entry.is_absolute():
            candidates.append(entry)
        else:
            candidates.append(self.workspace_root / entry)
            parts = entry.parts
            if len(parts) >= 3 and parts[0] == "src":
                share = _package_share_dir(parts[1])
                if share is not None:
                    candidates.append(Path(share).joinpath(*parts[2:]))
        for candidate in candidates:
            if candidate.exists():
                return str(candidate)
        raise FileNotFoundError(
            f"map yaml for {str(floor).upper()} not found: {entry} "
            f"(candidates tried: {[str(c) for c in candidates]})"
        )


def _package_share_dir(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package_name)
    except Exception:
        return None
