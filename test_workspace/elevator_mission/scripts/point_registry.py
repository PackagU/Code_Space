from dataclasses import dataclass
from pathlib import Path
import math
import yaml


@dataclass(frozen=True)
class NavPoint:
    floor: str
    point_id: str
    point_type: str
    x: float
    y: float
    yaw_deg: float

    @property
    def yaw_rad(self):
        return math.radians(self.yaw_deg)


class PointRegistry:
    def __init__(self, frame_id, floors):
        self.frame_id = frame_id
        self.floors = floors

    @classmethod
    def from_file(cls, path):
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        floors = {}
        for floor_id, floor_data in data["floors"].items():
            points = {}
            for point_id, point_data in floor_data["points"].items():
                points[str(point_id)] = NavPoint(
                    floor=floor_id,
                    point_id=str(point_id),
                    point_type=point_data["type"],
                    x=float(point_data["x"]),
                    y=float(point_data["y"]),
                    yaw_deg=float(point_data["yaw_deg"]),
                )
            floors[floor_id] = {
                "map_yaml": floor_data["map_yaml"],
                "initial_pose_id": floor_data["initial_pose_id"],
                "routing": floor_data.get("routing", {}),
                "points": points,
            }
        return cls(frame_id=data.get("frame_id", "map"), floors=floors)

    def get(self, floor, point_id):
        floor_key = str(floor).upper()
        point_key = str(point_id)
        if floor_key not in self.floors:
            raise KeyError(f"unknown floor: {floor_key}")
        points = self.floors[floor_key]["points"]
        if point_key not in points:
            raise KeyError(f"unknown point on {floor_key}: {point_key}")
        return points[point_key]

    def get_qualified(self, qualified_id):
        if "@" not in qualified_id:
            raise ValueError(f"expected point@floor, got: {qualified_id}")
        point_id, floor = qualified_id.split("@", 1)
        return self.get(floor, point_id)

    def initial_pose_id(self, floor):
        floor_key = str(floor).upper()
        if floor_key not in self.floors:
            raise KeyError(f"unknown floor: {floor_key}")
        return self.floors[floor_key]["initial_pose_id"]

    def routing(self, floor):
        floor_key = str(floor).upper()
        if floor_key not in self.floors:
            raise KeyError(f"unknown floor: {floor_key}")
        return self.floors[floor_key].get("routing", {})
