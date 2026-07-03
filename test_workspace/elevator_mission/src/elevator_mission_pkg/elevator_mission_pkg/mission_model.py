from dataclasses import dataclass
from pathlib import Path
import yaml


@dataclass(frozen=True)
class DeliveryMission:
    mission_id: str
    start_floor: str
    target_floor: str
    pickup_point: str
    elevator_entry_point: str
    elevator_inside_point: str
    elevator_exit_point: str
    destination_point: str
    mock_load_event: str
    mock_delivery_event: str


def load_mission(path, mission_id):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    mission_data = data["missions"][mission_id]
    return DeliveryMission(mission_id=mission_id, **mission_data)
