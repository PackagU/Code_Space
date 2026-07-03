"""Select exactly one Gazebo swap for each completed floor switch status."""

from __future__ import annotations

from dataclasses import dataclass
import json

from gazebo_world_swap_pkg.world_model import building_model_name, normalize_floor


@dataclass(frozen=True)
class SwapRequest:
    source_floor: str
    target_floor: str
    target_model: str
    spawn_point_id: str
    status: dict


class WorldSwapTrigger:
    """Deduplicates `/floor_orchestrator/status` JSON messages."""

    def __init__(self, initial_floor="F1"):
        self._last_floor = normalize_floor(initial_floor)

    @property
    def last_floor(self) -> str:
        return self._last_floor

    def observe(self, status_json):
        try:
            status = json.loads(status_json)
        except (TypeError, json.JSONDecodeError):
            return None
        if not isinstance(status, dict):
            return None
        if status.get("phase") != "ready":
            return None
        if bool(status.get("pending", True)):
            return None
        if not bool(status.get("map_loaded", False)):
            return None

        try:
            target_floor = normalize_floor(status.get("current_floor", ""))
            target_model = building_model_name(target_floor)
        except ValueError:
            return None

        if target_floor == self._last_floor:
            return None

        request = SwapRequest(
            source_floor=self._last_floor,
            target_floor=target_floor,
            target_model=target_model,
            spawn_point_id=str(status.get("spawn_point_id", "elevator_inside")),
            status=status,
        )
        self._last_floor = target_floor
        return request
