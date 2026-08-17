"""엘베 층 전환 완료 신호 트리거 순수 로직.

ROS 의존 없음 — host 오프라인 테스트 대상 (scripts/test_arm_sequence.py).
트리거 판정은 gazebo_world_swap_pkg.swap_trigger 와 동일 의미론:
phase=="ready" && pending==false && map_loaded==true, 층 변경 시 1회만 발동.
버튼 시퀀스 자체는 servo_protocol.PRESS_CYCLE (Kim 실기 프로토콜) 이 SSOT.
"""

from __future__ import annotations

import json


def normalize_floor(value):
    """"F1"/"f1"/"1" → "F1". 실패 시 ValueError."""
    text = str(value).strip().upper()
    if text.startswith("F"):
        text = text[1:]
    if not text.isdigit():
        raise ValueError(f"invalid floor: {value!r}")
    return f"F{int(text)}"


class FloorReadyTrigger:
    """`/floor_orchestrator/status` JSON에서 '층 전환 완료' 1회 이벤트를 뽑아낸다."""

    def __init__(self, initial_floor="F1"):
        self._last_floor = normalize_floor(initial_floor)

    @property
    def last_floor(self):
        return self._last_floor

    def observe(self, status_json):
        """전환 완료면 target_floor(str) 반환, 아니면 None. 같은 층 중복 신호는 무시."""
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
        except ValueError:
            return None
        if target_floor == self._last_floor:
            return None
        self._last_floor = target_floor
        return target_floor
