"""엘베 층 전환 완료 신호 트리거 순수 로직.

ROS 의존 없음 — host 오프라인 테스트 대상 (scripts/test_arm_sequence.py).
트리거 판정은 gazebo_world_swap_pkg.swap_trigger 와 동일 의미론:
phase=="ready" && pending==false && map_loaded==true, 층 변경 시 1회만 발동.
버튼 시퀀스 자체는 servo_protocol.PRESS_CYCLE (Kim 실기 프로토콜) 이 SSOT.
"""

from __future__ import annotations

import json
import re


REQUEST_ID_PATTERN = re.compile(r"^[A-Za-z0-9_.:-]{1,64}$")
BUTTON_PATTERN = re.compile(r"^[A-Za-z0-9_-]{1,16}$")


def normalize_floor(value):
    """"F1"/"f1"/"1" → "F1". 실패 시 ValueError."""
    text = str(value).strip().upper()
    if text.startswith("F"):
        text = text[1:]
    if not text.isdigit():
        raise ValueError(f"invalid floor: {value!r}")
    return f"F{int(text)}"


def parse_arm_command(command_json):
    """명시적인 mission 팔 명령을 검증해 정규화한다.

    ``press``는 호출 버튼과 목적층 버튼을 구분하고 버튼·사이클을 반드시 지정한다.
    ``home``은 원점 확인 절차를 시작하고, ``cancel``은 진행 중 요청을 중단한다.
    """
    try:
        command = json.loads(command_json)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError("arm command must be valid JSON") from exc
    if not isinstance(command, dict):
        raise ValueError("arm command must be a JSON object")
    request_id = str(command.get("request_id", "")).strip()
    if not REQUEST_ID_PATTERN.fullmatch(request_id):
        raise ValueError("request_id must be 1-64 safe characters")
    action = str(command.get("action", "")).strip().lower()
    if action not in ("press", "home", "cancel"):
        raise ValueError("action must be press, home, or cancel")
    if action == "home":
        return {"request_id": request_id, "action": action}
    if action == "cancel":
        target_request_id = str(command.get("target_request_id", "")).strip()
        if target_request_id and not REQUEST_ID_PATTERN.fullmatch(target_request_id):
            raise ValueError("target_request_id must be empty or 1-64 safe characters")
        return {
            "request_id": request_id,
            "action": action,
            "target_request_id": target_request_id,
        }
    target = str(command.get("target", "")).strip().lower()
    if target not in ("call", "destination"):
        raise ValueError("target must be call or destination")
    button = str(command.get("button", "")).strip().upper()
    if not BUTTON_PATTERN.fullmatch(button):
        raise ValueError("button must be 1-16 alphanumeric characters")
    try:
        cycle = int(command.get("press_cycle"))
    except (TypeError, ValueError) as exc:
        raise ValueError("press_cycle must be 1, 2, or 3") from exc
    if cycle not in (1, 2, 3):
        raise ValueError("press_cycle must be 1, 2, or 3")
    return {
        "request_id": request_id,
        "action": action,
        "target": target,
        "button": button,
        "press_cycle": cycle,
    }


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
