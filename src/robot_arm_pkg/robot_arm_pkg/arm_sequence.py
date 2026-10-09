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


def validate_command_pose(pose):
    from . import servo_protocol as sp
    if not isinstance(pose, dict) or set(pose) != set(sp.SERVO_IDS):
        raise ValueError("pose requires all four servo IDs")
    for joint, value in pose.items():
        if type(value) is not int:
            raise ValueError("PWM must be an integer")
        sp.check_pwm(joint, value)
    return dict(pose)


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
    if action not in ("press", "home", "stow", "cancel", "view", "move", "read"):
        raise ValueError("unknown arm action")
    if action == "read":
        return {"request_id": request_id, "action": action}
    if action == "move":
        pose = validate_command_pose(command.get("pose"))
        duration = command.get("duration_ms", 2000)
        if type(duration) is not int or not 1 <= duration <= 5000:
            raise ValueError("duration_ms must be 1-5000")
        return dict(request_id=request_id, action=action, pose=pose, duration_ms=duration)
    if action == "view":
        view = command.get("view")
        if view not in ("front_view", "floor_view"):
            raise ValueError("view must be front_view or floor_view")
        return {"request_id": request_id, "action": action, "view": view}
    if action in ("home", "stow"):
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
        raise ValueError("press_cycle must be 1 (menu 6) or 2 (menu 7)") from exc
    if cycle not in (1, 2):
        raise ValueError("press_cycle must be 1 (menu 6) or 2 (menu 7)")
    result = {
        "request_id": request_id,
        "action": action,
        "target": target,
        "button": button,
        "press_cycle": cycle,
    }
    if "press_pose" in command:
        from . import servo_protocol as sp
        result["press_pose"] = validate_command_pose(command["press_pose"])
        sp.calibrated_press_poses(result["press_pose"])
    return result


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
