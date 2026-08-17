"""엘베 층 전환 완료 신호 → 하드코딩 버튼-누르기 시퀀스 순수 로직.

ROS 의존 없음 — host 오프라인 테스트 대상 (scripts/test_arm_sequence.py).
트리거 판정은 gazebo_world_swap_pkg.swap_trigger 와 동일 의미론:
phase=="ready" && pending==false && map_loaded==true, 층 변경 시 1회만 발동.
"""

from __future__ import annotations

import json

# 4 DOF 하드코딩 포즈: (구간 시간 s, [base, shoulder, elbow, wrist] rad)
# TODO(Kim): 실측 버튼 위치 기준 각도로 교체 — 시퀀스 형태(홈→접근→누름→유지→복귀→홈)는 유지
BUTTON_PRESS_SEQUENCE = [
    (2.0, [0.00, 0.00, 0.00, 0.00]),  # home
    (3.0, [0.60, 0.50, -0.40, 0.20]),  # reach
    (1.5, [0.60, 0.70, -0.55, 0.35]),  # press
    (1.0, [0.60, 0.70, -0.55, 0.35]),  # hold
    (3.0, [0.60, 0.50, -0.40, 0.20]),  # retract
    (2.5, [0.00, 0.00, 0.00, 0.00]),  # home
]

JOINT_NAMES = ["arm_base_joint", "arm_shoulder_joint", "arm_elbow_joint", "arm_wrist_joint"]


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


def sequence_duration(sequence=BUTTON_PRESS_SEQUENCE):
    return sum(step for step, _ in sequence)


def interpolate(elapsed, sequence=BUTTON_PRESS_SEQUENCE):
    """시퀀스 시작 후 elapsed(s) 시점의 관절 각도(선형 보간). 범위 밖은 양끝 포즈로 clamp."""
    if elapsed <= 0.0:
        return list(sequence[0][1])
    t = 0.0
    prev_pose = sequence[0][1]
    for step, pose in sequence:
        if elapsed < t + step:
            ratio = (elapsed - t) / step
            return [a + (b - a) * ratio for a, b in zip(prev_pose, pose)]
        t += step
        prev_pose = pose
    return list(sequence[-1][1])
