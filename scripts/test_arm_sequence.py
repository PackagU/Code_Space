#!/usr/bin/env python3
"""robot_arm_pkg.arm_sequence 오프라인 테스트 (ROS 불필요, host 실행)."""
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "src" / "robot_arm_pkg" / "robot_arm_pkg" / "arm_sequence.py"


def load_module():
    spec = importlib.util.spec_from_file_location("arm_sequence", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ready_status(floor, pending=False, map_loaded=True, phase="ready"):
    return json.dumps(
        {"phase": phase, "pending": pending, "map_loaded": map_loaded, "current_floor": floor}
    )


def main():
    mod = load_module()

    # 1) 트리거: ready+맵로드+층 변경에서만 1회 발동
    trigger = mod.FloorReadyTrigger("F1")
    assert trigger.observe("not json") is None
    assert trigger.observe(ready_status("F2", phase="moving")) is None
    assert trigger.observe(ready_status("F2", pending=True)) is None
    assert trigger.observe(ready_status("F2", map_loaded=False)) is None
    assert trigger.observe(ready_status("F1")) is None, "같은 층은 무시"
    assert trigger.observe(ready_status("F2")) == "F2"
    assert trigger.observe(ready_status("F2")) is None, "중복 신호 dedupe"
    assert trigger.observe(ready_status("f3")) == "F3", "소문자 층 표기 허용"

    # 2) 시퀀스: 홈에서 시작해 홈으로 복귀, 총 길이 일치
    seq = mod.BUTTON_PRESS_SEQUENCE
    total = mod.sequence_duration(seq)
    assert abs(total - sum(s for s, _ in seq)) < 1e-9
    assert mod.interpolate(0.0, seq) == seq[0][1]
    assert mod.interpolate(total + 1.0, seq) == seq[-1][1]
    assert mod.interpolate(-1.0, seq) == seq[0][1]

    # 3) 보간 연속성: 50Hz 틱 간 관절 변화가 급격하지 않음 (하드코딩 시퀀스 검증)
    rate = 50.0
    prev = mod.interpolate(0.0, seq)
    max_step = 0.0
    ticks = int(total * rate) + 1
    for i in range(1, ticks):
        cur = mod.interpolate(i / rate, seq)
        max_step = max(max_step, max(abs(a - b) for a, b in zip(cur, prev)))
        prev = cur
    assert max_step < 0.05, f"틱당 관절 점프 {max_step:.4f} rad — 시퀀스가 불연속"

    # 4) 포즈 4 DOF 고정
    assert all(len(pose) == 4 for _, pose in seq)
    assert len(mod.JOINT_NAMES) == 4

    print(f"PASS arm_sequence: trigger dedupe + interpolation ({ticks} ticks, max_step {max_step:.4f} rad)")


if __name__ == "__main__":
    main()
