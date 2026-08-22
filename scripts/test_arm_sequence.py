#!/usr/bin/env python3
"""robot_arm_pkg 트리거 + Kim 서보 프로토콜 오프라인 테스트 (ROS 불필요, host 실행)."""
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "robot_arm_pkg" / "robot_arm_pkg"


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, PKG / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def ready_status(floor, pending=False, map_loaded=True, phase="ready"):
    return json.dumps(
        {"phase": phase, "pending": pending, "map_loaded": map_loaded, "current_floor": floor}
    )


def expect_raise(fn, *args):
    try:
        fn(*args)
    except ValueError:
        return
    raise AssertionError(f"{fn.__name__}{args} 이 ValueError 를 내지 않음")


def main():
    seq = load_module("arm_sequence")
    sp = load_module("servo_protocol")

    # 1) 트리거: ready+맵로드+층 변경에서만 1회 발동
    trigger = seq.FloorReadyTrigger("F1")
    assert trigger.observe("not json") is None
    assert trigger.observe(ready_status("F2", phase="moving")) is None
    assert trigger.observe(ready_status("F2", pending=True)) is None
    assert trigger.observe(ready_status("F2", map_loaded=False)) is None
    assert trigger.observe(ready_status("F1")) is None, "같은 층은 무시"
    assert trigger.observe(ready_status("F2")) == "F2"
    assert trigger.observe(ready_status("F2")) is None, "중복 신호 dedupe"
    assert trigger.observe(ready_status("f3")) == "F3", "소문자 층 표기 허용"

    # 2) 명령 문자열: Kim 벤치 스크립트 포맷과 정확히 일치해야 함
    assert (
        sp.pose_command("press_ready", 2000)
        == "{#000P1500T2000!#001P1500T2000!#002P1500T2000!#003P1500T2000!}"
    )
    assert (
        sp.pose_command("home", 3000)
        == "{#000P1500T3000!#001P1200T3000!#002P2000T3000!#003P1500T3000!}"
    )
    assert sp.stop_command("002") == "#002PDPT!"
    assert sp.read_position_command("001") == "#001PRAD!"

    # 3) 안전 가드: 잘못된 포즈/시간/PWM/ID 는 ValueError
    expect_raise(sp.pose_command, "no_such_pose", 1000)
    expect_raise(sp.pose_command, "home", 10000)
    expect_raise(sp.check_pwm, "000", 900)
    expect_raise(sp.check_pwm, "000", 2600)
    expect_raise(sp.check_pwm, "999", 1500)

    # 4) 위치 응답 파싱
    assert sp.parse_position("000", "#000P1500!") == 1500
    assert sp.parse_position("000", "garbage") is None
    assert sp.parse_position("000", None) is None

    # 5) 사이클: run_press_cycle 과 동일 구성 (총 9초, hold 는 재전송 없음)
    assert sp.cycle_duration_ms() == 9000
    assert [(p, s) for p, _, s in sp.PRESS_CYCLE] == [
        ("press_ready", True),
        ("pre_press", True),
        ("press", True),
        ("press", False),
        ("retreat", True),
        ("press_ready", True),
        ("home", True),
    ]

    # 6) PWM 보간: home 에서 시작해 home 으로 복귀, 50Hz 틱 간 점프가 완만
    assert sp.interpolate_pwm(0) == sp.POSES["home"]
    assert sp.interpolate_pwm(sp.cycle_duration_ms() + 1000) == sp.POSES["home"]
    tick_ms = 20.0
    prev = sp.interpolate_pwm(0)
    max_step = 0.0
    ticks = int(sp.cycle_duration_ms() / tick_ms) + 1
    for i in range(1, ticks):
        cur = sp.interpolate_pwm(i * tick_ms)
        max_step = max(max_step, max(abs(cur[s] - prev[s]) for s in sp.SERVO_IDS))
        for servo_id in sp.SERVO_IDS:
            sp.check_pwm(servo_id, cur[servo_id])  # 보간 전 구간이 안전 범위 안
        prev = cur
    assert max_step < 10.0, f"틱당 PWM 점프 {max_step:.2f} — 사이클이 불연속"

    print(f"PASS arm trigger + servo protocol ({ticks} ticks, max_step {max_step:.2f} PWM/tick)")


if __name__ == "__main__":
    main()
