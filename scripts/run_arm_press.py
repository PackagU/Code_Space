#!/usr/bin/env python3
"""Explicit arm bench command with measured feedback; never auto-selects a USB port.

This script is not part of normal startup.  Real execution requires the exact arm
udev alias and an explicit ``--execute`` flag.  ``--dry-run`` only prints protocol
payloads and reports SIMULATED, never physical completion.
"""

import argparse
import importlib.util
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "robot_arm_pkg" / "robot_arm_pkg"
DEFAULT_PORT = "/dev/arm_servo"


def load_servo_protocol():
    spec = importlib.util.spec_from_file_location("servo_protocol", PKG / "servo_protocol.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stop_unverified(driver):
    try:
        driver.stop_all()
    except Exception as exc:  # noqa: BLE001
        print(f"정지 명령 실패: {type(exc).__name__}: {exc}", file=sys.stderr)
    print("요청 중단: 정지 명령은 전송했지만 기계적 정지는 별도 확인 필요", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="로봇팔 버튼 누르기 실물 벤치 명령")
    parser.add_argument("cycle", nargs="?", type=int, default=1)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--execute", action="store_true", help="현장 승인 후 실물 명령 허용")
    parser.add_argument("--dry-run", action="store_true", help="전송 없이 payload만 출력")
    parser.add_argument("--tolerance-pwm", type=int, default=30)
    parser.add_argument("--feedback-timeout", type=float, default=1.0)
    parser.add_argument("--countdown", type=int, default=3)
    args = parser.parse_args()
    if args.execute == args.dry_run:
        parser.error("--execute 또는 --dry-run 중 정확히 하나를 지정해야 합니다")
    if args.feedback_timeout <= 0:
        parser.error("--feedback-timeout은 양수여야 합니다")

    sp = load_servo_protocol()
    cycle = sp.get_cycle(args.cycle)
    poses = sp.get_poses(args.cycle)
    if args.dry_run:
        print(sp.homing_command())
        for pose_name, duration_ms, send in cycle:
            if send:
                print(sp.pose_command(pose_name, duration_ms, poses))
        print("SIMULATED: 명령 문자열만 생성함; 위치 도달·완료는 미확인")
        return

    port_path = Path(args.port)
    if args.port != DEFAULT_PORT:
        sys.exit("실물 실행은 고정 udev 별칭 /dev/arm_servo만 허용합니다")
    if not port_path.exists() or not port_path.is_char_device():
        sys.exit("/dev/arm_servo 실장치가 없습니다; ttyUSB 자동 대체는 안전상 금지합니다")

    if args.countdown > 0:
        print(f"현장 승인·팔 주변 공간·E-Stop 확인: {args.countdown}초 후 시작")
        for remaining in range(args.countdown, 0, -1):
            print(remaining)
            time.sleep(1)

    driver = sp.SerialPoseDriver(args.port, args.baud)

    def wait_and_measure(pose_name, duration_ms, pose_table):
        driver.send_pose(pose_name, duration_ms, pose_table)
        time.sleep(duration_ms / 1000.0)
        deadline = time.monotonic() + args.feedback_timeout
        last = None
        while time.monotonic() < deadline:
            last = driver.read_positions()
            if sp.positions_reached(last, pose_table[pose_name], args.tolerance_pwm):
                return last
        raise TimeoutError(f"{pose_name} feedback timeout; last={last}")

    try:
        measured = wait_and_measure(sp.HOME_POSE, sp.HOMING_DURATION_MS, {sp.HOME_POSE: sp.HOME})
        print(f"home measured: {measured}")
        for index, (pose_name, duration_ms, send) in enumerate(cycle, start=1):
            if send:
                measured = wait_and_measure(pose_name, duration_ms, poses)
            else:
                time.sleep(duration_ms / 1000.0)
                measured = driver.read_positions()
                if not sp.positions_reached(measured, poses[pose_name], args.tolerance_pwm):
                    raise TimeoutError(f"hold feedback mismatch: {measured}")
            print(f"step {index}/{len(cycle)} measured: {pose_name} {measured}")
        print("COMPLETED: 모든 단계와 최종 home을 위치 피드백으로 확인함")
    except KeyboardInterrupt:
        stop_unverified(driver)
        raise SystemExit(130)
    except Exception as exc:  # noqa: BLE001
        stop_unverified(driver)
        raise SystemExit(f"FAILED: {type(exc).__name__}: {exc}")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
