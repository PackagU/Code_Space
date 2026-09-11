#!/usr/bin/env python3
"""로봇팔 버튼 누르기 원커맨드 벤치 실행 (ROS/Docker 불필요, host 직결 시리얼).

사용법 (VSCode 터미널 한 줄):
    python3 scripts/run_arm_press.py 2                    # cycle 2, 포트 /dev/arm_servo
    python3 scripts/run_arm_press.py 3 --port /dev/ttyUSB0
    python3 scripts/run_arm_press.py 1 --dry-run          # 하드웨어 없이 명령 문자열만 출력

포즈/사이클 값은 src/robot_arm_pkg/robot_arm_pkg/servo_protocol.py 가 SSOT —
여기서는 그대로 읽어 쓰기만 한다(튜닝은 그 파일에서).
⚠ 실행 전 팔 주변 공간 확보. Ctrl+C 시 전 모터 정지 명령 전송 후 종료.
"""
import argparse
import glob
import importlib.util
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "src" / "robot_arm_pkg" / "robot_arm_pkg"


def load_servo_protocol():
    spec = importlib.util.spec_from_file_location("servo_protocol", PKG / "servo_protocol.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class DryRunConn:
    """--dry-run 백엔드 — 전송 대신 명령 문자열을 출력한다."""

    def write(self, data):
        print(f"  [dry-run] {data.decode('ascii')}")

    def close(self):
        pass


DEFAULT_PORT = "/dev/arm_servo"


def resolve_port(port):
    """포트 자동 탐지 — 기본 포트(/dev/arm_servo)가 없으면 연결된 USB-시리얼을 찾는다."""
    if Path(port).exists():
        return port
    candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if port != DEFAULT_PORT:  # 사용자가 직접 지정한 포트는 대체하지 않는다
        sys.exit(f"지정한 포트 {port} 없음 — 연결된 시리얼: {candidates or '없음'}")
    if len(candidates) == 1:
        print(f"{DEFAULT_PORT} 없음 → 자동 탐지: {candidates[0]}")
        return candidates[0]
    if not candidates:
        sys.exit(
            "서보 컨트롤러가 이 PC에 연결돼 있지 않습니다 (USB-시리얼 장치 없음).\n"
            "  1) 컨트롤러 USB 케이블을 이 PC에 연결\n"
            "  2) ls /dev/ttyUSB* /dev/ttyACM* 로 포트 생성 확인\n"
            "  3) 다시 실행 (자동 탐지됨). 하드웨어 없이 확인만 하려면 --dry-run"
        )
    sys.exit(f"USB-시리얼이 여러 개 감지됨: {candidates} — --port 로 하나를 지정하세요")


def open_serial(port, baud):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial 미설치 — pip install pyserial 후 재실행 (또는 --dry-run)")
    try:
        return serial.Serial(port, baud, timeout=0.1)
    except Exception as exc:  # noqa: BLE001
        if "Permission denied" in str(exc) or "Errno 13" in str(exc):
            sys.exit(f"권한 없음: {port} — sudo usermod -aG dialout $USER 후 재로그인, 또는 sudo chmod 666 {port}")
        sys.exit(f"시리얼 열기 실패: {port} ({exc})")


def main():
    parser = argparse.ArgumentParser(description="로봇팔 버튼 누르기 사이클 1회 실행")
    parser.add_argument("cycle", nargs="?", type=int, default=1, help="press cycle 번호 1~3 (기본 1)")
    parser.add_argument("--port", default="/dev/arm_servo", help="서보 컨트롤러 시리얼 포트")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--skip-home", action="store_true", help="기동 homing 생략 (이미 home 자세일 때)")
    parser.add_argument("--countdown", type=int, default=3, help="시작 전 카운트다운 초 (기본 3, 0=즉시)")
    parser.add_argument("--dry-run", action="store_true", help="전송 없이 명령 문자열만 출력")
    args = parser.parse_args()

    sp = load_servo_protocol()
    try:
        cycle = sp.get_cycle(args.cycle)
        poses = sp.get_poses(args.cycle)
    except ValueError as exc:
        sys.exit(str(exc))

    # 포트 확인·오픈을 카운트다운보다 먼저 — 3초 기다린 뒤 실패하는 일이 없게
    if args.dry_run:
        conn, port = DryRunConn(), args.port
    else:
        port = resolve_port(args.port)
        conn = open_serial(port, args.baud)

    total_s = sp.cycle_duration_ms(cycle) / 1000.0
    print(f"press cycle #{args.cycle} — {len(cycle)} steps, {total_s:.1f}s, port={port}"
          f"{' (dry-run)' if args.dry_run else ''}")

    if not args.dry_run and args.countdown > 0:
        print(f"⚠ 팔 주변 공간 확보! {args.countdown}초 후 시작 (중단: Ctrl+C)")
        try:
            for remaining in range(args.countdown, 0, -1):
                print(f"  {remaining}...")
                time.sleep(1)
        except KeyboardInterrupt:
            conn.close()
            sys.exit("\n시작 전 중단 — 명령 미전송")

    def sleep_ms(duration_ms):
        if not args.dry_run:
            time.sleep(duration_ms / 1000.0)

    try:
        if not args.skip_home:
            print(f"homing: 중립(1500) → home {sp.HOMING_DURATION_MS / 1000.0:.1f}s")
            conn.write(sp.homing_command().encode("ascii"))
            sleep_ms(sp.HOMING_DURATION_MS)

        for index, (pose_name, duration_ms, send) in enumerate(cycle, start=1):
            action = pose_name if send else f"{pose_name} (유지)"
            print(f"step {index}/{len(cycle)}: {action} {duration_ms / 1000.0:.1f}s")
            if send:
                conn.write(sp.pose_command(pose_name, duration_ms, poses).encode("ascii"))
            sleep_ms(duration_ms)
        print("사이클 완료 — home 대기 자세")
    except KeyboardInterrupt:
        print("\n중단 — 전 모터 정지 명령 전송")
        for servo_id in sp.SERVO_IDS:
            conn.write(sp.stop_command(servo_id).encode("ascii"))
    finally:
        conn.close()


if __name__ == "__main__":
    main()
