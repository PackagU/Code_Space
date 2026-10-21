#!/usr/bin/env python3
"""신공학관 E2E 촬영 리허설 — 한 터미널, 한 명령.

F1 출발 → 택배 싣기 → 엘리베이터(▲, 4) → 4층(F2 지도) 배달 → 엘리베이터(▼, 1) → 출발점 복귀.
로봇이 움직이는 단계는 운영자가 Enter 로 시작한다. 탑승·하차만 키보드 조종.

Jetson 호스트에서 저장소 루트 기준으로 실행한다(fieldctl 과 같은 위치):

    python3 scripts/field_video_rehearsal.py                 # 전체 리허설
    python3 scripts/field_video_rehearsal.py --list          # 단계 목록
    python3 scripts/field_video_rehearsal.py --only S07      # 한 단계만 (예: 호출 버튼)
    python3 scripts/field_video_rehearsal.py --from S13      # 중간부터
    python3 scripts/field_video_rehearsal.py --dry-run --yes # 장비 없이 흐름만 (가짜 장치)

미리 켜 둘 것: ROS 컨테이너, 버튼 웹사이트(8091, 팔을 실제로 움직이는 모드).
base·Nav2 가 꺼져 있으면 점검 단계에서 켤지 묻는다. 기록은 logs/field_video/<시각>_<label>/.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from field_video import backends, runner  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENARIO = Path(__file__).resolve().parent / "field_video" / "scenario.json"


def parse_failures(items):
    plan = {}
    for item in items or []:
        key, _, count = item.rpartition("=")
        if not key:
            raise SystemExit(f"--fail expects key=N, got {item}")
        plan[key] = int(count)
    return plan


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", default=str(DEFAULT_SCENARIO))
    parser.add_argument("--list", action="store_true", help="단계 목록만 출력")
    parser.add_argument("--from", dest="start", help="이 단계부터")
    parser.add_argument("--to", dest="end", help="이 단계까지")
    parser.add_argument("--only", help="쉼표로 구분한 단계만 (예: S07,S10)")
    parser.add_argument("--label", default="rehearsal", help="기록 폴더 이름 끝에 붙는 말 (예: take1)")
    parser.add_argument("--nav-restart", action="store_true",
                        help="층 전환 때 지도 교체 대신 Nav2 재시작 (버튼 웹사이트가 --execute 모드일 때만 가능)")
    parser.add_argument("--no-record", action="store_true", help="rosbag 녹화를 하지 않음")
    parser.add_argument("--web-url", help="버튼 웹사이트 주소 (기본: 시나리오 settings.web_url)")
    parser.add_argument("--lift-port", help="지게팔 Uno 포트 (기본: 시나리오 settings.lift_port)")
    parser.add_argument("--ultrasonic-port", help="초음파 아두이노가 별도 보드일 때 포트 (기본: 지게팔 Uno 공유)")
    parser.add_argument("--container", help="ROS 컨테이너 이름 (기본: PACKAGU_CONTAINER_NAME 또는 ros2_humble)")
    parser.add_argument("--log-root", default=str(ROOT / "logs" / "field_video"))
    parser.add_argument("--dry-run", action="store_true", help="가짜 장치로 흐름만 확인 (로봇·팔·지게팔 안 움직임)")
    parser.add_argument("--yes", action="store_true", help="모든 질문에 Enter (--dry-run 전용)")
    parser.add_argument("--fail", action="append", help="(--dry-run) 실패 주입, 예: goal:f1_locker=1, press:4=1")
    parser.add_argument("--allow-preview-arm", action="store_true", help=argparse.SUPPRESS)
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    scenario = runner.load_scenario(args.scenario)
    steps = scenario["steps"]
    if args.list:
        for step in steps:
            flag = " (자동)" if step.get("auto") else ""
            print(f"{step['id']}  {step['type']:<8} {step['title']}{flag}")
        return 0
    if args.yes and not args.dry_run:
        raise SystemExit("--yes 는 --dry-run 에서만 쓸 수 있습니다 (실기는 사람이 시작 신호를 준다)")
    only = [s.strip() for s in args.only.split(",")] if args.only else None
    selected = runner.select_steps(steps, args.start, args.end, only)
    settings = scenario.get("settings", {})
    us_settings = settings.get("ultrasonic", {})
    us_port = args.ultrasonic_port if args.ultrasonic_port is not None else us_settings.get("port", "")
    if args.dry_run:
        failures = parse_failures(args.fail)
        field = backends.FakeField(failures)
        web = backends.FakeWeb(failures)
        lift = backends.FakeLift(failures)
        ultrasonic = backends.FakeUltrasonic()
        sleep = lambda _seconds: None  # noqa: E731
    else:
        if not (sys.stdin.isatty() and sys.stdout.isatty()):
            raise SystemExit("실기 리허설은 키보드 조종 때문에 터미널(TTY)에서 실행해야 합니다")
        field = backends.FieldStack(container=args.container, root=ROOT)
        web = backends.ButtonWeb(args.web_url or settings.get("web_url", "http://127.0.0.1:8091"))
        ultrasonic = backends.Ultrasonic(port=us_port, stale_sec=us_settings.get("stale_sec", 0.5))
        lift = backends.LiftUno(args.lift_port or settings["lift_port"], settings.get("lift_move_size", 166400),
                                line_hook=None if us_port else ultrasonic.feed)
        import time
        sleep = time.sleep
    log = runner.RunLog(args.log_root, args.label + ("_dryrun" if args.dry_run else ""))
    ui = runner.ConsoleUI(auto=args.yes)
    rehearsal = runner.Rehearsal(scenario, field, web, lift, ui, log, nav_restart=args.nav_restart,
                                 record=not args.no_record, allow_preview_arm=args.allow_preview_arm or args.dry_run,
                                 label=args.label, sleep=sleep, ultrasonic=ultrasonic)
    meta = rehearsal.run(selected, steps)
    return 0 if meta["verdict"].startswith("한 번에") else 1


if __name__ == "__main__":
    sys.exit(main())
