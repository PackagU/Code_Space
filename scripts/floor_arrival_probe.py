#!/usr/bin/env python3
"""Observe the mission's target-floor condition without ROS or motion commands."""
import argparse
import json
from pathlib import Path
import sys
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "test_workspace/elevator_mission/src/elevator_mission_pkg"))
from elevator_mission_pkg.elevator_camera import ElevatorArrivalGate, reader_observation


class FloorArrivalProbe:
    def __init__(self, target, wall_now):
        self.gate = ElevatorArrivalGate(target)
        self.gate.begin_looking(wall_now)

    def check(self, reader_state, now, wall_now):
        observation = reader_observation(reader_state, wall_now)
        self.gate.observe_floor(observation, now, wall_now)
        result = {
            "target_floor": self.gate.target,
            "current_floor": observation.get("floor", "UNKNOWN")
            if observation.get("valid") else "UNKNOWN",
            "confirmed_frames": min(self.gate.count, self.gate.required_frames),
            "required_frames": self.gate.required_frames,
            "exit_condition_met": self.gate.confirmed(now, wall_now),
            "motion_authorized": False,
            "reader_error": reader_state.get("error") or ""
            if isinstance(reader_state, dict) else "reader_unavailable",
        }
        return result

    def poll(self, reader_url):
        error = ""
        try:
            with urlopen(reader_url, timeout=.4) as response:
                state = json.loads(response.read(65536))
        except (OSError, ValueError, TypeError) as exc:
            state = None
            error = "reader_unavailable:" + type(exc).__name__
        result = self.check(state, time.monotonic(), time.time())
        if error:
            result["reader_error"] = error
        return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", default="4", help="목표 층: 4, F4, B1 등")
    parser.add_argument("--reader-url", default="http://127.0.0.1:8765/api/state")
    args = parser.parse_args(argv)
    try:
        probe = FloorArrivalProbe(args.target, time.time())
    except ValueError as exc:
        parser.error(str(exc))
    print("목표 %s · 층수 판정 시험 · 실제 이동 명령 없음 (Ctrl+C 종료)" % probe.gate.target,
          flush=True)
    last, printed_at = None, float("-inf")
    try:
        while True:
            result = probe.poll(args.reader_url)
            now = time.monotonic()
            if result != last or now - printed_at >= 1:
                print(json.dumps(result, ensure_ascii=False), flush=True)
                last, printed_at = result, now
            time.sleep(.1)
    except KeyboardInterrupt:
        print("시험 종료", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
