#!/usr/bin/env python3
"""Static contract checks for world-swap smoke scripts."""

from __future__ import annotations

from pathlib import Path
import os
import sys


def main():
    repo_root = Path(__file__).resolve().parents[3]
    scripts = repo_root / "test_workspace" / "gazebo_world_swap" / "scripts"
    runner = scripts / "run_l3_world_swap_smoke.sh"
    verifier = scripts / "verify_world_swap_state.py"

    assert runner.exists(), "run_l3_world_swap_smoke.sh missing"
    assert verifier.exists(), "verify_world_swap_state.py missing"
    assert os.access(runner, os.X_OK), "runner is not executable"
    assert os.access(verifier, os.X_OK), "verifier is not executable"

    runner_text = runner.read_text(encoding="utf-8")
    verifier_text = verifier.read_text(encoding="utf-8")

    assert "sync_docs_to_notion.py" not in runner_text + verifier_text
    assert "WITH_RVIZ" in runner_text
    assert "rviz:=$WITH_RVIZ" in runner_text
    assert "method:=model_swap" in runner_text
    assert "spawn_point:=charge_station" in runner_text
    assert "publish_initial_pose charge_station" in runner_text
    assert "send_nav_goal f1_parcel_storage" in runner_text
    assert "send_nav_goal f1_elevator_entry" in runner_text
    assert "send_nav_goal f1_elevator_inside" in runner_text
    assert "returning to F1 elevator for floor transfer" in runner_text
    assert "--times 10 --rate 2 /elevator/state" in runner_text
    assert "--once /elevator/state" not in runner_text
    assert "/navigate_to_pose" in runner_text
    assert "send_nav_goal f2_corridor 2.5 12.0" in runner_text
    assert "status: SUCCEEDED" in runner_text
    assert "/get_model_list" in verifier_text
    # 맵 치수 단일 소스: verifier 는 map_expectations 를 통해서만 기대값을 얻는다.
    assert "from map_expectations import" in verifier_text, (
        "verifier must read map dims from map_expectations.py (no hardcoding)"
    )
    for stale_dim in ("498", "348", "477", "299"):
        assert stale_dim not in verifier_text, (
            f"verifier must not hardcode map dimension {stale_dim}"
        )

    # --- 회귀 보호: 새 gated 옵션이 유지되고 기본값이 결정적인지 확인 ---
    for opt in ("WITH_PEDESTRIAN", "WITH_RECOVERY", "WITH_LOC_FAULT", "WITH_PROFILE",
                "WITH_STRESS", "WITH_RT_PRIORITY", "WITH_F3", "WITH_ARM", "WITH_RETURN"):
        assert f'{opt}="${{{opt}:-0}}"' in runner_text, f"{opt} default must be 0 (deterministic)"
    assert "verify_arm_sequence F2" in runner_text, "arm sequence F2 verification missing"
    assert "verify_arm_sequence F3" in runner_text, "arm sequence F3 verification missing"
    assert 'GAZEBO_GUI="${GAZEBO_GUI:-false}"' in runner_text, "GAZEBO_GUI default must be false (headless)"
    assert "gui:=$GAZEBO_GUI" in runner_text, "gazebo launch must pass gui flag"
    assert 'GAZEBO_REMOTE="${GAZEBO_REMOTE:-0}"' in runner_text, "GAZEBO_REMOTE default must be 0 (local gazebo)"
    assert "--floor F3 --from-floor F2" in runner_text, "F3 hop verification missing"
    assert "expect_nav_abort unreachable" in runner_text, "recovery check missing"
    assert "inject_wrong_initialpose" in runner_text, "localization fault hook missing"
    assert "finalize_artifacts" in runner_text, "artifact finalizer missing"
    assert "profile_resources.sh" in runner_text, "profiler hook missing"
    assert "ensure_maps" in runner_text, "fresh-clone map bootstrap missing"
    assert "colcon build --symlink-install --base-paths src" in runner_text, (
        "root build must not absorb test_workspace packages (stale ament index risk)"
    )

    # --- 회귀 보호: 왕복(F1<->F2) 체인 (WITH_RETURN) ---
    assert "--floor F1 --from-floor F2" in runner_text, "F2->F1 reverse verification missing"
    assert "send_nav_goal f1_charge_station 1.6 0.0" in runner_text, "charge station return goal missing"
    assert "verify_arm_sequence F1" in runner_text, "arm sequence F1 verification missing"
    assert "WITH_RETURN=1 과 WITH_F3=1" in runner_text, "WITH_RETURN/WITH_F3 mutual exclusion missing"
    assert "set_orchestrator_target_floor F1" in runner_text, "F1 target_floor helper call missing"
    for floor in ("F1", "F2", "F3"):
        assert f"request_floor_switch {floor}" in runner_text, (
            f"{floor} request_switch must use half-hang-absorbing helper (§1.23e)"
        )
    assert "set_orchestrator_target_floor F3" in runner_text, (
        "F3 target_floor must use timeout helper (bare 'ros2 param set' half-hang, §1.23e)"
    )

    # --- 회귀 보호: 왕복 반복 러너 (Roadmap 06 반복 안정성) ---
    repeat_runner = scripts / "run_roundtrip_repeat.sh"
    assert repeat_runner.exists(), "run_roundtrip_repeat.sh missing"
    assert os.access(repeat_runner, os.X_OK), "repeat runner is not executable"
    repeat_text = repeat_runner.read_text(encoding="utf-8")
    assert 'REPEAT_N="${REPEAT_N:-10}"' in repeat_text, "repeat default must be 10"
    assert "oom" in repeat_text.lower(), "OOM guard missing"
    assert "repeat_summary.md" in repeat_text, "aggregate summary missing"

    # --- 회귀 보호: 복도 폭 축소가 실수로 들어가지 않았는지 (CORRIDOR_HALF=2.5 고정) ---
    worlds_gen = repo_root / "scripts" / "generate_kku_worlds.py"
    assert worlds_gen.exists(), "generate_kku_worlds.py missing"
    gen_text = worlds_gen.read_text(encoding="utf-8")
    assert "CORRIDOR_HALF = 2.5" in gen_text, (
        "CORRIDOR_HALF must stay 2.5 (5m test corridor). "
        "A 2.0m measured-corridor change is out of scope for this guard."
    )

    # --- 회귀 보호: 층별 맵이 존재하고 단일 소스에서 파싱 가능한지 ---
    # (pgm 은 gitignore — 없으면 scripts/generate_kku_maps.py 로 먼저 생성)
    sys.path.insert(0, str(scripts))
    from map_expectations import KNOWN_FLOORS, load_expectation

    for floor in KNOWN_FLOORS:
        expectation = load_expectation(floor)
        assert expectation["width"] > 0 and expectation["height"] > 0, (
            f"{floor} map header unreadable"
        )
        assert expectation["resolution"] > 0, f"{floor} map resolution invalid"

    print("PASS smoke script contracts")


if __name__ == "__main__":
    main()
