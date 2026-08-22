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
                "WITH_STRESS", "WITH_RT_PRIORITY", "WITH_F3", "WITH_ARM", "WITH_RETURN",
                "WITH_STATIC_OBSTACLE", "WITH_LIFT", "SKIP_BUILD"):
        assert f'{opt}="${{{opt}:-0}}"' in runner_text, f"{opt} default must be 0 (deterministic)"

    # --- 회귀 보호: 2026-08-20 적대 리뷰 핫픽스 (review_report §3 findings 번호) ---
    assert "trap 'exit 130' INT" in runner_text and "trap 'exit 143' TERM" in runner_text, (
        "#5: INT/TERM trap must preserve non-zero exit (interrupted run counted as PASS before)"
    )
    assert 'log "RUN_DIR=$RUN_DIR"' in runner_text, "#17: RUN_DIR contract line for repeat runner missing"
    assert '&& grep -q "success=True" "$logf"' in runner_text, (
        "#4: request_switch must judge by response body success=True (ros2 service call rc=0 on rejection)"
    )
    assert "tail -n +$((anchor + 1))" in runner_text, "#19: armed-evidence grep must be time-anchored"
    assert runner_text.count("/navigate_to_pose/_action/cancel_goal") >= 2, (
        "#15: live goal must be cancelled before retry and on final failure"
    )
    assert "finalize_artifacts 2>/dev/null || true" in runner_text, "#13: EXIT trap must call finalize (idempotent)"
    assert 'if [[ "${FINALIZED:-0}" == "1" ]]; then return 0; fi' in runner_text, "#13: finalize must be idempotent"
    fin_body = runner_text.split("finalize_artifacts() {", 1)[1].split("\n}\n", 1)[0]
    fin_code = "\n".join(l for l in fin_body.splitlines() if not l.strip().startswith("#"))
    assert "| wc -l" not in fin_code, (
        "finalize must not use 'ls ... | wc -l' (pipefail rc=2 on no match killed finalize under set -e — G004 run4)"
    )
    assert fin_body.rstrip().endswith('log "artifacts collected -> $RUN_DIR (summary: $RUN_DIR/scenario_summary.md)"') \
        and "FINALIZED=1\n  log \"artifacts collected" in fin_body, "FINALIZED must be set only after the archive copy"
    assert 'finalize_artifacts || log "WARN: finalize_artifacts returned non-zero' in runner_text, (
        "main-flow finalize must not change the mission verdict"
    )
    assert 'kill -TERM "$PROFILE_PID"' in runner_text, "#3: profiler must be flushed before archive copy"
    assert 'if [[ ! "$missed" =~ ^[0-9]+$ ]]; then' in runner_text, "#12: missed-rate gate must fail closed"
    gate_idx = runner_text.index("CONTROL FIDELITY FAIL: missed-rate metric unavailable")
    final_idx = runner_text.rindex('\nfinalize_artifacts || log "WARN')
    assert gate_idx < final_idx, "#12: missed-rate gate must be evaluated before finalize_artifacts"
    for fn in ("wait_for_topic", "wait_for_service", "wait_for_action"):
        body = runner_text.split(f"{fn}() {{", 1)[1].split("\n}\n", 1)[0]
        assert "timeout 10 ros2" in body, f"#14: {fn} CLI must be wrapped in timeout"
    assert "local num_re='^-?[0-9]+([.][0-9]+)?([eE][+-]?[0-9]+)?$'" in runner_text, (
        "#16: realign parser must validate numerics before awk"
    )
    assert "nav2_goal_${name}_a${attempt}.fail.log" in runner_text, "#23: failed attempt logs must be preserved"
    # 2026-08-21 G004 run2: rmw 응답 유실 시 서버(bt_navigator) 로그 증거로 조기 판정 + F2 출구 스테이징 goal
    assert "SERVER-EVIDENCE: bt_navigator 'Goal succeeded' after anchor" in runner_text, (
        "send_nav_goal must absorb lost goal responses via anchored bt_navigator evidence"
    )
    assert '"$f" == *.responselost.log' in runner_text, "summary must skip response-lost attempt copies"
    assert '"$f" == *.dropped.log' in runner_text, "summary must skip dropped-request copies"
    assert "quick resend $quick/3 (no retry budget)" in runner_text, (
        "send_nav_goal must quick-resend when the server dropped the request (response race, no 'Begin navigating')"
    )
    assert "send_nav_goal f2_elevator_exit 1.8 0.0 0.0 1.0" in runner_text, (
        "F2 elevator-exit staging goal missing (straight exit before turning north — run2 collision-ahead)"
    )
    f2_exit_idx = runner_text.index("send_nav_goal f2_elevator_exit 1.8 0.0")
    f2_corr_idx = runner_text.index("send_nav_goal f2_corridor 2.5 12.0")
    assert f2_exit_idx < f2_corr_idx, "staging goal must precede f2_corridor"
    assert 'index($0,"버튼 시퀀스 완료"){c++}' in runner_text, "#20: arm completion must be paired per floor (anchored)"
    # 정적 장애물 / 리프트 옵션의 fail-closed 호출 경로
    assert "spawn_static_obstacles.py" in runner_text and "STATIC OBSTACLE SPAWN FAILED" in runner_text
    assert "lift_cycle.py" in runner_text and 'run_lift_cycle pickup' in runner_text \
        and 'run_lift_cycle f2_delivery' in runner_text, "lift cycle must run at pickup and F2 delivery"

    # --- 회귀 보호: pedestrians.py 구조 (AST) — a6a5115 형 재발 차단 ---
    import ast
    ped_src = (repo_root / "test_workspace" / "gazebo_world_swap" / "pedestrian" / "pedestrians.py")
    tree = ast.parse(ped_src.read_text(encoding="utf-8"))
    cls = next(n for n in ast.walk(tree) if isinstance(n, ast.ClassDef) and n.name == "Pedestrians")
    methods = {n.name: n for n in cls.body if isinstance(n, ast.FunctionDef)}
    init_calls = {n.func.attr for n in ast.walk(methods["__init__"])
                  if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert "create_timer" in init_calls and "_spawn" in init_calls, (
        "pedestrians.__init__ must create the tick timer and spawn pedestrians (a6a5115 regression)"
    )
    for cb in ("_on_amcl", "_on_model_states"):
        assert cb in methods, f"pedestrians.py must define {cb}"
        cb_calls = {n.func.attr for n in ast.walk(methods[cb])
                    if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
        for forbidden in ("create_timer", "_spawn", "wait_for_service", "create_subscription"):
            assert forbidden not in cb_calls, (
                f"pedestrians.{cb} must only update robot position — found {forbidden} (re-init regression)"
            )
    # 정적 장애물 배치 단일 소스 + 레이아웃 테스트 존재
    layout = repo_root / "test_workspace" / "gazebo_world_swap" / "pedestrian" / "static_obstacle_layout.py"
    assert layout.exists() and "<static>true</static>" in layout.read_text(encoding="utf-8")
    assert (scripts / "test_static_obstacles_layout.py").exists()
    assert (scripts / "test_pedestrians_stub.py").exists()
    assert (scripts / "check_idle_drift.sh").exists() and os.access(scripts / "check_idle_drift.sh", os.X_OK), (
        "wheel friction regression check (check_idle_drift.sh) missing"
    )
    # 리프트: URDF 의 lift_joint 는 자기잠금 dynamics(friction) 를 가져야 한다 (플러그인이 조인트를 놓음)
    urdf = repo_root / "src" / "common_pkg" / "urdf" / "delivery_robot.urdf.xacro"
    urdf_text = urdf.read_text(encoding="utf-8")
    lift_block = urdf_text.split('<joint name="lift_joint"', 1)[1].split("</joint>", 1)[0]
    assert "<dynamics" in lift_block and "friction=" in lift_block, (
        "lift_joint needs <dynamics friction> so the carrier holds position after joint_pose_trajectory releases it"
    )
    assert 'filename="libgazebo_ros_joint_pose_trajectory.so"' in urdf_text, "lift trajectory plugin missing"
    lift_cycle = (scripts / "lift_cycle.py").read_text(encoding="utf-8")
    assert 'traj.header.frame_id = "world"' in lift_cycle, (
        "lift_cycle must set frame_id='world' — joint_pose_trajectory aborts on empty frame_id (G004 run1)"
    )
    # 바퀴 물리 확정값 (2026-08-21, session_wiki/2026-08-20_review_followup §2): 차축 fdir1 + 캐스터 마찰.
    # "1 0 0" 은 θ≈90° 퇴화(정지 중 자발 yaw), 캐스터 mu 0 은 차축 fdir1 과 조합 시 6mm/s 크리프.
    assert urdf_text.count("<fdir1>0 0 1</fdir1>") == 2, "wheel fdir1 must be the axle direction '0 0 1' (collision frame)"
    assert "<fdir1>1 0 0</fdir1>" not in urdf_text, "fdir1 '1 0 0' rotates with the wheel (degenerate at 90deg) — forbidden"
    import re as _re
    for caster in ("caster_wheel", "caster_wheel_front"):
        blk = urdf_text.split(f'<gazebo reference="{caster}">', 1)[1].split("</gazebo>", 1)[0]
        mu = float(_re.search(r"<mu1>([0-9.]+)</mu1>", blk).group(1))
        assert mu >= 0.3, f"{caster} mu1={mu}: frictionless casters + axle fdir1 creep 6mm/s (need >= 0.3)"
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
