# 2026-09-14~15 자율주행 현장 리뷰와 개선 자료

브랜치 `lee/sim-real-maps`는 젯슨 미러 `lee/jetson-live` 커밋 `4bf35636466e073a6321de8ff098b4b09816d516`에서 시작한다. 리눅스 데스크톱에서 실측 지도 Gazebo 시뮬을 준비하기 위한 작업 브랜치다. 이 폴더의 판정은 원문대로 **오프라인 검증**이며, 실물 검증 근거로 쓰지 않는다.

## 먼저 읽을 것

1. [session_autonomy_improvement_20260915_175043_KST.md](session_autonomy_improvement_20260915_175043_KST.md): 젯슨에서 무엇을 바꿨고 무엇을 확인했는지
2. [2026-09-15_autonomy_test_review.md](2026-09-15_autonomy_test_review.md): 9/14 실패 원인 분석
3. [2026-09-15_autonomy_field_procedure.md](2026-09-15_autonomy_field_procedure.md): 다음 현장 절차와 기록 양식

## 문서 안 경로 대응표

문서는 Windows PC 저장소에서 쓴 원본 그대로 복사했다. 문서 속 링크는 PC 경로 기준이라 이 브랜치에서는 아래처럼 읽는다.

| 문서 속 경로 (PC) | 이 브랜치 |
|---|---|
| `artifacts/autonomy_improvement_20260915/` | `docs/field_review_20260915/autonomy_improvement_20260915/` |
| `artifacts/f1_manual_clean_20260914_v2/` | `docs/field_review_20260915/f1_manual_clean_20260914_v2/` |
| `artifacts/autonomy_test_review_20260915/*_crop.png` | `docs/field_review_20260915/photos/` |
| `logs/field_execution/20260915_autonomy_test_collection/{README.md,session_summary.csv}` | `docs/field_review_20260915/autonomy_test_collection_20260915/` |
| `logs/field_execution/autonomy_improvement_20260915_175043_KST/README.md` | `docs/field_review_20260915/session_autonomy_improvement_20260915_175043_KST.md` |
| 세션 `after/SHA256SUMS` | `docs/field_review_20260915/jetson_after_SHA256SUMS.txt` |
| 컨테이너 `/ros2_ws/maps/field/` | `src/slam_pkg/maps/field/` |
| 컨테이너 `/ros2_ws/scripts/` | `scripts/` |

`analyze_waypoints.py`와 `recount_logs.py`는 PC 저장소 경로(`logs/field_execution/...extracted/...`)를 입력으로 가정한다. 이 브랜치에서 돌리려면 경로를 `src/slam_pkg/maps/field/`로 바꿔야 한다. `build_map.cjs`는 PC 전용 `sharp` 절대경로를 쓴다.

## 지도 (`src/slam_pkg/maps/field/`)

PGM은 저장소 `.gitignore`(`*.pgm`) 대상이라 이 브랜치에만 강제로 추가했다. SHA256은 젯슨 `map_pins.json`·백업과 같다.

| 층 | 파일 | SHA256 |
|---|---|---|
| **F1 기본(pointer·핀, 2026-09-15 20:39~)** | `f1/f1_manual_clean_v3.yaml` / `.pgm` — v2 직각화(21.75°)·엘리베이터 네모·254/0/205·free_thresh 0.19 | yaml `d701d2b6…`, pgm `d525759b…` |
| F1 이전 기본 | `f1/f1_manual_clean_v2.yaml` / `.pgm` | yaml `5ed28bf0…`, pgm `3f9a40c4…` |
| F1 그 이전 | `f1/f1_manual_clean_v1.yaml` / `.pgm` | pgm `364fe2ad…` |
| F2 기본 | `f2/f2_nav_unknown_v1.yaml` + `f2/f2_raw_20260914.pgm` | yaml `b670cd2b…`, pgm `f89bfeb9…` |
| F3 기본 | `f3/f3_c192.yaml` / `.pgm` | yaml `2221c27f…`, pgm `ace8779d…` |

`waypoints.json`(`ebcfb132…`), `map_pins.json`(`eb5aaa29…`, F1=v3), 층별 `latest_map.txt`는 미러 커밋에 들어 있다. v1과 v2의 차이는 유리 고정문 occupied 선분 51셀이다. v3의 유리문은 `geometry_v3.json`의 `inner_walls_uv` 두 번째 선분이다.

**Nav2 기본값(2026-09-15 20:40~)**: `src/slam_pkg/config/nav2_params.yaml` = wall_push_v1 적용본(`80787840…`). 이전 기본값은 `nav2_params_pre_wallpush_20260915.yaml`(`e8cf213b…`), 증속 후보 v011/v012는 wall_push 포함본이다.

**리눅스 데스크톱 시뮬 지시문**: [LINUX_DESKTOP_SIM_PROMPT.md](LINUX_DESKTOP_SIM_PROMPT.md)

## 20:3x KST 추가분

- F2에서 벽에 붙은 지점 정정: 복도→로비 코너 직전 왼쪽 벽의 움푹 들어간 곳 `(-12.95, -5.25)`. 세션 문서 앞부분의 `(-11.05, -1.85)` 추정은 틀렸다.
- `src/slam_pkg/config/nav2_params_wall_push_v1.yaml`: global inflation 1.0 m·cost_scaling 2.0, planner cost_travel_multiplier 3.0(local·RPP 불변). 근거 `autonomy_improvement_20260915/evaluate_wall_push.py`·`wall_push_evaluation.json`. 시뮬 S4의 후보 중 하나로 쓸 수 있다.
- `f1_manual_clean_20260915_v3/`: v3 생성 스크립트와 v2 대비 그림.

| 문서 속 경로 (PC) | 이 브랜치 |
|---|---|
| `artifacts/f1_manual_clean_20260915_v3/` | `docs/field_review_20260915/f1_manual_clean_20260915_v3/` |

## 올리지 않은 것

- `.env`와 키·토큰·인증·세션 파일(값을 읽지 않음)
- SLAM 직렬화 바이너리 `*.posegraph`·`*.data`, 백업 사본 `*.bak*`·`*.before-*`
- 9/15 수집 로그 tar(12.7 MB)와 풀어 놓은 ROS 로그
- 12 MB급 현장 사진 원본 3장(`f1_idle_stop_view_1/2.png`, `f1_wrong_door_forced_stop.png`), 보조 바퀴 사진
- hwp 파일
