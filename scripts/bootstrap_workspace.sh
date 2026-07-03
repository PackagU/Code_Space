#!/usr/bin/env bash
# fresh clone 후 최초 1회 실행 — 저장소에 없는 생성물(.pgm 등)을 만든다.
# 사용: bash scripts/bootstrap_workspace.sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "[1/3] Gazebo world 생성 (kku_pre_simulation_map.yaml -> worlds/)"
python3 "${ROOT_DIR}/scripts/generate_kku_worlds.py"

echo "[2/3] Nav2 맵 생성 (world 기하 -> .pgm/.yaml)"
python3 "${ROOT_DIR}/scripts/generate_kku_maps.py"

echo "[3/3] 결과 검증"
for floor in f1 f2 f3; do
  pgm="${ROOT_DIR}/src/slam_pkg/maps/kku_virtual/${floor}/kku_${floor}.pgm"
  if [[ ! -f "${pgm}" ]]; then
    echo "error: ${pgm} 생성 실패" >&2
    exit 1
  fi
done

echo "bootstrap OK — 다음 단계:"
echo "  개발(데스크톱): ./scripts/run_kku_sim.sh F1"
echo "  오프라인 테스트: bash scripts/run_offline_tests.sh"
echo "  Jetson 배포:    docs/deployment/01_portability_policy.md 참조"
