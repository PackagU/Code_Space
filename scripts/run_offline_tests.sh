#!/usr/bin/env bash
# 오프라인 테스트 스위트 원커맨드 러너 (host / 컨테이너 / CI 공용).
#
# 규칙:
# - ROS(rclpy 등)가 있는 환경에서는 전부 PASS 해야 한다.
# - ROS가 없는 환경(CI ubuntu 등)에서는 ROS 모듈 ImportError 로 죽는
#   테스트만 SKIP 으로 집계하고, 나머지는 전부 PASS 해야 한다.
# 사용: bash scripts/run_offline_tests.sh
set -uo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

HAVE_ROS=0
python3 -c "import rclpy" 2>/dev/null && HAVE_ROS=1
echo "ROS python available: ${HAVE_ROS}"

ROS_MODULE_PATTERN='ModuleNotFoundError.*(rclpy|launch|launch_ros|ament_index_python|geometry_msgs|std_msgs|std_srvs|nav_msgs|nav2_msgs|sensor_msgs|gazebo_msgs|py_trees_ros|tf2)'

PY_TESTS=(
  scripts/test_wasd_teleop.py
  scripts/test_opencr_protocol.py
  scripts/test_diff_drive_odometry.py
  scripts/test_opencr_bridge_dryrun.py
  scripts/test_field_mapping_launch.py
  scripts/test_field_scripts_contract.py
  scripts/test_udev_contract.py
  scripts/test_jetson_deployment_contract.py
  scripts/test_kku_navigation_launch.py
  scripts/test_arm_sequence.py
  scripts/check_portability.py
  test_workspace/elevator_mission/scripts/test_behaviors_dryrun.py
  test_workspace/elevator_mission/scripts/test_point_registry.py
  test_workspace/elevator_mission/scripts/test_orthogonal_router.py
  test_workspace/elevator_mission/scripts/test_navigate_route_behavior.py
  test_workspace/elevator_mission/scripts/test_nav2_orthogonal_tuning.py
  test_workspace/elevator_auto_map_switch/scripts/test_auto_switch_state_machine.py
  test_workspace/elevator_auto_map_switch/scripts/test_floor_map_registry.py
  test_workspace/elevator_auto_map_switch/scripts/test_switch_floor_compatibility.py
  test_workspace/elevator_auto_map_switch/scripts/test_orchestrator_ros_smoke.py
  test_workspace/gazebo_world_swap/scripts/test_swap_trigger.py
  test_workspace/gazebo_world_swap/scripts/test_world_model.py
  test_workspace/gazebo_world_swap/scripts/test_world_swap_node_contract.py
  test_workspace/gazebo_world_swap/scripts/test_gazebo_launch_spawn_point.py
  test_workspace/gazebo_world_swap/scripts/test_package_metadata.py
  test_workspace/gazebo_world_swap/scripts/test_smoke_scripts_contract.py
  test_workspace/gazebo_world_swap/scripts/test_pedestrians_stub.py
  test_workspace/gazebo_world_swap/scripts/test_static_obstacles_layout.py
)

pass=0
fail=0
skip=0
failed_tests=()

for test in "${PY_TESTS[@]}"; do
  output="$(python3 "${test}" 2>&1)"
  rc=$?
  if [[ ${rc} -eq 0 ]]; then
    echo "PASS ${test}"
    pass=$((pass + 1))
  elif [[ ${HAVE_ROS} -eq 0 ]] && grep -qE "${ROS_MODULE_PATTERN}" <<<"${output}"; then
    echo "SKIP ${test} (ROS 모듈 없음)"
    skip=$((skip + 1))
  else
    echo "FAIL ${test} (exit ${rc})"
    echo "${output}" | tail -5 | sed 's/^/    /'
    fail=$((fail + 1))
    failed_tests+=("${test}")
  fi
done

# 셸 계약 테스트 (환경 무관, 항상 PASS 필요)
if bash scripts/test_kku_sim_scripts.sh >/dev/null 2>&1; then
  echo "PASS scripts/test_kku_sim_scripts.sh"
  pass=$((pass + 1))
else
  echo "FAIL scripts/test_kku_sim_scripts.sh"
  fail=$((fail + 1))
  failed_tests+=(scripts/test_kku_sim_scripts.sh)
fi

echo ""
echo "결과: PASS=${pass} SKIP=${skip} FAIL=${fail}"
if [[ ${fail} -gt 0 ]]; then
  printf '실패 목록:\n'
  printf '  %s\n' "${failed_tests[@]}"
  exit 1
fi
