#!/usr/bin/env bash
# Offline checks for sim/real_maps (same rule as scripts/run_offline_tests.sh: ROS-dependent tests SKIP
# when ROS python is unavailable). Not registered in the shared runner on purpose (field files untouched).
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
pass=0 skip=0 fail=0
for t in test_geometry.py test_world_params.py test_pose_capture_sim.py; do
  out="$(python3 "$t" 2>&1)"; code=$?
  if (( code != 0 )); then
    if grep -Eq 'ModuleNotFoundError.*(rclpy|nav2_msgs|nav_msgs|geometry_msgs)' <<<"$out"; then
      echo "SKIP $t (ROS unavailable)"; ((skip++))
    else
      echo "FAIL $t"; echo "$out" | tail -15; ((fail++))
    fi
  elif grep -q '^SKIP' <<<"$out"; then echo "SKIP $t"; ((skip++))
  else echo "PASS $t"; ((pass++)); fi
done
echo "sim tests: PASS $pass / SKIP $skip / FAIL $fail"
(( fail == 0 ))
