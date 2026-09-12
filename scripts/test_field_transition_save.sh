#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
OUTPUT_PATH="${1:?output base path required}"
if [[ "${OUTPUT_PATH}" != /ros2_ws/logs/pre_hw/*/result/OFFLINE_ONLY_* ]]; then
  echo "error: this test only writes OFFLINE_ONLY maps under P06 evidence" >&2
  exit 2
fi
if [[ -e "${OUTPUT_PATH}.yaml" || -e "${OUTPUT_PATH}.pgm" ]]; then
  echo "error: refusing to overwrite ${OUTPUT_PATH}.*" >&2
  exit 1
fi

ros2 run nav2_map_server map_saver_cli -f "${OUTPUT_PATH}"
serialize_output="$(
  ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '${OUTPUT_PATH}'}"
)"
printf '%s\n' "${serialize_output}"
grep -Eq 'result[=:][[:space:]]*0|result=0' <<<"${serialize_output}"
python3 -m slam_pkg.map_contract "${OUTPUT_PATH}.yaml"
sha256sum \
  "${OUTPUT_PATH}.yaml" "${OUTPUT_PATH}.pgm" \
  "${OUTPUT_PATH}.posegraph" "${OUTPUT_PATH}.data" \
  > "${OUTPUT_PATH}.sha256"
