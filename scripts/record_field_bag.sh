#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
BAG_NAME="${1:-field_$(date '+%Y%m%d_%H%M%S')}"
DURATION_SEC="${DURATION_SEC:-0}"
REQUIRE_IMU="${REQUIRE_IMU:-0}"
INCLUDE_CAMERA="${INCLUDE_CAMERA:-0}"
INCLUDE_CONTROL_DIAGNOSTICS="${INCLUDE_CONTROL_DIAGNOSTICS:-0}"
INCLUDE_NAV_DIAGNOSTICS="${INCLUDE_NAV_DIAGNOSTICS:-0}"
# 2026-09-15: rosbag2 기본 cache(100 MiB)는 저속 토픽에서 수 분간 파일이 커지지 않아 record check가
# 기록 여부를 판정할 수 없고, 전원 차단 시 잃는 구간도 길다. [제안값] 256 KiB로 자주 flush한다.
BAG_MAX_CACHE_BYTES="${BAG_MAX_CACHE_BYTES:-262144}"

[[ "${CONTAINER_NAME}" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "error: invalid container name" >&2; exit 2; }
[[ "${BAG_NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "error: invalid bag name" >&2; exit 2; }
[[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || { echo "error: DURATION_SEC must be a non-negative integer" >&2; exit 2; }
[[ "${BAG_MAX_CACHE_BYTES}" =~ ^[0-9]+$ ]] || { echo "error: BAG_MAX_CACHE_BYTES must be a non-negative integer" >&2; exit 2; }
for value in "${REQUIRE_IMU}" "${INCLUDE_CAMERA}" "${INCLUDE_CONTROL_DIAGNOSTICS}" "${INCLUDE_NAV_DIAGNOSTICS}"; do
  [[ "${value}" == "0" || "${value}" == "1" ]] || { echo "error: feature flags must be 0 or 1" >&2; exit 2; }
done

docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER_NAME}" || {
  echo "error: ${CONTAINER_NAME} is not running" >&2
  exit 1
}
logs_rw="$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/ros2_ws/logs"}}{{.RW}}{{end}}{{end}}' "${CONTAINER_NAME}")"
[[ "${logs_rw}" == "true" ]] || {
  echo "error: /ros2_ws/logs must be a writable persistent mount" >&2
  exit 1
}

in_container() {
  docker exec -i -w /ros2_ws "${CONTAINER_NAME}" bash -lc \
    "source /opt/ros/humble/setup.bash && source install/setup.bash && $*"
}

topic_list="$(in_container 'ros2 topic list')"
topics=(/scan /odom /tf /tf_static)
for required in "${topics[@]}"; do
  grep -Fxq "${required}" <<<"${topic_list}" || {
    echo "error: required topic missing: ${required}" >&2
    exit 1
  }
done
if grep -Fxq /imu <<<"${topic_list}"; then
  topics+=(/imu)
elif [[ "${REQUIRE_IMU}" == "1" ]]; then
  echo "error: REQUIRE_IMU=1 but /imu is missing" >&2
  exit 1
fi
if [[ "${INCLUDE_CAMERA}" == "1" ]]; then
  for camera_topic in /camera/image_raw /camera/camera_info; do
    grep -Fxq "${camera_topic}" <<<"${topic_list}" || {
      echo "error: requested camera topic missing: ${camera_topic}" >&2
      exit 1
    }
    topics+=("${camera_topic}")
  done
fi
if [[ "${INCLUDE_CONTROL_DIAGNOSTICS}" == "1" ]]; then
  for diagnostic_topic in /cmd_vel /cmd_vel_safe /diagnostics; do
    grep -Fxq "${diagnostic_topic}" <<<"${topic_list}" && topics+=("${diagnostic_topic}")
  done
fi
absent_nav_topics=()
if [[ "${INCLUDE_NAV_DIAGNOSTICS}" == "1" ]]; then
  # 2026-09-15: 실패 지점 재현용. 존재하는 토픽만 기록하고 없는 이름은 absent 목록으로 남긴다.
  # /map과 global costmap 전체는 transient-local 1회 + updates로만 커진다(always_send_full_costmap=False).
  nav_topics=(
    /amcl_pose /initialpose /goal_pose /map
    /plan /local_plan /received_global_plan /lookahead_point
    /cmd_vel_nav /cmd_vel_teleop
    /local_costmap/costmap /local_costmap/costmap_updates /local_costmap/published_footprint
    /global_costmap/costmap /global_costmap/costmap_updates
    /navigate_to_pose/_action/status /navigate_to_pose/_action/feedback
    /drive/ready /imu/ready /nav_safety/ready /nav_safety/stopped /nav_safety/status /nav_safety/stop
    /rosout
  )
  for nav_topic in "${nav_topics[@]}"; do
    if grep -Fxq "${nav_topic}" <<<"${topic_list}"; then
      topics+=("${nav_topic}")
    else
      absent_nav_topics+=("${nav_topic}")
    fi
  done
fi

output_dir="/ros2_ws/logs/field_bags/${BAG_NAME}"
in_container "test ! -e '${output_dir}'" || {
  echo "error: refusing to overwrite ${output_dir}" >&2
  exit 1
}
printf '[field-bag] output=%s\n' "${output_dir}"
printf '[field-bag] topics=%s\n' "${topics[*]}"
printf '[field-bag] absent_nav_topics=%s\n' "${absent_nav_topics[*]:-none}"
printf '[field-bag] replay never includes cmd_vel/arm/lift topics\n'

topic_args="${topics[*]}"
record_opts="--qos-profile-overrides-path scripts/rosbag_qos_overrides.yaml --max-cache-size ${BAG_MAX_CACHE_BYTES}"
set +e
if [[ "${DURATION_SEC}" == "0" ]]; then
  in_container "ros2 bag record ${record_opts} -o '${output_dir}' ${topic_args}"
  record_rc=$?
else
  in_container "timeout --signal=INT --kill-after=5 '${DURATION_SEC}' ros2 bag record ${record_opts} -o '${output_dir}' ${topic_args}"
  record_rc=$?
fi
set -e
if [[ ${record_rc} -ne 0 && ${record_rc} -ne 124 && ${record_rc} -ne 130 ]]; then
  echo "error: rosbag recorder exited ${record_rc}" >&2
  exit "${record_rc}"
fi

required_args="--require /scan --require /odom --require /tf --require /tf_static"
[[ "${REQUIRE_IMU}" == "1" ]] && required_args+=" --require /imu"
in_container "python3 scripts/bag_contract.py inspect '${output_dir}' ${required_args} > '${output_dir}/contract.json'"
in_container "ros2 bag info '${output_dir}' > '${output_dir}/info.txt'"
in_container "printf '%s\n' ${absent_nav_topics[*]:-} > '${output_dir}/absent_nav_topics.txt'"
in_container "cd '${output_dir}' && sha256sum metadata.yaml *.db3 > SHA256SUMS"
in_container "du -sb '${output_dir}'"
echo "BAG_PATH=${output_dir}"
