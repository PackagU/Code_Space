#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${PACKAGU_CONTAINER_NAME:-ros2_humble}"
BAG_NAME="${1:-field_$(date '+%Y%m%d_%H%M%S')}"
DURATION_SEC="${DURATION_SEC:-0}"
REQUIRE_IMU="${REQUIRE_IMU:-0}"
INCLUDE_CAMERA="${INCLUDE_CAMERA:-0}"
INCLUDE_CONTROL_DIAGNOSTICS="${INCLUDE_CONTROL_DIAGNOSTICS:-0}"

[[ "${CONTAINER_NAME}" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "error: invalid container name" >&2; exit 2; }
[[ "${BAG_NAME}" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "error: invalid bag name" >&2; exit 2; }
[[ "${DURATION_SEC}" =~ ^[0-9]+$ ]] || { echo "error: DURATION_SEC must be a non-negative integer" >&2; exit 2; }
for value in "${REQUIRE_IMU}" "${INCLUDE_CAMERA}" "${INCLUDE_CONTROL_DIAGNOSTICS}"; do
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

output_dir="/ros2_ws/logs/field_bags/${BAG_NAME}"
in_container "test ! -e '${output_dir}'" || {
  echo "error: refusing to overwrite ${output_dir}" >&2
  exit 1
}
printf '[field-bag] output=%s\n' "${output_dir}"
printf '[field-bag] topics=%s\n' "${topics[*]}"
printf '[field-bag] replay never includes cmd_vel/arm/lift topics\n'

topic_args="${topics[*]}"
set +e
if [[ "${DURATION_SEC}" == "0" ]]; then
  in_container "ros2 bag record --qos-profile-overrides-path scripts/rosbag_qos_overrides.yaml -o '${output_dir}' ${topic_args}"
  record_rc=$?
else
  in_container "timeout --signal=INT --kill-after=5 '${DURATION_SEC}' ros2 bag record --qos-profile-overrides-path scripts/rosbag_qos_overrides.yaml -o '${output_dir}' ${topic_args}"
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
in_container "cd '${output_dir}' && sha256sum metadata.yaml *.db3 > SHA256SUMS"
in_container "du -sb '${output_dir}'"
echo "BAG_PATH=${output_dir}"
