#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

check_paths() {
    local mode="$1"
    local raw_paths="$2"
    local path target
    local -a paths=()

    [[ -z "${raw_paths}" ]] && return 0
    IFS=',' read -r -a paths <<< "${raw_paths}"
    for path in "${paths[@]}"; do
        [[ -e "${path}" ]] || {
            echo "[entrypoint] ERROR: required ${mode} path is missing: ${path}" >&2
            exit 78
        }
        target="$(findmnt -n -T "${path}" -o TARGET 2>/dev/null || true)"
        [[ -n "${target}" && "${target}" != "/" ]] || {
            echo "[entrypoint] ERROR: required ${mode} path is not mounted: ${path}" >&2
            exit 78
        }
        if [[ "${mode}" == "writable" && ! -w "${path}" ]]; then
            echo "[entrypoint] ERROR: required writable path is read-only: ${path}" >&2
            exit 78
        fi
    done
}

check_paths readable "${PACKAGU_REQUIRED_READABLE_MOUNTS:-}"
check_paths writable "${PACKAGU_REQUIRED_WRITABLE_MOUNTS:-}"

echo "[entrypoint] ROS2 Humble ready. Domain ID: ${ROS_DOMAIN_ID}"
echo "[entrypoint] DISPLAY: ${DISPLAY:-unset}"

exec "$@"
