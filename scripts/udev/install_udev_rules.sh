#!/usr/bin/env bash
# Jetson 호스트에서 실행: sudo bash scripts/udev/install_udev_rules.sh
set -euo pipefail

RULES_SRC="$(cd "$(dirname "$0")" && pwd)/99-packagu-devices.rules"
RULES_DST="/etc/udev/rules.d/99-packagu-devices.rules"

if [[ ${EUID} -ne 0 ]]; then
  echo "error: run with sudo" >&2
  exit 1
fi

cp "${RULES_SRC}" "${RULES_DST}"
udevadm control --reload-rules
udevadm trigger
echo "installed ${RULES_DST}. 확인: ls -l /dev/rplidar /dev/opencr"
