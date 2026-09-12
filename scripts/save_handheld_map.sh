#!/usr/bin/env bash
# Active handheld SLAM map + posegraph save. Refuses overwrite and unsafe names.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

NAME="${1:-handheld_smoke_$(date +%Y%m%d-%H%M%S)}"
if [[ ! "${NAME}" =~ ^[A-Za-z0-9_-]+$ ]]; then
    echo "ERROR: map name must match [A-Za-z0-9_-]+" >&2
    exit 2
fi
DIR=/ros2_ws/maps/handheld
BASE="${DIR}/${NAME}"
mkdir -p "${DIR}"
if compgen -G "${BASE}*" >/dev/null; then
    echo "ERROR: refusing to overwrite ${BASE}*" >&2
    exit 3
fi

timeout 30 ros2 run nav2_map_server map_saver_cli -f "${BASE}"
timeout 30 ros2 service call /slam_toolbox/serialize_map \
    slam_toolbox/srv/SerializePoseGraph "{filename: '${BASE}'}" | tee "${BASE}_serialize.txt"

test -s "${BASE}.yaml"
test -s "${BASE}.pgm"
if ! compgen -G "${BASE}*.posegraph" >/dev/null && ! compgen -G "${BASE}*.data" >/dev/null; then
    echo "ERROR: posegraph/data file missing" >&2
    exit 4
fi
sha256sum "${BASE}"* >"${BASE}_sha256.txt"
echo "handheld_map_saved base=${BASE}"
ls -lh "${BASE}"*
