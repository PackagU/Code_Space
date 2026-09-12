#!/usr/bin/env bash
set -euo pipefail

echo "[deprecated wrapper] base must already be running via start_field_base.sh"
echo "Use docs/deployment/04_field_mapping_navigation.md and save_field_map.sh."
exec "$(dirname "$0")/start_field_mapping.sh" "$@"
