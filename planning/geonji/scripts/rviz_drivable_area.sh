#!/usr/bin/env bash
# Default area viewer now uses static-map obstacle evidence only (v3).
set -euo pipefail
area_workspace="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
if [[ ! -f "$area_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
exec bash "$area_workspace/src/aac_2026/planning/geonji/scripts/rviz_static_area.sh" "$@"
