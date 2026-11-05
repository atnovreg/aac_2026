#!/usr/bin/env bash
# Default area viewer now uses static-map obstacle evidence only (v3).
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
area_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$area_workspace/.f1tenth-workspace" && "$area_workspace" != / ]]; do
    area_workspace=$(dirname -- "$area_workspace")
done
if [[ ! -f "$area_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
exec bash "$area_workspace/src/aac_2026/planning/geonji/scripts/rviz_static_area.sh" "$@"
