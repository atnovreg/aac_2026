#!/usr/bin/env bash
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
static_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$static_workspace/.f1tenth-workspace" && "$static_workspace" != / ]]; do
    static_workspace=$(dirname -- "$static_workspace")
done
if [[ ! -f "$static_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$static_workspace"
bash src/aac_2026/navigation/map_review/scripts/rviz_map_review.sh
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m planning.geonji.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v3_static_only/' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec python3 -m planning.geonji.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v3_static_only/area.npz /review/static_only > /tmp/static-area.log 2>&1'
fi
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m planning.geonji.serve_gap_review --area results/건지광장/drivable_area/083146_v3_static_only' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec python3 -m planning.geonji.serve_gap_review --area results/건지광장/drivable_area/083146_v3_static_only --topic /review/static_only/gaps --remaining-only > /tmp/static-gaps.log 2>&1'
fi
docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec rviz2 -d /ws/src/aac_2026/planning/geonji/config/geonji_area_static.rviz > /tmp/static-area-rviz.log 2>&1'
