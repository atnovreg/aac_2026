#!/usr/bin/env bash
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
assumed_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$assumed_workspace/.f1tenth-workspace" && "$assumed_workspace" != / ]]; do
    assumed_workspace=$(dirname -- "$assumed_workspace")
done
if [[ ! -f "$assumed_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$assumed_workspace"
bash scripts/rviz_map_review.sh
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m tools.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v4_assumed_floor/' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 -m tools.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v4_assumed_floor/area.npz /review/assumed_floor > /tmp/assumed-floor.log 2>&1'
fi
docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec rviz2 -d /ws/config/geonji_assumed_floor.rviz > /tmp/assumed-floor-rviz.log 2>&1'
