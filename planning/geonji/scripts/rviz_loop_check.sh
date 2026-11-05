#!/usr/bin/env bash
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
loop_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$loop_workspace/.f1tenth-workspace" && "$loop_workspace" != / ]]; do
    loop_workspace=$(dirname -- "$loop_workspace")
done
if [[ ! -f "$loop_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$loop_workspace"
bash scripts/rviz_map_review.sh
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m tools.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v3_static_only/' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 -m tools.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v3_static_only/area.npz /review/static_only > /tmp/static-area.log 2>&1'
fi
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m tools.serve_loop_check' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 -m tools.serve_loop_check > /tmp/loop-check.log 2>&1'
fi
docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec rviz2 -d /ws/config/geonji_loop_check.rviz > /tmp/loop-check-rviz.log 2>&1'
