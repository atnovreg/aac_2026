#!/usr/bin/env bash
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
gap_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$gap_workspace/.f1tenth-workspace" && "$gap_workspace" != / ]]; do
    gap_workspace=$(dirname -- "$gap_workspace")
done
if [[ ! -f "$gap_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$gap_workspace"
bash scripts/rviz_map_review.sh
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m tools.serve_drivable_area ' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 -m tools.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v2_20cm/area.npz > /tmp/drivable-area.log 2>&1'
fi
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m tools.serve_gap_review' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec python3 -m tools.serve_gap_review > /tmp/gap-review.log 2>&1'
fi
docker exec -d f1tenth-map-review bash -c 'source /opt/ros/jazzy/setup.bash; exec rviz2 -d /ws/config/geonji_gap_review.rviz > /tmp/gap-review-rviz.log 2>&1'
