#!/usr/bin/env bash
set -euo pipefail
track_workspace="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
if [[ ! -f "$track_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$track_workspace"
bash src/aac_2026/navigation/map_review/scripts/rviz_map_review.sh
if ! docker exec f1tenth-map-review pgrep -f '^python3 -m planning.geonji.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v4_assumed_floor/' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec python3 -m planning.geonji.serve_drivable_area /ws/results/건지광장/drivable_area/083146_v4_assumed_floor/area.npz /review/assumed_floor > /tmp/assumed-floor.log 2>&1'
fi
# Restart only this viewer publisher to load the requested saved run.
docker exec f1tenth-map-review pkill -f '^python3 -m planning.geonji.serve_geonji_track' || true
if [[ $# -gt 0 ]]; then
 docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec python3 -m planning.geonji.serve_geonji_track --run "$1" > /tmp/geonji-track.log 2>&1' bash "$1"
else
 docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec python3 -m planning.geonji.serve_geonji_track > /tmp/geonji-track.log 2>&1'
fi
if ! docker exec f1tenth-map-review pgrep -f '^rviz2 -d /ws/src/aac_2026/planning/geonji/config/geonji_mpcc.rviz' >/dev/null; then
 docker exec -d f1tenth-map-review bash -c 'export PYTHONPATH=/ws/src/aac_2026; source /opt/ros/jazzy/setup.bash; exec rviz2 -d /ws/src/aac_2026/planning/geonji/config/geonji_mpcc.rviz > /tmp/geonji-mpcc-rviz.log 2>&1'
fi
