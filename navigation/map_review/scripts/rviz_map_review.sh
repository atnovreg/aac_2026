#!/usr/bin/env bash
set -euo pipefail
review_workspace="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
if [[ ! -f "$review_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
export PYTHONPATH="$review_workspace/src/aac_2026${PYTHONPATH:+:$PYTHONPATH}"
cd "$review_workspace"
review_container=f1tenth-map-review
if [[ "${1:-}" == stop ]]; then
    docker stop "$review_container"
    exit
fi
if docker container inspect "$review_container" >/dev/null 2>&1; then
    if [[ "$(docker inspect -f '{{.State.Running}}' "$review_container")" == true ]]; then
        echo 'RViz review is already running.'
        exit
    fi
    docker rm "$review_container" >/dev/null
fi
review_data='results/건지광장/rviz_review/083146'
if [[ ! -f "$review_data/clouds.npz" ]]; then
    .venv-dynamic-lio/bin/python -m navigation.map_review.map_review_cloud prepare \
      --run results/dynamic_lio/20260913T100858860234Z_indoor_front180_20260913_083146 \
      --metrics results/dynamic_lio/20260914_map_review/indoor_front180_20260913_083146/metrics.json \
      --output "$review_data"
fi
review_auth=${XAUTHORITY:-}
if [[ -z "$review_auth" || ! -f "$review_auth" ]]; then
    echo 'XAUTHORITY must point to the active desktop X11 authorization file.' >&2
    exit 1
fi
docker run -d --name "$review_container" --user "$(id -u):$(id -g)" \
 --workdir /ws --shm-size 1g \
 -e PYTHONPATH=/ws/src/aac_2026 -e HOME=/tmp/review-home -e DISPLAY="${DISPLAY:-:0}" \
 -e XAUTHORITY=/tmp/review.Xauthority -e QT_X11_NO_MITSHM=1 \
 -e QT_QPA_PLATFORM=xcb -e LIBGL_ALWAYS_SOFTWARE=1 \
 -e ROS_DOMAIN_ID=77 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
 -v "$review_workspace:/ws:ro" \
 -v "$review_workspace/bag:/ws/bag:ro" \
 -v "$review_workspace/tracks:/ws/tracks:ro" \
 -v "$review_workspace/results:/ws/results:ro" \
 -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
 -v "$review_auth:/tmp/review.Xauthority:ro" \
 aac-f1tenth-fastlio:jazzy bash -c '
   mkdir -p "$HOME"
   python3 -m navigation.map_review.map_review_cloud serve "/ws/results/건지광장/rviz_review/083146/clouds.npz" &
   review_publisher=$!
   trap "kill $review_publisher 2>/dev/null || true" EXIT
   rviz2 -d /ws/src/aac_2026/navigation/map_review/config/map_review.rviz
 '
