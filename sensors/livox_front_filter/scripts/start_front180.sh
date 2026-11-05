#!/usr/bin/env bash
# Run from the host desktop. Recording is started separately by the user.
set -euo pipefail
container="${AAC_CONTAINER:?Set AAC_CONTAINER to the explicitly prepared AAC Jetson container}"
container_workspace="${AAC_CONTAINER_WORKSPACE:-/ros_ws}"
if [[ $(docker inspect -f '{{.State.Running}}' "$container") != true ]]; then
  docker start "$container" >/dev/null
fi
export DISPLAY="${DISPLAY:-:1}"
xhost +si:localuser:root >/dev/null
docker_tty=(-i)
if [[ -t 0 && -t 1 ]]; then docker_tty+=(-t); fi
exec docker exec "${docker_tty[@]}" -e DISPLAY="$DISPLAY" "$container" \
  bash "$container_workspace/src/aac_2026/sensors/livox_front_filter/scripts/run_front180.sh"
