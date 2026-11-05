#!/usr/bin/env bash
set -euo pipefail
slam_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$slam_workspace/.f1tenth-workspace" && "$slam_workspace" != / ]]; do
    slam_workspace=$(dirname -- "$slam_workspace")
done
if [[ ! -f "$slam_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$slam_workspace"
export F1TENTH_WS="$slam_workspace"
export F1TENTH_ASSETS="${F1TENTH_ASSETS:-$(dirname -- "$(readlink -f -- "$slam_workspace/bags")")}"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
case "${1:-help}" in
  build)
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" build fastlio
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm -T -e CMAKE_BUILD_PARALLEL_LEVEL=2 fastlio bash /ws/scripts/build.sh
    ;;
  run)
    shift
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm -T fastlio python3 /ws/scripts/run_bag.py "$@"
    ;;
  shell)
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm fastlio bash
    ;;
  *)
    echo 'Usage: scripts/slam.sh build | run BAG_NAME [--rate 1.0] | run all | shell'
    ;;
esac
