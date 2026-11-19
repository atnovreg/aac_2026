#!/usr/bin/env bash
set -euo pipefail
slam_workspace="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
if [[ ! -f "$slam_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$slam_workspace"
export F1TENTH_WS="$slam_workspace"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
case "${1:-help}" in
  build)
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" build fastlio
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm -T -e CMAKE_BUILD_PARALLEL_LEVEL=2 fastlio bash /ws/src/aac_2026/operations/f1tenth/scripts/build_mapping.sh
    ;;
  run)
    shift
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm -T fastlio python3 /ws/src/aac_2026/navigation/fastlio/scripts/run_bag.py "$@"
    ;;
  shell)
    mounts=()
    if [[ "${2:-}" == --acados ]]; then
      mounts=(--volume "$(realpath -e "$slam_workspace/.venv-acados"):/opt/aac-venv:ro"
              --volume "$(realpath -e "$slam_workspace/vendor/acados"):/opt/aac-acados:ro"
              --env ACADOS_VENV=/opt/aac-venv --env ACADOS_SOURCE_DIR=/opt/aac-acados)
    fi
    docker compose --project-directory "$slam_workspace" -f "$slam_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm "${mounts[@]}" fastlio bash
    ;;
  *)
    echo 'Usage: src/aac_2026/navigation/fastlio/scripts/slam.sh build | run BAG_NAME [--rate 1.0] | run all | shell [--acados]'
    ;;
esac
