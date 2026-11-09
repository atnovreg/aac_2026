#!/usr/bin/env bash
# Replay saved acados poses. This does not run a Gazebo vehicle controller.
set -euo pipefail
replay_workspace="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
if [[ ! -f "$replay_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$replay_workspace"
export F1TENTH_WS="$replay_workspace"
if [[ "${1:-}" == build ]]; then
    exec docker compose --project-directory "$replay_workspace" -f "$replay_workspace/src/aac_2026/docker/f1tenth/compose.yaml" build gazebo
fi
replay_mounts=()
if [[ -n "${XAUTHORITY:-}" && -f "$XAUTHORITY" ]]; then
    replay_mounts+=(--volume "$XAUTHORITY:/tmp/gazebo.Xauthority:ro")
fi
exec docker compose --project-directory "$replay_workspace" -f "$replay_workspace/src/aac_2026/docker/f1tenth/compose.yaml" run --rm -T "${replay_mounts[@]}" gazebo bash src/aac_2026/system/f1tenth_gazebo_replay/scripts/gazebo_replay_container.sh "$@"
