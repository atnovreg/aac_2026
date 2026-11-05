#!/usr/bin/env bash
set -euo pipefail
replay_gui=true
if [[ "${1:-}" == --headless ]]; then
    replay_gui=false
    shift
fi
replay_build="${F1TENTH_REPLAY_BUILD:-$PWD/artifacts/f1tenth/jazzy-$(uname -m)/gazebo_replay}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$replay_build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
cmake -S tools/gazebo_replay -B "$replay_build" -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
cmake --build "$replay_build" -j4 >/dev/null
ctest --test-dir "$replay_build" --output-on-failure
replay_world=$(python3 -m tools.gazebo_replay.generate "$@")
echo "Gazebo trajectory replay: $replay_world"
echo 'Recorded acados poses; this is visualization, not a Gazebo physics/controller test.'
if [[ "$replay_gui" == true ]]; then
    gz sim -r -v 2 "$replay_world" &
    replay_pid=$!
    python3 -m tools.gazebo_replay.camera "$replay_world" &
    camera_pid=$!
    trap 'kill "$replay_pid" "$camera_pid" 2>/dev/null || true' EXIT INT TERM
    wait "$replay_pid"
else
    exec gz sim -s -r -v 3 "$replay_world"
fi
