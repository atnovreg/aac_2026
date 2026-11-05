#!/usr/bin/env bash
# Container entrypoint for start_front180.sh.
set -eo pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
workspace="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
while [[ ! -f "$workspace/.f1tenth-workspace" && "$workspace" != / ]]; do
  workspace="$(dirname -- "$workspace")"
done
[[ -f "$workspace/.f1tenth-workspace" ]] || { echo 'AAC workspace marker missing' >&2; exit 1; }
artifacts="$workspace/artifacts/f1tenth/${ROS_DISTRO}-$(uname -m)"
install_base="${F1TENTH_INSTALL_BASE:-$artifacts/install}"
source "$install_base/setup.bash"
set -u
filter_source="$workspace/src/aac_2026/sensors/livox_front_filter"
export F1TENTH_FRONT_BUILD="${F1TENTH_FRONT_BUILD:-$artifacts/front_filter}"
build_log=/tmp/front180_build.log
if ! { cmake -S "$filter_source" \
       -B "$F1TENTH_FRONT_BUILD" -DCMAKE_BUILD_TYPE=Release &&
       cmake --build "$F1TENTH_FRONT_BUILD" -j2; } > "$build_log" 2>&1; then
  cat "$build_log"
  exit 1
fi

# Shut down only the previous instances of this Livox display stack.
filter_pattern=$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$filter_source")
build_pattern=$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$F1TENTH_FRONT_BUILD")
install_pattern=$(python3 -c 'import re,sys; print(re.escape(sys.argv[1]))' "$install_base")
launch_pattern="^/usr/bin/python3 /opt/ros/${ROS_DISTRO}/bin/ros2 launch (${filter_pattern}/front180.launch.py)( |$)"
node_pattern="^(${build_pattern}/livox_front_filter|${install_pattern}/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node)( |$)"
pkill -INT -f "$launch_pattern" || true
pkill -INT -f "$node_pattern" || true
for ((attempt=0; attempt<50; attempt++)); do
  if ! pgrep -f "$launch_pattern|$node_pattern" >/dev/null; then break; fi
  sleep 0.2
done
if pgrep -f "$launch_pattern|$node_pattern" >/dev/null; then
  echo 'Previous Livox processes have not exited; close them and retry.' >&2
  exit 1
fi
exec ros2 launch "$filter_source/front180.launch.py"
