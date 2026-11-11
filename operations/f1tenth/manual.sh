#!/usr/bin/env bash
# Run on the Jetson. No service is installed and no hardware stage starts by default.
set -e
action="${1:-dry-run}"
case "$action" in build|dry-run|check|telemetry|drive) ;; *)
  echo 'Usage: manual.sh {build|dry-run|check|telemetry|drive --wheels-up}' >&2; exit 2;;
esac
script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
repo_root="$(cd -- "$(dirname -- "$script_path")/../.." && pwd)"
workspace="${F1TENTH_WS:-$(cd "$repo_root/../.." && pwd)}"
source /opt/ros/humble/setup.bash
deps="$workspace/.manual_ros_deps/opt/ros/humble"
if [[ -d "$deps" ]]; then
  export CMAKE_PREFIX_PATH="$deps:${CMAKE_PREFIX_PATH:-}"
fi
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42 SDL_VIDEODRIVER=offscreen
if [[ "$action" == build ]]; then
  vesc_source="$workspace/src/vesc"
  expected_revision=153998df8545fe1781b975df88e411b4e71d4bfe
  if [[ "$(git -C "$vesc_source" rev-parse HEAD)" != "$expected_revision" ]]; then
    echo "Expected VESC source revision $expected_revision; refusing an unverified build." >&2
    exit 1
  fi
  patch_file="$repo_root/system/f1tenth_manual/patches/0001-publish-primary-temperatures.patch"
  if ! git -C "$vesc_source" apply --reverse --check "$patch_file" 2>/dev/null; then
    git -C "$vesc_source" apply --check "$patch_file"
    git -C "$vesc_source" apply "$patch_file"
  fi
  cd "$workspace"
  export CMAKE_BUILD_PARALLEL_LEVEL=2
  exec colcon --log-base log/manual build --base-paths \
    "$repo_root/control/joy_control_node" "$repo_root/system/f1tenth_manual" \
    "$workspace/src/vesc" --build-base build/manual --install-base install/manual \
    --executor sequential --cmake-args -DBUILD_TESTING=OFF
fi
source "$workspace/install/manual/setup.bash"
if [[ "$action" == check || "$action" == drive ]]; then
  if [[ "$action" == drive && "${2:-}" != --wheels-up ]]; then
    echo 'Bench drive only: raise the driven wheels, watch the car, then use drive --wheels-up.' >&2
    exit 2
  fi
  # A bounded read-only process group is stopped even if the check fails.
  python3 - <<'PY'
import os, signal, subprocess, sys
from ament_index_python.packages import get_package_prefix
child = subprocess.Popen(['ros2', 'launch', 'f1tenth_manual', 'manual.launch.py',
                          'stage:=telemetry'], start_new_session=True)
try:
    checker = os.path.join(get_package_prefix('f1tenth_manual'), 'lib',
                           'f1tenth_manual', 'check_controller.py')
    result = subprocess.run([checker], timeout=12).returncode
finally:
    os.killpg(child.pid, signal.SIGINT)
    try:
        child.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGKILL)
        child.wait()
sys.exit(result)
PY
  [[ "$action" == check ]] && exit 0
fi
echo "Manual stage: $action | Circle arm, R1 hold, Cross stop | steering preview only"
launch_args=("stage:=$action")
if [[ -n "${F1TENTH_GAMEPAD_PROFILE:-}" ]]; then
  launch_args+=("gamepad_profile:=$F1TENTH_GAMEPAD_PROFILE")
fi
exec ros2 launch f1tenth_manual manual.launch.py "${launch_args[@]}"
