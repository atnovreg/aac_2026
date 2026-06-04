#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SESSION_NAME="AAC_2026_HARNESS"
CONFIG_FILE="${REPO_DIR}/operations/tmuxp_config/config.yaml"
LIVE_MODE="${1:-static}"

PROCESS_PATTERN='[r]os2 launch aac_vehicle spawn_vehicle.launch.py|[r]os2 launch joy_control_node joy_control.launch.py|[r]os_gz_sim|[r]os_gz_bridge|[p]arameter_bridge|[r]obot_state_publisher|[v]ehicle_state_publisher_exe|[i]gn gazebo|[g]z sim|[i]gnition gazebo|[g]zserver|[g]zclient'

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || {
    echo "Missing required command: $1" >&2
    exit 1
  }
}

source_ros() {
  set +u
  source /opt/ros/humble/setup.bash
  source /ros_ws/install/setup.bash
  set -u
}

assert_no_processes() {
  if pgrep -a -u "$(id -u)" -f "${PROCESS_PATTERN}" >/tmp/aac_harness_processes.txt; then
    echo "Unexpected AAC Gazebo/ROS processes are still running:" >&2
    cat /tmp/aac_harness_processes.txt >&2
    exit 1
  fi
}

run_static_checks() {
  require_cmd python3
  require_cmd tmux
  require_cmd tmuxp

  bash -n "${REPO_DIR}/start.sh" "${REPO_DIR}/stop.sh" "${REPO_DIR}/auto.sh" "${REPO_DIR}/kill_sys.sh"

  python3 - <<PY
import sys
import yaml

config_path = "${CONFIG_FILE}"
with open(config_path) as f:
    config = yaml.safe_load(f)

if config.get("session_name") != "AAC_2026":
    sys.exit("session_name must be AAC_2026")

panes = config["windows"][0]["panes"]
commands = [pane["shell_command"][0] for pane in panes]
missing_exec = [cmd for cmd in commands if not cmd.startswith("exec ")]
if missing_exec:
    sys.exit(f"pane commands should use exec: {missing_exec}")

print("tmuxp config OK")
PY

  source_ros
  require_cmd ros2
  ros2 launch aac_vehicle spawn_vehicle.launch.py --show-args >/dev/null

  echo "Static harness OK"
}

run_live_check() {
  require_cmd mktemp
  require_cmd tmux
  require_cmd tmuxp

  if tmux has-session -t AAC_2026 2>/dev/null; then
    echo "AAC_2026 is already running. Stop it before live harness." >&2
    exit 1
  fi

  if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}" || true
  fi

  assert_no_processes
  source_ros

  tmp_config="$(mktemp --suffix=.yaml)"
  sed "s/^session_name: AAC_2026$/session_name: ${SESSION_NAME}/" "${CONFIG_FILE}" > "${tmp_config}"
  trap 'rm -f "${tmp_config}"; bash "${REPO_DIR}/stop.sh" "${SESSION_NAME}" >/dev/null 2>&1 || true' EXIT

  tmuxp load -d "${tmp_config}"
  sleep "${HARNESS_STARTUP_SECONDS:-10}"

  tmux has-session -t "${SESSION_NAME}"
  bash "${REPO_DIR}/stop.sh" "${SESSION_NAME}"
  sleep 1

  if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo "${SESSION_NAME} tmux session is still running." >&2
    exit 1
  fi

  assert_no_processes
  echo "Live shutdown harness OK"
}

cd "${REPO_DIR}"

case "${LIVE_MODE}" in
  static)
    run_static_checks
    ;;
  --live|live)
    run_static_checks
    run_live_check
    ;;
  *)
    echo "Usage: $0 [static|--live]" >&2
    exit 2
    ;;
esac
