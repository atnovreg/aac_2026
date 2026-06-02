#!/usr/bin/env bash
set -euo pipefail

SESSION_NAME="${1:-AAC_2026}"
GRACE_SECONDS="${GRACE_SECONDS:-6}"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "Sending Ctrl-C to tmux session ${SESSION_NAME} ..."
  while IFS= read -r pane_id; do
    tmux send-keys -t "${pane_id}" C-c
  done < <(tmux list-panes -s -t "${SESSION_NAME}" -F '#{pane_id}')

  sleep "${GRACE_SECONDS}"

  if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    echo "Closing tmux session ${SESSION_NAME} ..."
    tmux kill-session -t "${SESSION_NAME}" || true
  fi
else
  echo "tmux session ${SESSION_NAME} is not running."
fi

echo "Stopping leftover AAC Gazebo/ROS processes ..."
pkill -TERM -u "$(id -u)" -f 'ros2 launch aac_vehicle spawn_vehicle.launch.py' || true
pkill -TERM -u "$(id -u)" -f 'ros2 launch joy_control_node joy_control.launch.py' || true
pkill -TERM -u "$(id -u)" -f 'ros_gz_sim|ros_gz_bridge|parameter_bridge|robot_state_publisher|vehicle_state_publisher_exe' || true
pkill -TERM -u "$(id -u)" -f 'ign gazebo|gz sim|ignition gazebo|gzserver|gzclient' || true

sleep 2

pkill -KILL -u "$(id -u)" -f 'ign gazebo|gz sim|ignition gazebo|gzserver|gzclient' || true

echo "Done."
