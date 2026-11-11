#!/usr/bin/env bash
# PC PS5 preview only. Directory events must remain visible across USB hotplug.
set -euo pipefail
script_path="$(readlink -f -- "${BASH_SOURCE[0]}")"
repo_root="$(cd -- "$(dirname -- "$script_path")/../.." && pwd)"
workspace="${F1TENTH_PC_WS:-$HOME/f1tenth_manual_pc_ws}"
input_group="$(getent group input | cut -d: -f3)"
exec docker run --rm --init --name f1tenth-manual-pc-preview --network none \
  --user "$(id -u):$(id -g)" --group-add "$input_group" \
  --device-cgroup-rule 'c 13:* r' \
  --mount type=bind,src=/dev/input,dst=/dev/input,readonly \
  --mount "type=bind,src=$workspace,dst=/ws" \
  --mount "type=bind,src=$repo_root,dst=/ws/src/aac_2026,readonly" \
  --entrypoint bash --workdir /ws \
  -e F1TENTH_WS=/ws \
  -e F1TENTH_GAMEPAD_PROFILE=/ws/src/aac_2026/system/f1tenth_manual/config/manual.pc.yaml \
  -e SDL_JOYSTICK_DISABLE_UDEV=1 \
  -e ROS_HOME=/tmp/manual-ros -e ROS_LOG_DIR=/tmp/manual-ros/log \
  -e ROS_LOCALHOST_ONLY=1 -e ROS_DOMAIN_ID=42 \
  aac-f1tenth-manual:humble-pc \
  -lc 'exec bash /ws/src/aac_2026/operations/f1tenth/manual.sh dry-run'
