#!/usr/bin/env bash
# SIGINT lets ROS launch shut down children and rosbag finalize metadata.
set -euo pipefail
session="${1:-AAC_2026}"
target="=$session"
wait_seconds="${STOP_WAIT_SECONDS:-15}"
[[ "$wait_seconds" =~ ^[0-9]+$ ]] || { echo 'STOP_WAIT_SECONDS must be a nonnegative integer.' >&2; exit 2; }
if ! tmux has-session -t "$target" 2>/dev/null; then
  echo "tmux session $session is not running."
  exit
fi
echo "Stopping $session; waiting for nodes and recordings to finish ..."
while read -r pane dead; do
  if [[ "$dead" == 0 ]]; then tmux send-keys -t "$pane" C-c || true; fi
done < <(tmux list-panes -s -t "$target" -F '#{pane_id} #{pane_dead}')
deadline=$((SECONDS + wait_seconds))
while tmux has-session -t "$target" 2>/dev/null; do
  active="$(tmux list-panes -s -t "$target" -F '#{pane_dead}' 2>/dev/null || true)"
  if [[ -n "$active" && "$active" != *0* ]]; then
    tmux kill-session -t "$target"
    echo "Stopped $session."
    exit
  fi
  if (( SECONDS >= deadline )); then
    echo "Some panes are still running. Session $session was kept; inspect it with tmux attach -t $session, then retry bash stop.sh $session." >&2
    exit 1
  fi
  sleep .25
done
echo "Stopped $session."
