#!/usr/bin/env bash
set -eo pipefail

live="${AAC_INDOOR_DIR:-$HOME/f1tenth_ws/live_indoor}"
tmux="${TMUX_BIN:-$HOME/.local/bin/tmux}"
self="$(readlink -f -- "$0")"
session=aac-drive-check
manual="$HOME/f1tenth_ws"

case "${1:-start}" in
  manual|shadow|record|manual-record)
    source /opt/ros/humble/setup.bash
    case "$1" in
      manual|manual-record)
        source "$manual/install/manual/setup.bash"
        export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
        ;;
      *)
        source "$live/msg-install/local_setup.bash"
        export ROS_DOMAIN_ID=43 ROS_LOCALHOST_ONLY=0
        export FASTRTPS_DEFAULT_PROFILES_FILE="$live/fastdds-peer.xml"
        ;;
    esac
    if [[ "$1" == manual ]]; then
      exec ros2 launch f1tenth_manual manual.launch.py stage:=drive steering:=true \
        config_file:="$manual/src/aac_2026/system/f1tenth_manual/config/manual.car.yaml"
    fi
    run="$2"
    exec > >(tee -a "$run/$1.log") 2>&1
    case "$1" in
      shadow)
        export ACADOS_SOURCE_DIR="$live/acados"
        export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
        export PYTHONPATH="$live/runtime${PYTHONPATH:+:$PYTHONPATH}:/usr/lib/python3/dist-packages"
        export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
        cd "$live/runtime"
        exec "$live/acados-venv/bin/python" -m control.acados_mpcc.ros_shadow \
          --ros-args -p use_sim_time:=false -p track_csv:="$live/track/track.csv" \
          -p log_csv:="$run/mpcc.csv"
        ;;
      record)
        exec ros2 bag record -o "$run/sensors" /livox/lidar /livox/imu /tf_static \
          /localization/odom /mpcc/state
        ;;
      manual-record)
        exec ros2 bag record -o "$run/ps5" /manual/joy /manual/ackermann_drive \
          /manual/motor_speed /manual/servo_position /vesc/sensors/core
        ;;
    esac
    ;;
esac

case "${1:-start}" in
  start)
    if "$tmux" has-session -t "=$session" 2>/dev/null; then
      "$self" status
      exit
    fi
    for path in start-lidar.sh start-localization.sh acados-venv/bin/python; do
      [[ -x "$live/$path" ]] || { echo "Missing $live/$path" >&2; exit 1; }
    done
    if ! pgrep -f '[/]opt/ros/humble/lib/joy/joy_node .*__node:=manual_joy' >/dev/null; then
      if pgrep -x vesc_driver_nod >/dev/null; then
        echo "An existing VESC driver is active. Check the PS5 session before starting." >&2
        exit 1
      fi
      "$tmux" new-session -d -s aac-manual "$self" manual
    fi
    for component in lidar localization; do
      target="aac-$component"
      if ! "$tmux" has-session -t "=$target" 2>/dev/null; then
        "$tmux" new-session -d -s "$target" "$live/start-$component.sh"
      elif [[ "$("$tmux" display-message -p -t "$target" '#{pane_dead}')" == 1 ]]; then
        "$tmux" respawn-pane -t "$target" "$live/start-$component.sh"
      fi
    done
    mkdir -p "$live/runs"
    run="$(mktemp -d "$live/runs/$(date +%Y%m%d_%H%M%S)_XXXXXX")"
    "$tmux" new-session -d -s "$session" -n mpcc -e "AAC_RUN=$run" "$self" shadow "$run"
    "$tmux" new-window -d -t "$session" -n sensors "$self" record "$run"
    "$tmux" new-window -d -t "$session" -n ps5 "$self" manual-record "$run"
    echo "Run: $run"
    for ((i=0; i<120; i++)); do
      if grep -q 'Shadow solver ready' "$run/shadow.log" 2>/dev/null &&
         grep -q 'All requested topics are subscribed' "$run/record.log" 2>/dev/null &&
         grep -q 'All requested topics are subscribed' "$run/manual-record.log" 2>/dev/null &&
         [[ -n "$(sed -n '2{p;q;}' "$run/mpcc.csv")" ]]; then
        echo "Ready: sensors, PS5 recorder and MPCC. Stop: $self stop"
        exit
      fi
      if [[ "$("$tmux" list-windows -t "$session" -F '#{window_name}' 2>/dev/null | wc -l)" != 3 ]]; then
        break
      fi
      sleep .5
    done
    echo "Startup not confirmed. See $run/*.log and run $self stop before retrying." >&2
    exit 1
    ;;
  status)
    "$tmux" list-sessions
    if "$tmux" has-session -t "=$session" 2>/dev/null; then
      "$tmux" show-environment -t "$session" AAC_RUN
      "$tmux" list-windows -t "$session"
    fi
    ;;
  trim)
    [[ -n "${2:-}" ]] || { echo "Usage: $0 trim RUN_DIRECTORY" >&2; exit 2; }
    source /opt/ros/humble/setup.bash
    source "$manual/install/manual/setup.bash"
    exec python3 "$live/trim_indoor_run.py" "$2" \
        "$manual/src/aac_2026/system/f1tenth_manual/config/manual.car.yaml"
    ;;
  stop)
    if "$tmux" has-session -t "=$session" 2>/dev/null; then
      run="$("$tmux" show-environment -t "$session" AAC_RUN)"
      run="${run#AAC_RUN=}"
      for window in sensors ps5 mpcc; do
        "$tmux" send-keys -t "$session:$window" C-c 2>/dev/null || true
      done
      for ((i=0; i<60; i++)); do
        if ! "$tmux" has-session -t "=$session" 2>/dev/null; then
          if [[ -f "$run/sensors/metadata.yaml" && -f "$run/ps5/metadata.yaml" ]]; then
            echo "Saved: $run"
            "$self" trim "$run"
            exit
          fi
          break
        fi
        sleep .25
      done
      echo "Check recording finalization in $run/*.log" >&2
      exit 1
    fi
    echo "No drive check is recording."
    ;;
  *)
    echo "Usage: $0 [start|stop|status|trim RUN_DIRECTORY]" >&2
    exit 2
    ;;
esac
