#!/usr/bin/env bash
# One environment and exec boundary for each live tmux pane.
set -eo pipefail
config_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export AAC_REPO="$(cd -- "$config_dir/../.." && pwd)"
export AAC_SETTINGS="${AAC_SETTINGS:-$config_dir/jetson.env}"
[[ -f "$AAC_SETTINGS" ]] || { echo "Missing settings: $AAC_SETTINGS" >&2; exit 1; }
set -a
source "$AAC_SETTINGS"
# A custom settings file can override only the values it needs.
source "$config_dir/jetson.env"
set +a
export AAC_RUN_MODE="${AAC_RUN_MODE:-autonomy}"
case "$AAC_RUN_MODE" in
  prepare|autonomy) ;;
  *) echo 'Use bash start.sh [autonomy|prepare], or bash start.sh check prepare.' >&2; exit 2 ;;
esac

builtin_autonomy=false
if [[ "$AAC_RUN_MODE" == autonomy && -n "$AAC_AUTONOMY_PROFILE" ]]; then
  builtin_autonomy=true
fi

check_config() {
  local errors=0 path
  if $builtin_autonomy; then
    if [[ -n "$AAC_CONTROL_START" || -n "$AAC_VEHICLE_START" ]]; then
      echo 'Choose AAC_AUTONOMY_PROFILE or both custom startup scripts.' >&2
      errors=1
    fi
    if [[ "$AAC_AUTONOMY_HARDWARE" != 0 && "$AAC_AUTONOMY_HARDWARE" != 1 ]]; then
      echo 'AAC_AUTONOMY_HARDWARE must be 0 or 1.' >&2
      errors=1
    fi
    if ! drive_speed_limit="$(cd "$AAC_REPO" && python3 - "$AAC_AUTONOMY_PROFILE" "$AAC_MANUAL_CONFIG" "$AAC_AUTONOMY_HARDWARE" <<'PY'
from pathlib import Path
import os
import sys

import yaml
from system.f1tenth.ros_drive_selector import load_config

profile, manual, hardware = sys.argv[1:]
try:
    limits, _ = load_config(profile, manual)
    if hardware == '1':
        serial = Path(yaml.safe_load(Path(manual).read_text())['/vesc/vesc_driver']['ros__parameters']['port'])
        if not serial.is_char_device() or not os.access(serial, os.R_OK | os.W_OK):
            raise ValueError(f'Cannot access VESC serial port {serial}')
    print(float(limits.auto_speed_mps))
except (KeyError, OSError, ValueError, TypeError, yaml.YAMLError) as error:
    sys.exit(f'Indoor autonomy configuration: {error}')
PY
    )"; then errors=1; fi
  elif [[ "$AAC_RUN_MODE" == autonomy ]]; then
    for path in AAC_CONTROL_START AAC_VEHICLE_START; do
      if [[ -z "${!path}" || ! -x "${!path}" || ! -f "${!path}" ]]; then
        echo "$path: set an executable real-vehicle startup script in $AAC_SETTINGS" >&2
        errors=1
      fi
    done
  fi
  # Normalize to DOUBLE_ARRAY, also rejecting booleans, NaN and infinity.
  if [[ "$AAC_INITIAL_BASE_POSE" != auto ]] && ! AAC_INITIAL_BASE_POSE="$(python3 - <<'PY'
import json, math, os, sys
try:
    pose = json.loads(os.environ.get('AAC_INITIAL_BASE_POSE', ''))
    if (not isinstance(pose, list) or len(pose) != 6
            or any(type(v) not in (int, float) or not math.isfinite(v) for v in pose)):
        raise ValueError()
    print(json.dumps([float(v) for v in pose], separators=(',', ':')))
except (ValueError, TypeError, OverflowError):
    sys.exit('AAC_INITIAL_BASE_POSE: set auto or six finite numbers [x,y,z,roll,pitch,yaw]')
PY
  )"; then errors=1; fi
  export AAC_INITIAL_BASE_POSE
  for path in "$AAC_ROS_SETUP" "$AAC_LIDAR_SETUP" "$AAC_LOCALIZATION_SETUP" "$AAC_MSG_SETUP" \
      "$AAC_MAP_DIR/map.pcd" "$AAC_MAP_DIR/map_evidence.npz" "$AAC_TRACK_CSV" \
      "$(dirname -- "$AAC_TRACK_CSV")/validation.json" "$(dirname -- "$AAC_TRACK_CSV")/soft_profile.npz" \
      "$AAC_DDS_PROFILE"; do
    [[ -f "$path" ]] || { echo "Missing file: $path" >&2; errors=1; }
  done
  if [[ "$AAC_INITIAL_BASE_POSE" == auto && ! -f "$AAC_MAP_DIR/occupancy.yaml" ]]; then
    echo "Missing file: $AAC_MAP_DIR/occupancy.yaml (required for automatic localization)" >&2
    errors=1
  fi
  if [[ "$AAC_RUN_MODE" == prepare ]] || $builtin_autonomy; then
    for path in "$AAC_MANUAL_SETUP" "$AAC_MANUAL_CONFIG"; do
      [[ -f "$path" ]] || { echo "Missing file: $path" >&2; errors=1; }
    done
    [[ -x "$AAC_MPCC_PYTHON" ]] || { echo "Missing MPCC Python: $AAC_MPCC_PYTHON" >&2; errors=1; }
    [[ -d "$AAC_ACADOS_DIR/lib" ]] || { echo "Missing acados libraries: $AAC_ACADOS_DIR/lib" >&2; errors=1; }
  fi
  (( errors == 0 ))
}

action="${1:-check}"
check_config || exit 1
if [[ "$action" == check ]]; then
  for path in tmux tmuxp; do
    command -v "$path" >/dev/null || { echo "Missing command: $path" >&2; exit 1; }
  done
  # Existing standalone sessions must be shut down by their own owner first.
  if pgrep -u "$(id -u)" -af '[l]ivox_ros_driver2_node|[f]astlio_mapping|[v]esc_driver_node|[v]esc_driver_nod|[j]oy_control_node_exe|[p]ython.*-m navigation.localization3d.ros_localizer|[p]ython.*-m control.acados_mpcc.ros_shadow|[p]ython.*-m system.f1tenth.ros_drive_selector'; then
    echo 'Existing vehicle/sensor/localization processes found. Reuse or stop their original session before starting AAC_2026.' >&2
    exit 1
  fi
  echo "Configuration checked: $AAC_RUN_MODE. Sensor readiness and vehicle operation are not verified."
  exit
fi

: "${AAC_RUN_DIR:?Use bash start.sh so each recording has its own directory}"
[[ -d "$AAC_RUN_DIR" ]] || { echo "Missing run directory: $AAC_RUN_DIR" >&2; exit 1; }
exec > >(tee -a "$AAC_RUN_DIR/$action.log") 2>&1
source "$AAC_ROS_SETUP"
export ROS_DOMAIN_ID="$AAC_SENSOR_DOMAIN" ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE="$AAC_DDS_PROFILE"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONUNBUFFERED=1
cd "$AAC_REPO"

if [[ "$AAC_RUN_MODE" == prepare && ( "$action" == vehicle || "$action" == record-vehicle ) ]]; then
  source "$AAC_MANUAL_SETUP"
  export ROS_DOMAIN_ID="$AAC_MANUAL_DOMAIN" ROS_LOCALHOST_ONLY=1
  unset FASTRTPS_DEFAULT_PROFILES_FILE
else
  if $builtin_autonomy; then source "$AAC_MANUAL_SETUP"; fi
  source "$AAC_LIDAR_SETUP"
  source "$AAC_LOCALIZATION_SETUP"
  source "$AAC_MSG_SETUP"
fi

case "$action" in
  lidar)
    exec ros2 launch "$AAC_REPO/system/f1tenth/config/agx/mid360.launch.py"
    ;;
  localization)
    exec ros2 launch "$AAC_REPO/navigation/localization3d/localization_mpcc.launch.py" \
      "map_dir:=$AAC_MAP_DIR" "track_csv:=$AAC_TRACK_CSV" \
      "initial_base_pose:=$AAC_INITIAL_BASE_POSE" use_sim_time:=false
    ;;
  controller)
    if [[ "$AAC_RUN_MODE" == autonomy ]] && ! $builtin_autonomy; then exec "$AAC_CONTROL_START"; fi
    export ACADOS_SOURCE_DIR="$AAC_ACADOS_DIR"
    export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    export PYTHONPATH="$AAC_REPO${PYTHONPATH:+:$PYTHONPATH}:/usr/lib/python3/dist-packages"
    drive_args=()
    if $builtin_autonomy; then
      drive_args=(-p "drive_speed_limit:=$drive_speed_limit")
    else
      echo 'PREPARE: MPCC computes and logs only; PS5 controls the car.'
    fi
    exec "$AAC_MPCC_PYTHON" -m control.acados_mpcc.ros_shadow --ros-args \
      -p use_sim_time:=false -p "track_csv:=$AAC_TRACK_CSV" -p "log_csv:=$AAC_RUN_DIR/mpcc.csv" "${drive_args[@]}"
    ;;
  vehicle)
    if $builtin_autonomy; then
      hardware=false
      if [[ "$AAC_AUTONOMY_HARDWARE" == 1 ]]; then hardware=true; fi
      exec ros2 launch "$AAC_REPO/system/f1tenth/config/agx/indoor_vehicle.launch.py" \
        "manual_config:=$AAC_MANUAL_CONFIG" "calibration_file:=$AAC_AUTONOMY_PROFILE" "hardware:=$hardware"
    fi
    if [[ "$AAC_RUN_MODE" == autonomy ]]; then exec "$AAC_VEHICLE_START"; fi
    echo 'PREPARE: existing manual-only PS5/VESC launch, disarmed on startup.'
    exec ros2 launch f1tenth_manual manual.launch.py stage:=drive steering:=true "config_file:=$AAC_MANUAL_CONFIG"
    ;;
  record-sensors)
    exec ros2 bag record -o "$AAC_RUN_DIR/sensors" /livox/lidar /livox/imu /tf /tf_static /localization/odom /mpcc/state
    ;;
  record-vehicle)
    if $builtin_autonomy; then
      output=/drive/preview
      if [[ "$AAC_AUTONOMY_HARDWARE" == 1 ]]; then output=/vesc/commands; fi
      exec ros2 bag record -o "$AAC_RUN_DIR/ps5" /manual/joy /manual/ackermann_drive \
        /mpcc/drive_request /drive/mode "$output/motor/speed" "$output/servo/position" /vesc/sensors/core
    fi
    exec ros2 bag record -o "$AAC_RUN_DIR/ps5" /manual/joy /manual/ackermann_drive \
      /manual/motor_speed /manual/servo_position /vesc/sensors/core /ackermann_auto /ackermann_drive
    ;;
  monitor)
    echo "Mode: $AAC_RUN_MODE | Run: $AAC_RUN_DIR | Stop: bash $AAC_REPO/stop.sh"
    exec ros2 topic hz /mpcc/state
    ;;
  *) echo "Unknown pane: $action" >&2; exit 2 ;;
esac
