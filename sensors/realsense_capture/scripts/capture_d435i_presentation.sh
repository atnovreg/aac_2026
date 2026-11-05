#!/usr/bin/env bash

set -Eeuo pipefail

capture_workspace="$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")"
while [[ ! -f "$capture_workspace/.f1tenth-workspace" && "$capture_workspace" != / ]]; do
  capture_workspace="$(dirname -- "$capture_workspace")"
done
[[ -f "$capture_workspace/.f1tenth-workspace" ]] || { echo 'AAC workspace marker missing' >&2; exit 1; }
WORKSPACE="${WORKSPACE:-$capture_workspace}"
ROS_SETUP_FILE="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
RSUSB_SETUP_FILE="${RSUSB_SETUP_FILE:-${WORKSPACE}/artifacts/f1tenth/jazzy-$(uname -m)/install/setup.bash}"
RSUSB_PREFIX="${RSUSB_PREFIX:-${WORKSPACE}/vendor/rsusb-install}"
CAPTURE_ROOT="${CAPTURE_ROOT:-${WORKSPACE}/results/camera/captures}"
WAIT_INTERVAL="${WAIT_INTERVAL:-0.5}"

CAMERA_NODE=/camera/camera
COLOR_TOPIC=/camera/camera/color/image_raw
DEPTH_TOPIC=/camera/camera/depth/image_rect_raw

CAMERA_PID=""
RGB_SAVER_PID=""
DEPTH_SAVER_PID=""
ORIGINAL_COLORIZER=""
COLORIZER_CHANGED=0
OUTPUT_DIR=""
LOG_DIR=""
FOUND_FILE=""
CAPTURE_SUCCEEDED=0

log() {
  printf '[D435i 캡처] %s\n' "$*"
}

fail() {
  printf '[D435i 캡처] 오류: %s\n' "$*" >&2
  exit 1
}

stop_owned_process() {
  local pid="${1:-}"
  local attempt

  [[ -n "${pid}" ]] || return 0
  kill -0 "${pid}" 2>/dev/null || {
    wait "${pid}" 2>/dev/null || true
    return 0
  }

  kill -INT "${pid}" 2>/dev/null || true
  for ((attempt = 0; attempt < 5; attempt++)); do
    kill -0 "${pid}" 2>/dev/null || break
    sleep "${WAIT_INTERVAL}"
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -TERM "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e

  if [[ "${COLORIZER_CHANGED}" == 1 && -n "${ORIGINAL_COLORIZER}" ]]; then
    ros2 param set "${CAMERA_NODE}" colorizer.enable "${ORIGINAL_COLORIZER}" \
      >/dev/null 2>&1 || true
    COLORIZER_CHANGED=0
  fi

  stop_owned_process "${DEPTH_SAVER_PID}"
  stop_owned_process "${RGB_SAVER_PID}"
  stop_owned_process "${CAMERA_PID}"

  if [[ "${CAPTURE_SUCCEEDED}" == 1 && -n "${LOG_DIR}" && \
        "${LOG_DIR}" == /tmp/d435i_capture.* && -d "${LOG_DIR}" ]]; then
    rm -rf -- "${LOG_DIR}"
  elif [[ "${status}" -ne 0 && -n "${LOG_DIR}" ]]; then
    printf '[D435i 캡처] 진단 로그: %s\n' "${LOG_DIR}" >&2
  fi

  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

node_exists() {
  ros2 node list 2>/dev/null | grep -Fx "$1" >/dev/null
}

topic_exists() {
  ros2 topic list 2>/dev/null | grep -Fx "$1" >/dev/null
}

service_exists() {
  ros2 service list 2>/dev/null | grep -Fx "$1" >/dev/null
}

wait_for_node() {
  local node="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    node_exists "${node}" && return 0
    if [[ -n "${CAMERA_PID}" ]] && ! kill -0 "${CAMERA_PID}" 2>/dev/null; then
      fail "카메라 launch가 조기 종료되었습니다. 로그: ${LOG_DIR}/camera.log"
    fi
    sleep "${WAIT_INTERVAL}"
  done
  fail "${timeout_seconds}초 안에 ROS 노드가 나타나지 않았습니다: ${node}"
}

wait_for_topic() {
  local topic="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    topic_exists "${topic}" && return 0
    sleep "${WAIT_INTERVAL}"
  done
  fail "${timeout_seconds}초 안에 ROS 토픽이 나타나지 않았습니다: ${topic}"
}

wait_for_service() {
  local service="$1"
  local timeout_seconds="$2"
  local saver_pid="$3"
  local saver_log="$4"
  local deadline=$((SECONDS + timeout_seconds))

  while ((SECONDS < deadline)); do
    service_exists "${service}" && return 0
    if ! kill -0 "${saver_pid}" 2>/dev/null; then
      fail "image_saver가 조기 종료되었습니다. 로그: ${saver_log}"
    fi
    sleep "${WAIT_INTERVAL}"
  done
  fail "${timeout_seconds}초 안에 저장 서비스가 나타나지 않았습니다: ${service}"
}

wait_for_capture_file() {
  local prefix="$1"
  local timeout_seconds="$2"
  local deadline=$((SECONDS + timeout_seconds))

  FOUND_FILE=""
  while ((SECONDS < deadline)); do
    FOUND_FILE="$(find "${OUTPUT_DIR}" -mindepth 1 -maxdepth 1 -type f \
      -name "${prefix}_*.jpg" | sort | head -n 1)"
    if [[ -n "${FOUND_FILE}" && -s "${FOUND_FILE}" ]]; then
      return 0
    fi
    sleep "${WAIT_INTERVAL}"
  done
  fail "${timeout_seconds}초 안에 저장 파일이 생성되지 않았습니다: ${prefix}_*.jpg"
}

create_output_directory() {
  local timestamp
  local base
  local suffix=1

  mkdir -p "${CAPTURE_ROOT}"
  timestamp="$(date +%Y%m%d_%H%M%S)"
  base="${CAPTURE_ROOT}/${timestamp}"
  OUTPUT_DIR="${base}"
  while [[ -e "${OUTPUT_DIR}" ]]; do
    OUTPUT_DIR="${base}_${suffix}"
    ((suffix += 1))
  done
  mkdir -p "${OUTPUT_DIR}"
}

capture_rgb() {
  local saver_log="${LOG_DIR}/rgb_saver.log"

  log 'RGB 프레임을 저장합니다.'
  /opt/ros/jazzy/lib/image_view/image_saver --ros-args \
    -r __node:=rgb_saver \
    -r save:=/rgb_saver/save \
    -r image:="${COLOR_TOPIC}" \
    -p save_all_image:=false \
    -p filename_format:="${OUTPUT_DIR}/d435i_rgb_%04i.%s" \
    >"${saver_log}" 2>&1 &
  RGB_SAVER_PID=$!

  wait_for_service /rgb_saver/save 15 "${RGB_SAVER_PID}" "${saver_log}"
  ros2 service call /rgb_saver/save std_srvs/srv/Empty '{}' \
    >>"${saver_log}" 2>&1
  wait_for_capture_file d435i_rgb 10
  mv -- "${FOUND_FILE}" "${OUTPUT_DIR}/d435i_rgb.jpg"
  stop_owned_process "${RGB_SAVER_PID}"
  RGB_SAVER_PID=""
}

read_colorizer_state() {
  local output
  output="$(ros2 param get "${CAMERA_NODE}" colorizer.enable 2>&1)" || \
    fail "colorizer.enable 값을 읽지 못했습니다: ${output}"
  ORIGINAL_COLORIZER="$(printf '%s\n' "${output}" | awk '{print tolower($NF)}')"
  case "${ORIGINAL_COLORIZER}" in
    true|false) ;;
    *) fail "알 수 없는 colorizer.enable 값입니다: ${output}" ;;
  esac
}

enable_colorizer() {
  read_colorizer_state
  if [[ "${ORIGINAL_COLORIZER}" != true ]]; then
    ros2 param set "${CAMERA_NODE}" colorizer.enable true >/dev/null
    COLORIZER_CHANGED=1
  fi
  sleep "${WAIT_INTERVAL}"
}

restore_colorizer() {
  if [[ "${COLORIZER_CHANGED}" == 1 ]]; then
    ros2 param set "${CAMERA_NODE}" colorizer.enable "${ORIGINAL_COLORIZER}" \
      >/dev/null
    COLORIZER_CHANGED=0
  fi
}

capture_colorized_depth() {
  local saver_log="${LOG_DIR}/depth_saver.log"

  log '컬러 Depth 프레임을 저장합니다.'
  enable_colorizer
  /opt/ros/jazzy/lib/image_view/image_saver --ros-args \
    -r __node:=depth_saver \
    -r save:=/depth_saver/save \
    -r image:="${DEPTH_TOPIC}" \
    -p save_all_image:=false \
    -p filename_format:="${OUTPUT_DIR}/d435i_depth_colorized_%04i.%s" \
    >"${saver_log}" 2>&1 &
  DEPTH_SAVER_PID=$!

  wait_for_service /depth_saver/save 15 "${DEPTH_SAVER_PID}" "${saver_log}"
  ros2 service call /depth_saver/save std_srvs/srv/Empty '{}' \
    >>"${saver_log}" 2>&1
  wait_for_capture_file d435i_depth_colorized 10
  mv -- "${FOUND_FILE}" "${OUTPUT_DIR}/d435i_depth_colorized.jpg"
  stop_owned_process "${DEPTH_SAVER_PID}"
  DEPTH_SAVER_PID=""
  restore_colorizer
}

verify_jpeg() {
  local image="$1"
  [[ -s "${image}" ]] || fail "이미지가 없거나 비어 있습니다: ${image}"
  file "${image}" | grep -qi 'JPEG image data' || \
    fail "JPEG 파일로 확인되지 않았습니다: ${image}"
}

main() {
  local host_output_dir

  [[ -r "${ROS_SETUP_FILE}" ]] || fail "ROS setup 파일이 없습니다: ${ROS_SETUP_FILE}"
  [[ -r "${RSUSB_SETUP_FILE}" ]] || fail "RSUSB overlay setup 파일이 없습니다: ${RSUSB_SETUP_FILE}"
  [[ -d "${RSUSB_PREFIX}" ]] || fail "RSUSB 설치 경로가 없습니다: ${RSUSB_PREFIX}"

  # shellcheck disable=SC1090
  set +u
  source "${ROS_SETUP_FILE}"
  # shellcheck disable=SC1090
  source "${RSUSB_SETUP_FILE}"
  set -u
  export PATH="${RSUSB_PREFIX}/bin:${PATH}"
  export LD_LIBRARY_PATH="${RSUSB_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

  command -v ros2 >/dev/null 2>&1 || fail 'ros2 명령을 찾을 수 없습니다.'
  command -v file >/dev/null 2>&1 || fail 'file 명령을 찾을 수 없습니다.'
  if ! ros2 pkg prefix image_view >/dev/null 2>&1; then
    printf '%s\n' '[D435i 캡처] image_view가 없습니다. 컨테이너에서 실행:' >&2
    printf '%s\n' 'apt update' >&2
    printf '%s\n' 'apt install -y ros-jazzy-image-view' >&2
    exit 1
  fi

  LOG_DIR="$(mktemp -d /tmp/d435i_capture.XXXXXX)"
  create_output_directory

  if node_exists "${CAMERA_NODE}"; then
    log '기존 RealSense 카메라 노드를 사용합니다.'
  else
    log 'RealSense 카메라 노드를 시작합니다.'
    ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true \
      enable_depth:=true \
      enable_gyro:=true \
      enable_accel:=true \
      >"${LOG_DIR}/camera.log" 2>&1 &
    CAMERA_PID=$!
  fi

  wait_for_node "${CAMERA_NODE}" 30
  wait_for_topic "${COLOR_TOPIC}" 30
  wait_for_topic "${DEPTH_TOPIC}" 30
  capture_rgb
  capture_colorized_depth

  verify_jpeg "${OUTPUT_DIR}/d435i_rgb.jpg"
  verify_jpeg "${OUTPUT_DIR}/d435i_depth_colorized.jpg"

  CAPTURE_SUCCEEDED=1
  host_output_dir="~/f1tenth_ws${OUTPUT_DIR#${WORKSPACE}}"
  log '저장이 완료되었습니다.'
  ls -lh \
    "${OUTPUT_DIR}/d435i_rgb.jpg" \
    "${OUTPUT_DIR}/d435i_depth_colorized.jpg"
  printf '[D435i 캡처] 컨테이너 경로: %s\n' "${OUTPUT_DIR}"
  printf '[D435i 캡처] Jetson 호스트 경로: %s\n' "${host_output_dir}"
}

main "$@"
