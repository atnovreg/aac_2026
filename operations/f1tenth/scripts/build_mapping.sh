#!/usr/bin/env bash
set -eo pipefail
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
set -u
workspace="${F1TENTH_WORKSPACE:-/ws}"
artifacts="$workspace/artifacts/f1tenth/${ROS_DISTRO}-$(uname -m)"
cd "$workspace"
bash "$workspace/src/aac_2026/perception/setup_colcon_ignores.sh"
colcon --log-base "$artifacts/log" build \
  --base-paths src/aac_2026 \
  --build-base "$artifacts/build" --install-base "$artifacts/install" \
  --packages-up-to bag_adapter fast_lio livox_ros_driver2 \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DROS_EDITION=ROS2 -DDISTRO_ROS="$ROS_DISTRO" "$@"
