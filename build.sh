#!/usr/bin/env bash
set -eo pipefail

. /opt/ros/$ROS_DISTRO/setup.bash
set -u
cd /ros_ws/src/aac_2026
./perception/setup_colcon_ignores.sh

cd /ros_ws
colcon build --base-paths src/* "$@" \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_INSTALL_PREFIX=/ros_ws/install
