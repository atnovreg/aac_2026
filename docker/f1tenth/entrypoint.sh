#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
f1tenth_install="${F1TENTH_INSTALL_BASE:-/ws/artifacts/f1tenth/jazzy-$(uname -m)/install}"
if [[ -f "$f1tenth_install/setup.bash" ]]; then source "$f1tenth_install/setup.bash"; fi
exec "$@"
