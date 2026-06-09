#!/usr/bin/env bash

source "/opt/ros/$ROS_DISTRO/setup.bash"
[ -f "/ros_ws/install/setup.bash" ] && source "/ros_ws/install/setup.bash"

# Default ROS_DOMAIN_ID to 3 if not set or empty
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-3}"
export HF_HOME="${HF_HOME:-/ros_ws/src/aac_2026/.hf_home}"
export HF_HUB_DISABLE_TELEMETRY="${HF_HUB_DISABLE_TELEMETRY:-1}"
# source "/opt/ACADOtoolkit/build/acado_env.sh"
# export PATH=$HOME/conda/bin:$HOME/conda/condabin:$PATH

for x in /opt/*; do
    if [[ -e "$x/.env.sh" ]]; then
	source "$x/.env.sh"
    fi
done

cd /ros_ws

echo 'aac-2026-image startup completed.'
exec "$@"
