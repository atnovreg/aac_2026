#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
export PYTHONPATH="/ws/src/aac_2026${PYTHONPATH:+:$PYTHONPATH}"
exec "$@"
