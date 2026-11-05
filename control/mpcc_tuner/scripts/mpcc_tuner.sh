#!/usr/bin/env bash
# Local offline MPCC tuning dashboard; never sends commands to a vehicle.
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
mpcc_tuner_root=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$mpcc_tuner_root/.f1tenth-workspace" && "$mpcc_tuner_root" != / ]]; do
    mpcc_tuner_root=$(dirname -- "$mpcc_tuner_root")
done
if [[ ! -f "$mpcc_tuner_root/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
cd "$mpcc_tuner_root"
exec .venv-dynamic-lio/bin/python -m tools.mpcc_tuner.server "$@"
