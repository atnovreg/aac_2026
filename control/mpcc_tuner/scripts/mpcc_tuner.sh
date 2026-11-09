#!/usr/bin/env bash
# Local offline MPCC tuning dashboard; never sends commands to a vehicle.
set -euo pipefail
mpcc_tuner_root="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../../../../.." && pwd)"
if [[ ! -f "$mpcc_tuner_root/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
export PYTHONPATH="$mpcc_tuner_root/src/aac_2026${PYTHONPATH:+:$PYTHONPATH}"
cd "$mpcc_tuner_root"
exec .venv-dynamic-lio/bin/python -m control.mpcc_tuner.server "$@"
