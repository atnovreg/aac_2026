#!/usr/bin/env bash
# Use the isolated acados environment without changing the caller's shell.
set -euo pipefail
# Resolve both the canonical source path and legacy script symlink.
acados_workspace=$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")
while [[ ! -f "$acados_workspace/.f1tenth-workspace" && "$acados_workspace" != / ]]; do
    acados_workspace=$(dirname -- "$acados_workspace")
done
if [[ ! -f "$acados_workspace/.f1tenth-workspace" ]]; then
    echo "Cannot locate .f1tenth-workspace for ${BASH_SOURCE[0]}" >&2
    exit 1
fi
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$acados_workspace/vendor/acados}"
export PYTHONPATH="$acados_workspace/src/aac_2026${PYTHONPATH:+:$PYTHONPATH}"
acados_venv="${ACADOS_VENV:-$acados_workspace/.venv-acados}"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export MPLBACKEND="${MPLBACKEND:-Agg}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/f1tenth-acados-matplotlib-${UID}}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
if [[ ! -x "$acados_venv/bin/python" || ! -f "$ACADOS_SOURCE_DIR/lib/libacados.so" ]]; then
    echo 'acados is not installed. Run control/acados_mpcc/scripts/setup_acados.sh from AAC first.' >&2
    exit 1
fi
exec "$acados_venv/bin/python" "$@"
