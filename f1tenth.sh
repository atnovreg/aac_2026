#!/usr/bin/env bash
# Explicit entry points for the integrated F1TENTH tools.
set -euo pipefail
aac_root="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd)"
workspace="$(cd -- "$aac_root/../.." && pwd)"
action="${1:-help}"
if (( $# )); then shift; fi
cd "$workspace"
case "$action" in
  check)
    python3 "$aac_root/operations/f1tenth/verify_layout.py" "$@"
    ;;
  python)
    exec "$aac_root/control/acados_mpcc/scripts/acados_python.sh" "$@"
    ;;
  tuner)
    exec "$aac_root/control/mpcc_tuner/scripts/mpcc_tuner.sh" "$@"
    ;;
  localization-test)
    cd "$aac_root/navigation/localization3d"
    exec "$aac_root/control/acados_mpcc/scripts/acados_python.sh" \
      -W ignore::DeprecationWarning -m unittest discover -s tests -v "$@"
    ;;
  help|-h|--help)
    printf '%s\n' 'Usage: bash f1tenth.sh {check|python [args...]|tuner [args...]|localization-test}'
    ;;
  *)
    printf 'Unknown action: %s\n' "$action" >&2
    exit 2
    ;;
esac
