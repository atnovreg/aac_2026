#!/usr/bin/env bash
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
action="${1:-autonomy}"
if (( $# )); then shift; fi
case "$action" in
  -h|--help|help)
    echo 'Usage: bash start.sh [autonomy|prepare|sim] [-d] | check [autonomy|prepare]'
    exit ;;
  check)
    export AAC_RUN_MODE="${1:-autonomy}"
    exec bash "$repo/operations/tmuxp_config/jetson.sh" check ;;
  autonomy|prepare)
    export AAC_RUN_MODE="$action"
    export AAC_SETTINGS="${AAC_SETTINGS:-$repo/operations/tmuxp_config/jetson.env}"
    bash "$repo/operations/tmuxp_config/jetson.sh" check
    config="$repo/operations/tmuxp_config/config.yaml"
    ;;
  sim) config="$repo/operations/tmuxp_config/simulation.yaml" ;;
  *) echo "Unknown mode: $action. See bash start.sh --help" >&2; exit 2 ;;
esac
if (( $# > 1 )) || { (( $# == 1 )) && [[ "$1" != -d ]]; }; then
  echo 'Only -d (detached) is supported after the mode. See bash start.sh --help' >&2
  exit 2
fi
if tmux has-session -t '=AAC_2026' 2>/dev/null; then
  echo 'AAC_2026 already exists. Attach with tmux attach -t AAC_2026, or stop it with bash stop.sh.' >&2
  exit 1
fi
if [[ "$action" != sim ]]; then
  # The checker runs in a subprocess; load the same settings for run allocation.
  set -a
  source "$AAC_SETTINGS"
  source "$repo/operations/tmuxp_config/jetson.env"
  set +a
  mkdir -p -- "$AAC_LIVE_DIR/runs"
  export AAC_RUN_DIR
  AAC_RUN_DIR="$(mktemp -d "$AAC_LIVE_DIR/runs/$(date +%Y%m%d_%H%M%S)_XXXXXX")"
  # Freeze settings for all panes; an already-running tmux server has stale env.
  while IFS= read -r name; do
    case "$name" in AAC_SETTINGS|AAC_REPO|AAC_RUN_DIR) continue ;; esac
    declare -p "$name"
  done < <(compgen -v AAC_) > "$AAC_RUN_DIR/settings.env"
  export AAC_SETTINGS="$AAC_RUN_DIR/settings.env"
  printf 'Mode: %s\nRun: %s\n' "$action" "$AAC_RUN_DIR"
fi
cd "$repo"
exec tmuxp load "$@" "$config"
