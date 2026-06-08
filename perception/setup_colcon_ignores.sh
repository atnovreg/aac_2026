#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace_root="$(cd "${repo_root}/../.." && pwd)"

external_dirs=(
  "${repo_root}/perception/BiSeNet"
  "${repo_root}/perception/PIDNet"
  "${repo_root}/perception/sam3"
  "${workspace_root}/wildgs-env"
)

for dir in "${external_dirs[@]}"; do
  if [[ -d "${dir}" ]]; then
    touch "${dir}/COLCON_IGNORE"
    echo "[colcon-ignore] ${dir}/COLCON_IGNORE"
  fi
done
