#!/usr/bin/env bash
# Install the pinned official release locally, or under explicit paths for Docker.
# Prerequisites: Python 3.12 with venv, git, C/C++ compiler, make, and cmake.
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
acados_lock="$acados_workspace/config/acados_versions.json"
export ACADOS_SOURCE_DIR="${ACADOS_SOURCE_DIR:-$acados_workspace/vendor/acados}"
export ACADOS_VENV="${ACADOS_VENV:-$acados_workspace/.venv-acados}"
acados_build_jobs="${ACADOS_BUILD_JOBS:-8}"
acados_python="${ACADOS_BOOTSTRAP_PYTHON:-python3}"
acados_commit=$("$acados_python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["acados"]["commit"])' "$acados_lock")
acados_release=$("$acados_python" -c 'import json,sys; print(json.load(open(sys.argv[1]))["acados"]["release"])' "$acados_lock")
"$acados_python" -c 'import sys; assert sys.version_info[:2] == (3,12), "Use Python 3.12 for this pinned environment"'
for acados_tool in git cmake make cc c++; do
    command -v "$acados_tool" >/dev/null || { echo "Missing prerequisite: $acados_tool" >&2; exit 1; }
done
if [[ ! -d "$ACADOS_SOURCE_DIR" ]]; then
    mkdir -p -- "$(dirname -- "$ACADOS_SOURCE_DIR")"
    git clone --branch "$acados_release" --depth 1 https://github.com/acados/acados.git "$ACADOS_SOURCE_DIR"
fi
if [[ "$(git -C "$ACADOS_SOURCE_DIR" rev-parse HEAD)" != "$acados_commit" ]]; then
    echo "Existing acados checkout does not match $acados_commit; preserving it and stopping." >&2
    exit 1
fi
git -C "$ACADOS_SOURCE_DIR" diff --quiet
git -C "$ACADOS_SOURCE_DIR" diff --cached --quiet
git -C "$ACADOS_SOURCE_DIR" submodule update --init --recursive --depth 1 -- external/blasfeo external/hpipm
"$acados_python" - "$acados_lock" "$ACADOS_SOURCE_DIR" <<'PY'
import json, pathlib, subprocess, sys
lock = json.load(open(sys.argv[1]))
source = pathlib.Path(sys.argv[2])
for name, expected in lock['submodules'].items():
    actual = subprocess.check_output(['git', '-C', str(source / name), 'rev-parse', 'HEAD'], text=True).strip()
    if actual != expected:
        raise SystemExit(f'{name}: expected {expected}, got {actual}')
    subprocess.run(['git', '-C', str(source / name), 'diff', '--exit-code'], check=True)
    subprocess.run(['git', '-C', str(source / name), 'diff', '--cached', '--exit-code'], check=True)
PY
cmake -S "$ACADOS_SOURCE_DIR" -B "$ACADOS_SOURCE_DIR/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DACADOS_WITH_OPENMP=OFF \
    -DBLASFEO_TARGET=GENERIC -DHPIPM_TARGET=GENERIC \
    -DACADOS_INSTALL_DIR="$ACADOS_SOURCE_DIR" -DCMAKE_INSTALL_PREFIX="$ACADOS_SOURCE_DIR"
cmake --build "$ACADOS_SOURCE_DIR/build" --parallel "$acados_build_jobs"
cmake --install "$ACADOS_SOURCE_DIR/build"
if [[ ! -x "$ACADOS_VENV/bin/python" ]]; then
    "$acados_python" -m venv "$ACADOS_VENV"
fi
"$ACADOS_VENV/bin/python" -m pip install --disable-pip-version-check -r "$acados_workspace/requirements/acados.txt"
"$ACADOS_VENV/bin/python" -m pip install --disable-pip-version-check --use-pep517 --no-build-isolation --no-deps \
    -e "$ACADOS_SOURCE_DIR/interfaces/acados_template"
export LD_LIBRARY_PATH="$ACADOS_SOURCE_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
"$ACADOS_VENV/bin/python" - "$acados_lock" <<'PY'
import json, subprocess, sys
from acados_template.utils import get_tera
version = json.load(open(sys.argv[1]))['tera_renderer']['version']
renderer = get_tera(version)
actual = subprocess.check_output([renderer, '--version'], text=True).strip()
if actual.split()[-1].lstrip('v') != version:
    raise SystemExit(f'Expected tera renderer {version}, got {actual}; preserving existing binary.')
print(actual)
PY
# Keep generated example code and figures outside the upstream checkout.
acados_smoke_dir="$ACADOS_SOURCE_DIR/build/smoke"
mkdir -p "$acados_smoke_dir"
(
    cd "$acados_smoke_dir"
    "$acados_workspace/scripts/acados_python.sh" - <<'PY'
import os, pathlib, runpy, sys
example = pathlib.Path(os.environ['ACADOS_SOURCE_DIR']) / 'examples/acados_python/getting_started/minimal_example_ocp.py'
sys.path.insert(0, str(example.parent))
runpy.run_path(str(example), run_name='__main__')
print('PASS: official pendulum OCP generated, compiled, and solved with SQP + HPIPM.')
PY
)
echo "acados ready: $ACADOS_SOURCE_DIR"
echo "Python: $ACADOS_VENV/bin/python"
