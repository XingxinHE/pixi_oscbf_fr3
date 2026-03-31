#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"

PYTHON_EXECUTABLE="$(which python)"
echo "Using Python: ${PYTHON_EXECUTABLE}"

if [[ -f "/opt/ros/humble/setup.bash" ]]; then
  # Optional fallback when running outside pixi shell.
  source /opt/ros/humble/setup.bash
fi

colcon build --symlink-install \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPython_EXECUTABLE="${PYTHON_EXECUTABLE}" \
    -DPython3_EXECUTABLE="${PYTHON_EXECUTABLE}" \
    ${CONDA_PREFIX:+-DPython_ROOT_DIR=${CONDA_PREFIX}} \
    ${CONDA_PREFIX:+-DPython3_ROOT_DIR=${CONDA_PREFIX}} \
    -DPython_FIND_STRATEGY=LOCATION \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -Wno-dev

echo "Build complete. Open a new shell or source scripts/setup.sh to refresh overlays."
