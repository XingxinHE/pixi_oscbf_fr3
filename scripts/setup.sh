#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-100}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
echo "Using ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "Using RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  source "${WS_ROOT}/install/setup.bash"
  echo "✓ Workspace overlay sourced"
else
  echo "WARN: install/setup.bash not found. Run 'pixi run build' first."
fi
