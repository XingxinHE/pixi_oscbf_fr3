#!/bin/bash
#
# This script configures the environment (ROS2 + python venv)

WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
VENV_DIR="$WS_ROOT/.venv"

if [[ ! -d "$VENV_DIR" ]]; then
  echo "ERROR: Could not find the virtual env"
  echo "Check that you've set up the venv with UV"
  exit 1
fi

export ROS_DOMAIN_ID=10
echo "Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

cd "$WS_ROOT"
source "$WS_ROOT/.venv/bin/activate"
source install/setup.bash
echo "✓ Environment configured"
