#!/bin/bash
#
# This script cleans up all of the build data

set -e

WS_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

echo "--- Cleaning build ---"
cd "$WS_ROOT"
rm -rf build/ install/ log/

echo "Done. Please use scripts/build.sh to re-build"
