#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_PLUGIN_DIR="${SCRIPT_DIR}/build/dynamic_world_cloud"
PLUGIN_LIB="${BUILD_PLUGIN_DIR}/libDynamicWorldCloud.so"

if [ "$#" -lt 1 ]; then
  cat <<'EOF'
Usage: ./run_gazebo.sh <world.sdf> [gz-sim-args...]

This script exports the plugin library path and launches Gazebo.
It assumes the plugin was compiled in build/dynamic_world_cloud.
EOF
  exit 1
fi

if [ ! -f "$PLUGIN_LIB" ]; then
  echo "ERROR: Plugin library not found: $PLUGIN_LIB"
  echo "Please build it first with: cmake --build build --target dynamic_world_cloud"
  exit 2
fi

export LD_LIBRARY_PATH="${BUILD_PLUGIN_DIR}:${LD_LIBRARY_PATH:-}"
export GZ_PLUGIN_PATH="${BUILD_PLUGIN_DIR}:${GZ_PLUGIN_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${BUILD_PLUGIN_DIR}:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_PARTITION="${GZ_PARTITION:-dynamic_cloud_test}"

WORLD_FILE="$1"
shift

# If the requested world file is not found directly, try ./gazebo/maps/<name>
if [ ! -f "$WORLD_FILE" ]; then
  CANDIDATE="${SCRIPT_DIR}/gazebo/maps/$WORLD_FILE"
  if [ -f "$CANDIDATE" ]; then
    WORLD_FILE="$CANDIDATE"
  else
    CANDIDATE2="${SCRIPT_DIR}/gazebo/maps/$(basename "$WORLD_FILE")"
    if [ -f "$CANDIDATE2" ]; then
      WORLD_FILE="$CANDIDATE2"
    fi
  fi
fi

if [ ! -f "$WORLD_FILE" ]; then
  echo "ERROR: World file not found: $1"
  echo "Searched:
  - $1
  - ${SCRIPT_DIR}/gazebo/maps/$1
  - ${SCRIPT_DIR}/gazebo/maps/$(basename "$1")"
  exit 3
fi

echo "Using build plugin dir: $BUILD_PLUGIN_DIR"
echo "Launching gz sim with world: $WORLD_FILE"
echo "Using Gazebo transport partition: $GZ_PARTITION"

gz sim "$WORLD_FILE" "$@"
