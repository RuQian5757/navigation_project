#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_PLUGIN_DIR="${PROJECT_ROOT}/build/dynamic_world_cloud"
PLUGIN_LIB="${BUILD_PLUGIN_DIR}/libDynamicWorldCloud.so"
GAZEBO_MAPS_DIR="${PROJECT_ROOT}/gazebo/maps"
GAZEBO_MODELS_DIR="${GAZEBO_MAPS_DIR}/models"

prepend_path_env() {
  local var_name="$1"
  local path_value="$2"
  local current_value="${!var_name:-}"

  if [ -z "$current_value" ]; then
    export "${var_name}=${path_value}"
  else
    export "${var_name}=${path_value}:${current_value}"
  fi
}

print_usage() {
  cat <<'EOF'
Usage: ./scripts/run_gazebo.sh <world.sdf> [gz-sim-args...]

This script exports the plugin library path and launches Gazebo.
It assumes the plugin was compiled in build/dynamic_world_cloud.

Examples:
  ./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -s -r -v 2
  ./scripts/run_gazebo.sh warehouse_world.sdf -s -r -v 2
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  print_usage
  exit 0
fi

if [ "$#" -lt 1 ]; then
  print_usage
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
prepend_path_env GZ_SIM_RESOURCE_PATH "${GAZEBO_MAPS_DIR}"
prepend_path_env GZ_SIM_RESOURCE_PATH "${GAZEBO_MODELS_DIR}"
prepend_path_env GZ_FILE_PATH "${GAZEBO_MAPS_DIR}"
prepend_path_env GZ_FILE_PATH "${GAZEBO_MODELS_DIR}"
export GZ_PARTITION="${GZ_PARTITION:-dynamic_cloud_test}"

WORLD_FILE="$1"
shift

# If the requested world file is not found directly, try repository-relative
# paths and then the common gazebo/maps location.
if [ ! -f "$WORLD_FILE" ]; then
  CANDIDATE="${PROJECT_ROOT}/$WORLD_FILE"
  if [ -f "$CANDIDATE" ]; then
    WORLD_FILE="$CANDIDATE"
  else
    CANDIDATE2="${GAZEBO_MAPS_DIR}/$WORLD_FILE"
    if [ -f "$CANDIDATE2" ]; then
      WORLD_FILE="$CANDIDATE2"
    else
      CANDIDATE3="${GAZEBO_MAPS_DIR}/$(basename "$WORLD_FILE")"
      if [ -f "$CANDIDATE3" ]; then
        WORLD_FILE="$CANDIDATE3"
      fi
    fi
  fi
fi

if [ ! -f "$WORLD_FILE" ]; then
  echo "ERROR: World file not found: $1"
  echo "Searched:
  - $1
  - ${PROJECT_ROOT}/$1
  - ${GAZEBO_MAPS_DIR}/$1
  - ${GAZEBO_MAPS_DIR}/$(basename "$1")"
  exit 3
fi

echo "Using project root: $PROJECT_ROOT"
echo "Using build plugin dir: $BUILD_PLUGIN_DIR"
echo "Launching gz sim with world: $WORLD_FILE"
echo "Using Gazebo transport partition: $GZ_PARTITION"

gz sim "$WORLD_FILE" "$@"
