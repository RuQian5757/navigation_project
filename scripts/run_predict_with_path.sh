#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# One-command Predict World + RF Octree A* Path Launcher
# ============================================================
#
# This script launches the predict world. The world itself loads the
# RFOctreePathPlanner system plugin, which builds the RF Octree, runs A*, and
# renders the path as visual-only cylinders.
#
# Default start / goal match the visual-only markers in
# gazebo/maps/warehouse_predict_world.sdf.
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

: "${PREDICT_WORLD:=${PROJECT_ROOT}/gazebo/maps/warehouse_predict_world.sdf}"
# Keep GUI enabled so the visual-only path cylinders can be seen.
# Do not include -s here if you want to see the path line in the GUI.
: "${PREDICT_GZ_ARGS:=-r -v 2}"
: "${PREDICT_PLANNER_DELAY:=2}"
: "${PREDICT_USE_EXTERNAL_PLANNER:=0}"
: "${PATH_START:=6,-2,1.2}"
: "${PATH_GOAL:=0,0,9.2}"

print_usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_predict_with_path.sh

Common overrides:
  PREDICT_WORLD             World file, default gazebo/maps/warehouse_predict_world.sdf
  PREDICT_GZ_ARGS           Extra gz sim args, default "-r -v 2"
  PREDICT_USE_EXTERNAL_PLANNER 1 also starts scripts/run_path_planning.sh for debugging
  PREDICT_PLANNER_DELAY     Seconds to wait before starting external planner
  PATH_START                Start position, default "6,-2,1.2"
  PATH_GOAL                 Goal position, default "0,0,9.2"

All PATH_* variables accepted by scripts/run_path_planning.sh can also be used.
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ] || [ "${1:-}" = "help" ]; then
  print_usage
  exit 0
fi

GAZEBO_PID=""
cleanup() {
  if [ -n "${GAZEBO_PID}" ] && kill -0 "${GAZEBO_PID}" 2>/dev/null; then
    kill "${GAZEBO_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ "${PREDICT_USE_EXTERNAL_PLANNER}" = "1" ]; then
  "${SCRIPT_DIR}/run_gazebo.sh" "${PREDICT_WORLD}" ${PREDICT_GZ_ARGS} &
  GAZEBO_PID="$!"

  sleep "${PREDICT_PLANNER_DELAY}"

  "${SCRIPT_DIR}/run_path_planning.sh" "$@"
else
  exec "${SCRIPT_DIR}/run_gazebo.sh" "${PREDICT_WORLD}" ${PREDICT_GZ_ARGS} "$@"
fi
