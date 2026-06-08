#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Octree + A* Performance Benchmark Launcher
# ============================================================
#
# Purpose:
#   Compare semantic Octree + A* against RF Octree + A* on the same
#   /world/dynamic_cloud frames.
#
# Start Gazebo first:
#   ./scripts/run_gazebo.sh gazebo/maps/warehouse_predict_world.sdf -r -v 2
#
# Then run:
#   ./scripts/run_performance_benchmark.sh
#
# Output:
#   data/performance_benchmark.csv
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

: "${BENCHMARK_BIN:=${PROJECT_ROOT}/build/octree_performance_benchmark}"
: "${BENCHMARK_OUTPUT:=${PROJECT_ROOT}/data/performance_benchmark.csv}"
: "${BENCHMARK_RF_MODEL:=${PROJECT_ROOT}/models/random_forest_voxel_model.rf.txt}"

# Gazebo transport settings. Must match scripts/run_gazebo.sh.
: "${GZ_PARTITION_VALUE:=dynamic_cloud_test}"
: "${GZ_POINTCLOUD_TOPIC:=/world/dynamic_cloud}"

# Benchmark sampling settings.
: "${BENCHMARK_FRAMES:=3}"
: "${BENCHMARK_TIMEOUT_SEC:=90}"
: "${BENCHMARK_MAX_DEPTH:=9}"
: "${BENCHMARK_MAX_POINTS:=50000}"

# Path planning start / goal.
: "${PATH_START:=6,-2,1.2}"
: "${PATH_GOAL:=0,0,9.2}"

# A* parameters. Keep these aligned with warehouse_predict_world.sdf unless
# you intentionally want to test a different planner configuration.
: "${PATH_BLOCK_PROBABILITY:=0.92}"
: "${PATH_PROBABILITY_WEIGHT:=6}"
: "${PATH_VERTICAL_WEIGHT:=0.75}"
: "${PATH_STAIR_CONNECTION_RADIUS:=1.25}"
: "${PATH_MAX_NON_STAIR_VERTICAL_STEP:=0.35}"
: "${PATH_STAIR_FLOOR_EXIT_TOLERANCE:=0.45}"

# Multi-floor height model.
: "${FEATURE_FLOOR_Z:=0}"
: "${FEATURE_STORY_HEIGHT:=4}"
: "${FEATURE_FLOOR_SURFACE_OFFSET:=1}"

print_usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_performance_benchmark.sh [extra benchmark args...]

Environment overrides:
  BENCHMARK_OUTPUT                 CSV output path
  BENCHMARK_RF_MODEL               C++ RF text model
  BENCHMARK_FRAMES                 Number of cloud frames to benchmark
  BENCHMARK_TIMEOUT_SEC            Timeout while waiting for frames
  BENCHMARK_MAX_DEPTH              Octree max depth
  BENCHMARK_MAX_POINTS             Max cloud points used per frame
  GZ_PARTITION_VALUE               Gazebo partition
  GZ_POINTCLOUD_TOPIC              Point cloud topic
  PATH_START / PATH_GOAL           A* start / goal
  PATH_BLOCK_PROBABILITY           Blocked probability threshold
  PATH_PROBABILITY_WEIGHT          A* risk cost weight
  PATH_VERTICAL_WEIGHT             A* vertical movement cost
  PATH_STAIR_CONNECTION_RADIUS     Stair connector radius
  PATH_MAX_NON_STAIR_VERTICAL_STEP Non-stair Z jump limit
  PATH_STAIR_FLOOR_EXIT_TOLERANCE  Stair exit floor-level tolerance
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ] || [ "${1:-}" = "help" ]; then
  print_usage
  exit 0
fi

if [ ! -x "${BENCHMARK_BIN}" ]; then
  cat >&2 <<EOF
ERROR: Benchmark binary not found:
  ${BENCHMARK_BIN}

Build it with:
  cmake --build build --target octree_performance_benchmark
EOF
  exit 2
fi

if [ ! -f "${BENCHMARK_RF_MODEL}" ]; then
  cat >&2 <<EOF
ERROR: RF model not found:
  ${BENCHMARK_RF_MODEL}

Generate it with:
  venv/bin/python3 python/train_model.py
EOF
  exit 3
fi

export GZ_PARTITION="${GZ_PARTITION_VALUE}"

exec "${BENCHMARK_BIN}" \
  --partition "${GZ_PARTITION_VALUE}" \
  --topic "${GZ_POINTCLOUD_TOPIC}" \
  --rf-model "${BENCHMARK_RF_MODEL}" \
  --output "${BENCHMARK_OUTPUT}" \
  --frames "${BENCHMARK_FRAMES}" \
  --timeout-sec "${BENCHMARK_TIMEOUT_SEC}" \
  --max-depth "${BENCHMARK_MAX_DEPTH}" \
  --max-points "${BENCHMARK_MAX_POINTS}" \
  --start "${PATH_START}" \
  --goal "${PATH_GOAL}" \
  --block-probability "${PATH_BLOCK_PROBABILITY}" \
  --probability-weight "${PATH_PROBABILITY_WEIGHT}" \
  --vertical-weight "${PATH_VERTICAL_WEIGHT}" \
  --stair-connection-radius "${PATH_STAIR_CONNECTION_RADIUS}" \
  --max-non-stair-vertical-step "${PATH_MAX_NON_STAIR_VERTICAL_STEP}" \
  --stair-floor-exit-tolerance "${PATH_STAIR_FLOOR_EXIT_TOLERANCE}" \
  --floor-z "${FEATURE_FLOOR_Z}" \
  --story-height "${FEATURE_STORY_HEIGHT}" \
  --floor-surface-offset "${FEATURE_FLOOR_SURFACE_OFFSET}" \
  "$@"
