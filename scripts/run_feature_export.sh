#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Navigation Project Gazebo Leaf Feature Export Launcher
# ============================================================
#
# Usage:
#   ./scripts/run_feature_export.sh
#
# This script subscribes to the Gazebo point cloud topic, rebuilds the
# Octree at a controlled rate, and exports leaf-node Random Forest
# features to CSV. It is intentionally separate from the 3D viewers so
# dataset generation can keep running without a rendering window.
#
# You can override any setting from the command line, for example:
#   FEATURE_EXPORT_HZ=0.5 FEATURE_OUTPUT=data/demo.csv ./scripts/run_feature_export.sh
#   FEATURE_ONCE=1 ./scripts/run_feature_export.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ============================================================
# Shared Gazebo Transport Settings
# ============================================================

# Gazebo Transport partition. This must match the partition used by Gazebo.
# scripts/run_gazebo.sh defaults to "dynamic_cloud_test".
: "${GZ_PARTITION_VALUE:=dynamic_cloud_test}"

# Point cloud topic published by DynamicWorldCloud.
: "${GZ_POINTCLOUD_TOPIC:=/world/dynamic_cloud}"

# ============================================================
# Feature Export Settings
# ============================================================

# Compiled C++ headless exporter.
: "${FEATURE_EXPORTER_BIN:=${PROJECT_ROOT}/build/leaf_feature_exporter_gazebo}"

# CSV output path. By default the exporter overwrites this file each export.
# Set FEATURE_TIMESTAMPED=1 to keep one CSV per exported frame.
: "${FEATURE_OUTPUT:=${PROJECT_ROOT}/data/leaf_features.csv}"

# Octree max depth used for feature extraction.
# Larger values can expose smaller voxels but may increase CSV size heavily.
: "${FEATURE_MAX_DEPTH:=9}"

# Maximum number of received cloud points used per exported frame.
# Lower values reduce CPU cost and CSV noise; higher values preserve detail.
: "${FEATURE_MAX_POINTS:=120000}"

# Maximum CSV export rate in Hz.
# 1 means rebuild Octree and write CSV at most once per second.
: "${FEATURE_EXPORT_HZ:=1}"

# Set to 1 to export only the first received point cloud and then exit.
: "${FEATURE_ONCE:=0}"

# Set to 1 to write FEATURE_OUTPUT stem plus _frameNNNNNN.csv.
# Example: data/leaf_features_frame000001.csv
: "${FEATURE_TIMESTAMPED:=0}"

# Set to 1 to fill label / obstacle_probability with simple rule-based weak labels.
# This is useful for bootstrapping a training CSV before manual correction.
: "${FEATURE_WEAK_LABELS:=0}"

# Multi-floor vertical model used by feature extraction and weak-label fallback.
# If each floor slab is 1m and each wall / indoor navigable height is 3m,
# the floor-to-floor story height is 4m.
: "${FEATURE_FLOOR_Z:=0}"
: "${FEATURE_STORY_HEIGHT:=4}"
: "${FEATURE_FLOOR_SURFACE_OFFSET:=1}"
: "${FEATURE_CEILING_OFFSET:=4}"
: "${FEATURE_NEAR_FLOOR:=0.4}"
: "${FEATURE_NEAR_CEILING:=0.4}"

print_usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_feature_export.sh [extra exporter args...]

Environment overrides:
  GZ_PARTITION_VALUE      Gazebo Transport partition
  GZ_POINTCLOUD_TOPIC     Gazebo PointCloudPacked topic
  FEATURE_EXPORTER_BIN    Built exporter binary path
  FEATURE_OUTPUT          CSV output path
  FEATURE_MAX_DEPTH       Octree max depth for extraction
  FEATURE_MAX_POINTS      Max input points per export
  FEATURE_EXPORT_HZ       Max export frequency
  FEATURE_ONCE            1 exports first cloud and exits
  FEATURE_TIMESTAMPED     1 writes one CSV per exported frame
  FEATURE_WEAK_LABELS     1 fills labels with rule-based weak labels
  FEATURE_FLOOR_Z         Story-0 origin height
  FEATURE_STORY_HEIGHT    Repeated floor-to-floor height
  FEATURE_FLOOR_SURFACE_OFFSET Local walkable floor surface offset in each story
  FEATURE_CEILING_OFFSET  Local ceiling offset in each story
  FEATURE_NEAR_FLOOR      Near-floor distance band
  FEATURE_NEAR_CEILING    Near-ceiling distance band
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ] || [ "${1:-}" = "help" ]; then
  print_usage
  exit 0
fi

if [ ! -x "${FEATURE_EXPORTER_BIN}" ]; then
  cat >&2 <<EOF
ERROR: Feature exporter not found or not executable:
  ${FEATURE_EXPORTER_BIN}

Build it with:
  cmake -S . -B build
  cmake --build build --target leaf_feature_exporter_gazebo
EOF
  exit 2
fi

export GZ_PARTITION="${GZ_PARTITION_VALUE}"

args=(
  --partition "${GZ_PARTITION_VALUE}"
  --topic "${GZ_POINTCLOUD_TOPIC}"
  --output "${FEATURE_OUTPUT}"
  --max-depth "${FEATURE_MAX_DEPTH}"
  --max-points "${FEATURE_MAX_POINTS}"
  --export-hz "${FEATURE_EXPORT_HZ}"
  --floor-z "${FEATURE_FLOOR_Z}"
  --story-height "${FEATURE_STORY_HEIGHT}"
  --floor-surface-offset "${FEATURE_FLOOR_SURFACE_OFFSET}"
  --ceiling-offset "${FEATURE_CEILING_OFFSET}"
  --near-floor "${FEATURE_NEAR_FLOOR}"
  --near-ceiling "${FEATURE_NEAR_CEILING}"
)

if [ "${FEATURE_ONCE}" = "1" ]; then
  args+=(--once)
fi

if [ "${FEATURE_TIMESTAMPED}" = "1" ]; then
  args+=(--timestamped)
fi

if [ "${FEATURE_WEAK_LABELS}" = "1" ]; then
  args+=(--weak-labels)
fi

exec "${FEATURE_EXPORTER_BIN}" "${args[@]}" "$@"
