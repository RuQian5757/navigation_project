#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Prediction / Evaluation Feature CSV Export Launcher
# ============================================================
#
# Purpose:
#   Generate a normal feature CSV for model evaluation.
#
# Output meaning:
#   label / obstacle_probability come from Gazebo semantic fields aggregated
#   into Octree leaves. They can be used as the reference columns when
#   python/train_model.py compares predicted_label against label.
#
# Usage:
#   ./scripts/run_feature_export_predict.sh
#   PREDICT_FEATURE_ONCE=1 ./scripts/run_feature_export_predict.sh
#

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# ============================================================
# Parameters You Usually Adjust
# ============================================================

# Output CSV. Keep this without train_ or predicted_ prefix so
# python/train_model.py can automatically pick it as predict data.
: "${PREDICT_FEATURE_OUTPUT:=${PROJECT_ROOT}/data/leaf_features.csv}"

# Export only one received cloud and exit. Useful for collecting one snapshot.
: "${PREDICT_FEATURE_ONCE:=0}"

# Keep one CSV per exported frame instead of overwriting the output path.
: "${PREDICT_FEATURE_TIMESTAMPED:=0}"

# Export frequency in Hz when not using PREDICT_FEATURE_ONCE=1.
: "${PREDICT_FEATURE_EXPORT_HZ:=1}"

# Octree and input point limits.
: "${PREDICT_FEATURE_MAX_DEPTH:=9}"
: "${PREDICT_FEATURE_MAX_POINTS:=120000}"

print_usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_feature_export_predict.sh [extra exporter args...]

Purpose:
  Export a Gazebo-semantic reference CSV for model prediction / accuracy checks.

Environment overrides:
  PREDICT_FEATURE_OUTPUT       Output CSV, default data/leaf_features.csv
  PREDICT_FEATURE_ONCE         1 exports first cloud and exits
  PREDICT_FEATURE_TIMESTAMPED  1 writes one CSV per exported frame
  PREDICT_FEATURE_EXPORT_HZ    Max export frequency
  PREDICT_FEATURE_MAX_DEPTH    Octree max depth
  PREDICT_FEATURE_MAX_POINTS   Max input points per export

Shared overrides forwarded to the exporter:
  GZ_PARTITION_VALUE
  GZ_POINTCLOUD_TOPIC
  FEATURE_FLOOR_Z
  FEATURE_STORY_HEIGHT
  FEATURE_FLOOR_SURFACE_OFFSET
  FEATURE_CEILING_OFFSET
  FEATURE_NEAR_FLOOR
  FEATURE_NEAR_CEILING
EOF
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ] || [ "${1:-}" = "help" ]; then
  print_usage
  exit 0
fi

# ============================================================
# Fixed Mode Settings
# ============================================================

# This mode intentionally does not load RF and does not weak-label.
# It produces reference / evaluation data from Gazebo semantic fields.
export FEATURE_OUTPUT="${PREDICT_FEATURE_OUTPUT}"
export FEATURE_ONCE="${PREDICT_FEATURE_ONCE}"
export FEATURE_TIMESTAMPED="${PREDICT_FEATURE_TIMESTAMPED}"
export FEATURE_EXPORT_HZ="${PREDICT_FEATURE_EXPORT_HZ}"
export FEATURE_MAX_DEPTH="${PREDICT_FEATURE_MAX_DEPTH}"
export FEATURE_MAX_POINTS="${PREDICT_FEATURE_MAX_POINTS}"
export FEATURE_TRAIN=0
export FEATURE_WEAK_LABELS=0
export FEATURE_RF_MODEL=

exec "${SCRIPT_DIR}/run_feature_export.sh" "$@"
