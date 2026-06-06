#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# Navigation Project 3D Visualization Launcher
# ============================================================
#
# Usage:
#   ./scripts/run_visualization.sh octree
#   ./scripts/run_visualization.sh pointcloud
#
# You can override any setting from the command line, for example:
#   OCTREE_MAX_VOXELS=1000 ./scripts/run_visualization.sh octree
#   POINTCLOUD_POINT_SIZE=2 ./scripts/run_visualization.sh pointcloud
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
# Octree 3D Display Settings
# ============================================================

# Compiled C++ realtime Octree viewer.
: "${OCTREE_VIEWER_BIN:=${PROJECT_ROOT}/build/octree_viewer/visualize_octree_gazebo}"

# Maximum Octree depth used by the viewer-side OctreeManager.
# Larger values allow smaller voxels, but may produce more leaves.
: "${OCTREE_MAX_DEPTH:=9}"

# Maximum number of leaf voxels rendered in PCLVisualizer.
# This is the most important performance knob.
: "${OCTREE_MAX_VOXELS:=6000}"

# Maximum number of received cloud points used to rebuild the viewer Octree.
# Lower values make the viewer smoother but reduce visual detail.
: "${OCTREE_MAX_RENDER_POINTS:=80000}"

# Maximum parse / Octree rebuild / render refresh rate in Hz.
# For demonstrations, 0.5 to 1.0 is usually enough.
: "${OCTREE_REBUILD_HZ:=1}"

# Voxel drawing mode:
#   centers      : draw only leaf voxel centers, fastest
#   boxes        : draw every sampled leaf voxel wireframe, clearest but slowest
#   hybrid       : draw centers plus a small number of boxes
#   center-boxes : draw centers and matching wireframe boxes for sampled voxels
: "${OCTREE_VOXEL_MODE:=center-boxes}"

# Whether to hide the raw white point cloud in Octree view.
# Set to 0 if you want to keep the original point cloud shape visible.
: "${OCTREE_HIDE_POINTS:=1}"

# If set to 1, color voxels by label instead of Octree depth.
# Current Gazebo cloud does not provide ML labels, so depth color is preferred.
: "${OCTREE_LABEL_COLOR:=0}"

# ============================================================
# Raw PointCloud 3D Display Settings
# ============================================================

# Python interpreter used for the PyVista point cloud viewer.
# If the project venv exists, the launcher will use it automatically unless
# PYTHON_BIN is already set.
if [ -z "${PYTHON_BIN:-}" ] && [ -x "${PROJECT_ROOT}/venv/bin/python3" ]; then
  PYTHON_BIN="${PROJECT_ROOT}/venv/bin/python3"
fi
: "${PYTHON_BIN:=python3}"

# Size of rendered raw point cloud points in PyVista.
: "${POINTCLOUD_POINT_SIZE:=3}"

# Colormap used for raw point cloud elevation coloring.
: "${POINTCLOUD_CMAP:=viridis}"

# Maximum PyVista refresh rate in Hz.
: "${POINTCLOUD_UPDATE_RATE:=20}"

# Maximum number of points rendered by the Python viewer.
: "${POINTCLOUD_MAX_RENDER_POINTS:=250000}"

# Viewer window size.
: "${POINTCLOUD_WINDOW_WIDTH:=1280}"
: "${POINTCLOUD_WINDOW_HEIGHT:=768}"

print_usage() {
  cat <<'EOF'
Usage:
  ./scripts/run_visualization.sh octree [extra octree viewer args...]
  ./scripts/run_visualization.sh pointcloud [extra pointcloud viewer args...]

Modes:
  octree      Subscribe /world/dynamic_cloud and show realtime Octree voxels.
  pointcloud  Subscribe /world/dynamic_cloud and show raw point cloud only.

Common environment overrides:
  GZ_PARTITION_VALUE
  GZ_POINTCLOUD_TOPIC

Octree overrides:
  OCTREE_MAX_DEPTH
  OCTREE_MAX_VOXELS
  OCTREE_MAX_RENDER_POINTS
  OCTREE_REBUILD_HZ
  OCTREE_VOXEL_MODE
  OCTREE_HIDE_POINTS
  OCTREE_LABEL_COLOR

Pointcloud overrides:
  PYTHON_BIN
  POINTCLOUD_POINT_SIZE
  POINTCLOUD_CMAP
  POINTCLOUD_UPDATE_RATE
  POINTCLOUD_MAX_RENDER_POINTS
  POINTCLOUD_WINDOW_WIDTH
  POINTCLOUD_WINDOW_HEIGHT
EOF
}

mode="${1:-octree}"
if [ "$#" -gt 0 ]; then
  shift
fi

export GZ_PARTITION="${GZ_PARTITION_VALUE}"

case "${mode}" in
  octree)
    if [ ! -x "${OCTREE_VIEWER_BIN}" ]; then
      cat >&2 <<EOF
ERROR: Octree viewer not found or not executable:
  ${OCTREE_VIEWER_BIN}

Build it with:
  cmake -S scripts -B build/octree_viewer
  cmake --build build/octree_viewer --target visualize_octree_gazebo
EOF
      exit 2
    fi

    octree_args=(
      --partition "${GZ_PARTITION_VALUE}"
      --topic "${GZ_POINTCLOUD_TOPIC}"
      --max-depth "${OCTREE_MAX_DEPTH}"
      --max-voxels "${OCTREE_MAX_VOXELS}"
      --max-render-points "${OCTREE_MAX_RENDER_POINTS}"
      --rebuild-hz "${OCTREE_REBUILD_HZ}"
      --voxel-mode "${OCTREE_VOXEL_MODE}"
    )

    if [ "${OCTREE_HIDE_POINTS}" = "1" ]; then
      octree_args+=(--no-points)
    fi
    if [ "${OCTREE_LABEL_COLOR}" = "1" ]; then
      octree_args+=(--label-color)
    fi

    exec "${OCTREE_VIEWER_BIN}" "${octree_args[@]}" "$@"
    ;;

  pointcloud)
    exec "${PYTHON_BIN}" "${PROJECT_ROOT}/scripts/visualize_pointcloud_realtime.py" \
      --partition "${GZ_PARTITION_VALUE}" \
      --topic "${GZ_POINTCLOUD_TOPIC}" \
      --point-size "${POINTCLOUD_POINT_SIZE}" \
      --cmap "${POINTCLOUD_CMAP}" \
      --update-rate "${POINTCLOUD_UPDATE_RATE}" \
      --max-render-points "${POINTCLOUD_MAX_RENDER_POINTS}" \
      --window-width "${POINTCLOUD_WINDOW_WIDTH}" \
      --window-height "${POINTCLOUD_WINDOW_HEIGHT}" \
      "$@"
    ;;

  -h|--help|help)
    print_usage
    ;;

  *)
    echo "Unknown visualization mode: ${mode}" >&2
    print_usage >&2
    exit 1
    ;;
esac
