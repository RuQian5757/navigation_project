#pragma once

#include "octree_manager.h"

#include <cstdint>
#include <string>
#include <vector>

namespace navigation {

struct LeafNode {
    int node_index = -1;
    uint32_t entity_id = 0;
    float min[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float max[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint64_t morton_code = 0;
    uint8_t depth = 0;
    int num_points = 0;
    float avg_normal[3] = {0.0f, 0.0f, 0.0f};
    float covariance_eigenvalues[3] = {0.0f, 0.0f, 0.0f};
    float pca_linearity = 0.0f;
    float pca_flatness = 0.0f;
    float pca_roughness = 0.0f;
    float pca_curvature = 0.0f;
    float center[3] = {0.0f, 0.0f, 0.0f};
    float size = 0.0f;
    int label = 0;
    float obstacle_probability = 0.0f;
};

struct FeatureExtractionConfig {
    float floor_z = 0.0f;
    float story_height = 4.0f;
    float floor_surface_offset = 0.0f;
    float ceiling_offset = 3.0f;
    float near_floor_z = 0.4f;
    float near_ceiling_z = 0.4f;
    float ceiling_z = 3.0f;
    float epsilon = 1e-6f;
};

struct WeakLabelingConfig {
    float floor_z = 0.0f;
    float story_height = 4.0f;
    float floor_surface_offset = 0.0f;
    float ceiling_offset = 3.0f;
    float ceiling_z = 3.0f;
    float floor_band = 0.35f;
    float free_max_slope_rad = 0.30f;
    float stair_min_slope_rad = 0.25f;
    float stair_max_slope_rad = 0.90f;
    float min_stair_height = 0.08f;
    int min_points_for_confident_label = 3;
};

void exportLeafFeaturesToCSV(const std::vector<LeafNode>& leaves,
                             const std::string& filename);

void exportLeafFeaturesToCSV(const std::vector<LeafNode>& leaves,
                             const std::string& filename,
                             const FeatureExtractionConfig& config);

void exportLeafFeaturesToCSV(const std::vector<LeafNode>& leaves,
                             const std::vector<int>& labels,
                             const std::vector<float>& obstacle_probabilities,
                             const std::string& filename);

void exportOctreeLeafFeaturesToCSV(const std::vector<OctreeNode>& nodes,
                                   const std::string& filename);

void exportOctreeLeafFeaturesToCSV(const std::vector<OctreeNode>& nodes,
                                   const std::string& filename,
                                   const FeatureExtractionConfig& config);

void assignWeakLabels(std::vector<LeafNode>& leaves,
                      const WeakLabelingConfig& config = WeakLabelingConfig{});

void exportWeakLabeledOctreeLeafFeaturesToCSV(
    const std::vector<OctreeNode>& nodes,
    const std::string& filename,
    const WeakLabelingConfig& config = WeakLabelingConfig{},
    const FeatureExtractionConfig& feature_config = FeatureExtractionConfig{});

} // namespace navigation
