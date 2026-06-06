#pragma once

#include "octree_manager.h"

#include <cstdint>
#include <string>
#include <vector>

namespace navigation {

struct LeafNode {
    float min[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float max[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint32_t morton_code = 0;
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
    float near_floor_z = 0.4f;
    float near_ceiling_z = 2.0f;
    float floor_z = 0.0f;
    float ceiling_z = 2.5f;
    float epsilon = 1e-6f;
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

} // namespace navigation
