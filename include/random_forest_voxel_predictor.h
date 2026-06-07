#pragma once

#include "octree_manager.h"

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace navigation {

struct RandomForestFeatureConfig {
    float floor_z = 0.0f;
    float story_height = 4.0f;
    float floor_surface_offset = 1.0f;
    float ceiling_offset = 4.0f;
    float near_floor_z = 0.4f;
    float near_ceiling_z = 0.4f;
    float epsilon = 1e-6f;
};

class RandomForestVoxelPredictor {
public:
    RandomForestVoxelPredictor() = default;
    explicit RandomForestVoxelPredictor(RandomForestFeatureConfig config);

    void loadFromTextModel(const std::string& filename);
    bool loaded() const;

    MLResult predict(const OctreeNode& node) const;
    OctreeManager::MLPredictor asMLPredictor() const;

    const std::vector<std::string>& featureNames() const { return feature_names_; }
    std::size_t classifierTreeCount() const;
    std::size_t regressorTreeCount() const;

private:
    struct ClassTreeNode {
        int left = -1;
        int right = -1;
        int feature = -1;
        float threshold = 0.0f;
        std::array<float, 3> values = {0.0f, 0.0f, 0.0f};
    };

    struct RegressionTreeNode {
        int left = -1;
        int right = -1;
        int feature = -1;
        float threshold = 0.0f;
        float value = 0.0f;
    };

    using ClassTree = std::vector<ClassTreeNode>;
    using RegressionTree = std::vector<RegressionTreeNode>;

    RandomForestFeatureConfig config_;
    std::vector<std::string> feature_names_;
    std::vector<ClassTree> classifier_trees_;
    std::vector<RegressionTree> regressor_trees_;

    std::vector<float> extractFeatures(const OctreeNode& node) const;
    std::array<float, 3> predictClassScores(const std::vector<float>& features) const;
    float predictProbability(const std::vector<float>& features) const;
};

} // namespace navigation
