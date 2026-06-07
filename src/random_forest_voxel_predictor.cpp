#include "random_forest_voxel_predictor.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace navigation {

namespace {

float clampFloat(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

float safeDivide(float numerator, float denominator, float fallback = 0.0f) {
    return std::abs(denominator) > 1e-6f ? numerator / denominator : fallback;
}

std::array<float, 3> sortedExtents(float dx, float dy, float dz) {
    std::array<float, 3> extents{std::abs(dx), std::abs(dy), std::abs(dz)};
    std::sort(extents.begin(), extents.end());
    return extents;
}

int computeStoryIndex(float z, float floor_z, float story_height, float epsilon) {
    if (story_height <= epsilon) {
        return 0;
    }
    return static_cast<int>(std::floor((z - floor_z) / story_height));
}

float computeStoryLocalZ(float z,
                         float floor_z,
                         float story_height,
                         int story_index,
                         float epsilon) {
    if (story_height <= epsilon) {
        return z - floor_z;
    }
    return z - (floor_z + static_cast<float>(story_index) * story_height);
}

template <typename TreeNode>
int descendTree(const std::vector<TreeNode>& tree, const std::vector<float>& features) {
    if (tree.empty()) {
        return -1;
    }

    int index = 0;
    for (int guard = 0; guard < static_cast<int>(tree.size()); ++guard) {
        const TreeNode& node = tree[static_cast<std::size_t>(index)];
        if (node.left < 0 || node.right < 0 || node.feature < 0) {
            return index;
        }
        const float value = node.feature < static_cast<int>(features.size())
                                ? features[static_cast<std::size_t>(node.feature)]
                                : 0.0f;
        index = value <= node.threshold ? node.left : node.right;
        if (index < 0 || index >= static_cast<int>(tree.size())) {
            return -1;
        }
    }
    return -1;
}

void expectToken(std::istream& in, const std::string& expected) {
    std::string token;
    if (!(in >> token) || token != expected) {
        throw std::runtime_error("Invalid RF voxel model format, expected token: " + expected);
    }
}

} // namespace

RandomForestVoxelPredictor::RandomForestVoxelPredictor(RandomForestFeatureConfig config)
    : config_(config) {
}

void RandomForestVoxelPredictor::loadFromTextModel(const std::string& filename) {
    std::ifstream in(filename);
    if (!in) {
        throw std::runtime_error("Failed to open RF voxel model: " + filename);
    }

    std::string magic;
    int version = 0;
    if (!(in >> magic >> version) || magic != "RFVOXEL_TEXT" || version != 1) {
        throw std::runtime_error("Unsupported RF voxel model: " + filename);
    }

    expectToken(in, "FEATURE_COUNT");
    int feature_count = 0;
    in >> feature_count;
    if (feature_count <= 0) {
        throw std::runtime_error("RF voxel model has no feature columns");
    }

    feature_names_.clear();
    feature_names_.reserve(static_cast<std::size_t>(feature_count));
    for (int i = 0; i < feature_count; ++i) {
        expectToken(in, "FEATURE");
        std::string feature;
        in >> feature;
        feature_names_.push_back(feature);
    }

    expectToken(in, "CLASS_COUNT");
    int class_count = 0;
    in >> class_count;
    if (class_count != 3) {
        throw std::runtime_error("RF voxel model must contain exactly 3 classes");
    }

    expectToken(in, "CLASSES");
    int c0 = -1;
    int c1 = -1;
    int c2 = -1;
    in >> c0 >> c1 >> c2;
    if (c0 != 0 || c1 != 1 || c2 != 2) {
        throw std::runtime_error("RF voxel model class order must be 0 1 2");
    }

    expectToken(in, "CLASSIFIER_TREES");
    int classifier_tree_count = 0;
    in >> classifier_tree_count;
    classifier_trees_.clear();
    classifier_trees_.reserve(static_cast<std::size_t>(std::max(0, classifier_tree_count)));
    for (int tree_index = 0; tree_index < classifier_tree_count; ++tree_index) {
        expectToken(in, "TREE");
        int node_count = 0;
        in >> node_count;
        ClassTree tree;
        tree.reserve(static_cast<std::size_t>(std::max(0, node_count)));
        for (int node_index = 0; node_index < node_count; ++node_index) {
            expectToken(in, "NODE");
            ClassTreeNode node;
            in >> node.left >> node.right >> node.feature >> node.threshold
               >> node.values[0] >> node.values[1] >> node.values[2];
            tree.push_back(node);
        }
        classifier_trees_.push_back(std::move(tree));
    }

    expectToken(in, "REGRESSOR_TREES");
    int regressor_tree_count = 0;
    in >> regressor_tree_count;
    regressor_trees_.clear();
    regressor_trees_.reserve(static_cast<std::size_t>(std::max(0, regressor_tree_count)));
    for (int tree_index = 0; tree_index < regressor_tree_count; ++tree_index) {
        expectToken(in, "TREE");
        int node_count = 0;
        in >> node_count;
        RegressionTree tree;
        tree.reserve(static_cast<std::size_t>(std::max(0, node_count)));
        for (int node_index = 0; node_index < node_count; ++node_index) {
            expectToken(in, "NODE");
            RegressionTreeNode node;
            in >> node.left >> node.right >> node.feature >> node.threshold >> node.value;
            tree.push_back(node);
        }
        regressor_trees_.push_back(std::move(tree));
    }

    std::string end_token;
    if (!(in >> end_token) || end_token != "END") {
        throw std::runtime_error("RF voxel model is truncated or missing END token");
    }
}

bool RandomForestVoxelPredictor::loaded() const {
    return !feature_names_.empty() && !classifier_trees_.empty() && !regressor_trees_.empty();
}

MLResult RandomForestVoxelPredictor::predict(const OctreeNode& node) const {
    if (!loaded()) {
        throw std::runtime_error("RandomForestVoxelPredictor used before loading a model");
    }

    const std::vector<float> features = extractFeatures(node);
    const std::array<float, 3> scores = predictClassScores(features);
    int label = 0;
    if (scores[1] > scores[label]) {
        label = 1;
    }
    if (scores[2] > scores[label]) {
        label = 2;
    }

    MLResult result;
    result.label = static_cast<VoxelLabel>(label);
    result.obstacle_probability = clampFloat(predictProbability(features), 0.0f, 1.0f);
    result.room_id = node.room_id;
    result.is_cross_floor = node.is_cross_floor || result.label == VoxelLabel::Stair;
    return result;
}

OctreeManager::MLPredictor RandomForestVoxelPredictor::asMLPredictor() const {
    auto self = std::make_shared<RandomForestVoxelPredictor>(*this);
    return [self](const OctreeNode& node) {
        return self->predict(node);
    };
}

std::size_t RandomForestVoxelPredictor::classifierTreeCount() const {
    return classifier_trees_.size();
}

std::size_t RandomForestVoxelPredictor::regressorTreeCount() const {
    return regressor_trees_.size();
}

std::vector<float> RandomForestVoxelPredictor::extractFeatures(const OctreeNode& node) const {
    const float dx = std::max(0.0f, node.bounds.max.x - node.bounds.min.x);
    const float dy = std::max(0.0f, node.bounds.max.y - node.bounds.min.y);
    const float dz = std::max(0.0f, node.bounds.max.z - node.bounds.min.z);
    const float voxel_volume = std::max(config_.epsilon, dx * dy * dz);
    const float density = static_cast<float>(node.point_count) / voxel_volume;
    const float surface_area = 2.0f * (dx * dy + dy * dz + dx * dz);
    const Point3D center = node.bounds.center();

    const int story_index = computeStoryIndex(
        center.z, config_.floor_z, config_.story_height, config_.epsilon);
    const float story_local_z = computeStoryLocalZ(
        center.z, config_.floor_z, config_.story_height, story_index, config_.epsilon);
    const float relative_height_from_floor = story_local_z - config_.floor_surface_offset;
    const float relative_distance_to_ceiling = config_.ceiling_offset - story_local_z;
    const float height_ratio = clampFloat(
        safeDivide(story_local_z - config_.floor_surface_offset,
                   config_.ceiling_offset - config_.floor_surface_offset),
        0.0f,
        1.0f);
    const bool is_near_floor = std::abs(relative_height_from_floor) <= config_.near_floor_z;
    const bool is_near_ceiling = std::abs(relative_distance_to_ceiling) <= config_.near_ceiling_z;

    const float nx = node.avg_normal.x;
    const float ny = node.avg_normal.y;
    const float nz = node.avg_normal.z;
    const float normal_magnitude = std::sqrt(nx * nx + ny * ny + nz * nz);
    const float verticality = std::abs(nz);
    const float horizontality = std::sqrt(nx * nx + ny * ny);
    const float slope_angle_rad = std::acos(clampFloat(verticality, 0.0f, 1.0f));
    const float roughness_proxy = clampFloat(1.0f - normal_magnitude, 0.0f, 1.0f);

    const std::array<float, 3> extents = sortedExtents(dx, dy, dz);
    const float min_extent = extents[0];
    const float mid_extent = extents[1];
    const float max_extent = std::max(extents[2], config_.epsilon);
    const float aabb_flatness = safeDivide(min_extent, max_extent);
    const float aabb_linearity = safeDivide(max_extent, std::max(mid_extent, config_.epsilon));
    const float aabb_anisotropy = safeDivide(max_extent - min_extent, max_extent);
    const float xy_area = dx * dy;
    const float z_extent = dz;
    const float normalized_depth = safeDivide(static_cast<float>(node.depth), 32.0f);
    const float morton_low_8bits = static_cast<float>(node.morton_code & 0xffULL);

    const std::unordered_map<std::string, float> values = {
        {"min_x", node.bounds.min.x},
        {"min_y", node.bounds.min.y},
        {"min_z", node.bounds.min.z},
        {"max_x", node.bounds.max.x},
        {"max_y", node.bounds.max.y},
        {"max_z", node.bounds.max.z},
        {"num_points", static_cast<float>(node.point_count)},
        {"voxel_volume", voxel_volume},
        {"density", density},
        {"log_density", std::log1p(density)},
        {"surface_density", safeDivide(static_cast<float>(node.point_count), surface_area)},
        {"voxel_size", dx},
        {"depth", static_cast<float>(node.depth)},
        {"normalized_depth", normalized_depth},
        {"center_x", center.x},
        {"center_y", center.y},
        {"center_z", center.z},
        {"story_index", static_cast<float>(story_index)},
        {"story_local_z", story_local_z},
        {"relative_height_from_floor", relative_height_from_floor},
        {"relative_distance_to_ceiling", relative_distance_to_ceiling},
        {"height_ratio", height_ratio},
        {"is_near_floor", is_near_floor ? 1.0f : 0.0f},
        {"is_near_ceiling", is_near_ceiling ? 1.0f : 0.0f},
        {"avg_normal_x", nx},
        {"avg_normal_y", ny},
        {"avg_normal_z", nz},
        {"normal_magnitude", normal_magnitude},
        {"verticality", verticality},
        {"horizontality", horizontality},
        {"slope_angle_rad", slope_angle_rad},
        {"roughness_proxy", roughness_proxy},
        {"eigenvalue_0", node.covariance_eigenvalues[0]},
        {"eigenvalue_1", node.covariance_eigenvalues[1]},
        {"eigenvalue_2", node.covariance_eigenvalues[2]},
        {"pca_linearity", node.linearity},
        {"pca_flatness", node.flatness},
        {"pca_roughness", node.roughness},
        {"pca_curvature", node.curvature},
        {"aabb_flatness", aabb_flatness},
        {"aabb_linearity", aabb_linearity},
        {"aabb_anisotropy", aabb_anisotropy},
        {"xy_area", xy_area},
        {"z_extent", z_extent},
        {"morton_low_8bits", morton_low_8bits},
    };

    std::vector<float> features;
    features.reserve(feature_names_.size());
    for (const std::string& name : feature_names_) {
        const auto found = values.find(name);
        features.push_back(found != values.end() ? found->second : 0.0f);
    }
    return features;
}

std::array<float, 3> RandomForestVoxelPredictor::predictClassScores(
    const std::vector<float>& features) const {
    std::array<float, 3> scores = {0.0f, 0.0f, 0.0f};
    for (const ClassTree& tree : classifier_trees_) {
        const int leaf = descendTree(tree, features);
        if (leaf < 0) {
            continue;
        }
        const ClassTreeNode& node = tree[static_cast<std::size_t>(leaf)];
        scores[0] += node.values[0];
        scores[1] += node.values[1];
        scores[2] += node.values[2];
    }
    return scores;
}

float RandomForestVoxelPredictor::predictProbability(const std::vector<float>& features) const {
    if (regressor_trees_.empty()) {
        return 0.0f;
    }

    float sum = 0.0f;
    int count = 0;
    for (const RegressionTree& tree : regressor_trees_) {
        const int leaf = descendTree(tree, features);
        if (leaf < 0) {
            continue;
        }
        sum += tree[static_cast<std::size_t>(leaf)].value;
        ++count;
    }
    return count > 0 ? sum / static_cast<float>(count) : 0.0f;
}

} // namespace navigation
