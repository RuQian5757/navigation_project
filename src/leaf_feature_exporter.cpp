#include "leaf_feature_exporter.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <stdexcept>

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

void writeHeader(std::ofstream& out) {
    out
        << "num_points,"
        << "voxel_volume,"
        << "density,"
        << "log_density,"
        << "surface_density,"
        << "voxel_size,"
        << "depth,"
        << "normalized_depth,"
        << "center_x,"
        << "center_y,"
        << "center_z,"
        << "relative_height_from_floor,"
        << "relative_distance_to_ceiling,"
        << "height_ratio,"
        << "is_near_floor,"
        << "is_near_ceiling,"
        << "avg_normal_x,"
        << "avg_normal_y,"
        << "avg_normal_z,"
        << "normal_magnitude,"
        << "verticality,"
        << "horizontality,"
        << "slope_angle_rad,"
        << "roughness_proxy,"
        << "eigenvalue_0,"
        << "eigenvalue_1,"
        << "eigenvalue_2,"
        << "pca_linearity,"
        << "pca_flatness,"
        << "pca_roughness,"
        << "pca_curvature,"
        << "aabb_flatness,"
        << "aabb_linearity,"
        << "aabb_anisotropy,"
        << "xy_area,"
        << "z_extent,"
        << "morton_low_8bits,"
        << "label,"
        << "obstacle_probability\n";
}

void writeLeafFeatures(std::ofstream& out,
                       const LeafNode& leaf,
                       int label,
                       float obstacle_probability,
                       const FeatureExtractionConfig& config) {
    const float dx = std::max(0.0f, leaf.max[0] - leaf.min[0]);
    const float dy = std::max(0.0f, leaf.max[1] - leaf.min[1]);
    const float dz = std::max(0.0f, leaf.max[2] - leaf.min[2]);
    const float voxel_volume = std::max(config.epsilon, dx * dy * dz);
    const float density = static_cast<float>(leaf.num_points) / voxel_volume;
    const float voxel_size = leaf.size > config.epsilon ? leaf.size : dx;
    const float surface_area = 2.0f * (dx * dy + dy * dz + dx * dz);

    const float nx = leaf.avg_normal[0];
    const float ny = leaf.avg_normal[1];
    const float nz = leaf.avg_normal[2];
    const float normal_magnitude = std::sqrt(nx * nx + ny * ny + nz * nz);
    const float verticality = std::abs(nz);
    const float horizontality = std::sqrt(nx * nx + ny * ny);
    const float slope_angle_rad = std::acos(clampFloat(verticality, 0.0f, 1.0f));
    const float roughness_proxy = clampFloat(1.0f - normal_magnitude, 0.0f, 1.0f);

    const float center_z = leaf.center[2];
    const float relative_height_from_floor = center_z - config.floor_z;
    const float relative_distance_to_ceiling = config.ceiling_z - center_z;
    const float height_ratio = clampFloat(
        safeDivide(center_z - config.floor_z, config.ceiling_z - config.floor_z), 0.0f, 1.0f);
    const int is_near_floor = center_z < config.near_floor_z ? 1 : 0;
    const int is_near_ceiling = center_z > config.near_ceiling_z ? 1 : 0;

    const std::array<float, 3> extents = sortedExtents(dx, dy, dz);
    const float min_extent = extents[0];
    const float mid_extent = extents[1];
    const float max_extent = std::max(extents[2], config.epsilon);
    const float aabb_flatness = safeDivide(min_extent, max_extent);
    const float aabb_linearity = safeDivide(max_extent, std::max(mid_extent, config.epsilon));
    const float aabb_anisotropy = safeDivide(max_extent - min_extent, max_extent);
    const float xy_area = dx * dy;
    const float z_extent = dz;
    const float normalized_depth = safeDivide(static_cast<float>(leaf.depth), 32.0f);
    const uint32_t morton_low_8bits = leaf.morton_code & 0xffU;

    out
        << leaf.num_points << ','
        << voxel_volume << ','
        << density << ','
        << std::log1p(density) << ','
        << safeDivide(static_cast<float>(leaf.num_points), surface_area) << ','
        << voxel_size << ','
        << static_cast<int>(leaf.depth) << ','
        << normalized_depth << ','
        << leaf.center[0] << ','
        << leaf.center[1] << ','
        << leaf.center[2] << ','
        << relative_height_from_floor << ','
        << relative_distance_to_ceiling << ','
        << height_ratio << ','
        << is_near_floor << ','
        << is_near_ceiling << ','
        << nx << ','
        << ny << ','
        << nz << ','
        << normal_magnitude << ','
        << verticality << ','
        << horizontality << ','
        << slope_angle_rad << ','
        << roughness_proxy << ','
        << leaf.covariance_eigenvalues[0] << ','
        << leaf.covariance_eigenvalues[1] << ','
        << leaf.covariance_eigenvalues[2] << ','
        << leaf.pca_linearity << ','
        << leaf.pca_flatness << ','
        << leaf.pca_roughness << ','
        << leaf.pca_curvature << ','
        << aabb_flatness << ','
        << aabb_linearity << ','
        << aabb_anisotropy << ','
        << xy_area << ','
        << z_extent << ','
        << morton_low_8bits << ','
        << label << ','
        << clampFloat(obstacle_probability, 0.0f, 1.0f)
        << '\n';
}

LeafNode toLeafNode(const OctreeNode& node) {
    LeafNode leaf;
    leaf.min[0] = node.bounds.min.x;
    leaf.min[1] = node.bounds.min.y;
    leaf.min[2] = node.bounds.min.z;
    leaf.max[0] = node.bounds.max.x;
    leaf.max[1] = node.bounds.max.y;
    leaf.max[2] = node.bounds.max.z;
    leaf.morton_code = static_cast<uint32_t>(node.morton_code & 0xffffffffULL);
    leaf.depth = static_cast<uint8_t>(std::max(0, std::min(node.depth, 255)));
    leaf.num_points = node.point_count;
    leaf.avg_normal[0] = node.avg_normal.x;
    leaf.avg_normal[1] = node.avg_normal.y;
    leaf.avg_normal[2] = node.avg_normal.z;
    leaf.covariance_eigenvalues[0] = node.covariance_eigenvalues[0];
    leaf.covariance_eigenvalues[1] = node.covariance_eigenvalues[1];
    leaf.covariance_eigenvalues[2] = node.covariance_eigenvalues[2];
    leaf.pca_linearity = node.linearity;
    leaf.pca_flatness = node.flatness;
    leaf.pca_roughness = node.roughness;
    leaf.pca_curvature = node.curvature;
    leaf.center[0] = node.bounds.center().x;
    leaf.center[1] = node.bounds.center().y;
    leaf.center[2] = node.bounds.center().z;
    leaf.size = node.bounds.width();
    leaf.label = static_cast<int>(node.label);
    leaf.obstacle_probability = node.obstacle_probability;
    return leaf;
}

} // namespace

void exportLeafFeaturesToCSV(const std::vector<LeafNode>& leaves,
                             const std::string& filename) {
    exportLeafFeaturesToCSV(leaves, filename, FeatureExtractionConfig{});
}

void exportLeafFeaturesToCSV(const std::vector<LeafNode>& leaves,
                             const std::string& filename,
                             const FeatureExtractionConfig& config) {
    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Failed to open CSV file: " + filename);
    }

    out << std::fixed << std::setprecision(6);
    writeHeader(out);
    for (const LeafNode& leaf : leaves) {
        writeLeafFeatures(out, leaf, leaf.label, leaf.obstacle_probability, config);
    }
}

void exportLeafFeaturesToCSV(const std::vector<LeafNode>& leaves,
                             const std::vector<int>& labels,
                             const std::vector<float>& obstacle_probabilities,
                             const std::string& filename) {
    if (labels.size() != leaves.size() || obstacle_probabilities.size() != leaves.size()) {
        throw std::invalid_argument("labels and obstacle_probabilities must match leaves.size()");
    }

    std::ofstream out(filename);
    if (!out) {
        throw std::runtime_error("Failed to open CSV file: " + filename);
    }

    out << std::fixed << std::setprecision(6);
    writeHeader(out);
    const FeatureExtractionConfig config;
    for (std::size_t i = 0; i < leaves.size(); ++i) {
        writeLeafFeatures(out, leaves[i], labels[i], obstacle_probabilities[i], config);
    }
}

void exportOctreeLeafFeaturesToCSV(const std::vector<OctreeNode>& nodes,
                                   const std::string& filename) {
    std::vector<LeafNode> leaves;
    leaves.reserve(nodes.size());
    for (const OctreeNode& node : nodes) {
        if (node.leaf) {
            leaves.push_back(toLeafNode(node));
        }
    }
    exportLeafFeaturesToCSV(leaves, filename);
}

} // namespace navigation
