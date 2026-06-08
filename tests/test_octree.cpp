#include "octree_manager.h"
#include "leaf_feature_exporter.h"
#include "astar_planner.h"

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

using namespace navigation;

void testAdaptiveSubdivision() {
    std::vector<Point3D> points;
    for (int x = 0; x < 14; ++x) {
        for (int y = 0; y < 14; ++y) {
            points.push_back({x * 0.5f, y * 0.5f, 1.0f});
        }
    }

    OctreeManager octree;
    octree.initialize(points);
    assert(octree.getNodeCount() > 1);
    assert(octree.getLeafCount() > 1);
}

void testLabelPropagation() {
    std::vector<Point3D> points;
    points.push_back({1.0f, 1.0f, 1.0f});
    points.push_back({1.5f, 1.5f, 1.0f});
    points.push_back({2.0f, 2.0f, 1.0f});

    OctreeManager octree;
    octree.initialize(points);
    assert(octree.getLeafCount() > 0);

    auto leaf_opt = octree.findLeafIndexByPoint({1.0f, 1.0f, 1.0f});
    assert(leaf_opt.has_value());
    int leaf = leaf_opt.value();
    octree.assignLeafLabel(leaf, VoxelLabel::Obstacle, 0.9f, 1, false);
    assert(octree.getNode(leaf)->label == VoxelLabel::Obstacle);
}

void testRoomClosureConstraint() {
    std::vector<Point3D> points;
    points.push_back({1.0f, 1.0f, 1.0f});
    points.push_back({1.4f, 1.0f, 1.0f});
    points.push_back({1.0f, 1.4f, 1.0f});

    OctreeManager octree;
    octree.initialize(points);
    auto first_leaf = octree.findLeafIndexByPoint({1.0f, 1.0f, 1.0f});
    auto second_leaf = octree.findLeafIndexByPoint({1.4f, 1.0f, 1.0f});
    assert(first_leaf.has_value() && second_leaf.has_value());

    const OctreeNode* node_a = octree.getNode(first_leaf.value());
    const OctreeNode* node_b = octree.getNode(second_leaf.value());
    assert(node_a != nullptr && node_b != nullptr);
    if (node_a->orthogonal_neighbors[POS_X] >= 0) {
        int neighbor = node_a->orthogonal_neighbors[POS_X];
        const OctreeNode* neighbor_node = octree.getNode(neighbor);
        assert(neighbor_node != nullptr);
    }
}

void testCrossFloorConnectivity() {
    std::vector<Point3D> points;
    points.push_back({5.0f, 5.0f, 0.5f});
    points.push_back({5.0f, 5.0f, 4.0f});

    OctreeManager octree;
    octree.initialize(points);

    auto lower_leaf = octree.findLeafIndexByPoint({5.0f, 5.0f, 0.5f});
    assert(lower_leaf.has_value());
    octree.assignLeafLabel(lower_leaf.value(), VoxelLabel::Stair, 0.1f, 1, true);
    assert(octree.isCrossFloorConnected(lower_leaf.value()));
}

void testTraversalCost() {
    std::vector<Point3D> points;
    points.push_back({2.0f, 2.0f, 1.0f});
    OctreeManager octree;
    octree.initialize(points);
    auto leaf = octree.findLeafIndexByPoint({2.0f, 2.0f, 1.0f});
    assert(leaf.has_value());
    float free_cost = octree.computeTraversalCost(leaf.value());
    octree.assignLeafLabel(leaf.value(), VoxelLabel::Obstacle, 0.85f, 1, false);
    float obstacle_cost = octree.computeTraversalCost(leaf.value());
    assert(obstacle_cost > free_cost);
}

void testSemanticPointCloudInitialization() {
    std::vector<PointCloudSample> samples;
    for (int i = 0; i < 24; ++i) {
        PointCloudSample sample;
        sample.point = {0.05f * static_cast<float>(i), 0.0f, 0.5f};
        sample.room_id = 3;
        sample.label = VoxelLabel::Stair;
        sample.obstacle_probability = 0.15f;
        sample.has_semantics = true;
        sample.is_cross_floor = true;
        samples.push_back(sample);
    }

    OctreeManager octree;
    octree.initialize(samples);
    auto leaf = octree.findLeafIndexByPoint(samples.front().point);
    assert(leaf.has_value());
    const OctreeNode* node = octree.getNode(leaf.value());
    assert(node != nullptr);
    assert(node->label == VoxelLabel::Stair);
    assert(node->room_id == 3);
    assert(octree.isCrossFloorConnected(leaf.value()));
}

void testDynamicIncrementalUpdateWithML() {
    std::vector<Point3D> points;
    for (int i = 0; i < 40; ++i) {
        points.push_back({0.1f * static_cast<float>(i), 0.0f, 1.0f});
    }

    OctreeManager octree;
    octree.setMLPredictor([](const OctreeNode& node) {
        MLResult result;
        result.room_id = 7;
        result.label = node.point_count > 2 ? VoxelLabel::Obstacle : VoxelLabel::Free;
        result.obstacle_probability = node.point_count > 2 ? 0.85f : 0.05f;
        return result;
    });
    octree.initialize(points);

    auto leaf = octree.findLeafIndexByPoint({1.0f, 0.0f, 1.0f});
    assert(leaf.has_value());
    PointCloudSample dynamic_sample;
    dynamic_sample.point = {1.0f, 0.0f, 1.0f};
    dynamic_sample.room_id = 7;
    dynamic_sample.label = VoxelLabel::Obstacle;
    dynamic_sample.obstacle_probability = 0.9f;
    dynamic_sample.has_semantics = true;
    octree.updateFromPointCloud(std::vector<PointCloudSample>{dynamic_sample});

    const OctreeNode* node = octree.getNode(leaf.value());
    assert(node != nullptr);
    assert(node->dynamic);
    assert(node->obstacle_probability >= 0.85f);
    assert(octree.computeTraversalCost(leaf.value()) > 1.0f);
}

void testLeafFeatureCSVExport() {
    LeafNode leaf;
    leaf.min[0] = 0.0f;
    leaf.min[1] = 0.0f;
    leaf.min[2] = 0.0f;
    leaf.max[0] = 0.5f;
    leaf.max[1] = 0.5f;
    leaf.max[2] = 0.5f;
    leaf.node_index = 7;
    leaf.entity_id = 99;
    leaf.morton_code = 42;
    leaf.depth = 3;
    leaf.num_points = 12;
    leaf.avg_normal[0] = 0.0f;
    leaf.avg_normal[1] = 0.0f;
    leaf.avg_normal[2] = 1.0f;
    leaf.center[0] = 0.25f;
    leaf.center[1] = 0.25f;
    leaf.center[2] = 0.25f;
    leaf.size = 0.5f;
    leaf.label = static_cast<int>(VoxelLabel::Free);
    leaf.obstacle_probability = 0.1f;

    const std::string path = "/tmp/navigation_octree_features_test.csv";
    exportLeafFeaturesToCSV(std::vector<LeafNode>{leaf}, path);

    std::ifstream in(path);
    assert(in.good());
    std::string header;
    std::string row;
    std::getline(in, header);
    std::getline(in, row);
    assert(header.find("node_index,entity_id,morton_code,min_x") == 0);
    assert(header.find("story_index,story_local_z") != std::string::npos);
    assert(header.find("num_points,voxel_volume,density") != std::string::npos);
    assert(header.rfind("label,obstacle_probability") != std::string::npos);
    assert(!row.empty());
}

void testWeakLabelExport() {
    LeafNode free_leaf;
    free_leaf.node_index = 1;
    free_leaf.num_points = 10;
    free_leaf.min[0] = -0.5f;
    free_leaf.min[1] = -0.5f;
    free_leaf.min[2] = 0.0f;
    free_leaf.max[0] = 0.5f;
    free_leaf.max[1] = 0.5f;
    free_leaf.max[2] = 0.2f;
    free_leaf.center[2] = 0.1f;
    free_leaf.avg_normal[2] = 1.0f;
    free_leaf.pca_flatness = 0.8f;

    LeafNode obstacle_leaf = free_leaf;
    obstacle_leaf.node_index = 2;
    obstacle_leaf.center[2] = 1.0f;
    obstacle_leaf.avg_normal[0] = 1.0f;
    obstacle_leaf.avg_normal[2] = 0.0f;

    std::vector<LeafNode> leaves{free_leaf, obstacle_leaf};
    WeakLabelingConfig config;
    config.floor_surface_offset = 0.0f;
    config.ceiling_offset = 3.0f;
    config.ceiling_z = 3.0f;
    assignWeakLabels(leaves, config);

    assert(leaves[0].label == static_cast<int>(VoxelLabel::Free));
    assert(leaves[0].obstacle_probability < 0.2f);
    assert(leaves[1].label == static_cast<int>(VoxelLabel::Obstacle));
    assert(leaves[1].obstacle_probability > 0.7f);
}

void testLeafPcaGeometryStats() {
    std::vector<Point3D> points = {
        {0.0f, 0.0f, 1.0f},
        {1.0f, 0.0f, 1.0f},
        {0.0f, 1.0f, 1.0f},
        {1.0f, 1.0f, 1.0f},
        {0.5f, 0.5f, 1.0f},
    };

    OctreeConfig config;
    config.max_depth = 0;
    OctreeManager octree(config);
    octree.initialize(points);

    assert(octree.getLeafCount() == 1);
    const OctreeNode* root = octree.getNode(0);
    assert(root != nullptr);
    assert(root->leaf);
    assert(root->avg_normal.z > 0.9f);
    assert(root->flatness > 0.8f);
    assert(root->curvature < 0.05f);

    const std::string path = "/tmp/navigation_octree_node_features_test.csv";
    exportOctreeLeafFeaturesToCSV(octree.nodes(), path);
    std::ifstream in(path);
    assert(in.good());
    std::string header;
    std::getline(in, header);
    assert(header.find("eigenvalue_0") != std::string::npos);
    assert(header.find("pca_flatness") != std::string::npos);
}

int findNearestLeafByCenter(const OctreeManager& octree, const Point3D& target) {
    int best_index = -1;
    float best_distance = 1e30f;
    const auto& nodes = octree.nodes();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (!nodes[i].leaf) {
            continue;
        }
        const Point3D c = nodes[i].bounds.center();
        const float dx = c.x - target.x;
        const float dy = c.y - target.y;
        const float dz = c.z - target.z;
        const float distance = dx * dx + dy * dy + dz * dz;
        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }
    return best_index;
}

void labelAllLeaves(OctreeManager& octree, VoxelLabel label, float probability) {
    const int node_count = octree.getNodeCount();
    for (int i = 0; i < node_count; ++i) {
        const OctreeNode* node = octree.getNode(i);
        if (node != nullptr && node->leaf) {
            octree.assignLeafLabel(i, label, probability, 0, label == VoxelLabel::Stair);
        }
    }
}

void testAStarSingleFloorPath() {
    std::vector<Point3D> points;
    for (int i = 0; i <= 32; ++i) {
        points.push_back({0.25f * static_cast<float>(i), 0.0f, 1.0f});
    }

    OctreeConfig config;
    config.max_depth = 5;
    config.corridor_max_voxel = 0.5f;
    config.min_points_to_split = 1;
    OctreeManager octree(config);
    octree.initialize(points);
    labelAllLeaves(octree, VoxelLabel::Free, 0.02f);

    const int start = findNearestLeafByCenter(octree, {0.0f, 0.0f, 1.0f});
    const int goal = findNearestLeafByCenter(octree, {8.0f, 0.0f, 1.0f});
    assert(start >= 0 && goal >= 0 && start != goal);

    AStarPlannerConfig planner_config;
    planner_config.obstacle_block_probability = 0.995f;
    planner_config.probability_weight = 40.0f;
    AStarPlanner planner(octree, planner_config);
    const AStarPath path = planner.findPathByNodeIndex(start, goal);
    assert(path.success);
    assert(path.node_indices.front() == start);
    assert(path.node_indices.back() == goal);
    assert(path.waypoints.size() >= 2);
    assert(path.total_cost > 0.0f);
}

void testAStarProbabilityIncreasesCost() {
    std::vector<Point3D> points;
    for (int i = 0; i <= 32; ++i) {
        points.push_back({0.25f * static_cast<float>(i), 0.0f, 1.0f});
    }

    OctreeConfig config;
    config.max_depth = 5;
    config.corridor_max_voxel = 0.5f;
    config.min_points_to_split = 1;
    OctreeManager octree(config);
    octree.initialize(points);
    labelAllLeaves(octree, VoxelLabel::Free, 0.02f);

    const int start = findNearestLeafByCenter(octree, {0.0f, 0.0f, 1.0f});
    const int goal = findNearestLeafByCenter(octree, {8.0f, 0.0f, 1.0f});
    const int high_risk = findNearestLeafByCenter(octree, {4.0f, 0.0f, 1.0f});
    assert(start >= 0 && goal >= 0 && high_risk >= 0);

    AStarPlannerConfig planner_config;
    planner_config.obstacle_block_probability = 0.995f;
    planner_config.probability_weight = 40.0f;
    AStarPlanner planner(octree, planner_config);
    const AStarPath baseline = planner.findPathByNodeIndex(start, goal);
    assert(baseline.success);

    octree.assignLeafLabel(high_risk, VoxelLabel::Free, 0.90f, 0, false);
    const AStarPath risky = planner.findPathByNodeIndex(start, goal);
    assert(risky.success);
    assert(risky.total_cost > baseline.total_cost);
}

void testAStarCrossFloorStairPath() {
    std::vector<Point3D> points;
    for (int z = 0; z <= 40; ++z) {
        points.push_back({0.0f, 0.0f, 0.10f * static_cast<float>(z)});
    }

    OctreeConfig config;
    config.max_depth = 5;
    config.stair_max_voxel = 0.5f;
    config.corridor_max_voxel = 0.5f;
    config.min_points_to_split = 1;
    OctreeManager octree(config);
    octree.initialize(points);
    labelAllLeaves(octree, VoxelLabel::Stair, 0.18f);

    const int start = findNearestLeafByCenter(octree, {0.0f, 0.0f, 0.0f});
    const int goal = findNearestLeafByCenter(octree, {0.0f, 0.0f, 4.0f});
    assert(start >= 0 && goal >= 0 && start != goal);

    AStarPlannerConfig planner_config;
    planner_config.allow_cross_floor = true;
    AStarPlanner planner(octree, planner_config);
    const AStarPath path = planner.findPathByNodeIndex(start, goal);
    assert(path.success);
    assert(path.node_indices.front() == start);
    assert(path.node_indices.back() == goal);
    bool has_cross_floor = false;
    for (const PathWaypoint& waypoint : path.waypoints) {
        has_cross_floor = has_cross_floor || waypoint.is_cross_floor;
    }
    assert(has_cross_floor);
}

int main() {
    testAdaptiveSubdivision();
    testLabelPropagation();
    testRoomClosureConstraint();
    testCrossFloorConnectivity();
    testTraversalCost();
    testSemanticPointCloudInitialization();
    testDynamicIncrementalUpdateWithML();
    testLeafFeatureCSVExport();
    testWeakLabelExport();
    testLeafPcaGeometryStats();
    testAStarSingleFloorPath();
    testAStarProbabilityIncreasesCost();
    testAStarCrossFloorStairPath();
    std::cout << "All Octree tests passed." << std::endl;
    return 0;
}
