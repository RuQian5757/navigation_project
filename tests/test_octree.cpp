#include "octree_manager.h"

#include <cassert>
#include <iostream>

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

int main() {
    testAdaptiveSubdivision();
    testLabelPropagation();
    testRoomClosureConstraint();
    testCrossFloorConnectivity();
    testTraversalCost();
    testSemanticPointCloudInitialization();
    testDynamicIncrementalUpdateWithML();
    std::cout << "All Octree tests passed." << std::endl;
    return 0;
}
