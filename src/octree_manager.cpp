#include "octree_manager.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <queue>

namespace navigation {

static float clampFloat(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

OctreeManager::OctreeManager(int max_depth)
    : max_depth_(max_depth) {
}

void OctreeManager::initialize(const std::vector<Point3D>& points) {
    buildLinearOctree(points);
    for (int i = 0, n = static_cast<int>(nodes_.size()); i < n; ++i) {
        updateNeighborLinks(i);
    }
}

void OctreeManager::updateFromPointCloud(const std::vector<Point3D>& updated_points) {
    // 首先檢查是否需要重新細分根節點
    if (nodes_.size() == 1 && nodes_[0].leaf) {
        // 如果只有根節點且是葉節點，根據新點雲重新構建
        buildLinearOctree(updated_points);
        return;
    }

    std::vector<int> touched;
    touched.reserve(updated_points.size());

    for (const auto& point : updated_points) {
        auto leaf_index = findLeafIndexByPoint(point);
        if (!leaf_index.has_value()) {
            continue;
        }

        int index = leaf_index.value();
        OctreeNode& node = nodes_[index];
        node.point_count += 1;
        node.node_volume = node.bounds.volume();
        
        // 檢查是否需要細分此節點
        if (node.leaf && node.depth < max_depth_) {
            float max_extent = std::max({node.bounds.width(), node.bounds.depth(), node.bounds.height()});
            float density = (node.node_volume > 0.0f) ? (node.point_count / node.node_volume) : 0.0f;
            
            if (shouldSubdivide(node.point_count, max_extent, density, node.depth)) {
                // 需要細分，但在動態更新中這比較複雜
                // 暫時只標記為已更新
            }
        }
        
        touched.push_back(index);
    }

    // 移除重複
    std::sort(touched.begin(), touched.end());
    touched.erase(std::unique(touched.begin(), touched.end()), touched.end());

    for (int index : touched) {
        floodFillLabelPropagation(index);
        updateNeighborLinks(index);
    }
}

std::array<int, NEIGHBOR_COUNT> OctreeManager::getOrthogonalNeighbors(int node_index) const {
    assert(node_index >= 0 && node_index < static_cast<int>(nodes_.size()));
    return nodes_[node_index].orthogonal_neighbors;
}

std::optional<int> OctreeManager::findLeafIndexByPoint(const Point3D& point) const {
    if (nodes_.empty()) {
        return std::nullopt;
    }
    return findContainingLeaf(0, point);
}

bool OctreeManager::isCrossFloorConnected(int node_index) const {
    if (node_index < 0 || node_index >= static_cast<int>(nodes_.size())) {
        return false;
    }
    const OctreeNode& node = nodes_[node_index];
    return node.is_cross_floor || node.label == VoxelLabel::Stair;
}

float OctreeManager::computeTraversalCost(int node_index, bool allow_cross_floor) const {
    assert(node_index >= 0 && node_index < static_cast<int>(nodes_.size()));
    const OctreeNode& node = nodes_[node_index];

    if (node.label == VoxelLabel::Obstacle) {
        return 1e6f;
    }

    float cost = 1.0f;
    cost += node.obstacle_probability * 8.0f;
    cost += (node.depth * 0.05f);
    if (node.label == VoxelLabel::Stair) {
        cost += 2.0f;
    }
    if (node.is_cross_floor && !allow_cross_floor) {
        cost += 5.0f;
    }
    return cost;
}

void OctreeManager::assignLeafLabel(int node_index, VoxelLabel label, float obstacle_probability, int room_id, bool is_cross_floor) {
    assert(node_index >= 0 && node_index < static_cast<int>(nodes_.size()));
    OctreeNode& node = nodes_[node_index];
    node.label = label;
    node.obstacle_probability = clampFloat(obstacle_probability, 0.0f, 1.0f);
    node.room_id = room_id;
    node.is_cross_floor = is_cross_floor;
    if (node.leaf) {
        floodFillLabelPropagation(node_index);
    }
}

int OctreeManager::getLeafCount() const {
    int leaf_count = 0;
    for (const auto& node : nodes_) {
        if (node.leaf) {
            ++leaf_count;
        }
    }
    return leaf_count;
}

int OctreeManager::getNodeCount() const {
    return static_cast<int>(nodes_.size());
}

const OctreeNode* OctreeManager::getNode(int index) const {
    if (index < 0 || index >= static_cast<int>(nodes_.size())) {
        return nullptr;
    }
    return &nodes_[index];
}

void OctreeManager::buildLinearOctree(const std::vector<Point3D>& points) {
    nodes_.clear();
    nodes_.reserve(1024);

    // 計算點雲的實際邊界
    if (points.empty()) {
        // 如果沒有點，使用默認邊界
        OctreeNode root;
        root.bounds = BBox{{0.0f, 0.0f, 0.0f}, {20.0f, 20.0f, 5.0f}};
        root.depth = 0;
        root.node_volume = root.bounds.volume();
        root.point_count = 0;
        root.morton_code = computeMortonCode(root.bounds.center(), 0);
        nodes_.push_back(root);
        return;
    }

    // 找出點雲邊界
    float min_x = points[0].x, max_x = points[0].x;
    float min_y = points[0].y, max_y = points[0].y;
    float min_z = points[0].z, max_z = points[0].z;

    for (const auto& pt : points) {
        min_x = std::min(min_x, pt.x); max_x = std::max(max_x, pt.x);
        min_y = std::min(min_y, pt.y); max_y = std::max(max_y, pt.y);
        min_z = std::min(min_z, pt.z); max_z = std::max(max_z, pt.z);
    }

    // 增加邊界 padding（每邊 0.5m）確保包含所有點
    float padding = 0.5f;
    OctreeNode root;
    root.bounds = BBox{
        {min_x - padding, min_y - padding, min_z - padding},
        {max_x + padding, max_y + padding, max_z + padding}
    };
    root.depth = 0;
    root.node_volume = root.bounds.volume();
    root.point_count = static_cast<int>(points.size());
    root.morton_code = computeMortonCode(root.bounds.center(), 0);
    nodes_.push_back(root);

    subdivideNode(0, points);
}

void OctreeManager::subdivideNode(int node_index, const std::vector<Point3D>& points) {
    OctreeNode& node = nodes_[node_index];
    node.node_volume = node.bounds.volume();

    if (node.depth >= max_depth_) {
        node.leaf = true;
        return;
    }

    float max_extent = std::max({node.bounds.width(), node.bounds.depth(), node.bounds.height()});
    float density = 0.0f;
    if (node.node_volume > 0.0f) {
        density = node.point_count / node.node_volume;
    }

    if (!shouldSubdivide(node.point_count, max_extent, density, node.depth)) {
        node.leaf = true;
        return;
    }

    node.leaf = false;
    std::array<std::vector<Point3D>, 8> child_points;
    const Point3D center = node.bounds.center();

    for (const auto& pt : points) {
        if (!node.bounds.contains(pt)) {
            continue;
        }
        int octant = getChildOctant(pt, node.bounds);
        child_points[octant].push_back(pt);
    }

    for (int child = 0; child < 8; ++child) {
        if (child_points[child].empty()) {
            node.children[child] = -1;
            continue;
        }

        OctreeNode child_node;
        child_node.depth = node.depth + 1;
        child_node.bounds.min = node.bounds.min;
        child_node.bounds.max = node.bounds.max;
        const Point3D child_center = computeOctantCenter(node.bounds, child);
        if (child & 1) {
            child_node.bounds.min.x = center.x;
        } else {
            child_node.bounds.max.x = center.x;
        }
        if (child & 2) {
            child_node.bounds.min.y = center.y;
        } else {
            child_node.bounds.max.y = center.y;
        }
        if (child & 4) {
            child_node.bounds.min.z = center.z;
        } else {
            child_node.bounds.max.z = center.z;
        }
        child_node.point_count = static_cast<int>(child_points[child].size());
        child_node.node_volume = child_node.bounds.volume();
        child_node.morton_code = computeMortonCode(child_center, child_node.depth);
        child_node.leaf = true;

        int child_index = static_cast<int>(nodes_.size());
        nodes_.push_back(child_node);
        node.children[child] = child_index;

        subdivideNode(child_index, child_points[child]);
    }

    updateNeighborLinks(node_index);
}

bool OctreeManager::shouldSubdivide(int point_count, float voxel_size, float density, int depth) const {
    if (point_count < 5) {
        return false;
    }
    if (depth >= max_depth_) {
        return false;
    }
    if (voxel_size <= 0.05f) {
        return false;
    }
    if (density > 250.0f) {
        return true;
    }
    if (point_count > 30) {
        return true;
    }
    if (voxel_size > 0.3f && point_count > 15) {
        return true;
    }
    if (voxel_size < 0.12f && point_count > 8) {
        return true;
    }
    return false;
}

void OctreeManager::updateNeighborLinks(int node_index) {
    OctreeNode& node = nodes_[node_index];
    for (int dir = 0; dir < NEIGHBOR_COUNT; ++dir) {
        auto neighbor = findNeighborByDirection(node_index, static_cast<NeighborDirection>(dir));
        node.orthogonal_neighbors[dir] = neighbor.has_value() ? neighbor.value() : -1;
    }
}

int OctreeManager::findContainingLeaf(int node_index, const Point3D& point) const {
    const OctreeNode& node = nodes_[node_index];
    if (!node.bounds.contains(point)) {
        return -1;
    }
    if (node.leaf) {
        return node_index;
    }
    int octant = getChildOctant(point, node.bounds);
    int child_index = node.children[octant];
    if (child_index < 0) {
        return node_index;
    }
    return findContainingLeaf(child_index, point);
}

std::optional<int> OctreeManager::findNeighborByDirection(int node_index, NeighborDirection dir) const {
    const OctreeNode& node = nodes_[node_index];
    const Point3D center = node.bounds.center();
    Point3D target = center;
    const float step = std::max({node.bounds.width(), node.bounds.depth(), node.bounds.height()}) * 0.51f;

    switch (dir) {
        case POS_X: target.x += step; break;
        case NEG_X: target.x -= step; break;
        case POS_Y: target.y += step; break;
        case NEG_Y: target.y -= step; break;
        case POS_Z: target.z += step; break;
        case NEG_Z: target.z -= step; break;
        default: break;
    }

    auto neighbor = findLeafIndexByPoint(target);
    if (!neighbor.has_value()) {
        return std::nullopt;
    }
    const OctreeNode& neighbor_node = nodes_[neighbor.value()];
    if (neighbor_node.room_id != node.room_id) {
        return std::nullopt;
    }
    return neighbor;
}

void OctreeManager::floodFillLabelPropagation(int start_node) {
    if (start_node < 0 || start_node >= static_cast<int>(nodes_.size())) {
        return;
    }

    std::queue<int> queue;
    std::vector<bool> visited(nodes_.size(), false);
    queue.push(start_node);
    visited[start_node] = true;

    const OctreeNode& start = nodes_[start_node];
    while (!queue.empty()) {
        int current = queue.front();
        queue.pop();
        OctreeNode& node = nodes_[current];

        for (int dir = 0; dir < NEIGHBOR_COUNT; ++dir) {
            int neighbor_index = node.orthogonal_neighbors[dir];
            if (neighbor_index < 0 || neighbor_index >= static_cast<int>(nodes_.size())) {
                continue;
            }
            if (visited[neighbor_index]) {
                continue;
            }
            OctreeNode& neighbor = nodes_[neighbor_index];
            if (neighbor.room_id != start.room_id) {
                continue;
            }
            if (start.label == VoxelLabel::Obstacle && neighbor.obstacle_probability < 0.7f) {
                neighbor.label = VoxelLabel::Obstacle;
                neighbor.obstacle_probability = std::max(neighbor.obstacle_probability, 0.7f);
            }
            if (neighbor.label == VoxelLabel::Free && start.label == VoxelLabel::Stair) {
                neighbor.label = VoxelLabel::Stair;
            }
            visited[neighbor_index] = true;
            queue.push(neighbor_index);
        }
    }
}

int OctreeManager::getChildOctant(const Point3D& point, const BBox& bounds) const {
    const Point3D center = bounds.center();
    int octant = 0;
    if (point.x >= center.x) {
        octant |= 1;
    }
    if (point.y >= center.y) {
        octant |= 2;
    }
    if (point.z >= center.z) {
        octant |= 4;
    }
    return octant;
}

Point3D OctreeManager::computeOctantCenter(const BBox& bounds, int child_index) const {
    const Point3D center = bounds.center();
    Point3D result = center;
    if (!(child_index & 1)) {
        result.x = 0.5f * (bounds.min.x + center.x);
    } else {
        result.x = 0.5f * (center.x + bounds.max.x);
    }
    if (!(child_index & 2)) {
        result.y = 0.5f * (bounds.min.y + center.y);
    } else {
        result.y = 0.5f * (center.y + bounds.max.y);
    }
    if (!(child_index & 4)) {
        result.z = 0.5f * (bounds.min.z + center.z);
    } else {
        result.z = 0.5f * (center.z + bounds.max.z);
    }
    return result;
}

uint64_t OctreeManager::computeMortonCode(const Point3D& point, int depth) const {
    const float scale_x = static_cast<float>((1u << max_depth_) - 1) / 20.0f;
    const float scale_y = static_cast<float>((1u << max_depth_) - 1) / 20.0f;
    const float scale_z = static_cast<float>((1u << max_depth_) - 1) / 5.0f;

    uint32_t x = static_cast<uint32_t>(clampFloat(point.x, 0.0f, 20.0f) * scale_x);
    uint32_t y = static_cast<uint32_t>(clampFloat(point.y, 0.0f, 20.0f) * scale_y);
    uint32_t z = static_cast<uint32_t>(clampFloat(point.z, 0.0f, 5.0f) * scale_z);
    return interleaveBits(x, y, z);
}

uint64_t OctreeManager::interleaveBits(uint32_t x, uint32_t y, uint32_t z) const {
    auto spread = [](uint64_t v) {
        v = (v | (v << 32)) & 0x1f00000000ffffULL;
        v = (v | (v << 16)) & 0x1f0000ff0000ffULL;
        v = (v | (v << 8)) & 0x100f00f00f00f00fULL;
        v = (v | (v << 4)) & 0x10c30c30c30c30c3ULL;
        v = (v | (v << 2)) & 0x1249249249249249ULL;
        return v;
    };
    return spread(x) | (spread(y) << 1) | (spread(z) << 2);
}

} // namespace navigation
