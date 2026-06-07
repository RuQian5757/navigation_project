#include "octree_manager.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace navigation {

namespace {

constexpr float kEpsilon = 1e-4f;
constexpr float kBlockedCost = 1e6f;

float clampFloat(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

float distanceBetween(const Point3D& a, const Point3D& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float maxExtent(const BBox& bounds) {
    return std::max({bounds.width(), bounds.depth(), bounds.height()});
}

bool nearlyTouches(float a, float b) {
    return std::abs(a - b) <= 2.0f * kEpsilon;
}

bool rangesOverlap(float a_min, float a_max, float b_min, float b_max) {
    return std::max(a_min, b_min) <= std::min(a_max, b_max) + 2.0f * kEpsilon;
}

std::vector<PointCloudSample> toSamples(const std::vector<Point3D>& points) {
    std::vector<PointCloudSample> samples;
    samples.reserve(points.size());
    for (const Point3D& point : points) {
        PointCloudSample sample;
        sample.point = point;
        samples.push_back(sample);
    }
    return samples;
}

using Matrix3 = std::array<std::array<float, 3>, 3>;

Matrix3 identityMatrix3() {
    return Matrix3{{
        {{1.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f}},
    }};
}

struct EigenDecomposition3 {
    std::array<float, 3> values = {0.0f, 0.0f, 0.0f};
    Matrix3 vectors = identityMatrix3();
};

EigenDecomposition3 jacobiEigenDecomposition(Matrix3 matrix) {
    EigenDecomposition3 result;
    Matrix3 vectors = identityMatrix3();

    for (int iter = 0; iter < 24; ++iter) {
        int p = 0;
        int q = 1;
        float max_offdiag = std::abs(matrix[0][1]);
        if (std::abs(matrix[0][2]) > max_offdiag) {
            p = 0;
            q = 2;
            max_offdiag = std::abs(matrix[0][2]);
        }
        if (std::abs(matrix[1][2]) > max_offdiag) {
            p = 1;
            q = 2;
            max_offdiag = std::abs(matrix[1][2]);
        }

        if (max_offdiag < 1e-8f) {
            break;
        }

        const float app = matrix[p][p];
        const float aqq = matrix[q][q];
        const float apq = matrix[p][q];
        const float angle = 0.5f * std::atan2(2.0f * apq, aqq - app);
        const float c = std::cos(angle);
        const float s = std::sin(angle);

        for (int k = 0; k < 3; ++k) {
            if (k == p || k == q) {
                continue;
            }
            const float akp = matrix[k][p];
            const float akq = matrix[k][q];
            matrix[k][p] = c * akp - s * akq;
            matrix[p][k] = matrix[k][p];
            matrix[k][q] = s * akp + c * akq;
            matrix[q][k] = matrix[k][q];
        }

        matrix[p][p] = c * c * app - 2.0f * s * c * apq + s * s * aqq;
        matrix[q][q] = s * s * app + 2.0f * s * c * apq + c * c * aqq;
        matrix[p][q] = 0.0f;
        matrix[q][p] = 0.0f;

        for (int k = 0; k < 3; ++k) {
            const float vip = vectors[k][p];
            const float viq = vectors[k][q];
            vectors[k][p] = c * vip - s * viq;
            vectors[k][q] = s * vip + c * viq;
        }
    }

    std::array<int, 3> order = {0, 1, 2};
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return matrix[a][a] < matrix[b][b];
    });

    Matrix3 sorted_vectors{};
    for (int sorted = 0; sorted < 3; ++sorted) {
        const int original = order[sorted];
        result.values[sorted] = std::max(0.0f, matrix[original][original]);
        for (int row = 0; row < 3; ++row) {
            sorted_vectors[row][sorted] = vectors[row][original];
        }
    }
    result.vectors = sorted_vectors;
    return result;
}

Point3D normalizedVector(float x, float y, float z) {
    const float norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 1e-8f) {
        return {};
    }
    return {x / norm, y / norm, z / norm};
}

} // namespace

OctreeManager::OctreeManager(int max_depth)
    : config_{} {
    config_.max_depth = max_depth;
}

OctreeManager::OctreeManager(OctreeConfig config)
    : config_(config) {
}

void OctreeManager::initialize(const std::vector<Point3D>& points) {
    initialize(toSamples(points));
}

void OctreeManager::initialize(const std::vector<PointCloudSample>& samples) {
    buildLinearOctree(samples);
    rebuildLeafIndex();
    refreshAllNeighborLinks();

    std::vector<int> leaves;
    leaves.reserve(nodes_.size());
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if (nodes_[i].leaf) {
            leaves.push_back(i);
        }
    }
    applyMLToLeaves(leaves);
    refreshAllNeighborLinks();
}

void OctreeManager::updateFromPointCloud(const std::vector<Point3D>& updated_points) {
    updateFromPointCloud(toSamples(updated_points));
}

void OctreeManager::updateFromPointCloud(const std::vector<PointCloudSample>& updated_samples) {
    if (nodes_.empty()) {
        initialize(updated_samples);
        return;
    }

    bool outside_root = false;
    for (const PointCloudSample& sample : updated_samples) {
        if (!root_bounds_.contains(sample.point)) {
            outside_root = true;
            break;
        }
    }

    if (outside_root) {
        initialize(updated_samples);
        return;
    }

    std::unordered_set<int> touched_set;
    for (const PointCloudSample& sample : updated_samples) {
        const auto leaf_index = findLeafIndexByPoint(sample.point);
        if (!leaf_index.has_value()) {
            continue;
        }

        OctreeNode& leaf = nodes_[leaf_index.value()];
        leaf.dynamic = true;
        leaf.point_count += 1;
        leaf.density = leaf.node_volume > 0.0f ? leaf.point_count / leaf.node_volume : 0.0f;

        if (sample.has_semantics) {
            leaf.room_id = sample.room_id;
            leaf.label = sample.label;
            leaf.obstacle_probability = clampFloat(sample.obstacle_probability, 0.0f, 1.0f);
            leaf.is_cross_floor = sample.is_cross_floor || sample.label == VoxelLabel::Stair;
        } else {
            leaf.obstacle_probability = clampFloat(
                std::max(leaf.obstacle_probability * config_.dynamic_decay, 0.35f), 0.0f, 1.0f);
        }
        touched_set.insert(leaf_index.value());
    }

    std::vector<int> touched(touched_set.begin(), touched_set.end());
    applyMLToLeaves(touched);

    for (int index : touched) {
        updateNeighborLinks(index);
        for (int neighbor : nodes_[index].orthogonal_neighbors) {
            if (neighbor >= 0) {
                updateNeighborLinks(neighbor);
            }
        }
    }

    for (int index : touched) {
        floodFillLabelPropagation(index);
    }
}

void OctreeManager::setMLPredictor(MLPredictor predictor) {
    ml_predictor_ = std::move(predictor);
}

std::array<int, NEIGHBOR_COUNT> OctreeManager::getOrthogonalNeighbors(int node_index) const {
    assert(node_index >= 0 && node_index < static_cast<int>(nodes_.size()));
    return nodes_[node_index].orthogonal_neighbors;
}

std::optional<int> OctreeManager::findLeafIndexByPoint(const Point3D& point) const {
    if (nodes_.empty() || !root_bounds_.contains(point)) {
        return std::nullopt;
    }
    const int result = findContainingLeaf(0, point);
    if (result < 0) {
        return std::nullopt;
    }
    return result;
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

    if (node.label == VoxelLabel::Obstacle || node.obstacle_probability >= 0.95f) {
        return kBlockedCost;
    }

    const float size_penalty = 0.08f * static_cast<float>(node.depth);
    const float probability_penalty = 9.0f * node.obstacle_probability;
    const float stair_penalty = node.label == VoxelLabel::Stair ? 1.75f : 0.0f;
    const float cross_floor_penalty = node.is_cross_floor && !allow_cross_floor ? 5.0f : 0.0f;
    const float dynamic_penalty = node.dynamic ? 0.75f : 0.0f;
    return 1.0f + size_penalty + probability_penalty + stair_penalty + cross_floor_penalty + dynamic_penalty;
}

std::optional<TraversalInfo> OctreeManager::computeTraversalInfo(int from_node, int to_node, bool allow_cross_floor) const {
    if (from_node < 0 || from_node >= static_cast<int>(nodes_.size()) ||
        to_node < 0 || to_node >= static_cast<int>(nodes_.size())) {
        return std::nullopt;
    }

    const OctreeNode& from = nodes_[from_node];
    const OctreeNode& to = nodes_[to_node];
    if (to.label == VoxelLabel::Obstacle || to.obstacle_probability >= 0.95f) {
        return std::nullopt;
    }
    if (from.room_id != to.room_id && !(isCrossFloorConnected(from_node) && isCrossFloorConnected(to_node))) {
        return std::nullopt;
    }

    TraversalInfo info;
    info.distance = distanceBetween(from.bounds.center(), to.bounds.center());
    info.obstacle_probability = to.obstacle_probability;
    info.label = to.label;
    info.is_cross_floor = to.is_cross_floor;
    info.room_id = to.room_id;
    info.cost = info.distance * computeTraversalCost(to_node, allow_cross_floor);
    return info;
}

void OctreeManager::assignLeafLabel(int node_index, VoxelLabel label, float obstacle_probability, int room_id, bool is_cross_floor) {
    assert(node_index >= 0 && node_index < static_cast<int>(nodes_.size()));
    OctreeNode& node = nodes_[node_index];
    node.label = label;
    node.obstacle_probability = clampFloat(obstacle_probability, 0.0f, 1.0f);
    node.room_id = room_id;
    node.is_cross_floor = is_cross_floor || label == VoxelLabel::Stair;
    if (node.leaf) {
        updateNeighborLinks(node_index);
        floodFillLabelPropagation(node_index);
    }
}

int OctreeManager::getLeafCount() const {
    int leaf_count = 0;
    for (const OctreeNode& node : nodes_) {
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

void OctreeManager::buildLinearOctree(const std::vector<PointCloudSample>& samples) {
    nodes_.clear();
    leaf_by_code_.clear();
    nodes_.reserve(std::max<std::size_t>(1024, samples.size() * 2));

    if (samples.empty()) {
        root_bounds_ = BBox{{0.0f, 0.0f, 0.0f}, {20.0f, 20.0f, 5.0f}};
    } else {
        float min_x = samples.front().point.x;
        float max_x = samples.front().point.x;
        float min_y = samples.front().point.y;
        float max_y = samples.front().point.y;
        float min_z = samples.front().point.z;
        float max_z = samples.front().point.z;
        for (const PointCloudSample& sample : samples) {
            min_x = std::min(min_x, sample.point.x);
            max_x = std::max(max_x, sample.point.x);
            min_y = std::min(min_y, sample.point.y);
            max_y = std::max(max_y, sample.point.y);
            min_z = std::min(min_z, sample.point.z);
            max_z = std::max(max_z, sample.point.z);
        }

        const float max_span = std::max({max_x - min_x, max_y - min_y, max_z - min_z, 1.0f});
        const float padding = std::max(0.5f, 0.02f * max_span);
        root_bounds_ = BBox{
            {min_x - padding, min_y - padding, min_z - padding},
            {max_x + padding, max_y + padding, max_z + padding},
        };
    }

    OctreeNode root;
    root.bounds = root_bounds_;
    root.depth = 0;
    root.point_count = static_cast<int>(samples.size());
    root.node_volume = root.bounds.volume();
    root.density = root.node_volume > 0.0f ? root.point_count / root.node_volume : 0.0f;
    root.morton_code = computeMortonCode(root.bounds.center(), 0);
    nodes_.push_back(root);
    aggregateSampleSemantics(0, samples);
    subdivideNode(0, samples);
}

void OctreeManager::subdivideNode(int node_index, const std::vector<PointCloudSample>& samples) {
    nodes_[node_index].node_volume = nodes_[node_index].bounds.volume();
    nodes_[node_index].density = nodes_[node_index].node_volume > 0.0f
                                      ? nodes_[node_index].point_count / nodes_[node_index].node_volume
                                      : 0.0f;

    bool has_stair_semantics = false;
    for (const PointCloudSample& sample : samples) {
        if (sample.has_semantics && (sample.label == VoxelLabel::Stair || sample.is_cross_floor)) {
            has_stair_semantics = true;
            break;
        }
    }

    if (!shouldSubdivide(nodes_[node_index], has_stair_semantics)) {
        nodes_[node_index].leaf = true;
        computeLeafGeometryStats(node_index, samples);
        return;
    }

    nodes_[node_index].leaf = false;
    std::array<std::vector<PointCloudSample>, 8> child_samples;
    const BBox parent_bounds = nodes_[node_index].bounds;
    const Point3D center = parent_bounds.center();

    for (const PointCloudSample& sample : samples) {
        if (parent_bounds.contains(sample.point)) {
            child_samples[getChildOctant(sample.point, parent_bounds)].push_back(sample);
        }
    }

    for (int child = 0; child < 8; ++child) {
        if (child_samples[child].empty()) {
            nodes_[node_index].children[child] = -1;
            continue;
        }

        OctreeNode child_node;
        child_node.depth = nodes_[node_index].depth + 1;
        child_node.bounds = parent_bounds;
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
        child_node.point_count = static_cast<int>(child_samples[child].size());
        child_node.node_volume = child_node.bounds.volume();
        child_node.density = child_node.node_volume > 0.0f ? child_node.point_count / child_node.node_volume : 0.0f;
        child_node.morton_code = computeMortonCode(computeOctantCenter(parent_bounds, child), child_node.depth);

        const int child_index = static_cast<int>(nodes_.size());
        nodes_.push_back(child_node);
        nodes_[node_index].children[child] = child_index;
        aggregateSampleSemantics(child_index, child_samples[child]);
        subdivideNode(child_index, child_samples[child]);
    }
}

bool OctreeManager::shouldSubdivide(const OctreeNode& node, bool has_stair_semantics) const {
    if (node.depth >= config_.max_depth || node.point_count < config_.min_points_to_split) {
        return false;
    }

    const float size = maxExtent(node.bounds);
    const float target_min = has_stair_semantics ? config_.stair_min_voxel : config_.corridor_min_voxel;
    const float target_max = has_stair_semantics ? config_.stair_max_voxel : config_.corridor_max_voxel;
    if (size <= target_min) {
        return false;
    }
    if (size > target_max) {
        return true;
    }
    if (node.density > config_.dense_points_per_m3) {
        return true;
    }
    return node.point_count > config_.dense_points_to_split;
}

void OctreeManager::rebuildLeafIndex() {
    leaf_by_code_.clear();
    leaf_by_code_.reserve(nodes_.size());
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if (nodes_[i].leaf) {
            leaf_by_code_[nodes_[i].morton_code] = i;
        }
    }
}

void OctreeManager::refreshAllNeighborLinks() {
    for (int i = 0; i < static_cast<int>(nodes_.size()); ++i) {
        if (nodes_[i].leaf) {
            updateNeighborLinks(i);
        }
    }
}

void OctreeManager::updateNeighborLinks(int node_index) {
    if (node_index < 0 || node_index >= static_cast<int>(nodes_.size()) || !nodes_[node_index].leaf) {
        return;
    }

    std::array<int, NEIGHBOR_COUNT> next;
    next.fill(-1);
    for (int dir = 0; dir < NEIGHBOR_COUNT; ++dir) {
        const auto neighbor = findNeighborByDirection(node_index, static_cast<NeighborDirection>(dir));
        next[dir] = neighbor.has_value() ? neighbor.value() : -1;
    }
    nodes_[node_index].orthogonal_neighbors = next;
}

int OctreeManager::findContainingLeaf(int node_index, const Point3D& point) const {
    if (node_index < 0 || node_index >= static_cast<int>(nodes_.size())) {
        return -1;
    }
    const OctreeNode& node = nodes_[node_index];
    if (!node.bounds.contains(point)) {
        return -1;
    }
    if (node.leaf) {
        return node_index;
    }

    const int octant = getChildOctant(point, node.bounds);
    const int child_index = node.children[octant];
    if (child_index >= 0) {
        const int result = findContainingLeaf(child_index, point);
        if (result >= 0) {
            return result;
        }
    }

    for (int child : node.children) {
        const int result = findContainingLeaf(child, point);
        if (result >= 0) {
            return result;
        }
    }
    return -1;
}

std::optional<int> OctreeManager::findNeighborByDirection(int node_index, NeighborDirection dir) const {
    const OctreeNode& node = nodes_[node_index];
    const BBox& b = node.bounds;
    std::array<Point3D, 5> probes;
    const Point3D c = b.center();

    switch (dir) {
        case POS_X:
        case NEG_X: {
            const float x = dir == POS_X ? b.max.x + kEpsilon : b.min.x - kEpsilon;
            probes = {{
                {x, c.y, c.z},
                {x, b.min.y + 0.25f * b.depth(), c.z},
                {x, b.max.y - 0.25f * b.depth(), c.z},
                {x, c.y, b.min.z + 0.25f * b.height()},
                {x, c.y, b.max.z - 0.25f * b.height()},
            }};
            break;
        }
        case POS_Y:
        case NEG_Y: {
            const float y = dir == POS_Y ? b.max.y + kEpsilon : b.min.y - kEpsilon;
            probes = {{
                {c.x, y, c.z},
                {b.min.x + 0.25f * b.width(), y, c.z},
                {b.max.x - 0.25f * b.width(), y, c.z},
                {c.x, y, b.min.z + 0.25f * b.height()},
                {c.x, y, b.max.z - 0.25f * b.height()},
            }};
            break;
        }
        case POS_Z:
        case NEG_Z: {
            const float z = dir == POS_Z ? b.max.z + kEpsilon : b.min.z - kEpsilon;
            probes = {{
                {c.x, c.y, z},
                {b.min.x + 0.25f * b.width(), c.y, z},
                {b.max.x - 0.25f * b.width(), c.y, z},
                {c.x, b.min.y + 0.25f * b.depth(), z},
                {c.x, b.max.y - 0.25f * b.depth(), z},
            }};
            break;
        }
        default:
            return std::nullopt;
    }

    int best = -1;
    float best_score = std::numeric_limits<float>::max();
    for (const Point3D& probe : probes) {
        const auto candidate = findLeafIndexByPoint(probe);
        if (!candidate.has_value() || candidate.value() == node_index) {
            continue;
        }
        const OctreeNode& other = nodes_[candidate.value()];
        if (!canTraverseBetween(node, other, dir)) {
            continue;
        }

        const float score = distanceBetween(c, other.bounds.center()) + 0.02f * static_cast<float>(other.depth);
        if (score < best_score) {
            best_score = score;
            best = candidate.value();
        }
    }

    if (best < 0) {
        return std::nullopt;
    }
    return best;
}

void OctreeManager::floodFillLabelPropagation(int start_node) {
    if (start_node < 0 || start_node >= static_cast<int>(nodes_.size()) || !nodes_[start_node].leaf) {
        return;
    }

    struct QueueItem {
        int index;
        int distance;
    };

    std::queue<QueueItem> queue;
    std::vector<bool> visited(nodes_.size(), false);
    queue.push({start_node, 0});
    visited[start_node] = true;

    const OctreeNode start = nodes_[start_node];
    while (!queue.empty()) {
        const QueueItem current = queue.front();
        queue.pop();
        if (current.distance >= config_.flood_fill_max_depth) {
            continue;
        }

        const OctreeNode node_snapshot = nodes_[current.index];
        for (int neighbor_index : node_snapshot.orthogonal_neighbors) {
            if (neighbor_index < 0 || neighbor_index >= static_cast<int>(nodes_.size()) || visited[neighbor_index]) {
                continue;
            }

            OctreeNode& neighbor = nodes_[neighbor_index];
            if (neighbor.room_id != start.room_id &&
                !(start.is_cross_floor && neighbor.is_cross_floor)) {
                continue;
            }

            const float attenuation = 1.0f - 0.30f * static_cast<float>(current.distance + 1);
            if (start.label == VoxelLabel::Obstacle) {
                neighbor.obstacle_probability = std::max(
                    neighbor.obstacle_probability,
                    clampFloat(start.obstacle_probability * attenuation, 0.0f, 1.0f));
                if (neighbor.obstacle_probability >= 0.70f) {
                    neighbor.label = VoxelLabel::Obstacle;
                }
            } else if (start.label == VoxelLabel::Stair && neighbor.label == VoxelLabel::Free) {
                neighbor.label = VoxelLabel::Stair;
                neighbor.is_cross_floor = true;
                neighbor.obstacle_probability = std::min(neighbor.obstacle_probability, 0.35f);
            }

            visited[neighbor_index] = true;
            queue.push({neighbor_index, current.distance + 1});
        }
    }
}

void OctreeManager::applyMLToLeaves(const std::vector<int>& leaf_indices) {
    if (!ml_predictor_) {
        return;
    }

    for (int index : leaf_indices) {
        if (index < 0 || index >= static_cast<int>(nodes_.size()) || !nodes_[index].leaf) {
            continue;
        }
        const MLResult result = ml_predictor_(nodes_[index]);
        nodes_[index].label = result.label;
        nodes_[index].obstacle_probability = clampFloat(result.obstacle_probability, 0.0f, 1.0f);
        nodes_[index].room_id = result.room_id;
        nodes_[index].is_cross_floor = result.is_cross_floor || result.label == VoxelLabel::Stair;
    }
}

void OctreeManager::aggregateSampleSemantics(int node_index, const std::vector<PointCloudSample>& samples) {
    if (samples.empty()) {
        return;
    }

    int semantic_count = 0;
    int free_count = 0;
    int obstacle_count = 0;
    int stair_count = 0;
    float probability_sum = 0.0f;
    std::unordered_map<int, int> room_votes;
    std::unordered_map<uint32_t, int> entity_votes;
    bool cross_floor = false;

    for (const PointCloudSample& sample : samples) {
        if (!sample.has_semantics) {
            continue;
        }
        ++semantic_count;
        probability_sum += sample.obstacle_probability;
        room_votes[sample.room_id] += 1;
        entity_votes[sample.entity_id] += 1;
        cross_floor = cross_floor || sample.is_cross_floor;
        switch (sample.label) {
            case VoxelLabel::Free: ++free_count; break;
            case VoxelLabel::Obstacle: ++obstacle_count; break;
            case VoxelLabel::Stair: ++stair_count; break;
        }
    }

    if (semantic_count == 0) {
        return;
    }

    OctreeNode& node = nodes_[node_index];
    node.obstacle_probability = clampFloat(probability_sum / semantic_count, 0.0f, 1.0f);
    node.is_cross_floor = cross_floor || stair_count > 0;
    if (stair_count >= obstacle_count && stair_count >= free_count) {
        node.label = VoxelLabel::Stair;
    } else if (obstacle_count >= free_count) {
        node.label = VoxelLabel::Obstacle;
    } else {
        node.label = VoxelLabel::Free;
    }

    int best_room = node.room_id;
    int best_votes = -1;
    for (const auto& vote : room_votes) {
        if (vote.second > best_votes) {
            best_votes = vote.second;
            best_room = vote.first;
        }
    }
    node.room_id = best_room;

    uint32_t best_entity = node.dominant_entity_id;
    best_votes = -1;
    for (const auto& vote : entity_votes) {
        if (vote.second > best_votes) {
            best_votes = vote.second;
            best_entity = vote.first;
        }
    }
    node.dominant_entity_id = best_entity;
}

void OctreeManager::computeLeafGeometryStats(int node_index, const std::vector<PointCloudSample>& samples) {
    if (node_index < 0 || node_index >= static_cast<int>(nodes_.size())) {
        return;
    }

    OctreeNode& node = nodes_[node_index];
    node.centroid = node.bounds.center();
    node.avg_normal = {};
    node.covariance_eigenvalues = {0.0f, 0.0f, 0.0f};
    node.linearity = 0.0f;
    node.flatness = 0.0f;
    node.roughness = 0.0f;
    node.curvature = 0.0f;

    if (samples.empty()) {
        return;
    }

    Point3D centroid;
    for (const PointCloudSample& sample : samples) {
        centroid.x += sample.point.x;
        centroid.y += sample.point.y;
        centroid.z += sample.point.z;
    }
    const float inv_count = 1.0f / static_cast<float>(samples.size());
    centroid.x *= inv_count;
    centroid.y *= inv_count;
    centroid.z *= inv_count;
    node.centroid = centroid;

    if (samples.size() < 3) {
        return;
    }

    Matrix3 covariance{};
    for (const PointCloudSample& sample : samples) {
        const float x = sample.point.x - centroid.x;
        const float y = sample.point.y - centroid.y;
        const float z = sample.point.z - centroid.z;
        covariance[0][0] += x * x;
        covariance[0][1] += x * y;
        covariance[0][2] += x * z;
        covariance[1][1] += y * y;
        covariance[1][2] += y * z;
        covariance[2][2] += z * z;
    }
    covariance[1][0] = covariance[0][1];
    covariance[2][0] = covariance[0][2];
    covariance[2][1] = covariance[1][2];

    for (auto& row : covariance) {
        for (float& value : row) {
            value *= inv_count;
        }
    }

    const EigenDecomposition3 eigen = jacobiEigenDecomposition(covariance);
    node.covariance_eigenvalues = eigen.values;

    Point3D normal = normalizedVector(eigen.vectors[0][0], eigen.vectors[1][0], eigen.vectors[2][0]);
    if (normal.z < 0.0f) {
        normal.x *= -1.0f;
        normal.y *= -1.0f;
        normal.z *= -1.0f;
    }
    node.avg_normal = normal;

    const float lambda0 = eigen.values[0];
    const float lambda1 = eigen.values[1];
    const float lambda2 = eigen.values[2];
    const float lambda_sum = lambda0 + lambda1 + lambda2;
    if (lambda_sum < 1e-8f || lambda2 < 1e-8f) {
        node.avg_normal = {};
        return;
    }

    node.linearity = clampFloat((lambda2 - lambda1) / lambda2, 0.0f, 1.0f);
    node.flatness = clampFloat((lambda1 - lambda0) / lambda2, 0.0f, 1.0f);
    node.roughness = clampFloat(lambda0 / lambda2, 0.0f, 1.0f);
    node.curvature = clampFloat(lambda0 / lambda_sum, 0.0f, 1.0f);
}

bool OctreeManager::canTraverseBetween(const OctreeNode& from, const OctreeNode& to, NeighborDirection dir) const {
    if (to.label == VoxelLabel::Obstacle || to.obstacle_probability >= 0.95f) {
        return false;
    }

    const bool same_room = from.room_id == to.room_id;
    const bool stair_cross_floor = (dir == POS_Z || dir == NEG_Z) &&
                                   (from.is_cross_floor || from.label == VoxelLabel::Stair) &&
                                   (to.is_cross_floor || to.label == VoxelLabel::Stair);
    if (!same_room && !stair_cross_floor) {
        return false;
    }

    switch (dir) {
        case POS_X:
            return nearlyTouches(from.bounds.max.x, to.bounds.min.x) &&
                   rangesOverlap(from.bounds.min.y, from.bounds.max.y, to.bounds.min.y, to.bounds.max.y) &&
                   rangesOverlap(from.bounds.min.z, from.bounds.max.z, to.bounds.min.z, to.bounds.max.z);
        case NEG_X:
            return nearlyTouches(from.bounds.min.x, to.bounds.max.x) &&
                   rangesOverlap(from.bounds.min.y, from.bounds.max.y, to.bounds.min.y, to.bounds.max.y) &&
                   rangesOverlap(from.bounds.min.z, from.bounds.max.z, to.bounds.min.z, to.bounds.max.z);
        case POS_Y:
            return nearlyTouches(from.bounds.max.y, to.bounds.min.y) &&
                   rangesOverlap(from.bounds.min.x, from.bounds.max.x, to.bounds.min.x, to.bounds.max.x) &&
                   rangesOverlap(from.bounds.min.z, from.bounds.max.z, to.bounds.min.z, to.bounds.max.z);
        case NEG_Y:
            return nearlyTouches(from.bounds.min.y, to.bounds.max.y) &&
                   rangesOverlap(from.bounds.min.x, from.bounds.max.x, to.bounds.min.x, to.bounds.max.x) &&
                   rangesOverlap(from.bounds.min.z, from.bounds.max.z, to.bounds.min.z, to.bounds.max.z);
        case POS_Z:
            return nearlyTouches(from.bounds.max.z, to.bounds.min.z) &&
                   rangesOverlap(from.bounds.min.x, from.bounds.max.x, to.bounds.min.x, to.bounds.max.x) &&
                   rangesOverlap(from.bounds.min.y, from.bounds.max.y, to.bounds.min.y, to.bounds.max.y);
        case NEG_Z:
            return nearlyTouches(from.bounds.min.z, to.bounds.max.z) &&
                   rangesOverlap(from.bounds.min.x, from.bounds.max.x, to.bounds.min.x, to.bounds.max.x) &&
                   rangesOverlap(from.bounds.min.y, from.bounds.max.y, to.bounds.min.y, to.bounds.max.y);
        default:
            return false;
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
    return Point3D{
        child_index & 1 ? 0.5f * (center.x + bounds.max.x) : 0.5f * (bounds.min.x + center.x),
        child_index & 2 ? 0.5f * (center.y + bounds.max.y) : 0.5f * (bounds.min.y + center.y),
        child_index & 4 ? 0.5f * (center.z + bounds.max.z) : 0.5f * (bounds.min.z + center.z),
    };
}

uint64_t OctreeManager::computeMortonCode(const Point3D& point, int depth) const {
    const float grid_max = static_cast<float>((1u << std::min(config_.max_depth, 20)) - 1u);
    const float sx = root_bounds_.width() > 0.0f ? grid_max / root_bounds_.width() : 0.0f;
    const float sy = root_bounds_.depth() > 0.0f ? grid_max / root_bounds_.depth() : 0.0f;
    const float sz = root_bounds_.height() > 0.0f ? grid_max / root_bounds_.height() : 0.0f;

    const uint32_t x = static_cast<uint32_t>(clampFloat((point.x - root_bounds_.min.x) * sx, 0.0f, grid_max));
    const uint32_t y = static_cast<uint32_t>(clampFloat((point.y - root_bounds_.min.y) * sy, 0.0f, grid_max));
    const uint32_t z = static_cast<uint32_t>(clampFloat((point.z - root_bounds_.min.z) * sz, 0.0f, grid_max));
    return (static_cast<uint64_t>(depth) << 60U) | interleaveBits(x, y, z);
}

uint64_t OctreeManager::interleaveBits(uint32_t x, uint32_t y, uint32_t z) const {
    auto spread = [](uint64_t v) {
        v &= 0x1fffffULL;
        v = (v | (v << 32U)) & 0x1f00000000ffffULL;
        v = (v | (v << 16U)) & 0x1f0000ff0000ffULL;
        v = (v | (v << 8U)) & 0x100f00f00f00f00fULL;
        v = (v | (v << 4U)) & 0x10c30c30c30c30c3ULL;
        v = (v | (v << 2U)) & 0x1249249249249249ULL;
        return v;
    };
    return spread(x) | (spread(y) << 1U) | (spread(z) << 2U);
}

} // namespace navigation
