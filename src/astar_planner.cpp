#include "astar_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace navigation {

namespace {

constexpr float kInfinity = std::numeric_limits<float>::infinity();

float clampFloat(float value, float min_value, float max_value) {
    return std::max(min_value, std::min(max_value, value));
}

float distanceBetween(const Point3D& a, const Point3D& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

float planarLength(float x, float y) {
    return std::sqrt(x * x + y * y);
}

float planarDistance(const Point3D& a, const Point3D& b) {
    return planarLength(a.x - b.x, a.y - b.y);
}

float distancePointToSegment3D(const Point3D& point,
                               const Point3D& a,
                               const Point3D& b) {
    const float abx = b.x - a.x;
    const float aby = b.y - a.y;
    const float abz = b.z - a.z;
    const float length_sq = abx * abx + aby * aby + abz * abz;
    if (length_sq < 1e-8f) {
        return distanceBetween(point, a);
    }

    const float apx = point.x - a.x;
    const float apy = point.y - a.y;
    const float apz = point.z - a.z;
    const float t = clampFloat((apx * abx + apy * aby + apz * abz) / length_sq,
                               0.0f,
                               1.0f);
    const Point3D closest{
        a.x + t * abx,
        a.y + t * aby,
        a.z + t * abz,
    };
    return distanceBetween(point, closest);
}

int gridCell(float value, float cell_size) {
    return static_cast<int>(std::floor(value / cell_size));
}

long long gridKey(int x, int y) {
    return (static_cast<long long>(x) << 32) ^
           static_cast<unsigned int>(y);
}

Point3D normalizedPlanar(float x, float y) {
    const float length = planarLength(x, y);
    if (length < 1e-5f) {
        return {};
    }
    return {x / length, y / length, 0.0f};
}

float distanceToNearestFloorSurface(float z, const AStarPlannerConfig& config) {
    const float story_height = std::max(config.story_height, 1e-3f);
    const float first_surface = config.floor_z + config.floor_surface_offset;
    const float story = std::round((z - first_surface) / story_height);
    const float nearest_surface = first_surface + story * story_height;
    return std::abs(z - nearest_surface);
}

bool isNearFloorSurface(float z, const AStarPlannerConfig& config) {
    if (!config.constrain_stair_exits_to_floor_levels) {
        return true;
    }
    return distanceToNearestFloorSurface(z, config) <=
           std::max(0.0f, config.stair_floor_exit_tolerance);
}

int labelRank(VoxelLabel label) {
    switch (label) {
        case VoxelLabel::Free: return 0;
        case VoxelLabel::Stair: return 1;
        case VoxelLabel::Obstacle: return 2;
    }
    return 2;
}

bool sameDirection(const Point3D& a, const Point3D& b, const Point3D& c) {
    const float abx = b.x - a.x;
    const float aby = b.y - a.y;
    const float abz = b.z - a.z;
    const float bcx = c.x - b.x;
    const float bcy = c.y - b.y;
    const float bcz = c.z - b.z;

    const float ab_norm = std::sqrt(abx * abx + aby * aby + abz * abz);
    const float bc_norm = std::sqrt(bcx * bcx + bcy * bcy + bcz * bcz);
    if (ab_norm < 1e-5f || bc_norm < 1e-5f) {
        return false;
    }

    const float dot = (abx * bcx + aby * bcy + abz * bcz) / (ab_norm * bc_norm);
    return dot > 0.995f;
}

struct QueueItem {
    int node_index = -1;
    float f_score = 0.0f;
    float g_score = 0.0f;
};

struct QueueCompare {
    bool operator()(const QueueItem& lhs, const QueueItem& rhs) const {
        if (lhs.f_score == rhs.f_score) {
            return lhs.g_score < rhs.g_score;
        }
        return lhs.f_score > rhs.f_score;
    }
};

} // namespace

AStarPlanner::AStarPlanner(const OctreeManager& octree, AStarPlannerConfig config)
    : octree_(octree), config_(config) {
}

AStarPath AStarPlanner::findPath(const Point3D& start, const Point3D& goal) const {
    AStarPath result;
    bool start_snapped = false;
    bool goal_snapped = false;
    float start_snap_distance = 0.0f;
    float goal_snap_distance = 0.0f;
    const auto start_node = resolveEndpointNode(start, &start_snapped, &start_snap_distance);
    const auto goal_node = resolveEndpointNode(goal, &goal_snapped, &goal_snap_distance);
    if (!start_node.has_value()) {
        result.message = "Start point cannot be resolved to a traversable leaf";
        return result;
    }
    if (!goal_node.has_value()) {
        result.message = "Goal point cannot be resolved to a traversable leaf";
        return result;
    }
    result = findPathByNodeIndex(start_node.value(), goal_node.value());
    result.start_snapped = start_snapped;
    result.goal_snapped = goal_snapped;
    result.start_snap_distance = start_snap_distance;
    result.goal_snap_distance = goal_snap_distance;
    if (result.success && (start_snapped || goal_snapped)) {
        result.message += " (";
        if (start_snapped) {
            result.message += "start snapped " + std::to_string(start_snap_distance) + "m";
        }
        if (start_snapped && goal_snapped) {
            result.message += ", ";
        }
        if (goal_snapped) {
            result.message += "goal snapped " + std::to_string(goal_snap_distance) + "m";
        }
        result.message += ")";
    }
    return result;
}

AStarPath AStarPlanner::findPathByNodeIndex(int start_node, int goal_node) const {
    AStarPath failed;
    failed.start_node = start_node;
    failed.goal_node = goal_node;

    const int node_count = octree_.getNodeCount();
    if (start_node < 0 || start_node >= node_count ||
        goal_node < 0 || goal_node >= node_count) {
        failed.message = "Start or goal node index is out of range";
        return failed;
    }
    if (!isTraversable(start_node)) {
        failed.message = "Start node is not traversable";
        return failed;
    }
    if (!isTraversable(goal_node)) {
        failed.message = "Goal node is not traversable";
        return failed;
    }

    if (start_node == goal_node) {
        std::vector<int> came_from(static_cast<std::size_t>(node_count), -1);
        std::vector<float> g_score(static_cast<std::size_t>(node_count), kInfinity);
        g_score[static_cast<std::size_t>(start_node)] = 0.0f;
        return reconstructPath(start_node, goal_node, came_from, g_score, 0);
    }

    std::priority_queue<QueueItem, std::vector<QueueItem>, QueueCompare> open_set;
    std::vector<int> came_from(static_cast<std::size_t>(node_count), -1);
    std::vector<float> g_score(static_cast<std::size_t>(node_count), kInfinity);
    std::vector<bool> closed(static_cast<std::size_t>(node_count), false);

    g_score[static_cast<std::size_t>(start_node)] = 0.0f;
    open_set.push({start_node, heuristicCost(start_node, goal_node), 0.0f});

    int expanded = 0;
    while (!open_set.empty()) {
        const QueueItem current = open_set.top();
        open_set.pop();

        if (current.node_index < 0 || current.node_index >= node_count) {
            continue;
        }
        if (closed[static_cast<std::size_t>(current.node_index)]) {
            continue;
        }

        closed[static_cast<std::size_t>(current.node_index)] = true;
        ++expanded;

        if (current.node_index == goal_node) {
            return reconstructPath(start_node, goal_node, came_from, g_score, expanded);
        }

        if (expanded >= config_.max_expansions) {
            failed.message = "A* stopped because max_expansions was reached";
            failed.expanded_nodes = expanded;
            return failed;
        }

        const auto neighbors = getTraversalNeighbors(current.node_index);
        for (int neighbor : neighbors) {
            if (neighbor < 0 || neighbor >= node_count ||
                closed[static_cast<std::size_t>(neighbor)] ||
                !isTraversable(neighbor)) {
                continue;
            }

            const float traversal_cost = edgeCost(current.node_index, neighbor);
            if (!std::isfinite(traversal_cost)) {
                continue;
            }

            const float tentative_g =
                g_score[static_cast<std::size_t>(current.node_index)] + traversal_cost;
            if (tentative_g < g_score[static_cast<std::size_t>(neighbor)]) {
                came_from[static_cast<std::size_t>(neighbor)] = current.node_index;
                g_score[static_cast<std::size_t>(neighbor)] = tentative_g;
                const float f_score = tentative_g + heuristicCost(neighbor, goal_node);
                open_set.push({neighbor, f_score, tentative_g});
            }
        }
    }

    failed.message = "No path found";
    failed.expanded_nodes = expanded;
    return failed;
}

std::optional<int> AStarPlanner::resolveEndpointNode(const Point3D& point,
                                                     bool* snapped,
                                                     float* snap_distance) const {
    if (snapped != nullptr) {
        *snapped = false;
    }
    if (snap_distance != nullptr) {
        *snap_distance = 0.0f;
    }

    const auto containing_leaf = octree_.findLeafIndexByPoint(point);
    if (containing_leaf.has_value() && isTraversable(containing_leaf.value())) {
        return containing_leaf;
    }

    if (!config_.snap_endpoints_to_traversable) {
        return containing_leaf;
    }

    int best_node = -1;
    float best_distance = kInfinity;
    const float max_distance = std::max(0.0f, config_.endpoint_snap_radius);
    const auto& nodes = octree_.nodes();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        const OctreeNode& node = nodes[static_cast<std::size_t>(i)];
        if (!node.leaf) {
            continue;
        }
        const float distance = distanceBetween(point, node.bounds.center());
        if (distance <= max_distance && distance < best_distance) {
            if (!isTraversable(i)) {
                continue;
            }
            best_distance = distance;
            best_node = i;
        }
    }

    if (best_node < 0) {
        return std::nullopt;
    }
    if (snapped != nullptr) {
        *snapped = true;
    }
    if (snap_distance != nullptr) {
        *snap_distance = best_distance;
    }
    return best_node;
}

void AStarPlanner::ensureStairCache() const {
    if (stair_cache_valid_) {
        return;
    }

    stair_nodes_.clear();
    stair_component_by_node_.assign(octree_.nodes().size(), -1);
    stair_components_.clear();

    const auto& nodes = octree_.nodes();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        const OctreeNode& node = nodes[static_cast<std::size_t>(i)];
        if (node.leaf &&
            (node.label == VoxelLabel::Stair || node.is_cross_floor) &&
            isTraversable(i)) {
            stair_nodes_.push_back(i);
        }
    }

    for (int stair_node : stair_nodes_) {
        if (stair_component_by_node_[static_cast<std::size_t>(stair_node)] >= 0) {
            continue;
        }

        const int component_id = static_cast<int>(stair_components_.size());
        StairComponent component;
        component.min_z = kInfinity;
        component.max_z = -kInfinity;
        std::vector<int> component_nodes;

        std::queue<int> pending;
        pending.push(stair_node);
        stair_component_by_node_[static_cast<std::size_t>(stair_node)] = component_id;

        while (!pending.empty()) {
            const int current = pending.front();
            pending.pop();

            const OctreeNode* current_node = octree_.getNode(current);
            if (current_node == nullptr) {
                continue;
            }
            const float current_z = current_node->bounds.center().z;
            component.min_z = std::min(component.min_z, current_z);
            component.max_z = std::max(component.max_z, current_z);
            ++component.count;
            component_nodes.push_back(current);

            for (int candidate : stair_nodes_) {
                if (stair_component_by_node_[static_cast<std::size_t>(candidate)] >= 0) {
                    continue;
                }
                if (!stairNodesCanConnect(current, candidate)) {
                    continue;
                }
                stair_component_by_node_[static_cast<std::size_t>(candidate)] = component_id;
                pending.push(candidate);
            }
        }

        if (component.count > 0) {
            const float end_band =
                std::max(0.25f, std::min(config_.stair_endpoint_tolerance,
                                         std::max(config_.stair_step_max_vertical, 0.25f)));
            Point3D low_sum{};
            Point3D high_sum{};
            int low_count = 0;
            int high_count = 0;
            for (int node_index : component_nodes) {
                const OctreeNode* node = octree_.getNode(node_index);
                if (node == nullptr) {
                    continue;
                }
                const Point3D center = node->bounds.center();
                if (center.z <= component.min_z + end_band) {
                    low_sum.x += center.x;
                    low_sum.y += center.y;
                    low_sum.z += center.z;
                    ++low_count;
                }
                if (center.z >= component.max_z - end_band) {
                    high_sum.x += center.x;
                    high_sum.y += center.y;
                    high_sum.z += center.z;
                    ++high_count;
                }
            }

            if (low_count > 0 && high_count > 0) {
                const float inv_low = 1.0f / static_cast<float>(low_count);
                const float inv_high = 1.0f / static_cast<float>(high_count);
                component.low_center = {
                    low_sum.x * inv_low,
                    low_sum.y * inv_low,
                    low_sum.z * inv_low,
                };
                component.high_center = {
                    high_sum.x * inv_high,
                    high_sum.y * inv_high,
                    high_sum.z * inv_high,
                };
                component.ascent_direction =
                    normalizedPlanar(component.high_center.x - component.low_center.x,
                                     component.high_center.y - component.low_center.y);
                component.has_direction =
                    planarLength(component.ascent_direction.x,
                                 component.ascent_direction.y) > 0.5f;

                if (component.has_direction) {
                    const Point3D lateral{
                        -component.ascent_direction.y,
                        component.ascent_direction.x,
                        0.0f,
                    };
                    for (int node_index : component_nodes) {
                        const OctreeNode* node = octree_.getNode(node_index);
                        if (node == nullptr) {
                            continue;
                        }
                        const Point3D center = node->bounds.center();
                        if (center.z <= component.min_z + end_band) {
                            const float lx = center.x - component.low_center.x;
                            const float ly = center.y - component.low_center.y;
                            component.low_half_width = std::max(
                                component.low_half_width,
                                std::abs(lx * lateral.x + ly * lateral.y) +
                                    0.5f * std::max(node->bounds.width(), node->bounds.depth()));
                        }
                        if (center.z >= component.max_z - end_band) {
                            const float hx = center.x - component.high_center.x;
                            const float hy = center.y - component.high_center.y;
                            component.high_half_width = std::max(
                                component.high_half_width,
                                std::abs(hx * lateral.x + hy * lateral.y) +
                                    0.5f * std::max(node->bounds.width(), node->bounds.depth()));
                        }
                    }
                }
            }
            stair_components_.push_back(component);
        }
    }

    stair_cache_valid_ = true;
}

bool AStarPlanner::isStairLike(int node_index) const {
    const OctreeNode* node = octree_.getNode(node_index);
    return node != nullptr &&
           (node->label == VoxelLabel::Stair || node->is_cross_floor);
}

bool AStarPlanner::stairNodesCanConnect(int from_node, int to_node) const {
    const OctreeNode* from = octree_.getNode(from_node);
    const OctreeNode* to = octree_.getNode(to_node);
    if (from == nullptr || to == nullptr) {
        return false;
    }
    if (!isStairLike(from_node) || !isStairLike(to_node)) {
        return false;
    }

    const Point3D from_center = from->bounds.center();
    const Point3D to_center = to->bounds.center();
    if (distanceBetween(from_center, to_center) > config_.stair_connection_radius) {
        return false;
    }

    if (from->dominant_entity_id != 0 &&
        to->dominant_entity_id != 0 &&
        from->dominant_entity_id != to->dominant_entity_id) {
        return false;
    }

    if (config_.constrain_stair_transitions) {
        const float dz = std::abs(from_center.z - to_center.z);
        const float max_vertical =
            std::max(config_.stair_step_max_vertical,
                     std::max(from->bounds.height(), to->bounds.height()) * 1.5f);
        if (dz > max_vertical) {
            return false;
        }
    }

    return true;
}

bool AStarPlanner::isStairEndpoint(int node_index) const {
    if (!config_.constrain_stair_transitions) {
        return true;
    }
    if (!isStairLike(node_index)) {
        return false;
    }

    ensureStairCache();
    if (node_index < 0 ||
        node_index >= static_cast<int>(stair_component_by_node_.size())) {
        return false;
    }

    const int component_id =
        stair_component_by_node_[static_cast<std::size_t>(node_index)];
    if (component_id < 0 ||
        component_id >= static_cast<int>(stair_components_.size())) {
        return false;
    }

    const OctreeNode* node = octree_.getNode(node_index);
    if (node == nullptr) {
        return false;
    }

    const StairComponent& component =
        stair_components_[static_cast<std::size_t>(component_id)];
    const float z = node->bounds.center().z;
    const float tolerance =
        std::max(config_.stair_endpoint_tolerance, node->bounds.height() * 1.5f);
    return z <= component.min_z + tolerance ||
           z >= component.max_z - tolerance;
}

bool AStarPlanner::stairDirectionAllowsTransition(
    int stair_node,
    int free_node,
    bool entering_stair,
    const StairComponent& component) const {
    if (!config_.constrain_stair_direction || !component.has_direction) {
        return true;
    }

    const OctreeNode* stair = octree_.getNode(stair_node);
    const OctreeNode* free = octree_.getNode(free_node);
    if (stair == nullptr || free == nullptr) {
        return false;
    }

    const Point3D stair_center = stair->bounds.center();
    const Point3D free_center = free->bounds.center();
    const float stair_endpoint_tolerance =
        std::max(config_.stair_endpoint_tolerance, stair->bounds.height() * 1.5f);
    const float floor_connection_tolerance =
        std::max(config_.stair_floor_connection_max_vertical,
                 free->bounds.height() * 1.5f);

    const bool stair_at_low_end =
        stair_center.z <= component.min_z + stair_endpoint_tolerance;
    const bool stair_at_high_end =
        stair_center.z >= component.max_z - stair_endpoint_tolerance;
    const bool free_at_low_side =
        free_center.z <= component.min_z + floor_connection_tolerance;
    const bool free_at_high_side =
        free_center.z >= component.max_z - floor_connection_tolerance;

    Point3D movement;
    if (entering_stair) {
        movement = normalizedPlanar(stair_center.x - free_center.x,
                                    stair_center.y - free_center.y);
    } else {
        movement = normalizedPlanar(free_center.x - stair_center.x,
                                    free_center.y - stair_center.y);
    }

    if (planarLength(movement.x, movement.y) < 1e-5f) {
        return true;
    }

    Point3D expected{};
    Point3D endpoint_center{};
    float half_width = 0.0f;
    float expected_forward_sign = 0.0f;
    if (entering_stair && stair_at_low_end && free_at_low_side) {
        expected = component.ascent_direction;
        endpoint_center = component.low_center;
        half_width = component.low_half_width;
        expected_forward_sign = -1.0f;
    } else if (entering_stair && stair_at_high_end && free_at_high_side) {
        expected = {-component.ascent_direction.x, -component.ascent_direction.y, 0.0f};
        endpoint_center = component.high_center;
        half_width = component.high_half_width;
        expected_forward_sign = 1.0f;
    } else if (!entering_stair && stair_at_low_end && free_at_low_side) {
        expected = {-component.ascent_direction.x, -component.ascent_direction.y, 0.0f};
        endpoint_center = component.low_center;
        half_width = component.low_half_width;
        expected_forward_sign = -1.0f;
    } else if (!entering_stair && stair_at_high_end && free_at_high_side) {
        expected = component.ascent_direction;
        endpoint_center = component.high_center;
        half_width = component.high_half_width;
        expected_forward_sign = 1.0f;
    } else {
        return false;
    }

    const Point3D lateral{-component.ascent_direction.y,
                          component.ascent_direction.x,
                          0.0f};
    const float rel_x = free_center.x - endpoint_center.x;
    const float rel_y = free_center.y - endpoint_center.y;
    const float lateral_error = std::abs(rel_x * lateral.x + rel_y * lateral.y);
    const float allowed_half_width =
        std::max(half_width, 0.25f) + config_.stair_entry_lateral_margin;
    if (lateral_error > allowed_half_width) {
        return false;
    }

    const float forward_offset =
        rel_x * component.ascent_direction.x + rel_y * component.ascent_direction.y;
    if (expected_forward_sign < 0.0f &&
        forward_offset > config_.stair_entry_forward_margin) {
        return false;
    }
    if (expected_forward_sign > 0.0f &&
        forward_offset < -config_.stair_entry_forward_margin) {
        return false;
    }

    const float dot = movement.x * expected.x + movement.y * expected.y;
    return dot >= config_.stair_direction_dot_min;
}

bool AStarPlanner::isAllowedStairTransition(int from_node, int to_node) const {
    if (!config_.constrain_stair_transitions) {
        return true;
    }

    const bool from_stair = isStairLike(from_node);
    const bool to_stair = isStairLike(to_node);
    if (!from_stair && !to_stair) {
        return true;
    }

    const OctreeNode* from = octree_.getNode(from_node);
    const OctreeNode* to = octree_.getNode(to_node);
    if (from == nullptr || to == nullptr) {
        return false;
    }

    if (from_stair && to_stair) {
        return stairNodesCanConnect(from_node, to_node);
    }

    const int stair_node = from_stair ? from_node : to_node;
    if (!isStairEndpoint(stair_node)) {
        return false;
    }

    ensureStairCache();
    const int component_id =
        stair_component_by_node_[static_cast<std::size_t>(stair_node)];
    if (component_id < 0 ||
        component_id >= static_cast<int>(stair_components_.size())) {
        return false;
    }

    const StairComponent& component =
        stair_components_[static_cast<std::size_t>(component_id)];
    const OctreeNode* stair = octree_.getNode(stair_node);
    if (stair == nullptr) {
        return false;
    }
    const OctreeNode* free_node = from_stair ? to : from;
    const float free_z = free_node->bounds.center().z;
    const bool free_near_component_end =
        free_z <= component.min_z + config_.stair_floor_connection_max_vertical ||
        free_z >= component.max_z - config_.stair_floor_connection_max_vertical;
    if (!free_near_component_end) {
        return false;
    }
    if (!isNearFloorSurface(free_z, config_) ||
        !isNearFloorSurface(stair->bounds.center().z, config_)) {
        return false;
    }

    if (!stairDirectionAllowsTransition(stair_node,
                                        from_stair ? to_node : from_node,
                                        !from_stair,
                                        component)) {
        return false;
    }

    const float dz = std::abs(from->bounds.center().z - to->bounds.center().z);
    const float max_entry_step =
        std::max(config_.stair_floor_connection_max_vertical,
                 std::max(from->bounds.height(), to->bounds.height()) * 1.5f);
    return dz <= max_entry_step;
}

void AStarPlanner::ensureObstacleCache() const {
    if (obstacle_cache_valid_) {
        return;
    }

    obstacle_nodes_.clear();
    obstacle_grid_.clear();
    obstacle_max_planar_radius_ = 0.0f;
    const auto& nodes = octree_.nodes();
    obstacle_clearance_cache_.assign(nodes.size(), -1);
    obstacle_grid_cell_size_ =
        std::max(0.25f, config_.obstacle_clearance_radius + 0.35f);
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        const OctreeNode& node = nodes[static_cast<std::size_t>(i)];
        if (!node.leaf) {
            continue;
        }
        if (node.label == VoxelLabel::Obstacle ||
            node.obstacle_probability >= config_.obstacle_block_probability) {
            obstacle_nodes_.push_back(i);
            obstacle_max_planar_radius_ =
                std::max(obstacle_max_planar_radius_,
                         0.5f * std::max(node.bounds.width(), node.bounds.depth()));
            const Point3D center = node.bounds.center();
            const int cell_x = gridCell(center.x, obstacle_grid_cell_size_);
            const int cell_y = gridCell(center.y, obstacle_grid_cell_size_);
            obstacle_grid_[gridKey(cell_x, cell_y)].push_back(i);
        }
    }
    obstacle_cache_valid_ = true;
}

bool AStarPlanner::isInsideObstacleClearance(int node_index) const {
    if (!config_.enable_obstacle_clearance ||
        config_.obstacle_clearance_radius <= 0.0f) {
        return false;
    }

    const OctreeNode* node = octree_.getNode(node_index);
    if (node == nullptr || !node->leaf || node->label == VoxelLabel::Obstacle) {
        return false;
    }

    ensureObstacleCache();
    if (node_index >= 0 &&
        node_index < static_cast<int>(obstacle_clearance_cache_.size()) &&
        obstacle_clearance_cache_[static_cast<std::size_t>(node_index)] >= 0) {
        return obstacle_clearance_cache_[static_cast<std::size_t>(node_index)] != 0;
    }

    if (isStairLike(node_index)) {
        if (node_index >= 0 &&
            node_index < static_cast<int>(obstacle_clearance_cache_.size())) {
            obstacle_clearance_cache_[static_cast<std::size_t>(node_index)] = 0;
        }
        return false;
    }

    const Point3D center = node->bounds.center();
    const float node_radius =
        0.5f * std::max(node->bounds.width(), node->bounds.depth());
    const float max_search_radius =
        config_.obstacle_clearance_radius +
        node_radius +
        obstacle_max_planar_radius_ +
        obstacle_grid_cell_size_;
    const int search_cells =
        std::max(1, static_cast<int>(std::ceil(max_search_radius / obstacle_grid_cell_size_)));
    const int center_cell_x = gridCell(center.x, obstacle_grid_cell_size_);
    const int center_cell_y = gridCell(center.y, obstacle_grid_cell_size_);

    for (int dx = -search_cells; dx <= search_cells; ++dx) {
        for (int dy = -search_cells; dy <= search_cells; ++dy) {
            const auto it = obstacle_grid_.find(
                gridKey(center_cell_x + dx, center_cell_y + dy));
            if (it == obstacle_grid_.end()) {
                continue;
            }

            for (int obstacle_index : it->second) {
                const OctreeNode* obstacle = octree_.getNode(obstacle_index);
                if (obstacle == nullptr || !obstacle->leaf) {
                    continue;
                }
                const Point3D obstacle_center = obstacle->bounds.center();
                const float dz = std::abs(center.z - obstacle_center.z);
                const float z_tolerance =
                    std::max(config_.obstacle_clearance_z_tolerance,
                             0.5f * (node->bounds.height() + obstacle->bounds.height()));
                if (dz > z_tolerance) {
                    continue;
                }

                const float obstacle_radius =
                    0.5f * std::max(obstacle->bounds.width(), obstacle->bounds.depth());
                const float clearance =
                    config_.obstacle_clearance_radius + node_radius + obstacle_radius;
                if (planarDistance(center, obstacle_center) <= clearance) {
                    if (node_index >= 0 &&
                        node_index < static_cast<int>(obstacle_clearance_cache_.size())) {
                        obstacle_clearance_cache_[static_cast<std::size_t>(node_index)] = 1;
                    }
                    return true;
                }
            }
        }
    }

    if (node_index >= 0 &&
        node_index < static_cast<int>(obstacle_clearance_cache_.size())) {
        obstacle_clearance_cache_[static_cast<std::size_t>(node_index)] = 0;
    }
    return false;
}

bool AStarPlanner::segmentIntersectsBlockedObstacle(int from_node, int to_node) const {
    if (!config_.block_edges_through_obstacles) {
        return false;
    }

    const OctreeNode* from = octree_.getNode(from_node);
    const OctreeNode* to = octree_.getNode(to_node);
    if (from == nullptr || to == nullptr || !from->leaf || !to->leaf) {
        return true;
    }

    ensureObstacleCache();
    if (obstacle_nodes_.empty()) {
        return false;
    }

    const Point3D a = from->bounds.center();
    const Point3D b = to->bounds.center();
    const float segment_min_x = std::min(a.x, b.x) - obstacle_max_planar_radius_;
    const float segment_max_x = std::max(a.x, b.x) + obstacle_max_planar_radius_;
    const float segment_min_y = std::min(a.y, b.y) - obstacle_max_planar_radius_;
    const float segment_max_y = std::max(a.y, b.y) + obstacle_max_planar_radius_;
    const int min_cell_x = gridCell(segment_min_x, obstacle_grid_cell_size_);
    const int max_cell_x = gridCell(segment_max_x, obstacle_grid_cell_size_);
    const int min_cell_y = gridCell(segment_min_y, obstacle_grid_cell_size_);
    const int max_cell_y = gridCell(segment_max_y, obstacle_grid_cell_size_);

    for (int cell_x = min_cell_x; cell_x <= max_cell_x; ++cell_x) {
        for (int cell_y = min_cell_y; cell_y <= max_cell_y; ++cell_y) {
            const auto it = obstacle_grid_.find(gridKey(cell_x, cell_y));
            if (it == obstacle_grid_.end()) {
                continue;
            }

            for (int obstacle_index : it->second) {
                if (obstacle_index == from_node || obstacle_index == to_node) {
                    continue;
                }
                const OctreeNode* obstacle = octree_.getNode(obstacle_index);
                if (obstacle == nullptr || !obstacle->leaf) {
                    continue;
                }

                const Point3D obstacle_center = obstacle->bounds.center();
                const float obstacle_radius =
                    0.5f * std::sqrt(obstacle->bounds.width() * obstacle->bounds.width() +
                                      obstacle->bounds.depth() * obstacle->bounds.depth() +
                                      obstacle->bounds.height() * obstacle->bounds.height());
                const float clearance =
                    obstacle_radius + std::max(0.0f, config_.edge_obstacle_clearance_radius);
                if (distancePointToSegment3D(obstacle_center, a, b) <= clearance) {
                    return true;
                }
            }
        }
    }

    return false;
}

std::vector<int> AStarPlanner::getTraversalNeighbors(int node_index) const {
    std::vector<int> neighbors;
    const int node_count = octree_.getNodeCount();
    if (node_index < 0 || node_index >= node_count) {
        return neighbors;
    }

    const auto orthogonal = octree_.getOrthogonalNeighbors(node_index);
    for (int neighbor : orthogonal) {
        if (neighbor >= 0 && neighbor < node_count &&
            isAllowedStairTransition(node_index, neighbor) &&
            std::find(neighbors.begin(), neighbors.end(), neighbor) == neighbors.end()) {
            neighbors.push_back(neighbor);
        }
    }

    if (!config_.enable_stair_connection_edges || config_.stair_connection_radius <= 0.0f) {
        return neighbors;
    }

    const OctreeNode* current = octree_.getNode(node_index);
    if (current == nullptr || !isTraversable(node_index)) {
        return neighbors;
    }

    ensureStairCache();
    const Point3D current_center = current->bounds.center();
    const bool current_is_stair =
        current->label == VoxelLabel::Stair || current->is_cross_floor;
    const float radius = config_.stair_connection_radius;

    auto append_if_near = [&](int candidate_index) {
        if (candidate_index == node_index ||
            candidate_index < 0 ||
            candidate_index >= node_count ||
            std::find(neighbors.begin(), neighbors.end(), candidate_index) != neighbors.end()) {
            return;
        }

        const OctreeNode* candidate = octree_.getNode(candidate_index);
        if (candidate == nullptr || !candidate->leaf) {
            return;
        }
        const Point3D candidate_center = candidate->bounds.center();
        if (distanceBetween(current_center, candidate_center) > radius) {
            return;
        }

        const bool candidate_is_stair = isStairLike(candidate_index);
        if (!current_is_stair && !candidate_is_stair) {
            return;
        }

        if (!isTraversable(candidate_index)) {
            return;
        }

        if (!isAllowedStairTransition(node_index, candidate_index)) {
            return;
        }

        if (!config_.allow_cross_floor) {
            const float dz = std::abs(current_center.z - candidate_center.z);
            if (dz > std::max(current->bounds.height(), candidate->bounds.height())) {
                return;
            }
        }

        neighbors.push_back(candidate_index);
    };

    if (current_is_stair) {
        const auto& nodes = octree_.nodes();
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            if (nodes[static_cast<std::size_t>(i)].leaf) {
                append_if_near(i);
            }
        }
    } else {
        for (int stair_node : stair_nodes_) {
            append_if_near(stair_node);
        }
    }

    return neighbors;
}

float AStarPlanner::heuristicCost(int node_index, int goal_node) const {
    const OctreeNode* node = octree_.getNode(node_index);
    const OctreeNode* goal = octree_.getNode(goal_node);
    if (node == nullptr || goal == nullptr) {
        return kInfinity;
    }

    const Point3D node_center = node->bounds.center();
    const Point3D goal_center = goal->bounds.center();
    const float distance = distanceBetween(node_center, goal_center);
    const float vertical_distance = std::abs(node_center.z - goal_center.z);
    const float probability = clampFloat(node->obstacle_probability, 0.0f, 1.0f);

    float label_penalty = 0.0f;
    if (node->label == VoxelLabel::Stair) {
        label_penalty += config_.stair_weight;
    } else if (node->label == VoxelLabel::Obstacle) {
        label_penalty += 1000.0f;
    }

    float cross_floor_bias = 0.0f;
    if (vertical_distance > std::max(node->bounds.height(), goal->bounds.height()) &&
        !(node->is_cross_floor || node->label == VoxelLabel::Stair)) {
        cross_floor_bias = config_.cross_floor_weight;
    }

    const float risk_multiplier = 1.0f + config_.probability_weight * probability;
    return config_.heuristic_weight *
           (distance * risk_multiplier +
            config_.vertical_weight * vertical_distance +
            label_penalty +
            cross_floor_bias);
}

float AStarPlanner::edgeCost(int from_node, int to_node) const {
    if (!isAllowedStairTransition(from_node, to_node)) {
        return kInfinity;
    }

    const auto traversal =
        octree_.computeTraversalInfo(from_node, to_node, config_.allow_cross_floor);
    if (!traversal.has_value()) {
        return kInfinity;
    }

    const OctreeNode* to = octree_.getNode(to_node);
    if (to == nullptr) {
        return kInfinity;
    }
    const OctreeNode* from = octree_.getNode(from_node);
    if (from == nullptr) {
        return kInfinity;
    }
    const float dz = std::abs(from->bounds.center().z - to->bounds.center().z);
    const bool stair_transition = isStairLike(from_node) || isStairLike(to_node);
    if (!stair_transition &&
        dz > std::max(0.0f, config_.max_non_stair_vertical_step)) {
        return kInfinity;
    }
    if (to->obstacle_probability >= config_.obstacle_block_probability ||
        to->label == VoxelLabel::Obstacle) {
        return kInfinity;
    }

    const auto max_node_extent = [](const OctreeNode* node) {
        return std::max({node->bounds.width(),
                         node->bounds.depth(),
                         node->bounds.height()});
    };
    const float adjacent_edge_limit =
        1.75f * std::max(max_node_extent(from), max_node_extent(to));
    if (traversal->distance > adjacent_edge_limit &&
        segmentIntersectsBlockedObstacle(from_node, to_node)) {
        return kInfinity;
    }

    float cost = traversal->cost;
    const float p = clampFloat(traversal->obstacle_probability, 0.0f, 1.0f);
    cost += traversal->distance * config_.probability_weight * p * p;
    if (isInsideObstacleClearance(to_node)) {
        cost += traversal->distance * (4.0f + config_.probability_weight);
    }
    if (traversal->label == VoxelLabel::Stair) {
        cost += config_.stair_weight * traversal->distance;
    }
    if (traversal->is_cross_floor) {
        cost += config_.cross_floor_weight * traversal->distance;
    }

    cost += config_.vertical_weight * dz;
    return cost;
}

bool AStarPlanner::isTraversable(int node_index) const {
    const OctreeNode* node = octree_.getNode(node_index);
    if (node == nullptr || !node->leaf) {
        return false;
    }
    if (node->label == VoxelLabel::Obstacle) {
        return false;
    }
    if (node->obstacle_probability >= config_.obstacle_block_probability) {
        return false;
    }
    return true;
}

AStarPath AStarPlanner::reconstructPath(int start_node,
                                        int goal_node,
                                        const std::vector<int>& came_from,
                                        const std::vector<float>& g_score,
                                        int expanded_nodes) const {
    AStarPath result;
    result.success = true;
    result.message = "Path found";
    result.start_node = start_node;
    result.goal_node = goal_node;
    result.expanded_nodes = expanded_nodes;
    result.total_cost = g_score[static_cast<std::size_t>(goal_node)];

    int current = goal_node;
    while (current >= 0) {
        result.node_indices.push_back(current);
        if (current == start_node) {
            break;
        }
        current = came_from[static_cast<std::size_t>(current)];
    }

    if (result.node_indices.empty() || result.node_indices.back() != start_node) {
        result.success = false;
        result.message = "Failed to reconstruct path";
        result.node_indices.clear();
        result.waypoints.clear();
        return result;
    }

    std::reverse(result.node_indices.begin(), result.node_indices.end());
    result.waypoints = buildWaypoints(result.node_indices);
    if (config_.smooth_collinear_waypoints) {
        result.waypoints = smoothWaypoints(result.waypoints);
    }
    return result;
}

std::vector<PathWaypoint> AStarPlanner::buildWaypoints(
    const std::vector<int>& node_indices) const {
    std::vector<PathWaypoint> waypoints;
    waypoints.reserve(node_indices.size());
    for (int node_index : node_indices) {
        const OctreeNode* node = octree_.getNode(node_index);
        if (node == nullptr) {
            continue;
        }
        PathWaypoint waypoint;
        waypoint.node_index = node_index;
        waypoint.position = node->bounds.center();
        waypoint.label = node->label;
        waypoint.obstacle_probability = node->obstacle_probability;
        waypoint.is_cross_floor = node->is_cross_floor;
        waypoint.room_id = node->room_id;
        waypoints.push_back(waypoint);
    }
    return waypoints;
}

std::vector<PathWaypoint> AStarPlanner::smoothWaypoints(
    const std::vector<PathWaypoint>& waypoints) const {
    if (waypoints.size() <= 2) {
        return waypoints;
    }

    std::vector<PathWaypoint> smoothed;
    smoothed.reserve(waypoints.size());
    smoothed.push_back(waypoints.front());

    for (std::size_t i = 1; i + 1 < waypoints.size(); ++i) {
        const PathWaypoint& previous = smoothed.back();
        const PathWaypoint& current = waypoints[i];
        const PathWaypoint& next = waypoints[i + 1];

        const bool semantic_break =
            current.label != previous.label ||
            current.label != next.label ||
            current.is_cross_floor != previous.is_cross_floor ||
            current.is_cross_floor != next.is_cross_floor ||
            current.room_id != previous.room_id ||
            current.room_id != next.room_id ||
            labelRank(current.label) > labelRank(previous.label);
        const bool stair_or_cross_floor =
            previous.label == VoxelLabel::Stair ||
            current.label == VoxelLabel::Stair ||
            next.label == VoxelLabel::Stair ||
            previous.is_cross_floor ||
            current.is_cross_floor ||
            next.is_cross_floor;
        const bool vertical_break =
            std::abs(current.position.z - previous.position.z) > 0.15f ||
            std::abs(next.position.z - current.position.z) > 0.15f;
        if (semantic_break ||
            stair_or_cross_floor ||
            vertical_break ||
            !sameDirection(previous.position, current.position, next.position)) {
            smoothed.push_back(current);
        }
    }

    smoothed.push_back(waypoints.back());
    return smoothed;
}

} // namespace navigation
