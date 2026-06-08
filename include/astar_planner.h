#pragma once

#include "octree_manager.h"

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace navigation {

struct AStarPlannerConfig {
    bool allow_cross_floor = true;
    int max_expansions = 200000;
    float heuristic_weight = 1.15f;
    float probability_weight = 6.0f;
    float stair_weight = 1.25f;
    float cross_floor_weight = 2.0f;
    float vertical_weight = 0.75f;
    float obstacle_block_probability = 0.92f;
    bool enable_obstacle_clearance = true;
    float obstacle_clearance_radius = 0.15f;
    float obstacle_clearance_z_tolerance = 1.5f;
    bool block_edges_through_obstacles = true;
    float edge_obstacle_clearance_radius = 0.03f;
    bool smooth_collinear_waypoints = true;
    bool snap_endpoints_to_traversable = true;
    float endpoint_snap_radius = 1.5f;
    bool enable_stair_connection_edges = true;
    float stair_connection_radius = 1.25f;
    bool constrain_stair_transitions = true;
    float stair_endpoint_tolerance = 0.6f;
    float stair_step_max_vertical = 0.8f;
    float stair_floor_connection_max_vertical = 1.2f;
    float max_non_stair_vertical_step = 0.35f;
    bool constrain_stair_exits_to_floor_levels = true;
    float floor_z = 0.0f;
    float story_height = 4.0f;
    float floor_surface_offset = 1.0f;
    float stair_floor_exit_tolerance = 0.45f;
    bool constrain_stair_direction = true;
    float stair_direction_dot_min = 0.5f;
    float stair_entry_lateral_margin = 0.25f;
    float stair_entry_forward_margin = 0.55f;
};

struct PathWaypoint {
    int node_index = -1;
    Point3D position;
    VoxelLabel label = VoxelLabel::Free;
    float obstacle_probability = 0.0f;
    bool is_cross_floor = false;
    int room_id = 0;
};

struct AStarPath {
    bool success = false;
    std::string message;
    int start_node = -1;
    int goal_node = -1;
    bool start_snapped = false;
    bool goal_snapped = false;
    float start_snap_distance = 0.0f;
    float goal_snap_distance = 0.0f;
    float total_cost = 0.0f;
    int expanded_nodes = 0;
    std::vector<int> node_indices;
    std::vector<PathWaypoint> waypoints;
};

class AStarPlanner {
public:
    explicit AStarPlanner(const OctreeManager& octree,
                          AStarPlannerConfig config = AStarPlannerConfig{});

    AStarPath findPath(const Point3D& start, const Point3D& goal) const;
    AStarPath findPathByNodeIndex(int start_node, int goal_node) const;

private:
    struct StairComponent {
        float min_z = 0.0f;
        float max_z = 0.0f;
        Point3D low_center;
        Point3D high_center;
        Point3D ascent_direction;
        float low_half_width = 0.0f;
        float high_half_width = 0.0f;
        bool has_direction = false;
        int count = 0;
    };

    const OctreeManager& octree_;
    AStarPlannerConfig config_;
    mutable bool stair_cache_valid_ = false;
    mutable std::vector<int> stair_nodes_;
    mutable std::vector<int> stair_component_by_node_;
    mutable std::vector<StairComponent> stair_components_;
    mutable bool obstacle_cache_valid_ = false;
    mutable std::vector<int> obstacle_nodes_;
    mutable std::vector<int8_t> obstacle_clearance_cache_;
    mutable float obstacle_grid_cell_size_ = 0.5f;
    mutable float obstacle_max_planar_radius_ = 0.0f;
    mutable std::unordered_map<long long, std::vector<int>> obstacle_grid_;

    std::optional<int> resolveEndpointNode(const Point3D& point,
                                           bool* snapped,
                                           float* snap_distance) const;
    std::vector<int> getTraversalNeighbors(int node_index) const;
    void ensureStairCache() const;
    bool isStairLike(int node_index) const;
    bool stairNodesCanConnect(int from_node, int to_node) const;
    bool isStairEndpoint(int node_index) const;
    bool stairDirectionAllowsTransition(int stair_node,
                                        int free_node,
                                        bool entering_stair,
                                        const StairComponent& component) const;
    bool isAllowedStairTransition(int from_node, int to_node) const;
    void ensureObstacleCache() const;
    bool isInsideObstacleClearance(int node_index) const;
    bool segmentIntersectsBlockedObstacle(int from_node, int to_node) const;
    float heuristicCost(int node_index, int goal_node) const;
    float edgeCost(int from_node, int to_node) const;
    bool isTraversable(int node_index) const;
    AStarPath reconstructPath(int start_node,
                              int goal_node,
                              const std::vector<int>& came_from,
                              const std::vector<float>& g_score,
                              int expanded_nodes) const;
    std::vector<PathWaypoint> buildWaypoints(const std::vector<int>& node_indices) const;
    std::vector<PathWaypoint> smoothWaypoints(const std::vector<PathWaypoint>& waypoints) const;
};

} // namespace navigation
