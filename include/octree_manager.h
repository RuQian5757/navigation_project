#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <vector>

namespace navigation {

struct Point3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

struct BBox {
    Point3D min;
    Point3D max;

    float width() const { return max.x - min.x; }
    float depth() const { return max.y - min.y; }
    float height() const { return max.z - min.z; }
    float volume() const { return width() * depth() * height(); }
    Point3D center() const {
        return Point3D{
            0.5f * (min.x + max.x),
            0.5f * (min.y + max.y),
            0.5f * (min.z + max.z),
        };
    }
    bool contains(const Point3D& p) const {
        return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y && p.z >= min.z && p.z <= max.z;
    }
};

enum class VoxelLabel : uint8_t {
    Free = 0,
    Obstacle = 1,
    Stair = 2,
};

enum NeighborDirection : int {
    POS_X = 0,
    NEG_X = 1,
    POS_Y = 2,
    NEG_Y = 3,
    POS_Z = 4,
    NEG_Z = 5,
    NEIGHBOR_COUNT = 6
};

struct OctreeNode {
    uint64_t morton_code = 0;
    int depth = 0;
    bool leaf = true;
    bool is_cross_floor = false;
    VoxelLabel label = VoxelLabel::Free;
    float obstacle_probability = 0.0f;
    int room_id = 0;
    int point_count = 0;
    float node_volume = 0.0f;
    std::array<int, 8> children;
    std::array<int, NEIGHBOR_COUNT> orthogonal_neighbors;
    BBox bounds;

    OctreeNode() {
        children.fill(-1);
        orthogonal_neighbors.fill(-1);
    }
};

class OctreeManager {
public:
    explicit OctreeManager(int max_depth = 9);

    void initialize(const std::vector<Point3D>& points);
    void updateFromPointCloud(const std::vector<Point3D>& updated_points);

    std::array<int, NEIGHBOR_COUNT> getOrthogonalNeighbors(int node_index) const;
    std::optional<int> findLeafIndexByPoint(const Point3D& point) const;
    bool isCrossFloorConnected(int node_index) const;
    float computeTraversalCost(int node_index, bool allow_cross_floor = false) const;
    void assignLeafLabel(int node_index, VoxelLabel label, float obstacle_probability, int room_id, bool is_cross_floor = false);

    int getLeafCount() const;
    int getNodeCount() const;
    const OctreeNode* getNode(int index) const;

private:
    int max_depth_;
    std::vector<OctreeNode> nodes_;

    void buildLinearOctree(const std::vector<Point3D>& points);
    void subdivideNode(int node_index, const std::vector<Point3D>& points);
    bool shouldSubdivide(int point_count, float voxel_size, float density, int depth) const;
    void updateNeighborLinks(int node_index);
    int findContainingLeaf(int node_index, const Point3D& point) const;
    std::optional<int> findNeighborByDirection(int node_index, NeighborDirection dir) const;
    void floodFillLabelPropagation(int start_node);
    int getChildOctant(const Point3D& point, const BBox& bounds) const;
    Point3D computeOctantCenter(const BBox& bounds, int child_index) const;
    uint64_t computeMortonCode(const Point3D& point, int depth) const;
    uint64_t interleaveBits(uint32_t x, uint32_t y, uint32_t z) const;
};

} // namespace navigation
