#include <iostream>
#include "octree_manager.h"

int main(int argc, char** argv) {
    std::vector<navigation::Point3D> points;
    for (int x = 0; x < 10; ++x) {
        for (int y = 0; y < 10; ++y) {
            points.push_back({x * 1.0f, y * 1.0f, 1.0f});
        }
    }

    navigation::OctreeManager octree;
    octree.initialize(points);

    std::cout << "Navigation Octree build complete." << std::endl;
    std::cout << "Node count: " << octree.getNodeCount() << std::endl;
    std::cout << "Leaf count: " << octree.getLeafCount() << std::endl;
    return 0;
}
