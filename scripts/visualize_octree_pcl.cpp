#include "octree_manager.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

struct Options {
    std::string pcd_path;
    int max_depth = 9;
    int max_voxels = 25000;
    bool show_points = true;
    bool show_voxels = true;
    bool color_by_depth = true;
};

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " <cloud.pcd> [options]\n\n"
        << "Options:\n"
        << "  --max-depth N      Octree max depth, default 9\n"
        << "  --max-voxels N     Maximum rendered leaf voxels, default 25000\n"
        << "  --no-points        Hide original point cloud\n"
        << "  --no-voxels        Hide octree voxel boxes\n"
        << "  --label-color      Color voxels by ML/navigation label instead of depth\n"
        << "  --help             Show this message\n\n"
        << "Build example:\n"
        << "  cmake -S scripts -B build/octree_viewer\n"
        << "  cmake --build build/octree_viewer\n";
}

bool parseArgs(int argc, char** argv, Options& options) {
    if (argc < 2) {
        printUsage(argv[0]);
        return false;
    }

    options.pcd_path = argv[1];
    if (options.pcd_path == "--help" || options.pcd_path == "-h") {
        printUsage(argv[0]);
        return false;
    }

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--max-depth" && i + 1 < argc) {
            options.max_depth = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--max-voxels" && i + 1 < argc) {
            options.max_voxels = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--no-points") {
            options.show_points = false;
        } else if (arg == "--no-voxels") {
            options.show_voxels = false;
        } else if (arg == "--label-color") {
            options.color_by_depth = false;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }

    return true;
}

std::vector<navigation::Point3D> convertCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) {
    std::vector<navigation::Point3D> points;
    points.reserve(cloud->size());
    for (const pcl::PointXYZ& p : cloud->points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
            continue;
        }
        points.push_back({p.x, p.y, p.z});
    }
    return points;
}

void colorByDepth(int depth, int max_depth, double& r, double& g, double& b) {
    const double t = max_depth > 0
                         ? std::min(1.0, static_cast<double>(depth) / static_cast<double>(max_depth))
                         : 0.0;
    r = 0.15 + 0.85 * t;
    g = 0.85 - 0.55 * t;
    b = 1.00 - 0.75 * t;
}

void colorByLabel(navigation::VoxelLabel label, float probability, bool cross_floor,
                  double& r, double& g, double& b) {
    if (cross_floor || label == navigation::VoxelLabel::Stair) {
        r = 0.10;
        g = 0.45;
        b = 1.00;
        return;
    }
    if (label == navigation::VoxelLabel::Obstacle || probability >= 0.70f) {
        r = 1.00;
        g = 0.20;
        b = 0.12;
        return;
    }
    r = 0.12;
    g = 0.85;
    b = 0.35;
}

std::vector<int> collectLeafIndices(const navigation::OctreeManager& octree, int max_voxels) {
    std::vector<int> leaves;
    const auto& nodes = octree.nodes();
    leaves.reserve(nodes.size());
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        if (nodes[i].leaf) {
            leaves.push_back(i);
        }
    }

    if (static_cast<int>(leaves.size()) <= max_voxels) {
        return leaves;
    }

    std::vector<int> sampled;
    sampled.reserve(max_voxels);
    const double step = static_cast<double>(leaves.size()) / static_cast<double>(max_voxels);
    for (int i = 0; i < max_voxels; ++i) {
        const std::size_t idx = static_cast<std::size_t>(std::floor(i * step));
        sampled.push_back(leaves[std::min(idx, leaves.size() - 1)]);
    }
    return sampled;
}

void addOctreeVoxels(pcl::visualization::PCLVisualizer& viewer,
                     const navigation::OctreeManager& octree,
                     const Options& options) {
    const std::vector<int> leaves = collectLeafIndices(octree, options.max_voxels);
    const auto& nodes = octree.nodes();

    for (std::size_t i = 0; i < leaves.size(); ++i) {
        const navigation::OctreeNode& node = nodes[leaves[i]];

        double r = 0.0;
        double g = 0.0;
        double b = 0.0;
        if (options.color_by_depth) {
            colorByDepth(node.depth, options.max_depth, r, g, b);
        } else {
            colorByLabel(node.label, node.obstacle_probability, node.is_cross_floor, r, g, b);
        }

        std::ostringstream id;
        id << "voxel_" << i;
        viewer.addCube(node.bounds.min.x, node.bounds.max.x,
                       node.bounds.min.y, node.bounds.max.y,
                       node.bounds.min.z, node.bounds.max.z,
                       r, g, b, id.str());
        viewer.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
            id.str());
        viewer.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.0, id.str());
        viewer.setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.35, id.str());
    }
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) {
        return argc < 2 ? 1 : 0;
    }

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(options.pcd_path, *cloud) != 0) {
        std::cerr << "Failed to load PCD file: " << options.pcd_path << "\n";
        return 1;
    }

    const std::vector<navigation::Point3D> points = convertCloud(cloud);
    if (points.empty()) {
        std::cerr << "PCD contains no finite XYZ points.\n";
        return 1;
    }

    navigation::OctreeConfig config;
    config.max_depth = options.max_depth;
    navigation::OctreeManager octree(config);
    octree.initialize(points);

    std::cout << "Loaded points: " << points.size() << "\n";
    std::cout << "Octree nodes:  " << octree.getNodeCount() << "\n";
    std::cout << "Octree leaves: " << octree.getLeafCount() << "\n";

    pcl::visualization::PCLVisualizer viewer("Navigation Octree PCL Viewer");
    viewer.setBackgroundColor(0.04, 0.04, 0.05);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    if (options.show_points) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            point_color(cloud, 220, 220, 220);
        viewer.addPointCloud<pcl::PointXYZ>(cloud, point_color, "cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud");
    }

    if (options.show_voxels) {
        addOctreeVoxels(viewer, octree, options);
    }

    while (!viewer.wasStopped()) {
        viewer.spinOnce(16);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    return 0;
}
