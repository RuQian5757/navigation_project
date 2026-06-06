#include "octree_manager.h"

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace {

enum class VoxelMode {
    Centers,
    Boxes,
    Hybrid,
    CenterBoxes,
};

struct Options {
    std::string topic = "/world/dynamic_cloud";
    std::string partition = "dynamic_cloud_test";
    int max_depth = 9;
    int max_voxels = 6000;
    int max_render_points = 80000;
    double rebuild_hz = 1.0;
    bool show_points = true;
    bool show_voxels = true;
    bool color_by_depth = true;
    VoxelMode voxel_mode = VoxelMode::Centers;
};

struct LatestCloud {
    std::mutex mutex;
    std::vector<navigation::Point3D> points;
    std::uint64_t frame = 0;
    std::uint64_t source_points = 0;
    std::string status = "waiting";
};

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " [options]\n\n"
        << "Options:\n"
        << "  --topic NAME          Gazebo PointCloudPacked topic, default /world/dynamic_cloud\n"
        << "  --partition NAME      Gazebo partition, default dynamic_cloud_test\n"
        << "  --max-depth N         Octree max depth, default 9\n"
        << "  --max-voxels N        Maximum rendered leaf voxels, default 6000\n"
        << "  --max-render-points N Maximum rendered points, default 80000\n"
        << "  --rebuild-hz HZ       Max parse/octree/render rate, default 1\n"
        << "  --voxel-mode MODE     centers, boxes, hybrid, or center-boxes; default centers\n"
        << "  --no-points           Hide original point cloud\n"
        << "  --no-voxels           Hide octree voxel boxes\n"
        << "  --label-color         Color voxels by ML/navigation label instead of depth\n"
        << "  --help                Show this message\n\n"
        << "Build:\n"
        << "  cmake -S scripts -B build/octree_viewer\n"
        << "  cmake --build build/octree_viewer\n\n"
        << "Run with Gazebo:\n"
        << "  GZ_PARTITION=dynamic_cloud_test ./build/octree_viewer/visualize_octree_gazebo\n";
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc) {
            options.topic = argv[++i];
        } else if (arg == "--partition" && i + 1 < argc) {
            options.partition = argv[++i];
        } else if (arg == "--max-depth" && i + 1 < argc) {
            options.max_depth = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--max-voxels" && i + 1 < argc) {
            options.max_voxels = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--max-render-points" && i + 1 < argc) {
            options.max_render_points = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--rebuild-hz" && i + 1 < argc) {
            options.rebuild_hz = std::max(0.25, std::stod(argv[++i]));
        } else if (arg == "--voxel-mode" && i + 1 < argc) {
            const std::string mode = argv[++i];
            if (mode == "centers") {
                options.voxel_mode = VoxelMode::Centers;
            } else if (mode == "boxes") {
                options.voxel_mode = VoxelMode::Boxes;
            } else if (mode == "hybrid") {
                options.voxel_mode = VoxelMode::Hybrid;
            } else if (mode == "center-boxes" || mode == "both") {
                options.voxel_mode = VoxelMode::CenterBoxes;
            } else {
                std::cerr << "Unknown voxel mode: " << mode << "\n";
                return false;
            }
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

bool findXYZOffsets(const gz::msgs::PointCloudPacked& msg,
                    int& x_offset, int& y_offset, int& z_offset) {
    x_offset = -1;
    y_offset = -1;
    z_offset = -1;

    for (int i = 0; i < msg.field_size(); ++i) {
        const auto& field = msg.field(i);
        if (field.datatype() != gz::msgs::PointCloudPacked::Field::FLOAT32) {
            continue;
        }

        if (field.name() == "x") {
            x_offset = static_cast<int>(field.offset());
        } else if (field.name() == "y") {
            y_offset = static_cast<int>(field.offset());
        } else if (field.name() == "z") {
            z_offset = static_cast<int>(field.offset());
        } else if (field.name() == "xyz") {
            x_offset = static_cast<int>(field.offset());
            y_offset = x_offset + static_cast<int>(sizeof(float));
            z_offset = y_offset + static_cast<int>(sizeof(float));
        }
    }

    return x_offset >= 0 && y_offset >= 0 && z_offset >= 0;
}

std::vector<navigation::Point3D> parsePointCloudPacked(
    const gz::msgs::PointCloudPacked& msg, int max_render_points,
    std::uint64_t& source_points) {
    source_points = static_cast<std::uint64_t>(msg.width()) *
                    static_cast<std::uint64_t>(msg.height());
    if (source_points == 0 || msg.point_step() == 0) {
        return {};
    }

    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    if (!findXYZOffsets(msg, x_offset, y_offset, z_offset)) {
        throw std::runtime_error("PointCloudPacked does not contain FLOAT32 x/y/z or xyz fields");
    }

    const std::size_t point_step = static_cast<std::size_t>(msg.point_step());
    const std::size_t required = static_cast<std::size_t>(source_points) * point_step;
    if (msg.data().size() < required) {
        throw std::runtime_error("PointCloudPacked data is shorter than width*height*point_step");
    }

    const std::size_t stride = source_points > static_cast<std::uint64_t>(max_render_points)
                                   ? static_cast<std::size_t>(
                                         std::ceil(static_cast<double>(source_points) /
                                                   static_cast<double>(max_render_points)))
                                   : 1U;

    std::vector<navigation::Point3D> points;
    points.reserve(static_cast<std::size_t>(
        std::min<std::uint64_t>(source_points, static_cast<std::uint64_t>(max_render_points))));

    const char* data = msg.data().data();
    for (std::size_t i = 0; i < source_points; i += stride) {
        const std::size_t base = i * point_step;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, data + base + static_cast<std::size_t>(x_offset), sizeof(float));
        std::memcpy(&y, data + base + static_cast<std::size_t>(y_offset), sizeof(float));
        std::memcpy(&z, data + base + static_cast<std::size_t>(z_offset), sizeof(float));
        if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
            points.push_back({x, y, z});
        }
    }

    return points;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr toPclCloud(const std::vector<navigation::Point3D>& points) {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->reserve(points.size());
    for (const auto& p : points) {
        cloud->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
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

void clearVoxelShapes(pcl::visualization::PCLVisualizer& viewer, std::size_t count) {
    for (std::size_t i = 0; i < count; ++i) {
        std::ostringstream id;
        id << "voxel_" << i;
        viewer.removeShape(id.str());
    }
}

void clearVoxelCenterClouds(pcl::visualization::PCLVisualizer& viewer, int max_depth) {
    for (int depth = 0; depth <= max_depth; ++depth) {
        std::ostringstream id;
        id << "voxel_centers_depth_" << depth;
        viewer.removePointCloud(id.str());
    }
}

std::size_t addOctreeVoxels(pcl::visualization::PCLVisualizer& viewer,
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

    return leaves.size();
}

std::size_t addOrUpdateVoxelCenters(pcl::visualization::PCLVisualizer& viewer,
                                    const navigation::OctreeManager& octree,
                                    const Options& options) {
    const std::vector<int> leaves = collectLeafIndices(octree, options.max_voxels);
    const auto& nodes = octree.nodes();
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds(
        static_cast<std::size_t>(options.max_depth + 1));

    for (auto& cloud : clouds) {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    for (int leaf : leaves) {
        const navigation::OctreeNode& node = nodes[leaf];
        const int depth = std::max(0, std::min(options.max_depth, node.depth));
        const navigation::Point3D c = node.bounds.center();
        clouds[static_cast<std::size_t>(depth)]->push_back(pcl::PointXYZ(c.x, c.y, c.z));
    }

    for (int depth = 0; depth <= options.max_depth; ++depth) {
        auto& cloud = clouds[static_cast<std::size_t>(depth)];
        cloud->width = static_cast<std::uint32_t>(cloud->size());
        cloud->height = 1;
        cloud->is_dense = true;

        std::ostringstream id;
        id << "voxel_centers_depth_" << depth;

        if (cloud->empty()) {
            viewer.removePointCloud(id.str());
            continue;
        }

        double r = 0.0;
        double g = 0.0;
        double b = 0.0;
        colorByDepth(depth, options.max_depth, r, g, b);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            color(cloud, static_cast<int>(255.0 * r),
                  static_cast<int>(255.0 * g),
                  static_cast<int>(255.0 * b));

        if (!viewer.updatePointCloud<pcl::PointXYZ>(cloud, color, id.str())) {
            viewer.addPointCloud<pcl::PointXYZ>(cloud, color, id.str());
        }

        const double depth_ratio = options.max_depth > 0
                                       ? static_cast<double>(depth) / options.max_depth
                                       : 1.0;
        const double point_size = std::max(2.0, 11.0 - 8.0 * depth_ratio);
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size, id.str());
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_OPACITY, 0.85, id.str());
    }

    return leaves.size();
}

void addOrUpdatePointCloud(pcl::visualization::PCLVisualizer& viewer,
                           const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        point_color(cloud, 220, 220, 220);
    if (!viewer.updatePointCloud<pcl::PointXYZ>(cloud, point_color, "cloud")) {
        viewer.addPointCloud<pcl::PointXYZ>(cloud, point_color, "cloud");
        viewer.setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "cloud");
    }
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) {
        return 0;
    }

    setenv("GZ_PARTITION", options.partition.c_str(), 1);

    LatestCloud latest;
    std::atomic_bool running{true};
    std::atomic<std::int64_t> last_parse_ns{0};

    gz::transport::Node node;
    const int max_render_points = options.max_render_points;
    const auto parse_period_ns = static_cast<std::int64_t>(
        1.0e9 / std::max(0.25, options.rebuild_hz));
    std::function<void(const gz::msgs::PointCloudPacked&)> callback =
        [&latest, &last_parse_ns, max_render_points, parse_period_ns](
            const gz::msgs::PointCloudPacked& msg) {
            const auto now = std::chrono::steady_clock::now().time_since_epoch();
            const auto now_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            std::int64_t previous_ns = last_parse_ns.load(std::memory_order_relaxed);
            if (now_ns - previous_ns < parse_period_ns) {
                return;
            }
            if (!last_parse_ns.compare_exchange_strong(
                    previous_ns, now_ns, std::memory_order_relaxed)) {
                return;
            }

            try {
                std::uint64_t source_points = 0;
                std::vector<navigation::Point3D> points =
                    parsePointCloudPacked(msg, max_render_points, source_points);

                std::lock_guard<std::mutex> lock(latest.mutex);
                latest.points = std::move(points);
                latest.source_points = source_points;
                latest.frame += 1;
                latest.status = "streaming";
            } catch (const std::exception& e) {
                std::lock_guard<std::mutex> lock(latest.mutex);
                latest.status = std::string("parse error: ") + e.what();
            }
        };
    const bool subscribed = node.Subscribe<gz::msgs::PointCloudPacked>(
        options.topic, callback);

    if (!subscribed) {
        std::cerr << "Failed to subscribe to topic: " << options.topic << "\n";
        return 1;
    }

    std::cout << "Subscribed to " << options.topic
              << " on GZ_PARTITION=" << options.partition << "\n";

    pcl::visualization::PCLVisualizer viewer("Realtime Navigation Octree");
    viewer.setBackgroundColor(0.04, 0.04, 0.05);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();

    navigation::OctreeConfig config;
    config.max_depth = options.max_depth;

    std::uint64_t rendered_frame = 0;
    std::size_t rendered_voxels = 0;
    auto last_rebuild = std::chrono::steady_clock::time_point{};
    const auto rebuild_period = std::chrono::duration<double>(1.0 / options.rebuild_hz);

    while (running && !viewer.wasStopped()) {
        std::vector<navigation::Point3D> points;
        std::uint64_t frame = 0;
        std::uint64_t source_points = 0;
        std::string status;
        {
            std::lock_guard<std::mutex> lock(latest.mutex);
            frame = latest.frame;
            source_points = latest.source_points;
            status = latest.status;
            if (frame != rendered_frame) {
                points = latest.points;
            }
        }

        const auto now = std::chrono::steady_clock::now();
        if (!points.empty() && frame != rendered_frame &&
            now - last_rebuild >= rebuild_period) {
            navigation::OctreeManager octree(config);
            octree.initialize(points);

            if (options.show_points) {
                addOrUpdatePointCloud(viewer, toPclCloud(points));
            }
            if (options.show_voxels) {
                if (options.voxel_mode == VoxelMode::Centers) {
                    clearVoxelShapes(viewer, rendered_voxels);
                    rendered_voxels = addOrUpdateVoxelCenters(viewer, octree, options);
                } else if (options.voxel_mode == VoxelMode::Boxes) {
                    clearVoxelCenterClouds(viewer, options.max_depth);
                    clearVoxelShapes(viewer, rendered_voxels);
                    rendered_voxels = addOctreeVoxels(viewer, octree, options);
                } else if (options.voxel_mode == VoxelMode::CenterBoxes) {
                    clearVoxelShapes(viewer, rendered_voxels);
                    rendered_voxels = addOrUpdateVoxelCenters(viewer, octree, options);
                    addOctreeVoxels(viewer, octree, options);
                } else {
                    rendered_voxels = addOrUpdateVoxelCenters(viewer, octree, options);
                    clearVoxelShapes(viewer, rendered_voxels);
                    Options box_options = options;
                    box_options.max_voxels = std::min(options.max_voxels, 600);
                    addOctreeVoxels(viewer, octree, box_options);
                }
            }

            rendered_frame = frame;
            last_rebuild = now;

            std::ostringstream title;
            title << "Realtime Navigation Octree | frame=" << frame
                  << " source_points=" << source_points
                  << " rendered_points=" << points.size()
                  << " leaves=" << octree.getLeafCount()
                  << " voxels=" << rendered_voxels;
            viewer.setWindowName(title.str());
            std::cout << title.str() << "\n";
        } else if (frame == 0) {
            viewer.setWindowName("Realtime Navigation Octree | " + status);
        }

        viewer.spinOnce(16);
        std::this_thread::sleep_for(std::chrono::milliseconds(16));
    }

    running = false;
    return 0;
}
