#include "leaf_feature_exporter.h"
#include "octree_manager.h"

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic_bool g_running{true};

struct Options {
    std::string topic = "/world/dynamic_cloud";
    std::string partition = "dynamic_cloud_test";
    std::string output = "data/leaf_features.csv";
    int max_depth = 9;
    int max_points = 120000;
    double export_hz = 1.0;
    bool once = false;
    bool timestamped = false;
};

void onSignal(int) {
    g_running = false;
}

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " [options]\n\n"
        << "Options:\n"
        << "  --topic NAME          Gazebo PointCloudPacked topic, default /world/dynamic_cloud\n"
        << "  --partition NAME      Gazebo partition, default dynamic_cloud_test\n"
        << "  --output FILE         CSV output path, default data/leaf_features.csv\n"
        << "  --max-depth N         Viewer/export Octree max depth, default 9\n"
        << "  --max-points N        Max received points used per export, default 120000\n"
        << "  --export-hz HZ        Max CSV export rate, default 1\n"
        << "  --once                Export the first received cloud and exit\n"
        << "  --timestamped         Write output_stem_frameNNNNNN.csv instead of overwriting\n"
        << "  --help                Show this message\n";
}

enum class ParseResult {
    Ok,
    Help,
    Error,
};

ParseResult parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc) {
            options.topic = argv[++i];
        } else if (arg == "--partition" && i + 1 < argc) {
            options.partition = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            options.output = argv[++i];
        } else if (arg == "--max-depth" && i + 1 < argc) {
            options.max_depth = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--max-points" && i + 1 < argc) {
            options.max_points = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--export-hz" && i + 1 < argc) {
            options.export_hz = std::max(0.05, std::stod(argv[++i]));
        } else if (arg == "--once") {
            options.once = true;
        } else if (arg == "--timestamped") {
            options.timestamped = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return ParseResult::Help;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return ParseResult::Error;
        }
    }
    return ParseResult::Ok;
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
    const gz::msgs::PointCloudPacked& msg,
    int max_points,
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

    const std::size_t stride = source_points > static_cast<std::uint64_t>(max_points)
                                   ? static_cast<std::size_t>(
                                         std::ceil(static_cast<double>(source_points) /
                                                   static_cast<double>(max_points)))
                                   : 1U;

    std::vector<navigation::Point3D> points;
    points.reserve(static_cast<std::size_t>(
        std::min<std::uint64_t>(source_points, static_cast<std::uint64_t>(max_points))));

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

std::string outputPathForFrame(const std::string& output, std::uint64_t frame, bool timestamped) {
    if (!timestamped) {
        return output;
    }

    const std::filesystem::path base(output);
    const std::filesystem::path parent = base.parent_path();
    const std::string stem = base.stem().string();
    const std::string ext = base.extension().empty() ? ".csv" : base.extension().string();

    std::ostringstream filename;
    filename << stem << "_frame" << std::setw(6) << std::setfill('0') << frame << ext;
    return (parent / filename.str()).string();
}

void ensureParentDirectory(const std::string& output) {
    const std::filesystem::path parent = std::filesystem::path(output).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    const ParseResult parse_result = parseArgs(argc, argv, options);
    if (parse_result == ParseResult::Help) {
        return 0;
    } else if (parse_result == ParseResult::Error) {
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    setenv("GZ_PARTITION", options.partition.c_str(), 1);
    ensureParentDirectory(options.output);

    std::atomic_bool busy{false};
    std::atomic_bool exported_once{false};
    std::atomic<std::int64_t> last_export_ns{0};
    std::atomic<std::uint64_t> frame_counter{0};
    const auto export_period_ns = static_cast<std::int64_t>(1.0e9 / options.export_hz);

    gz::transport::Node node;
    std::function<void(const gz::msgs::PointCloudPacked&)> callback =
        [&](const gz::msgs::PointCloudPacked& msg) {
            if (options.once && exported_once.load(std::memory_order_relaxed)) {
                return;
            }

            const auto now = std::chrono::steady_clock::now().time_since_epoch();
            const auto now_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            std::int64_t previous_ns = last_export_ns.load(std::memory_order_relaxed);
            if (!options.once && now_ns - previous_ns < export_period_ns) {
                return;
            }
            if (!last_export_ns.compare_exchange_strong(previous_ns, now_ns, std::memory_order_relaxed)) {
                return;
            }
            bool expected_busy = false;
            if (!busy.compare_exchange_strong(expected_busy, true, std::memory_order_acq_rel)) {
                return;
            }

            try {
                std::uint64_t source_points = 0;
                const std::vector<navigation::Point3D> points =
                    parsePointCloudPacked(msg, options.max_points, source_points);
                if (points.empty()) {
                    std::cerr << "[leaf_feature_exporter] Received empty point cloud\n";
                    busy = false;
                    return;
                }

                navigation::OctreeConfig config;
                config.max_depth = options.max_depth;
                navigation::OctreeManager octree(config);
                octree.initialize(points);

                const std::uint64_t frame = ++frame_counter;
                const std::string output_path =
                    outputPathForFrame(options.output, frame, options.timestamped);
                ensureParentDirectory(output_path);
                navigation::exportOctreeLeafFeaturesToCSV(octree.nodes(), output_path);

                std::cout << "[leaf_feature_exporter] frame=" << frame
                          << " source_points=" << source_points
                          << " used_points=" << points.size()
                          << " leaves=" << octree.getLeafCount()
                          << " output='" << output_path << "'\n";

                exported_once = true;
                if (options.once) {
                    g_running = false;
                }
            } catch (const std::exception& e) {
                std::cerr << "[leaf_feature_exporter] export failed: " << e.what() << "\n";
            }

            busy = false;
        };

    const bool subscribed = node.Subscribe<gz::msgs::PointCloudPacked>(options.topic, callback);
    if (!subscribed) {
        std::cerr << "Failed to subscribe to topic: " << options.topic << "\n";
        return 1;
    }

    std::cout << "[leaf_feature_exporter] subscribed topic='" << options.topic
              << "' partition='" << options.partition
              << "' output='" << options.output << "'\n";

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
