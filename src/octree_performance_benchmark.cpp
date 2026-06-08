#include "astar_planner.h"
#include "octree_manager.h"
#include "random_forest_voxel_predictor.h"

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

std::atomic_bool g_running{true};

struct PackedFieldOffsets {
    int x = -1;
    int y = -1;
    int z = -1;
    int label = -1;
    int obstacle_probability = -1;
    int entity_id = -1;
};

struct ParsedCloud {
    std::vector<navigation::PointCloudSample> samples;
    bool has_semantics = false;
};

struct Options {
    std::string topic = "/world/dynamic_cloud";
    std::string partition = "dynamic_cloud_test";
    std::string rf_model = "models/random_forest_voxel_model.rf.txt";
    std::string output = "data/performance_benchmark.csv";
    navigation::Point3D start{6.0f, -2.0f, 1.2f};
    navigation::Point3D goal{0.0f, 0.0f, 9.2f};
    int max_depth = 9;
    int max_points = 50000;
    int frames = 3;
    double timeout_sec = 60.0;
    navigation::RandomForestFeatureConfig feature_config;
    navigation::AStarPlannerConfig planner_config;
};

struct BenchmarkResult {
    std::uint64_t frame = 0;
    std::string mode;
    std::uint64_t source_points = 0;
    std::size_t used_points = 0;
    double parse_ms = 0.0;
    double build_ms = 0.0;
    double plan_ms = 0.0;
    double total_ms = 0.0;
    std::size_t node_count = 0;
    std::size_t leaf_count = 0;
    double estimated_node_memory_kb = 0.0;
    long rss_kb_after = 0;
    long peak_rss_kb = 0;
    bool path_success = false;
    int expanded_nodes = 0;
    std::size_t waypoints = 0;
    float path_cost = 0.0f;
    std::string message;
};

double nowMs() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double, std::milli>(
               clock::now().time_since_epoch())
        .count();
}

void onSignal(int) {
    g_running = false;
}

long currentRssKb() {
    std::ifstream statm("/proc/self/statm");
    long pages = 0;
    long resident = 0;
    statm >> pages >> resident;
    const long page_size_kb = sysconf(_SC_PAGESIZE) / 1024;
    return resident * page_size_kb;
}

long peakRssKb() {
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) != 0) {
        return 0;
    }
    return usage.ru_maxrss;
}

bool parsePoint3D(const std::string& text, navigation::Point3D& point) {
    std::stringstream ss(text);
    std::string token;
    std::vector<float> values;
    while (std::getline(ss, token, ',')) {
        if (token.empty()) {
            return false;
        }
        try {
            values.push_back(static_cast<float>(std::stof(token)));
        } catch (...) {
            return false;
        }
    }
    if (values.size() != 3) {
        return false;
    }
    point = {values[0], values[1], values[2]};
    return true;
}

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " [options]\n\n"
        << "Options:\n"
        << "  --topic NAME              Gazebo PointCloudPacked topic, default /world/dynamic_cloud\n"
        << "  --partition NAME          Gazebo transport partition, default dynamic_cloud_test\n"
        << "  --rf-model FILE           C++ RF text model, default models/random_forest_voxel_model.rf.txt\n"
        << "  --output FILE             CSV output, default data/performance_benchmark.csv\n"
        << "  --start X,Y,Z             Start position, default 6,-2,1.2\n"
        << "  --goal X,Y,Z              Goal position, default 0,0,9.2\n"
        << "  --frames N                Number of point cloud frames to benchmark, default 3\n"
        << "  --timeout-sec S           Stop waiting after this many seconds, default 60\n"
        << "  --max-depth N             Octree max depth, default 9\n"
        << "  --max-points N            Max points used per frame, default 50000\n"
        << "  --block-probability P     Block leaves above probability, default 0.92\n"
        << "  --probability-weight W    A* probability cost weight, default 6\n"
        << "  --vertical-weight W       Vertical movement cost weight, default 0.75\n"
        << "  --stair-connection-radius M Stair connector radius, default 1.25\n"
        << "  --max-non-stair-vertical-step M Non-stair vertical jump limit, default 0.35\n"
        << "  --stair-floor-exit-tolerance M Stair exit floor-level tolerance, default 0.45\n"
        << "  --floor-z Z               Story-0 origin, default 0\n"
        << "  --story-height H          Floor-to-floor height, default 4\n"
        << "  --floor-surface-offset Z  Walkable floor surface offset, default 1\n"
        << "  --help                    Show this message\n";
}

bool parseArgs(int argc, char** argv, Options& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--topic" && i + 1 < argc) {
            options.topic = argv[++i];
        } else if (arg == "--partition" && i + 1 < argc) {
            options.partition = argv[++i];
        } else if (arg == "--rf-model" && i + 1 < argc) {
            options.rf_model = argv[++i];
        } else if (arg == "--output" && i + 1 < argc) {
            options.output = argv[++i];
        } else if (arg == "--start" && i + 1 < argc) {
            if (!parsePoint3D(argv[++i], options.start)) {
                std::cerr << "Invalid --start, expected X,Y,Z\n";
                return false;
            }
        } else if (arg == "--goal" && i + 1 < argc) {
            if (!parsePoint3D(argv[++i], options.goal)) {
                std::cerr << "Invalid --goal, expected X,Y,Z\n";
                return false;
            }
        } else if (arg == "--frames" && i + 1 < argc) {
            options.frames = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--timeout-sec" && i + 1 < argc) {
            options.timeout_sec = std::max(1.0, std::stod(argv[++i]));
        } else if (arg == "--max-depth" && i + 1 < argc) {
            options.max_depth = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--max-points" && i + 1 < argc) {
            options.max_points = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--block-probability" && i + 1 < argc) {
            options.planner_config.obstacle_block_probability =
                std::clamp(static_cast<float>(std::stof(argv[++i])), 0.0f, 1.0f);
        } else if (arg == "--probability-weight" && i + 1 < argc) {
            options.planner_config.probability_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--vertical-weight" && i + 1 < argc) {
            options.planner_config.vertical_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-connection-radius" && i + 1 < argc) {
            options.planner_config.stair_connection_radius =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--max-non-stair-vertical-step" && i + 1 < argc) {
            options.planner_config.max_non_stair_vertical_step =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-floor-exit-tolerance" && i + 1 < argc) {
            options.planner_config.stair_floor_exit_tolerance =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--floor-z" && i + 1 < argc) {
            options.feature_config.floor_z = static_cast<float>(std::stof(argv[++i]));
            options.planner_config.floor_z = options.feature_config.floor_z;
        } else if (arg == "--story-height" && i + 1 < argc) {
            options.feature_config.story_height =
                std::max(0.001f, static_cast<float>(std::stof(argv[++i])));
            options.planner_config.story_height = options.feature_config.story_height;
        } else if (arg == "--floor-surface-offset" && i + 1 < argc) {
            options.feature_config.floor_surface_offset =
                static_cast<float>(std::stof(argv[++i]));
            options.planner_config.floor_surface_offset =
                options.feature_config.floor_surface_offset;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }
    return true;
}

bool findFieldOffsets(const gz::msgs::PointCloudPacked& msg,
                      PackedFieldOffsets& offsets) {
    for (int i = 0; i < msg.field_size(); ++i) {
        const auto& field = msg.field(i);
        if (field.datatype() == gz::msgs::PointCloudPacked::Field::FLOAT32) {
            if (field.name() == "x") {
                offsets.x = static_cast<int>(field.offset());
            } else if (field.name() == "y") {
                offsets.y = static_cast<int>(field.offset());
            } else if (field.name() == "z") {
                offsets.z = static_cast<int>(field.offset());
            } else if (field.name() == "xyz") {
                offsets.x = static_cast<int>(field.offset());
                offsets.y = offsets.x + static_cast<int>(sizeof(float));
                offsets.z = offsets.y + static_cast<int>(sizeof(float));
            } else if (field.name() == "obstacle_probability") {
                offsets.obstacle_probability = static_cast<int>(field.offset());
            }
        } else if (field.datatype() == gz::msgs::PointCloudPacked::Field::UINT32) {
            if (field.name() == "label") {
                offsets.label = static_cast<int>(field.offset());
            } else if (field.name() == "entity_id") {
                offsets.entity_id = static_cast<int>(field.offset());
            }
        }
    }
    return offsets.x >= 0 && offsets.y >= 0 && offsets.z >= 0;
}

navigation::VoxelLabel toVoxelLabel(std::uint32_t label) {
    if (label == static_cast<std::uint32_t>(navigation::VoxelLabel::Obstacle)) {
        return navigation::VoxelLabel::Obstacle;
    }
    if (label == static_cast<std::uint32_t>(navigation::VoxelLabel::Stair)) {
        return navigation::VoxelLabel::Stair;
    }
    return navigation::VoxelLabel::Free;
}

float clampProbability(float value) {
    if (!std::isfinite(value)) {
        return 0.0f;
    }
    return std::max(0.0f, std::min(1.0f, value));
}

ParsedCloud parsePointCloudPacked(const gz::msgs::PointCloudPacked& msg,
                                  int max_points,
                                  std::uint64_t& source_points) {
    source_points = static_cast<std::uint64_t>(msg.width()) *
                    static_cast<std::uint64_t>(msg.height());
    if (source_points == 0 || msg.point_step() == 0) {
        return {};
    }

    PackedFieldOffsets offsets;
    if (!findFieldOffsets(msg, offsets)) {
        throw std::runtime_error("PointCloudPacked does not contain x/y/z fields");
    }

    const std::size_t point_step = static_cast<std::size_t>(msg.point_step());
    const std::size_t required = static_cast<std::size_t>(source_points) * point_step;
    if (msg.data().size() < required) {
        throw std::runtime_error("PointCloudPacked data shorter than width*height*point_step");
    }

    const std::size_t stride =
        source_points > static_cast<std::uint64_t>(max_points)
            ? static_cast<std::size_t>(std::ceil(static_cast<double>(source_points) /
                                                 static_cast<double>(max_points)))
            : 1U;

    ParsedCloud parsed;
    parsed.has_semantics = offsets.label >= 0 && offsets.obstacle_probability >= 0;
    parsed.samples.reserve(static_cast<std::size_t>(
        std::min<std::uint64_t>(source_points, static_cast<std::uint64_t>(max_points))));

    const char* data = msg.data().data();
    for (std::size_t i = 0; i < source_points; i += stride) {
        const std::size_t base = i * point_step;
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        std::memcpy(&x, data + base + static_cast<std::size_t>(offsets.x), sizeof(float));
        std::memcpy(&y, data + base + static_cast<std::size_t>(offsets.y), sizeof(float));
        std::memcpy(&z, data + base + static_cast<std::size_t>(offsets.z), sizeof(float));
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
            continue;
        }

        navigation::PointCloudSample sample;
        sample.point = {x, y, z};
        if (parsed.has_semantics) {
            std::uint32_t label = 0;
            std::uint32_t entity_id = 0;
            float probability = 0.0f;
            std::memcpy(&label, data + base + static_cast<std::size_t>(offsets.label),
                        sizeof(label));
            std::memcpy(&probability,
                        data + base + static_cast<std::size_t>(offsets.obstacle_probability),
                        sizeof(probability));
            if (offsets.entity_id >= 0) {
                std::memcpy(&entity_id,
                            data + base + static_cast<std::size_t>(offsets.entity_id),
                            sizeof(entity_id));
            }
            sample.label = toVoxelLabel(label);
            sample.obstacle_probability = clampProbability(probability);
            sample.entity_id = entity_id;
            sample.has_semantics = true;
            sample.is_cross_floor = sample.label == navigation::VoxelLabel::Stair;
        }
        parsed.samples.push_back(sample);
    }
    return parsed;
}

std::string csvEscape(const std::string& text) {
    if (text.find_first_of(",\"\n") == std::string::npos) {
        return text;
    }
    std::string escaped = "\"";
    for (char c : text) {
        if (c == '"') {
            escaped += "\"\"";
        } else {
            escaped += c;
        }
    }
    escaped += '"';
    return escaped;
}

void writeHeaderIfNeeded(const std::string& output) {
    const bool exists = std::filesystem::exists(output) &&
                        std::filesystem::file_size(output) > 0;
    if (exists) {
        return;
    }
    const auto parent = std::filesystem::path(output).parent_path();
    if (!parent.empty()) {
        std::filesystem::create_directories(parent);
    }
    std::ofstream file(output, std::ios::app);
    file << "frame,mode,source_points,used_points,parse_ms,build_ms,plan_ms,total_ms,"
         << "node_count,leaf_count,estimated_node_memory_kb,rss_kb_after,peak_rss_kb,"
         << "path_success,expanded_nodes,waypoints,path_cost,message\n";
}

void appendResultCsv(const std::string& output, const BenchmarkResult& r) {
    std::ofstream file(output, std::ios::app);
    file << r.frame << ','
         << r.mode << ','
         << r.source_points << ','
         << r.used_points << ','
         << r.parse_ms << ','
         << r.build_ms << ','
         << r.plan_ms << ','
         << r.total_ms << ','
         << r.node_count << ','
         << r.leaf_count << ','
         << r.estimated_node_memory_kb << ','
         << r.rss_kb_after << ','
         << r.peak_rss_kb << ','
         << (r.path_success ? 1 : 0) << ','
         << r.expanded_nodes << ','
         << r.waypoints << ','
         << r.path_cost << ','
         << csvEscape(r.message) << '\n';
}

BenchmarkResult runMode(const Options& options,
                        const ParsedCloud& parsed,
                        std::uint64_t frame,
                        std::uint64_t source_points,
                        double parse_ms,
                        const std::string& mode,
                        const navigation::RandomForestVoxelPredictor* rf_predictor) {
    BenchmarkResult result;
    result.frame = frame;
    result.mode = mode;
    result.source_points = source_points;
    result.used_points = parsed.samples.size();
    result.parse_ms = parse_ms;

    navigation::OctreeConfig octree_config;
    octree_config.max_depth = options.max_depth;

    const double build_start = nowMs();
    navigation::OctreeManager octree(octree_config);
    if (rf_predictor != nullptr) {
        octree.setMLPredictor(rf_predictor->asMLPredictor());
    }
    octree.initialize(parsed.samples);
    const double build_done = nowMs();

    navigation::AStarPlanner planner(octree, options.planner_config);
    const double plan_start = nowMs();
    const navigation::AStarPath path = planner.findPath(options.start, options.goal);
    const double plan_done = nowMs();

    result.build_ms = build_done - build_start;
    result.plan_ms = plan_done - plan_start;
    result.total_ms = result.parse_ms + result.build_ms + result.plan_ms;
    result.node_count = octree.nodes().size();
    result.leaf_count = static_cast<std::size_t>(octree.getLeafCount());
    result.estimated_node_memory_kb =
        static_cast<double>(octree.nodes().size() * sizeof(navigation::OctreeNode)) / 1024.0;
    result.rss_kb_after = currentRssKb();
    result.peak_rss_kb = peakRssKb();
    result.path_success = path.success;
    result.expanded_nodes = path.expanded_nodes;
    result.waypoints = path.waypoints.size();
    result.path_cost = path.total_cost;
    result.message = path.message;
    return result;
}

void printResult(const BenchmarkResult& r) {
    std::cout << "[benchmark] frame=" << r.frame
              << " mode=" << r.mode
              << " points=" << r.used_points
              << " leaves=" << r.leaf_count
              << " path=" << (r.path_success ? "success" : "failed")
              << " expanded=" << r.expanded_nodes
              << " waypoints=" << r.waypoints
              << " parse_ms=" << r.parse_ms
              << " build_ms=" << r.build_ms
              << " plan_ms=" << r.plan_ms
              << " total_ms=" << r.total_ms
              << " rss_kb=" << r.rss_kb_after
              << " message='" << r.message << "'\n";
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArgs(argc, argv, options)) {
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    setenv("GZ_PARTITION", options.partition.c_str(), 1);

    navigation::RandomForestVoxelPredictor rf_predictor(options.feature_config);
    try {
        rf_predictor.loadFromTextModel(options.rf_model);
    } catch (const std::exception& e) {
        std::cerr << "[benchmark] Failed to load RF model '" << options.rf_model
                  << "': " << e.what() << "\n";
        return 2;
    }

    writeHeaderIfNeeded(options.output);

    std::atomic_bool busy{false};
    std::atomic_int processed_frames{0};
    std::mutex output_mutex;
    const double start_wall = nowMs();

    gz::transport::Node node;
    const bool subscribed =
        node.Subscribe<gz::msgs::PointCloudPacked>(
            options.topic,
            [&](const gz::msgs::PointCloudPacked& msg) {
                if (!g_running || processed_frames.load() >= options.frames) {
                    return;
                }
                bool expected = false;
                if (!busy.compare_exchange_strong(expected, true)) {
                    return;
                }

                try {
                    std::uint64_t source_points = 0;
                    const double parse_start = nowMs();
                    const ParsedCloud parsed =
                        parsePointCloudPacked(msg, options.max_points, source_points);
                    const double parse_done = nowMs();
                    if (parsed.samples.empty()) {
                        busy = false;
                        return;
                    }

                    const std::uint64_t frame =
                        static_cast<std::uint64_t>(processed_frames.load() + 1);
                    const double parse_ms = parse_done - parse_start;
                    const BenchmarkResult semantic =
                        runMode(options, parsed, frame, source_points, parse_ms,
                                "semantic_octree_astar", nullptr);
                    const BenchmarkResult rf =
                        runMode(options, parsed, frame, source_points, parse_ms,
                                "rf_octree_astar", &rf_predictor);

                    {
                        std::lock_guard<std::mutex> lock(output_mutex);
                        appendResultCsv(options.output, semantic);
                        appendResultCsv(options.output, rf);
                        printResult(semantic);
                        printResult(rf);

                        const double delta_ms = rf.total_ms - semantic.total_ms;
                        const double expansion_delta =
                            static_cast<double>(rf.expanded_nodes - semantic.expanded_nodes);
                        std::cout << "[benchmark] frame=" << frame
                                  << " rf_minus_semantic_total_ms=" << delta_ms
                                  << " rf_minus_semantic_expanded=" << expansion_delta
                                  << " output='" << options.output << "'\n";
                    }

                    ++processed_frames;
                } catch (const std::exception& e) {
                    std::cerr << "[benchmark] frame failed: " << e.what() << "\n";
                }

                busy = false;
            });

    if (!subscribed) {
        std::cerr << "[benchmark] Failed to subscribe topic '" << options.topic << "'\n";
        return 3;
    }

    std::cout << "[benchmark] subscribed topic='" << options.topic
              << "' partition='" << options.partition
              << "' frames=" << options.frames
              << " max_points=" << options.max_points
              << " output='" << options.output << "'\n";

    while (g_running && processed_frames.load() < options.frames) {
        if ((nowMs() - start_wall) / 1000.0 > options.timeout_sec) {
            std::cerr << "[benchmark] timeout after " << options.timeout_sec << " sec\n";
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return processed_frames.load() > 0 ? 0 : 4;
}
