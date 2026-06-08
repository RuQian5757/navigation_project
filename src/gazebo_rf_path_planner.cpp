#include "astar_planner.h"
#include "octree_manager.h"
#include "random_forest_voxel_predictor.h"

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/marker.pb.h>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
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

struct MarkerTransport {
    gz::transport::Node* node = nullptr;
    gz::transport::Node::Publisher publisher;
    std::string topic = "/marker";
    unsigned int service_timeout_ms = 250;
};

struct Options {
    std::string topic = "/world/dynamic_cloud";
    std::string partition = "dynamic_cloud_test";
    std::string rf_model = "models/random_forest_voxel_model.rf.txt";
    std::string marker_topic = "/marker";
    std::string marker_namespace = "rf_astar_path";
    navigation::Point3D start;
    navigation::Point3D goal;
    bool has_start = false;
    bool has_goal = false;
    int max_depth = 9;
    int max_points = 120000;
    double plan_hz = 1.0;
    bool once = false;
    float line_width = 0.08f;
    navigation::RandomForestFeatureConfig feature_config;
    navigation::AStarPlannerConfig planner_config;
};

void onSignal(int) {
    g_running = false;
}

void printUsage(const char* program) {
    std::cout
        << "Usage:\n"
        << "  " << program << " --start X,Y,Z --goal X,Y,Z [options]\n\n"
        << "Options:\n"
        << "  --topic NAME              Gazebo PointCloudPacked topic, default /world/dynamic_cloud\n"
        << "  --partition NAME          Gazebo transport partition, default dynamic_cloud_test\n"
        << "  --rf-model FILE           C++ Random Forest model, default models/random_forest_voxel_model.rf.txt\n"
        << "  --marker-topic NAME       Gazebo marker topic, default /marker\n"
        << "  --marker-namespace NAME   Marker namespace, default rf_astar_path\n"
        << "  --start X,Y,Z             Path start position in world coordinates\n"
        << "  --goal X,Y,Z              Path goal position in world coordinates\n"
        << "  --max-depth N             Octree max depth, default 9\n"
        << "  --max-points N            Max input points used per planning frame, default 120000\n"
        << "  --plan-hz HZ              Max planning frequency, default 1\n"
        << "  --line-width M            Path line width in Gazebo, default 0.08\n"
        << "  --once                    Plan once on first received cloud and exit\n"
        << "  --no-cross-floor          Disallow cross-floor traversal\n"
        << "  --endpoint-snap-radius M  Snap start/goal to nearest traversable leaf, default 1.5\n"
        << "  --no-endpoint-snap        Disable start/goal snapping\n"
        << "  --stair-connection-radius M Nearby stair connector radius, default 1.25\n"
        << "  --no-stair-connections    Disable virtual stair connector edges\n"
        << "  --no-stair-transition-constraint Allow entering/leaving stair voxels anywhere\n"
        << "  --stair-endpoint-tolerance M Stair endpoint entry/exit height band, default 0.6\n"
        << "  --stair-step-max-vertical M Max vertical jump between stair voxels, default 0.8\n"
        << "  --stair-floor-connection-max-vertical M Max free<->stair vertical gap, default 1.2\n"
        << "  --max-non-stair-vertical-step M Max non-stair vertical step, default 0.35\n"
        << "  --stair-floor-exit-tolerance M Distance to known floor levels for stair exits, default 0.45\n"
        << "  --no-stair-floor-level-exit-constraint Allow stair exits away from known floor levels\n"
        << "  --no-stair-direction-constraint Disable stair facing direction checks\n"
        << "  --stair-direction-dot-min D Minimum direction dot product, default 0.5\n"
        << "  --stair-entry-lateral-margin M Extra side margin at stair entrance, default 0.25\n"
        << "  --stair-entry-forward-margin M Front/back entrance margin, default 0.55\n"
        << "  --max-expansions N        A* expansion cap, default 200000\n"
        << "  --heuristic-weight W      A* heuristic multiplier, default 1.15\n"
        << "  --probability-weight W    Obstacle probability cost weight, default 6\n"
        << "  --stair-weight W          Stair cost weight, default 1.25\n"
        << "  --cross-floor-weight W    Cross-floor edge cost weight, default 2\n"
        << "  --vertical-weight W       Vertical movement cost weight, default 0.75\n"
        << "  --block-probability P     Treat leaf as blocked above probability, default 0.92\n"
        << "  --no-obstacle-clearance   Disable obstacle clearance cost around obstacles\n"
        << "  --obstacle-clearance-radius M Add extra cost near obstacles, default 0.15\n"
        << "  --obstacle-clearance-z-tolerance M Vertical clearance band, default 1.5\n"
        << "  --no-edge-obstacle-blocking Disable hard segment-vs-obstacle checks\n"
        << "  --edge-obstacle-clearance-radius M Segment obstacle margin, default 0.03\n"
        << "  --floor-z Z               Story-0 origin height for RF features, default 0\n"
        << "  --story-height H          Floor-to-floor height for RF features, default 4\n"
        << "  --floor-surface-offset Z  Local floor surface offset, default 1\n"
        << "  --ceiling-offset Z        Local ceiling offset, default 4\n"
        << "  --near-floor Z            Near-floor distance band, default 0.4\n"
        << "  --near-ceiling Z          Near-ceiling distance band, default 0.4\n"
        << "  --help                    Show this message\n";
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

    point = navigation::Point3D{values[0], values[1], values[2]};
    return true;
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
        } else if (arg == "--rf-model" && i + 1 < argc) {
            options.rf_model = argv[++i];
        } else if (arg == "--marker-topic" && i + 1 < argc) {
            options.marker_topic = argv[++i];
        } else if (arg == "--marker-namespace" && i + 1 < argc) {
            options.marker_namespace = argv[++i];
        } else if (arg == "--start" && i + 1 < argc) {
            options.has_start = parsePoint3D(argv[++i], options.start);
            if (!options.has_start) {
                std::cerr << "Invalid --start value, expected X,Y,Z\n";
                return ParseResult::Error;
            }
        } else if (arg == "--goal" && i + 1 < argc) {
            options.has_goal = parsePoint3D(argv[++i], options.goal);
            if (!options.has_goal) {
                std::cerr << "Invalid --goal value, expected X,Y,Z\n";
                return ParseResult::Error;
            }
        } else if (arg == "--max-depth" && i + 1 < argc) {
            options.max_depth = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--max-points" && i + 1 < argc) {
            options.max_points = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--plan-hz" && i + 1 < argc) {
            options.plan_hz = std::max(0.05, std::stod(argv[++i]));
        } else if (arg == "--line-width" && i + 1 < argc) {
            options.line_width = std::max(0.001f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--once") {
            options.once = true;
        } else if (arg == "--no-cross-floor") {
            options.planner_config.allow_cross_floor = false;
        } else if (arg == "--endpoint-snap-radius" && i + 1 < argc) {
            options.planner_config.endpoint_snap_radius =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--no-endpoint-snap") {
            options.planner_config.snap_endpoints_to_traversable = false;
        } else if (arg == "--stair-connection-radius" && i + 1 < argc) {
            options.planner_config.stair_connection_radius =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--no-stair-connections") {
            options.planner_config.enable_stair_connection_edges = false;
        } else if (arg == "--no-stair-transition-constraint") {
            options.planner_config.constrain_stair_transitions = false;
        } else if (arg == "--stair-endpoint-tolerance" && i + 1 < argc) {
            options.planner_config.stair_endpoint_tolerance =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-step-max-vertical" && i + 1 < argc) {
            options.planner_config.stair_step_max_vertical =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-floor-connection-max-vertical" && i + 1 < argc) {
            options.planner_config.stair_floor_connection_max_vertical =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--max-non-stair-vertical-step" && i + 1 < argc) {
            options.planner_config.max_non_stair_vertical_step =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-floor-exit-tolerance" && i + 1 < argc) {
            options.planner_config.stair_floor_exit_tolerance =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--no-stair-floor-level-exit-constraint") {
            options.planner_config.constrain_stair_exits_to_floor_levels = false;
        } else if (arg == "--no-stair-direction-constraint") {
            options.planner_config.constrain_stair_direction = false;
        } else if (arg == "--stair-direction-dot-min" && i + 1 < argc) {
            options.planner_config.stair_direction_dot_min =
                std::max(-1.0f, std::min(1.0f, static_cast<float>(std::stof(argv[++i]))));
        } else if (arg == "--stair-entry-lateral-margin" && i + 1 < argc) {
            options.planner_config.stair_entry_lateral_margin =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-entry-forward-margin" && i + 1 < argc) {
            options.planner_config.stair_entry_forward_margin =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--max-expansions" && i + 1 < argc) {
            options.planner_config.max_expansions = std::max(1, std::stoi(argv[++i]));
        } else if (arg == "--heuristic-weight" && i + 1 < argc) {
            options.planner_config.heuristic_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--probability-weight" && i + 1 < argc) {
            options.planner_config.probability_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--stair-weight" && i + 1 < argc) {
            options.planner_config.stair_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--cross-floor-weight" && i + 1 < argc) {
            options.planner_config.cross_floor_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--vertical-weight" && i + 1 < argc) {
            options.planner_config.vertical_weight =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--block-probability" && i + 1 < argc) {
            options.planner_config.obstacle_block_probability =
                std::max(0.0f, std::min(1.0f, static_cast<float>(std::stof(argv[++i]))));
        } else if (arg == "--no-obstacle-clearance") {
            options.planner_config.enable_obstacle_clearance = false;
        } else if (arg == "--obstacle-clearance-radius" && i + 1 < argc) {
            options.planner_config.obstacle_clearance_radius =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--obstacle-clearance-z-tolerance" && i + 1 < argc) {
            options.planner_config.obstacle_clearance_z_tolerance =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--no-edge-obstacle-blocking") {
            options.planner_config.block_edges_through_obstacles = false;
        } else if (arg == "--edge-obstacle-clearance-radius" && i + 1 < argc) {
            options.planner_config.edge_obstacle_clearance_radius =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--floor-z" && i + 1 < argc) {
            options.feature_config.floor_z = static_cast<float>(std::stof(argv[++i]));
            options.planner_config.floor_z = options.feature_config.floor_z;
        } else if (arg == "--story-height" && i + 1 < argc) {
            options.feature_config.story_height =
                std::max(0.001f, static_cast<float>(std::stof(argv[++i])));
            options.planner_config.story_height = options.feature_config.story_height;
        } else if (arg == "--floor-surface-offset" && i + 1 < argc) {
            options.feature_config.floor_surface_offset = static_cast<float>(std::stof(argv[++i]));
            options.planner_config.floor_surface_offset =
                options.feature_config.floor_surface_offset;
        } else if (arg == "--ceiling-offset" && i + 1 < argc) {
            options.feature_config.ceiling_offset = static_cast<float>(std::stof(argv[++i]));
        } else if (arg == "--near-floor" && i + 1 < argc) {
            options.feature_config.near_floor_z =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--near-ceiling" && i + 1 < argc) {
            options.feature_config.near_ceiling_z =
                std::max(0.0f, static_cast<float>(std::stof(argv[++i])));
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return ParseResult::Help;
        } else {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage(argv[0]);
            return ParseResult::Error;
        }
    }

    if (!options.has_start || !options.has_goal) {
        std::cerr << "Both --start X,Y,Z and --goal X,Y,Z are required\n";
        printUsage(argv[0]);
        return ParseResult::Error;
    }

    return ParseResult::Ok;
}

bool findFieldOffsets(const gz::msgs::PointCloudPacked& msg, PackedFieldOffsets& offsets) {
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
            float obstacle_probability = 0.0f;
            std::memcpy(&label,
                        data + base + static_cast<std::size_t>(offsets.label),
                        sizeof(label));
            std::memcpy(&obstacle_probability,
                        data + base + static_cast<std::size_t>(offsets.obstacle_probability),
                        sizeof(obstacle_probability));
            if (offsets.entity_id >= 0) {
                std::memcpy(&entity_id,
                            data + base + static_cast<std::size_t>(offsets.entity_id),
                            sizeof(entity_id));
            }
            sample.label = toVoxelLabel(label);
            sample.obstacle_probability = clampProbability(obstacle_probability);
            sample.entity_id = entity_id;
            sample.has_semantics = true;
            sample.is_cross_floor = sample.label == navigation::VoxelLabel::Stair;
        }
        parsed.samples.push_back(sample);
    }

    return parsed;
}

void setMarkerColor(gz::msgs::Marker& marker, float r, float g, float b, float a) {
    auto* diffuse = marker.mutable_material()->mutable_diffuse();
    diffuse->set_r(r);
    diffuse->set_g(g);
    diffuse->set_b(b);
    diffuse->set_a(a);
}

void setMarkerScale(gz::msgs::Marker& marker, float x, float y, float z) {
    marker.mutable_scale()->set_x(x);
    marker.mutable_scale()->set_y(y);
    marker.mutable_scale()->set_z(z);
}

void setMarkerIdentityPose(gz::msgs::Marker& marker) {
    marker.mutable_pose()->mutable_position()->set_x(0.0);
    marker.mutable_pose()->mutable_position()->set_y(0.0);
    marker.mutable_pose()->mutable_position()->set_z(0.0);
    marker.mutable_pose()->mutable_orientation()->set_w(1.0);
}

void sendMarker(MarkerTransport& transport, const gz::msgs::Marker& marker) {
    if (transport.publisher) {
        transport.publisher.Publish(marker);
    }

    if (transport.node != nullptr) {
        gz::msgs::Boolean reply;
        bool result = false;
        const bool request_sent =
            transport.node->Request<gz::msgs::Marker, gz::msgs::Boolean>(
                transport.topic, marker, transport.service_timeout_ms, reply, result);
        if (!request_sent) {
            return;
        }
        if (!result || !reply.data()) {
            // Topic publishing remains as a fallback. Keep this quiet to avoid
            // spamming when Gazebo is running headless without MarkerManager.
        }
    }
}

void publishDeleteMarker(MarkerTransport& transport,
                         const std::string& ns,
                         std::uint64_t id) {
    gz::msgs::Marker marker;
    marker.set_ns(ns);
    marker.set_id(id);
    marker.set_action(gz::msgs::Marker::DELETE_MARKER);
    sendMarker(transport, marker);
}

void publishSphereMarker(MarkerTransport& transport,
                         const std::string& ns,
                         std::uint64_t id,
                         const navigation::Point3D& position,
                         float r,
                         float g,
                         float b) {
    gz::msgs::Marker marker;
    marker.set_ns(ns);
    marker.set_id(id);
    marker.set_action(gz::msgs::Marker::ADD_MODIFY);
    marker.set_type(gz::msgs::Marker::SPHERE);
    marker.set_visibility(gz::msgs::Marker::GUI);
    setMarkerScale(marker, 0.28f, 0.28f, 0.28f);
    setMarkerColor(marker, r, g, b, 1.0f);
    marker.mutable_pose()->mutable_position()->set_x(position.x);
    marker.mutable_pose()->mutable_position()->set_y(position.y);
    marker.mutable_pose()->mutable_position()->set_z(position.z);
    marker.mutable_pose()->mutable_orientation()->set_w(1.0);
    sendMarker(transport, marker);
}

void publishPathMarker(MarkerTransport& transport,
                       const Options& options,
                       const navigation::AStarPath& path) {
    if (!path.success || path.waypoints.size() < 2) {
        publishDeleteMarker(transport, options.marker_namespace, 0);
        return;
    }

    gz::msgs::Marker marker;
    marker.set_ns(options.marker_namespace);
    marker.set_id(0);
    marker.set_action(gz::msgs::Marker::ADD_MODIFY);
    marker.set_type(gz::msgs::Marker::LINE_STRIP);
    marker.set_visibility(gz::msgs::Marker::GUI);
    marker.set_layer(0);
    setMarkerIdentityPose(marker);
    setMarkerScale(marker, options.line_width, options.line_width, options.line_width);
    setMarkerColor(marker, 0.0f, 0.95f, 1.0f, 1.0f);

    for (const auto& waypoint : path.waypoints) {
        auto* point = marker.add_point();
        point->set_x(waypoint.position.x);
        point->set_y(waypoint.position.y);
        point->set_z(waypoint.position.z + 0.08f);
    }

    sendMarker(transport, marker);
    publishSphereMarker(transport, options.marker_namespace, 1, options.start, 0.0f, 1.0f, 0.25f);
    publishSphereMarker(transport, options.marker_namespace, 2, options.goal, 1.0f, 0.15f, 0.1f);
}

void printPoint(const char* name, const navigation::Point3D& point) {
    std::cout << name << "=(" << point.x << ", " << point.y << ", " << point.z << ")";
}

const char* labelName(navigation::VoxelLabel label) {
    switch (label) {
        case navigation::VoxelLabel::Free: return "free";
        case navigation::VoxelLabel::Obstacle: return "obstacle";
        case navigation::VoxelLabel::Stair: return "stair";
    }
    return "unknown";
}

void appendEndpointSummary(std::ostream& os,
                           const char* name,
                           const navigation::OctreeManager& octree,
                           int node_index,
                           bool snapped,
                           float snap_distance) {
    const navigation::OctreeNode* node = octree.getNode(node_index);
    if (node == nullptr) {
        os << " " << name << "_node=none";
        return;
    }
    const navigation::Point3D center = node->bounds.center();
    os << " " << name << "_node=" << node_index
       << " " << name << "_center=(" << center.x << "," << center.y << "," << center.z << ")"
       << " " << name << "_label=" << labelName(node->label)
       << " " << name << "_prob=" << node->obstacle_probability;
    if (snapped) {
        os << " " << name << "_snapped=" << snap_distance << "m";
    }
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    const ParseResult parse_result = parseArgs(argc, argv, options);
    if (parse_result == ParseResult::Help) {
        return 0;
    }
    if (parse_result == ParseResult::Error) {
        return 1;
    }

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    setenv("GZ_PARTITION", options.partition.c_str(), 1);

    auto rf_predictor =
        std::make_shared<navigation::RandomForestVoxelPredictor>(options.feature_config);
    try {
        rf_predictor->loadFromTextModel(options.rf_model);
    } catch (const std::exception& e) {
        std::cerr << "[rf_path_planner] Failed to load RF model '" << options.rf_model
                  << "': " << e.what() << "\n";
        return 2;
    }

    gz::transport::Node node;
    MarkerTransport marker_transport;
    marker_transport.node = &node;
    marker_transport.topic = options.marker_topic;
    marker_transport.publisher = node.Advertise<gz::msgs::Marker>(options.marker_topic);
    if (!marker_transport.publisher) {
        std::cerr << "[rf_path_planner] Failed to advertise marker topic: "
                  << options.marker_topic << "\n";
        return 3;
    }

    std::atomic_bool busy{false};
    std::atomic_bool planned_once{false};
    std::atomic<std::int64_t> last_plan_ns{0};
    std::atomic<std::uint64_t> frame_counter{0};
    const auto plan_period_ns = static_cast<std::int64_t>(1.0e9 / options.plan_hz);

    std::function<void(const gz::msgs::PointCloudPacked&)> callback =
        [&](const gz::msgs::PointCloudPacked& msg) {
            if (options.once && planned_once.load(std::memory_order_relaxed)) {
                return;
            }

            const auto now = std::chrono::steady_clock::now().time_since_epoch();
            const auto now_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
            std::int64_t previous_ns = last_plan_ns.load(std::memory_order_relaxed);
            if (!options.once && now_ns - previous_ns < plan_period_ns) {
                return;
            }
            if (!last_plan_ns.compare_exchange_strong(previous_ns, now_ns, std::memory_order_relaxed)) {
                return;
            }

            bool expected_busy = false;
            if (!busy.compare_exchange_strong(expected_busy, true, std::memory_order_acq_rel)) {
                return;
            }

            try {
                std::uint64_t source_points = 0;
                const ParsedCloud parsed =
                    parsePointCloudPacked(msg, options.max_points, source_points);
                if (parsed.samples.empty()) {
                    std::cerr << "[rf_path_planner] Received empty point cloud\n";
                    busy = false;
                    return;
                }

                navigation::OctreeConfig config;
                config.max_depth = options.max_depth;
                navigation::OctreeManager octree(config);
                octree.setMLPredictor(rf_predictor->asMLPredictor());
                octree.initialize(parsed.samples);

                navigation::AStarPlanner planner(octree, options.planner_config);
                const navigation::AStarPath path = planner.findPath(options.start, options.goal);
                publishPathMarker(marker_transport, options, path);

                const std::uint64_t frame = ++frame_counter;
                std::cout << "[rf_path_planner] frame=" << frame
                          << " source_points=" << source_points
                          << " used_points=" << parsed.samples.size()
                          << " leaves=" << octree.getLeafCount()
                          << " path=" << (path.success ? "success" : "failed")
                          << " expanded=" << path.expanded_nodes
                          << " waypoints=" << path.waypoints.size()
                          << " cost=" << path.total_cost;
                appendEndpointSummary(std::cout, "start", octree, path.start_node,
                                      path.start_snapped, path.start_snap_distance);
                appendEndpointSummary(std::cout, "goal", octree, path.goal_node,
                                      path.goal_snapped, path.goal_snap_distance);
                std::cout << " message='" << path.message << "'\n";

                planned_once = true;
                if (options.once) {
                    g_running = false;
                }
            } catch (const std::exception& e) {
                std::cerr << "[rf_path_planner] planning failed: " << e.what() << "\n";
            }

            busy = false;
        };

    const bool subscribed = node.Subscribe<gz::msgs::PointCloudPacked>(options.topic, callback);
    if (!subscribed) {
        std::cerr << "[rf_path_planner] Failed to subscribe to topic: "
                  << options.topic << "\n";
        return 4;
    }

    std::cout << "[rf_path_planner] loaded RF model '" << options.rf_model
              << "' classifier_trees=" << rf_predictor->classifierTreeCount()
              << " regressor_trees=" << rf_predictor->regressorTreeCount() << "\n";
    std::cout << "[rf_path_planner] subscribed topic='" << options.topic
              << "' marker_topic='" << options.marker_topic
              << "' partition='" << options.partition << "' ";
    printPoint("start", options.start);
    std::cout << " ";
    printPoint("goal", options.goal);
    std::cout << "\n";

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    publishDeleteMarker(marker_transport, options.marker_namespace, 0);
    return 0;
}
