#pragma once

#include "astar_planner.h"
#include "octree_manager.h"
#include "random_forest_voxel_predictor.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace gz::sim::systems
{

/// \brief Build an RF-predicted Octree from DynamicWorldCloud, run A*, and
/// render the path as visual-only cylinders in Gazebo.
class RFOctreePathPlanner : public gz::sim::System,
                            public gz::sim::ISystemConfigure,
                            public gz::sim::ISystemPreUpdate
{
public:
  RFOctreePathPlanner();
  ~RFOctreePathPlanner() override;

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override;

private:
  struct PackedFieldOffsets
  {
    int x{-1};
    int y{-1};
    int z{-1};
    int label{-1};
    int obstacle_probability{-1};
    int entity_id{-1};
  };

  struct ParsedCloud
  {
    std::vector<navigation::PointCloudSample> samples;
    bool has_semantics{false};
  };

  void OnPointCloud(const gz::msgs::PointCloudPacked &_msg);
  ParsedCloud ParsePointCloudPacked(const gz::msgs::PointCloudPacked &_msg,
                                    std::uint64_t &_sourcePoints) const;
  bool FindFieldOffsets(const gz::msgs::PointCloudPacked &_msg,
                        PackedFieldOffsets &_offsets) const;
  std::string BuildPathVisualModelSdf(const std::vector<navigation::PathWaypoint> &_waypoints,
                                      std::uint64_t _revision) const;
  void ReplacePathVisual(gz::sim::EntityComponentManager &_ecm);
  void ClearPathVisual(gz::sim::EntityComponentManager &_ecm);

  static navigation::VoxelLabel ToVoxelLabel(std::uint32_t _label);
  static float ClampProbability(float _value);

private:
  gz::transport::Node transport_node_;
  gz::sim::EventManager *event_mgr_{nullptr};
  gz::sim::Entity world_entity_{gz::sim::kNullEntity};
  gz::sim::Entity path_model_entity_{gz::sim::kNullEntity};

  std::string topic_{"/world/dynamic_cloud"};
  std::string rf_model_{"models/random_forest_voxel_model.rf.txt"};
  std::string model_name_prefix_{"rf_astar_path_visual"};
  navigation::Point3D start_{0.0f, -5.0f, 1.2f};
  navigation::Point3D goal_{0.0f, 0.0f, 9.2f};
  int max_depth_{9};
  int max_points_{50000};
  double plan_hz_{1.0};
  double last_plan_wall_time_{-1.0};
  double line_radius_{0.06};
  double z_offset_{0.10};
  navigation::RandomForestFeatureConfig feature_config_;
  navigation::AStarPlannerConfig planner_config_;
  std::unique_ptr<navigation::RandomForestVoxelPredictor> rf_predictor_;

  std::mutex path_mutex_;
  std::vector<navigation::PathWaypoint> pending_waypoints_;
  std::uint64_t pending_revision_{0};
  std::uint64_t rendered_revision_{0};
  bool pending_clear_{false};
  std::atomic_bool busy_{false};
};

} // namespace gz::sim::systems
