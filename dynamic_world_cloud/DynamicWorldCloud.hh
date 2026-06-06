#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Mesh.hh>
#include <sdf/Plane.hh>

namespace gz::sim::systems
{

/// \brief Gazebo Sim system plugin that produces a dynamic ground-truth
/// point cloud from all collision geometry in the world.
///
/// Local geometry clouds are generated once per collision entity. Each
/// PostUpdate only transforms those cached points by the current entity pose,
/// so moving models, spawned models, and deleted models are reflected without
/// re-sampling geometry every frame.
class DynamicWorldCloud : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPostUpdate
{
public:
  DynamicWorldCloud();
  ~DynamicWorldCloud() override;

  /// \brief Read plugin parameters, advertise transport topic, and build
  /// local clouds for entities already present when the world starts.
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override;

  /// \brief Detect spawned/deleted entities, transform local clouds to world
  /// coordinates, publish them, and periodically save PCD snapshots.
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Access the latest global world cloud for future navigation modules.
  const pcl::PointCloud<pcl::PointXYZ> &GetCurrentPointCloud() const;

private:
  /// \brief Cached data for one collision entity.
  struct EntityCloud
  {
    gz::sim::Entity collision_entity{gz::sim::kNullEntity};
    gz::sim::Entity link_entity{gz::sim::kNullEntity};
    gz::sim::Entity model_entity{gz::sim::kNullEntity};
    std::string scoped_name;
    pcl::PointCloud<pcl::PointXYZ> local_cloud;
  };

  /// \brief Traverse Model -> Link -> Collision hierarchy and return live
  /// collision entities that have geometry.
  std::vector<gz::sim::Entity> CollectCollisionEntities(
      const gz::sim::EntityComponentManager &_ecm) const;

  /// \brief Build local clouds for current world contents.
  void BuildInitialLocalClouds(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Add new collision clouds and remove cache entries whose entities
  /// no longer exist.
  void UpdateEntityCache(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Build and cache a local cloud for one collision entity.
  bool BuildEntityLocalCloud(const gz::sim::Entity &_collisionEntity,
                             const gz::sim::EntityComponentManager &_ecm);

  /// \brief Sample one SDF collision geometry into collision-local points.
  pcl::PointCloud<pcl::PointXYZ> SampleCollisionGeometry(
      const sdf::Geometry &_geometry) const;

  /// \brief Sample all six box faces.
  pcl::PointCloud<pcl::PointXYZ> SampleBox(
      const gz::math::Vector3d &_size,
      double _spacing) const;

  /// \brief Sample cylinder side, top, and bottom surfaces.
  pcl::PointCloud<pcl::PointXYZ> SampleCylinder(double _radius,
                                                double _length,
                                                double _spacing) const;

  /// \brief Sample sphere surface using a deterministic Fibonacci layout.
  pcl::PointCloud<pcl::PointXYZ> SampleSphere(double _radius,
                                              double _spacing) const;

  /// \brief Load mesh through gz::common::MeshManager and use vertices.
  pcl::PointCloud<pcl::PointXYZ> SampleMesh(const sdf::Mesh &_mesh) const;

  /// \brief Sample a finite SDF plane, useful for floor collision geometry.
  pcl::PointCloud<pcl::PointXYZ> SamplePlane(const sdf::Plane &_plane,
                                             double _spacing) const;

  /// \brief Transform one cached local point to world coordinates.
  pcl::PointXYZ TransformLocalToWorld(const pcl::PointXYZ &_pt,
                                      const gz::math::Pose3d &_pose) const;

  /// \brief Rebuild the global cloud from current world poses.
  void RebuildGlobalCloud(const gz::sim::EntityComponentManager &_ecm);

  /// \brief Save the latest global cloud to binary PCD.
  void SaveCurrentCloudToPCD(double _simTimeSec);

  /// \brief Write a minimal binary PCD file without depending on pcl_io.
  bool WriteBinaryPCD(const std::string &_path,
                      const pcl::PointCloud<pcl::PointXYZ> &_cloud) const;

  /// \brief Publish latest global cloud as gz::msgs::PointCloudPacked.
  void PublishPointCloud();

  /// \brief Convert PCL cloud to Gazebo transport message.
  gz::msgs::PointCloudPacked BuildPointCloudMessage() const;

  /// \brief Stable unordered_map key for an entity ID.
  static uint64_t EntityToKey(const gz::sim::Entity &_entity);

private:
  double spacing_{0.05};
  double update_rate_{10.0};
  double pcd_save_interval_{5.0};
  bool publish_enabled_{true};
  uint32_t max_points_per_publish_{0};
  std::string pcd_directory_{"."};
  std::string transport_topic_{"/world/dynamic_cloud"};

  std::unordered_map<uint64_t, EntityCloud> entity_clouds_;
  mutable std::unordered_map<std::string, pcl::PointCloud<pcl::PointXYZ>>
      mesh_vertex_cache_;
  pcl::PointCloud<pcl::PointXYZ> global_cloud_;

  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher cloud_pub_;

  double last_publish_time_{-1.0};
  double last_pcd_save_time_{0.0};
  uint64_t publish_count_{0};
};

} // namespace gz::sim::systems
