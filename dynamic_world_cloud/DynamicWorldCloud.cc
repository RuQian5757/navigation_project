#include "DynamicWorldCloud.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_set>

#include <gz/common/Mesh.hh>
#include <gz/common/MeshManager.hh>
#include <gz/common/SubMesh.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/PointCloudPackedUtils.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Collision.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <sdf/Box.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Plane.hh>
#include <sdf/Sphere.hh>

namespace gz::sim::systems
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultSpacing = 0.05;

double SafeSpacing(double _spacing)
{
  return _spacing > 0.0 ? _spacing : kDefaultSpacing;
}

void AppendPoint(pcl::PointCloud<pcl::PointXYZ> &_cloud,
                 double _x,
                 double _y,
                 double _z)
{
  _cloud.emplace_back(static_cast<float>(_x),
                      static_cast<float>(_y),
                      static_cast<float>(_z));
}
} // namespace

DynamicWorldCloud::DynamicWorldCloud() = default;

DynamicWorldCloud::~DynamicWorldCloud() = default;

uint64_t DynamicWorldCloud::EntityToKey(const gz::sim::Entity &_entity)
{
  return static_cast<uint64_t>(_entity);
}

void DynamicWorldCloud::Configure(
    const gz::sim::Entity &/*_entity*/,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &/*_eventMgr*/)
{
  // Read optional SDF settings. Defaults target dense ground-truth clouds while
  // keeping publication and PCD I/O throttled.
  if (_sdf)
  {
    if (_sdf->HasElement("point_spacing"))
      this->spacing_ = _sdf->Get<double>("point_spacing");
    if (_sdf->HasElement("update_rate"))
      this->update_rate_ = _sdf->Get<double>("update_rate");
    if (_sdf->HasElement("pcd_save_interval"))
      this->pcd_save_interval_ = _sdf->Get<double>("pcd_save_interval");
    if (_sdf->HasElement("publish_enabled"))
      this->publish_enabled_ = _sdf->Get<bool>("publish_enabled");
    if (_sdf->HasElement("max_points_per_publish"))
      this->max_points_per_publish_ =
          _sdf->Get<uint32_t>("max_points_per_publish");
    if (_sdf->HasElement("pcd_directory"))
      this->pcd_directory_ = _sdf->Get<std::string>("pcd_directory");
    if (_sdf->HasElement("transport_topic"))
      this->transport_topic_ = _sdf->Get<std::string>("transport_topic");
  }

  this->spacing_ = SafeSpacing(this->spacing_);
  this->update_rate_ = std::max(0.001, this->update_rate_);
  this->pcd_save_interval_ = std::max(0.0, this->pcd_save_interval_);
  this->last_publish_time_ = -1.0 / this->update_rate_;

  if (this->publish_enabled_)
  {
    this->cloud_pub_ =
        this->transport_node_.Advertise<gz::msgs::PointCloudPacked>(
            this->transport_topic_);
  }

  // The ECM may not expose all model/link/collision entities during Configure
  // in every loading path. We do an eager scan for worlds that are ready, then
  // scan again on the first PostUpdate before publishing.
  this->BuildInitialLocalClouds(_ecm);

  std::cerr << "[DynamicWorldCloud] Configured. cached_entities="
            << this->entity_clouds_.size()
            << " topic='" << this->transport_topic_
            << "' spacing=" << this->spacing_
            << "m update_rate=" << this->update_rate_
            << "Hz publish=" << (this->publish_enabled_ ? "true" : "false")
            << " max_points_per_publish=" << this->max_points_per_publish_
            << "\n";
}

void DynamicWorldCloud::BuildInitialLocalClouds(
    const gz::sim::EntityComponentManager &_ecm)
{
  for (const auto &collisionEntity : this->CollectCollisionEntities(_ecm))
  {
    this->BuildEntityLocalCloud(collisionEntity, _ecm);
  }
}

std::vector<gz::sim::Entity> DynamicWorldCloud::CollectCollisionEntities(
    const gz::sim::EntityComponentManager &_ecm) const
{
  std::vector<gz::sim::Entity> collisionEntities;

  // Requirement-level traversal: discover every model, then every link under
  // that model, then every collision under each link.
  _ecm.Each<gz::sim::components::Model>(
      [&](const gz::sim::Entity &_model,
          const gz::sim::components::Model *) -> bool
      {
        _ecm.Each<gz::sim::components::Link,
                  gz::sim::components::ParentEntity>(
            [&](const gz::sim::Entity &_link,
                const gz::sim::components::Link *,
                const gz::sim::components::ParentEntity *_linkParent) -> bool
            {
              if (!_linkParent || _linkParent->Data() != _model)
                return true;

              _ecm.Each<gz::sim::components::CollisionElement,
                        gz::sim::components::ParentEntity>(
                  [&](const gz::sim::Entity &_collision,
                      const gz::sim::components::CollisionElement *,
                      const gz::sim::components::ParentEntity *_collisionParent)
                      -> bool
                  {
                    if (_collisionParent && _collisionParent->Data() == _link)
                      collisionEntities.push_back(_collision);
                    return true;
                  });
              return true;
            });
        return true;
      });

  return collisionEntities;
}

bool DynamicWorldCloud::BuildEntityLocalCloud(
    const gz::sim::Entity &_collisionEntity,
    const gz::sim::EntityComponentManager &_ecm)
{
  const uint64_t key = EntityToKey(_collisionEntity);
  if (this->entity_clouds_.find(key) != this->entity_clouds_.end())
    return true;

  const auto *collisionComp =
      _ecm.Component<gz::sim::components::CollisionElement>(_collisionEntity);
  if (!collisionComp)
    return false;

  const sdf::Collision &collision = collisionComp->Data();
  const sdf::Geometry *geometry = collision.Geom();
  if (!geometry)
  {
    std::cerr << "[DynamicWorldCloud] Collision has no geometry: entity="
              << key << "\n";
    return false;
  }

  pcl::PointCloud<pcl::PointXYZ> localCloud =
      this->SampleCollisionGeometry(*geometry);
  if (localCloud.empty())
  {
    std::cerr << "[DynamicWorldCloud] No points sampled for "
              << gz::sim::scopedName(_collisionEntity, _ecm)
              << " geometry_type=" << static_cast<int>(geometry->Type())
              << "\n";
    return false;
  }

  const gz::sim::Entity linkEntity = _ecm.ParentEntity(_collisionEntity);
  const gz::sim::Entity modelEntity = _ecm.ParentEntity(linkEntity);

  EntityCloud cache;
  cache.collision_entity = _collisionEntity;
  cache.link_entity = linkEntity;
  cache.model_entity = modelEntity;
  cache.scoped_name = gz::sim::scopedName(_collisionEntity, _ecm);
  cache.local_cloud = std::move(localCloud);
  cache.local_cloud.width = static_cast<uint32_t>(cache.local_cloud.size());
  cache.local_cloud.height = 1;
  cache.local_cloud.is_dense = true;

  std::cerr << "[DynamicWorldCloud] Cached local cloud for "
            << cache.scoped_name << " points=" << cache.local_cloud.size()
            << "\n";

  this->entity_clouds_.emplace(key, std::move(cache));
  return true;
}

pcl::PointCloud<pcl::PointXYZ> DynamicWorldCloud::SampleCollisionGeometry(
    const sdf::Geometry &_geometry) const
{
  switch (_geometry.Type())
  {
    case sdf::GeometryType::BOX:
    {
      const sdf::Box *box = _geometry.BoxShape();
      if (!box)
        return {};
      return this->SampleBox(box->Size(), this->spacing_);
    }
    case sdf::GeometryType::CYLINDER:
    {
      const sdf::Cylinder *cylinder = _geometry.CylinderShape();
      if (!cylinder)
        return {};
      return this->SampleCylinder(cylinder->Radius(),
                                  cylinder->Length(),
                                  this->spacing_);
    }
    case sdf::GeometryType::SPHERE:
    {
      const sdf::Sphere *sphere = _geometry.SphereShape();
      if (!sphere)
        return {};
      return this->SampleSphere(sphere->Radius(), this->spacing_);
    }
    case sdf::GeometryType::PLANE:
    {
      const sdf::Plane *plane = _geometry.PlaneShape();
      if (!plane)
        return {};
      return this->SamplePlane(*plane, this->spacing_);
    }
    case sdf::GeometryType::MESH:
    {
      const sdf::Mesh *mesh = _geometry.MeshShape();
      if (!mesh)
        return {};
      return this->SampleMesh(*mesh);
    }
    default:
      std::cerr << "[DynamicWorldCloud] Unsupported collision geometry type="
                << static_cast<int>(_geometry.Type()) << "\n";
      return {};
  }
}

pcl::PointCloud<pcl::PointXYZ> DynamicWorldCloud::SampleBox(
    const gz::math::Vector3d &_size,
    double _spacing) const
{
  pcl::PointCloud<pcl::PointXYZ> points;
  _spacing = SafeSpacing(_spacing);

  const double halfX = 0.5 * _size.X();
  const double halfY = 0.5 * _size.Y();
  const double halfZ = 0.5 * _size.Z();
  const int nx = std::max(1, static_cast<int>(std::ceil(_size.X() / _spacing)));
  const int ny = std::max(1, static_cast<int>(std::ceil(_size.Y() / _spacing)));
  const int nz = std::max(1, static_cast<int>(std::ceil(_size.Z() / _spacing)));

  // z-min and z-max faces.
  for (int ix = 0; ix <= nx; ++ix)
  {
    const double x = -halfX + _size.X() * ix / nx;
    for (int iy = 0; iy <= ny; ++iy)
    {
      const double y = -halfY + _size.Y() * iy / ny;
      AppendPoint(points, x, y, -halfZ);
      AppendPoint(points, x, y, halfZ);
    }
  }

  // y-min and y-max faces.
  for (int ix = 0; ix <= nx; ++ix)
  {
    const double x = -halfX + _size.X() * ix / nx;
    for (int iz = 0; iz <= nz; ++iz)
    {
      const double z = -halfZ + _size.Z() * iz / nz;
      AppendPoint(points, x, -halfY, z);
      AppendPoint(points, x, halfY, z);
    }
  }

  // x-min and x-max faces.
  for (int iy = 0; iy <= ny; ++iy)
  {
    const double y = -halfY + _size.Y() * iy / ny;
    for (int iz = 0; iz <= nz; ++iz)
    {
      const double z = -halfZ + _size.Z() * iz / nz;
      AppendPoint(points, -halfX, y, z);
      AppendPoint(points, halfX, y, z);
    }
  }

  return points;
}

pcl::PointCloud<pcl::PointXYZ> DynamicWorldCloud::SampleCylinder(
    double _radius,
    double _length,
    double _spacing) const
{
  pcl::PointCloud<pcl::PointXYZ> points;
  _spacing = SafeSpacing(_spacing);

  const int heightSteps =
      std::max(1, static_cast<int>(std::ceil(_length / _spacing)));
  const int thetaSteps = std::max(
      12, static_cast<int>(std::ceil(2.0 * kPi * _radius / _spacing)));

  // Side surface.
  for (int iz = 0; iz <= heightSteps; ++iz)
  {
    const double z = -0.5 * _length + _length * iz / heightSteps;
    for (int it = 0; it < thetaSteps; ++it)
    {
      const double theta = 2.0 * kPi * it / thetaSteps;
      AppendPoint(points, _radius * std::cos(theta),
                  _radius * std::sin(theta), z);
    }
  }

  // Top and bottom disks.
  const int radialSteps =
      std::max(1, static_cast<int>(std::ceil(_radius / _spacing)));
  for (int cap = 0; cap < 2; ++cap)
  {
    const double z = cap == 0 ? -0.5 * _length : 0.5 * _length;
    AppendPoint(points, 0.0, 0.0, z);
    for (int ir = 1; ir <= radialSteps; ++ir)
    {
      const double r = _radius * ir / radialSteps;
      const int ringSteps = std::max(
          8, static_cast<int>(std::ceil(2.0 * kPi * r / _spacing)));
      for (int it = 0; it < ringSteps; ++it)
      {
        const double theta = 2.0 * kPi * it / ringSteps;
        AppendPoint(points, r * std::cos(theta), r * std::sin(theta), z);
      }
    }
  }

  return points;
}

pcl::PointCloud<pcl::PointXYZ> DynamicWorldCloud::SampleSphere(
    double _radius,
    double _spacing) const
{
  pcl::PointCloud<pcl::PointXYZ> points;
  _spacing = SafeSpacing(_spacing);

  const double area = 4.0 * kPi * _radius * _radius;
  const int sampleCount =
      std::max(32, static_cast<int>(std::ceil(area / (_spacing * _spacing))));
  const double goldenAngle = kPi * (3.0 - std::sqrt(5.0));

  // Fibonacci sphere sampling gives a nearly uniform deterministic surface
  // cloud without random state or per-frame work.
  for (int i = 0; i < sampleCount; ++i)
  {
    const double zUnit =
        1.0 - (2.0 * static_cast<double>(i) + 1.0) / sampleCount;
    const double radial = std::sqrt(std::max(0.0, 1.0 - zUnit * zUnit));
    const double theta = goldenAngle * i;
    AppendPoint(points,
                _radius * radial * std::cos(theta),
                _radius * radial * std::sin(theta),
                _radius * zUnit);
  }

  return points;
}

pcl::PointCloud<pcl::PointXYZ> DynamicWorldCloud::SampleMesh(
    const sdf::Mesh &_mesh) const
{
  const std::string uri = _mesh.Uri();
  if (uri.empty())
    return {};

  const gz::math::Vector3d scale = _mesh.Scale();
  const std::string cacheKey =
      uri + "|" + std::to_string(scale.X()) + "," +
      std::to_string(scale.Y()) + "," + std::to_string(scale.Z());

  if (auto it = this->mesh_vertex_cache_.find(cacheKey);
      it != this->mesh_vertex_cache_.end())
  {
    return it->second;
  }

  pcl::PointCloud<pcl::PointXYZ> vertices;
  const gz::common::Mesh *mesh =
      gz::common::MeshManager::Instance()->Load(uri);
  if (!mesh)
  {
    std::cerr << "[DynamicWorldCloud] Failed to load mesh uri='" << uri
              << "'\n";
    return {};
  }

  for (unsigned int si = 0; si < mesh->SubMeshCount(); ++si)
  {
    auto submesh = mesh->SubMeshByIndex(si).lock();
    if (!submesh)
      continue;

    for (unsigned int vi = 0; vi < submesh->VertexCount(); ++vi)
    {
      const gz::math::Vector3d v = submesh->Vertex(vi) * scale;
      AppendPoint(vertices, v.X(), v.Y(), v.Z());
    }
  }

  vertices.width = static_cast<uint32_t>(vertices.size());
  vertices.height = 1;
  vertices.is_dense = true;
  this->mesh_vertex_cache_.emplace(cacheKey, vertices);
  return vertices;
}

pcl::PointCloud<pcl::PointXYZ> DynamicWorldCloud::SamplePlane(
    const sdf::Plane &_plane,
    double _spacing) const
{
  pcl::PointCloud<pcl::PointXYZ> points;
  _spacing = SafeSpacing(_spacing);

  const gz::math::Vector2d size = _plane.Size();
  const double halfX = 0.5 * size.X();
  const double halfY = 0.5 * size.Y();
  const int nx = std::max(1, static_cast<int>(std::ceil(size.X() / _spacing)));
  const int ny = std::max(1, static_cast<int>(std::ceil(size.Y() / _spacing)));

  gz::math::Quaterniond rotateToNormal;
  rotateToNormal.SetFrom2Axes(gz::math::Vector3d::UnitZ, _plane.Normal());

  for (int ix = 0; ix <= nx; ++ix)
  {
    const double x = -halfX + size.X() * ix / nx;
    for (int iy = 0; iy <= ny; ++iy)
    {
      const double y = -halfY + size.Y() * iy / ny;
      const gz::math::Vector3d local =
          rotateToNormal * gz::math::Vector3d(x, y, 0.0);
      AppendPoint(points, local.X(), local.Y(), local.Z());
    }
  }

  points.width = static_cast<uint32_t>(points.size());
  points.height = 1;
  points.is_dense = true;
  return points;
}

pcl::PointXYZ DynamicWorldCloud::TransformLocalToWorld(
    const pcl::PointXYZ &_pt,
    const gz::math::Pose3d &_pose) const
{
  const gz::math::Vector3d local(_pt.x, _pt.y, _pt.z);
  const gz::math::Vector3d world = _pose.Rot() * local + _pose.Pos();
  return pcl::PointXYZ(static_cast<float>(world.X()),
                       static_cast<float>(world.Y()),
                       static_cast<float>(world.Z()));
}

void DynamicWorldCloud::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  const double currentTime =
      std::chrono::duration<double>(_info.simTime).count();
  const double publishPeriod = 1.0 / std::max(0.001, this->update_rate_);
  const bool shouldPublish =
      this->publish_enabled_ &&
      currentTime >= this->last_publish_time_ + publishPeriod;
  const bool shouldSave =
      this->pcd_save_interval_ > 0.0 &&
      currentTime >= this->last_pcd_save_time_ + this->pcd_save_interval_;

  // Transforming every point in the world cloud can be expensive. We still
  // enter PostUpdate every simulation tick, but only refresh the point cloud
  // when a new published/saved snapshot is due.
  if (!shouldPublish && !shouldSave)
    return;

  this->UpdateEntityCache(_ecm);
  this->RebuildGlobalCloud(_ecm);

  if (shouldPublish)
  {
    this->PublishPointCloud();
    this->last_publish_time_ = currentTime;
  }

  if (shouldSave)
  {
    this->SaveCurrentCloudToPCD(currentTime);
    this->last_pcd_save_time_ = currentTime;
  }
}

void DynamicWorldCloud::UpdateEntityCache(
    const gz::sim::EntityComponentManager &_ecm)
{
  const std::vector<gz::sim::Entity> liveCollisions =
      this->CollectCollisionEntities(_ecm);
  std::unordered_set<uint64_t> liveKeys;
  liveKeys.reserve(liveCollisions.size());

  for (const auto &collisionEntity : liveCollisions)
  {
    const uint64_t key = EntityToKey(collisionEntity);
    liveKeys.insert(key);
    if (this->entity_clouds_.find(key) == this->entity_clouds_.end())
    {
      if (this->BuildEntityLocalCloud(collisionEntity, _ecm))
      {
        std::cerr << "[DynamicWorldCloud] Added spawned entity "
                  << gz::sim::scopedName(collisionEntity, _ecm) << "\n";
      }
    }
  }

  std::vector<uint64_t> removedKeys;
  for (const auto &[key, cache] : this->entity_clouds_)
  {
    if (liveKeys.find(key) == liveKeys.end() ||
        !_ecm.HasEntity(cache.collision_entity))
    {
      removedKeys.push_back(key);
    }
  }

  for (const uint64_t key : removedKeys)
  {
    std::cerr << "[DynamicWorldCloud] Removed deleted entity "
              << this->entity_clouds_[key].scoped_name << "\n";
    this->entity_clouds_.erase(key);
  }
}

void DynamicWorldCloud::RebuildGlobalCloud(
    const gz::sim::EntityComponentManager &_ecm)
{
  std::size_t totalPoints = 0;
  for (const auto &[_, cache] : this->entity_clouds_)
    totalPoints += cache.local_cloud.size();

  this->global_cloud_.clear();
  this->global_cloud_.reserve(totalPoints);

  for (const auto &[_, cache] : this->entity_clouds_)
  {
    if (!_ecm.HasEntity(cache.collision_entity))
      continue;

    // worldPose composes model, link, and collision poses, preserving full XYZ
    // coordinates for multi-floor worlds and dynamic obstacles.
    const gz::math::Pose3d worldPose =
        gz::sim::worldPose(cache.collision_entity, _ecm);

    for (const auto &localPoint : cache.local_cloud)
    {
      this->global_cloud_.push_back(
          this->TransformLocalToWorld(localPoint, worldPose));
    }
  }

  this->global_cloud_.width = static_cast<uint32_t>(this->global_cloud_.size());
  this->global_cloud_.height = 1;
  this->global_cloud_.is_dense = true;
}

void DynamicWorldCloud::SaveCurrentCloudToPCD(double _simTimeSec)
{
  if (this->global_cloud_.empty())
    return;

  std::error_code ec;
  std::filesystem::create_directories(this->pcd_directory_, ec);
  if (ec)
  {
    std::cerr << "[DynamicWorldCloud] Could not create PCD directory '"
              << this->pcd_directory_ << "': " << ec.message() << "\n";
    return;
  }

  std::ostringstream path;
  path << this->pcd_directory_ << "/dynamic_world_cloud_"
       << static_cast<int>(_simTimeSec) << ".pcd";

  if (this->WriteBinaryPCD(path.str(), this->global_cloud_))
  {
    std::cerr << "[DynamicWorldCloud] Saved PCD '" << path.str()
              << "' points=" << this->global_cloud_.size() << "\n";
  }
  else
  {
    std::cerr << "[DynamicWorldCloud] Failed to save PCD '" << path.str()
              << "'\n";
  }
}

bool DynamicWorldCloud::WriteBinaryPCD(
    const std::string &_path,
    const pcl::PointCloud<pcl::PointXYZ> &_cloud) const
{
  std::ofstream out(_path, std::ios::binary);
  if (!out)
    return false;

  out << "# .PCD v0.7 - Point Cloud Data file format\n";
  out << "VERSION 0.7\n";
  out << "FIELDS x y z\n";
  out << "SIZE 4 4 4\n";
  out << "TYPE F F F\n";
  out << "COUNT 1 1 1\n";
  out << "WIDTH " << _cloud.size() << "\n";
  out << "HEIGHT 1\n";
  out << "VIEWPOINT 0 0 0 1 0 0 0\n";
  out << "POINTS " << _cloud.size() << "\n";
  out << "DATA binary\n";

  for (const auto &pt : _cloud)
  {
    out.write(reinterpret_cast<const char *>(&pt.x), sizeof(pt.x));
    out.write(reinterpret_cast<const char *>(&pt.y), sizeof(pt.y));
    out.write(reinterpret_cast<const char *>(&pt.z), sizeof(pt.z));
  }

  return static_cast<bool>(out);
}

void DynamicWorldCloud::PublishPointCloud()
{
  if (this->global_cloud_.empty())
    return;

  this->cloud_pub_.Publish(this->BuildPointCloudMessage());
  ++this->publish_count_;
  std::cerr << "[DynamicWorldCloud] Published cloud #" << this->publish_count_
            << " full_points=" << this->global_cloud_.size()
            << " topic_points="
            << (this->max_points_per_publish_ > 0
                    ? std::min<std::size_t>(this->global_cloud_.size(),
                                            this->max_points_per_publish_)
                    : this->global_cloud_.size())
            << " topic='" << this->transport_topic_ << "'\n";
}

gz::msgs::PointCloudPacked DynamicWorldCloud::BuildPointCloudMessage() const
{
  gz::msgs::PointCloudPacked msg;
  gz::msgs::InitPointCloudPacked(
      msg, "world", false,
      {{"xyz", gz::msgs::PointCloudPacked::Field::FLOAT32}});

  const std::size_t sourceCount = this->global_cloud_.size();
  const std::size_t publishCount =
      (this->max_points_per_publish_ > 0 &&
       sourceCount > this->max_points_per_publish_)
          ? this->max_points_per_publish_
          : sourceCount;

  msg.set_height(1);
  msg.set_width(static_cast<uint32_t>(publishCount));
  msg.set_is_bigendian(false);
  msg.set_is_dense(this->global_cloud_.is_dense);
  msg.set_row_step(msg.point_step() * msg.width());
  msg.mutable_data()->resize(publishCount * msg.point_step());

  for (std::size_t i = 0; i < publishCount; ++i)
  {
    const std::size_t sourceIndex =
        publishCount == sourceCount ? i : (i * sourceCount) / publishCount;
    const auto &pt = this->global_cloud_[sourceIndex];
    const std::size_t offset = i * msg.point_step();
    const float xyz[3] = {pt.x, pt.y, pt.z};
    std::memcpy(msg.mutable_data()->data() + offset, xyz, sizeof(xyz));
  }

  return msg;
}

const pcl::PointCloud<pcl::PointXYZ> &
DynamicWorldCloud::GetCurrentPointCloud() const
{
  return this->global_cloud_;
}

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(::gz::sim::systems::DynamicWorldCloud,
              ::gz::sim::System,
              ::gz::sim::ISystemConfigure,
              ::gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(::gz::sim::systems::DynamicWorldCloud,
                    "DynamicWorldCloud")
GZ_ADD_PLUGIN_ALIAS(::gz::sim::systems::DynamicWorldCloud,
                    "dynamic_world_cloud")
