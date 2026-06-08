#include "RFOctreePathPlanner.hh"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>

#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <sdf/Root.hh>

namespace gz::sim::systems
{
namespace
{
double WallTimeSec()
{
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

bool ParsePoint3D(const std::string &_text, navigation::Point3D &_point)
{
  std::stringstream ss(_text);
  std::string token;
  std::vector<float> values;
  while (std::getline(ss, token, ','))
  {
    if (token.empty())
      return false;
    try
    {
      values.push_back(static_cast<float>(std::stof(token)));
    }
    catch (...)
    {
      return false;
    }
  }

  if (values.size() != 3)
    return false;

  _point = navigation::Point3D{values[0], values[1], values[2]};
  return true;
}

double Distance(const navigation::Point3D &_a, const navigation::Point3D &_b)
{
  const double dx = static_cast<double>(_a.x - _b.x);
  const double dy = static_cast<double>(_a.y - _b.y);
  const double dz = static_cast<double>(_a.z - _b.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool SameWaypoints(const std::vector<navigation::PathWaypoint> &_a,
                   const std::vector<navigation::PathWaypoint> &_b,
                   double _tolerance)
{
  if (_a.size() != _b.size())
    return false;
  for (std::size_t i = 0; i < _a.size(); ++i)
  {
    if (Distance(_a[i].position, _b[i].position) > _tolerance)
      return false;
  }
  return true;
}
} // namespace

RFOctreePathPlanner::RFOctreePathPlanner() = default;

RFOctreePathPlanner::~RFOctreePathPlanner() = default;

void RFOctreePathPlanner::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &/*_ecm*/,
    gz::sim::EventManager &_eventMgr)
{
  this->event_mgr_ = &_eventMgr;
  this->world_entity_ = _entity;

  if (_sdf)
  {
    if (_sdf->HasElement("transport_topic"))
      this->topic_ = _sdf->Get<std::string>("transport_topic");
    if (_sdf->HasElement("rf_model"))
      this->rf_model_ = _sdf->Get<std::string>("rf_model");
    if (_sdf->HasElement("start"))
    {
      navigation::Point3D parsed;
      if (ParsePoint3D(_sdf->Get<std::string>("start"), parsed))
        this->start_ = parsed;
    }
    if (_sdf->HasElement("goal"))
    {
      navigation::Point3D parsed;
      if (ParsePoint3D(_sdf->Get<std::string>("goal"), parsed))
        this->goal_ = parsed;
    }
    if (_sdf->HasElement("max_depth"))
      this->max_depth_ = std::max(1, _sdf->Get<int>("max_depth"));
    if (_sdf->HasElement("max_points"))
      this->max_points_ = std::max(1, _sdf->Get<int>("max_points"));
    if (_sdf->HasElement("plan_hz"))
      this->plan_hz_ = std::max(0.05, _sdf->Get<double>("plan_hz"));
    if (_sdf->HasElement("line_radius"))
      this->line_radius_ = std::max(0.005, _sdf->Get<double>("line_radius"));
    if (_sdf->HasElement("z_offset"))
      this->z_offset_ = _sdf->Get<double>("z_offset");
    if (_sdf->HasElement("block_probability"))
      this->planner_config_.obstacle_block_probability =
          std::clamp(static_cast<float>(_sdf->Get<double>("block_probability")), 0.0f, 1.0f);
    if (_sdf->HasElement("enable_obstacle_clearance"))
      this->planner_config_.enable_obstacle_clearance =
          _sdf->Get<bool>("enable_obstacle_clearance");
    if (_sdf->HasElement("obstacle_clearance_radius"))
      this->planner_config_.obstacle_clearance_radius =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("obstacle_clearance_radius")));
    if (_sdf->HasElement("obstacle_clearance_z_tolerance"))
      this->planner_config_.obstacle_clearance_z_tolerance =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("obstacle_clearance_z_tolerance")));
    if (_sdf->HasElement("block_edges_through_obstacles"))
      this->planner_config_.block_edges_through_obstacles =
          _sdf->Get<bool>("block_edges_through_obstacles");
    if (_sdf->HasElement("edge_obstacle_clearance_radius"))
      this->planner_config_.edge_obstacle_clearance_radius =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("edge_obstacle_clearance_radius")));
    if (_sdf->HasElement("endpoint_snap_radius"))
      this->planner_config_.endpoint_snap_radius =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("endpoint_snap_radius")));
    if (_sdf->HasElement("stair_connection_radius"))
      this->planner_config_.stair_connection_radius =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_connection_radius")));
    if (_sdf->HasElement("constrain_stair_transitions"))
      this->planner_config_.constrain_stair_transitions =
          _sdf->Get<bool>("constrain_stair_transitions");
    if (_sdf->HasElement("stair_endpoint_tolerance"))
      this->planner_config_.stair_endpoint_tolerance =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_endpoint_tolerance")));
    if (_sdf->HasElement("stair_step_max_vertical"))
      this->planner_config_.stair_step_max_vertical =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_step_max_vertical")));
    if (_sdf->HasElement("stair_floor_connection_max_vertical"))
      this->planner_config_.stair_floor_connection_max_vertical =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_floor_connection_max_vertical")));
    if (_sdf->HasElement("max_non_stair_vertical_step"))
      this->planner_config_.max_non_stair_vertical_step =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("max_non_stair_vertical_step")));
    if (_sdf->HasElement("constrain_stair_exits_to_floor_levels"))
      this->planner_config_.constrain_stair_exits_to_floor_levels =
          _sdf->Get<bool>("constrain_stair_exits_to_floor_levels");
    if (_sdf->HasElement("stair_floor_exit_tolerance"))
      this->planner_config_.stair_floor_exit_tolerance =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_floor_exit_tolerance")));
    if (_sdf->HasElement("constrain_stair_direction"))
      this->planner_config_.constrain_stair_direction =
          _sdf->Get<bool>("constrain_stair_direction");
    if (_sdf->HasElement("stair_direction_dot_min"))
      this->planner_config_.stair_direction_dot_min =
          std::clamp(static_cast<float>(_sdf->Get<double>("stair_direction_dot_min")),
                     -1.0f, 1.0f);
    if (_sdf->HasElement("stair_entry_lateral_margin"))
      this->planner_config_.stair_entry_lateral_margin =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_entry_lateral_margin")));
    if (_sdf->HasElement("stair_entry_forward_margin"))
      this->planner_config_.stair_entry_forward_margin =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("stair_entry_forward_margin")));
    if (_sdf->HasElement("probability_weight"))
      this->planner_config_.probability_weight =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("probability_weight")));
    if (_sdf->HasElement("vertical_weight"))
      this->planner_config_.vertical_weight =
          std::max(0.0f, static_cast<float>(_sdf->Get<double>("vertical_weight")));
    if (_sdf->HasElement("allow_cross_floor"))
      this->planner_config_.allow_cross_floor = _sdf->Get<bool>("allow_cross_floor");
    if (_sdf->HasElement("floor_z"))
    {
      this->feature_config_.floor_z = static_cast<float>(_sdf->Get<double>("floor_z"));
      this->planner_config_.floor_z = this->feature_config_.floor_z;
    }
    if (_sdf->HasElement("story_height"))
    {
      this->feature_config_.story_height =
          std::max(0.001f, static_cast<float>(_sdf->Get<double>("story_height")));
      this->planner_config_.story_height = this->feature_config_.story_height;
    }
    if (_sdf->HasElement("floor_surface_offset"))
    {
      this->feature_config_.floor_surface_offset =
          static_cast<float>(_sdf->Get<double>("floor_surface_offset"));
      this->planner_config_.floor_surface_offset =
          this->feature_config_.floor_surface_offset;
    }
    if (_sdf->HasElement("ceiling_offset"))
      this->feature_config_.ceiling_offset =
          static_cast<float>(_sdf->Get<double>("ceiling_offset"));
  }

  this->rf_predictor_ =
      std::make_unique<navigation::RandomForestVoxelPredictor>(this->feature_config_);
  try
  {
    this->rf_predictor_->loadFromTextModel(this->rf_model_);
  }
  catch (const std::exception &_e)
  {
    std::cerr << "[RFOctreePathPlanner] Failed to load RF model '"
              << this->rf_model_ << "': " << _e.what() << "\n";
    this->rf_predictor_.reset();
    return;
  }

  const bool subscribed =
      this->transport_node_.Subscribe(
          this->topic_, &RFOctreePathPlanner::OnPointCloud, this);
  if (!subscribed)
  {
    std::cerr << "[RFOctreePathPlanner] Failed to subscribe topic '"
              << this->topic_ << "'\n";
    return;
  }

  std::cerr << "[RFOctreePathPlanner] Configured topic='" << this->topic_
            << "' rf_model='" << this->rf_model_
            << "' start=(" << this->start_.x << "," << this->start_.y << "," << this->start_.z << ")"
            << " goal=(" << this->goal_.x << "," << this->goal_.y << "," << this->goal_.z << ")"
            << " plan_hz=" << this->plan_hz_
            << " line_radius=" << this->line_radius_
            << " constrain_stair_transitions="
            << (this->planner_config_.constrain_stair_transitions ? "true" : "false")
            << " stair_endpoint_tolerance=" << this->planner_config_.stair_endpoint_tolerance
            << " stair_step_max_vertical=" << this->planner_config_.stair_step_max_vertical
            << " stair_floor_connection_max_vertical="
            << this->planner_config_.stair_floor_connection_max_vertical
            << " max_non_stair_vertical_step="
            << this->planner_config_.max_non_stair_vertical_step
            << " constrain_stair_exits_to_floor_levels="
            << (this->planner_config_.constrain_stair_exits_to_floor_levels ? "true" : "false")
            << " stair_floor_exit_tolerance="
            << this->planner_config_.stair_floor_exit_tolerance
            << " obstacle_clearance_radius="
            << this->planner_config_.obstacle_clearance_radius
            << " block_edges_through_obstacles="
            << (this->planner_config_.block_edges_through_obstacles ? "true" : "false")
            << " edge_obstacle_clearance_radius="
            << this->planner_config_.edge_obstacle_clearance_radius
            << " constrain_stair_direction="
            << (this->planner_config_.constrain_stair_direction ? "true" : "false")
            << " stair_direction_dot_min=" << this->planner_config_.stair_direction_dot_min
            << " stair_entry_lateral_margin="
            << this->planner_config_.stair_entry_lateral_margin
            << "\n";
}

void RFOctreePathPlanner::PreUpdate(const gz::sim::UpdateInfo &/*_info*/,
                                    gz::sim::EntityComponentManager &_ecm)
{
  std::uint64_t revision = 0;
  bool shouldClear = false;
  {
    std::lock_guard<std::mutex> lock(this->path_mutex_);
    revision = this->pending_revision_;
    shouldClear = this->pending_clear_;
  }

  if (revision == this->rendered_revision_)
    return;

  if (shouldClear)
    this->ClearPathVisual(_ecm);
  else
    this->ReplacePathVisual(_ecm);

  this->rendered_revision_ = revision;
}

void RFOctreePathPlanner::OnPointCloud(const gz::msgs::PointCloudPacked &_msg)
{
  if (!this->rf_predictor_)
    return;

  const double now = WallTimeSec();
  if (this->last_plan_wall_time_ > 0.0 &&
      now - this->last_plan_wall_time_ < 1.0 / this->plan_hz_)
  {
    return;
  }

  bool expected = false;
  if (!this->busy_.compare_exchange_strong(expected, true))
    return;
  this->last_plan_wall_time_ = now;

  try
  {
    std::uint64_t sourcePoints = 0;
    const double parseStart = WallTimeSec();
    const ParsedCloud parsed = this->ParsePointCloudPacked(_msg, sourcePoints);
    const double parseDone = WallTimeSec();
    if (parsed.samples.empty())
    {
      this->busy_ = false;
      return;
    }
    std::cerr << "[RFOctreePathPlanner] received cloud points=" << sourcePoints
              << " used=" << parsed.samples.size()
              << " parse_ms=" << ((parseDone - parseStart) * 1000.0) << "\n";

    navigation::OctreeConfig octreeConfig;
    octreeConfig.max_depth = this->max_depth_;
    navigation::OctreeManager octree(octreeConfig);
    octree.setMLPredictor(this->rf_predictor_->asMLPredictor());
    const double octreeStart = WallTimeSec();
    octree.initialize(parsed.samples);
    const double octreeDone = WallTimeSec();
    std::cerr << "[RFOctreePathPlanner] octree leaves=" << octree.getLeafCount()
              << " build_ms=" << ((octreeDone - octreeStart) * 1000.0) << "\n";

    navigation::AStarPlanner planner(octree, this->planner_config_);
    const double planStart = WallTimeSec();
    const navigation::AStarPath path = planner.findPath(this->start_, this->goal_);
    const double planDone = WallTimeSec();

    {
      std::lock_guard<std::mutex> lock(this->path_mutex_);
      if (path.success && path.waypoints.size() >= 2 &&
          !this->pending_clear_ &&
          SameWaypoints(path.waypoints, this->pending_waypoints_, 0.05))
      {
        this->busy_ = false;
        return;
      }

      if ((!path.success || path.waypoints.size() < 2) &&
          this->pending_clear_ &&
          this->pending_waypoints_.empty())
      {
        this->busy_ = false;
        return;
      }

      ++this->pending_revision_;
      this->pending_clear_ = !path.success || path.waypoints.size() < 2;
      this->pending_waypoints_ = path.success ? path.waypoints
                                              : std::vector<navigation::PathWaypoint>{};
    }

    std::cerr << "[RFOctreePathPlanner] points=" << sourcePoints
              << " used=" << parsed.samples.size()
              << " leaves=" << octree.getLeafCount()
              << " path=" << (path.success ? "success" : "failed")
              << " waypoints=" << path.waypoints.size()
              << " expanded=" << path.expanded_nodes
              << " plan_ms=" << ((planDone - planStart) * 1000.0)
              << " message='" << path.message << "'\n";
  }
  catch (const std::exception &_e)
  {
    std::cerr << "[RFOctreePathPlanner] planning failed: " << _e.what() << "\n";
  }

  this->busy_ = false;
}

bool RFOctreePathPlanner::FindFieldOffsets(
    const gz::msgs::PointCloudPacked &_msg,
    PackedFieldOffsets &_offsets) const
{
  for (int i = 0; i < _msg.field_size(); ++i)
  {
    const auto &field = _msg.field(i);
    if (field.datatype() == gz::msgs::PointCloudPacked::Field::FLOAT32)
    {
      if (field.name() == "x")
        _offsets.x = static_cast<int>(field.offset());
      else if (field.name() == "y")
        _offsets.y = static_cast<int>(field.offset());
      else if (field.name() == "z")
        _offsets.z = static_cast<int>(field.offset());
      else if (field.name() == "xyz")
      {
        _offsets.x = static_cast<int>(field.offset());
        _offsets.y = _offsets.x + static_cast<int>(sizeof(float));
        _offsets.z = _offsets.y + static_cast<int>(sizeof(float));
      }
      else if (field.name() == "obstacle_probability")
      {
        _offsets.obstacle_probability = static_cast<int>(field.offset());
      }
    }
    else if (field.datatype() == gz::msgs::PointCloudPacked::Field::UINT32)
    {
      if (field.name() == "label")
        _offsets.label = static_cast<int>(field.offset());
      else if (field.name() == "entity_id")
        _offsets.entity_id = static_cast<int>(field.offset());
    }
  }
  return _offsets.x >= 0 && _offsets.y >= 0 && _offsets.z >= 0;
}

RFOctreePathPlanner::ParsedCloud RFOctreePathPlanner::ParsePointCloudPacked(
    const gz::msgs::PointCloudPacked &_msg,
    std::uint64_t &_sourcePoints) const
{
  _sourcePoints = static_cast<std::uint64_t>(_msg.width()) *
                  static_cast<std::uint64_t>(_msg.height());
  if (_sourcePoints == 0 || _msg.point_step() == 0)
    return {};

  PackedFieldOffsets offsets;
  if (!this->FindFieldOffsets(_msg, offsets))
    throw std::runtime_error("PointCloudPacked does not contain x/y/z or xyz fields");

  const std::size_t pointStep = static_cast<std::size_t>(_msg.point_step());
  const std::size_t required = static_cast<std::size_t>(_sourcePoints) * pointStep;
  if (_msg.data().size() < required)
    throw std::runtime_error("PointCloudPacked data is shorter than width*height*point_step");

  const std::size_t stride =
      _sourcePoints > static_cast<std::uint64_t>(this->max_points_)
          ? static_cast<std::size_t>(std::ceil(static_cast<double>(_sourcePoints) /
                                               static_cast<double>(this->max_points_)))
          : 1U;

  ParsedCloud parsed;
  parsed.has_semantics = offsets.label >= 0 && offsets.obstacle_probability >= 0;
  parsed.samples.reserve(static_cast<std::size_t>(
      std::min<std::uint64_t>(_sourcePoints, static_cast<std::uint64_t>(this->max_points_))));

  const char *data = _msg.data().data();
  for (std::size_t i = 0; i < _sourcePoints; i += stride)
  {
    const std::size_t base = i * pointStep;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    std::memcpy(&x, data + base + static_cast<std::size_t>(offsets.x), sizeof(float));
    std::memcpy(&y, data + base + static_cast<std::size_t>(offsets.y), sizeof(float));
    std::memcpy(&z, data + base + static_cast<std::size_t>(offsets.z), sizeof(float));
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      continue;

    navigation::PointCloudSample sample;
    sample.point = {x, y, z};
    if (parsed.has_semantics)
    {
      std::uint32_t label = 0;
      std::uint32_t entityId = 0;
      float probability = 0.0f;
      std::memcpy(&label, data + base + static_cast<std::size_t>(offsets.label), sizeof(label));
      std::memcpy(&probability,
                  data + base + static_cast<std::size_t>(offsets.obstacle_probability),
                  sizeof(probability));
      if (offsets.entity_id >= 0)
      {
        std::memcpy(&entityId,
                    data + base + static_cast<std::size_t>(offsets.entity_id),
                    sizeof(entityId));
      }
      sample.label = ToVoxelLabel(label);
      sample.obstacle_probability = ClampProbability(probability);
      sample.entity_id = entityId;
      sample.has_semantics = true;
      sample.is_cross_floor = sample.label == navigation::VoxelLabel::Stair;
    }
    parsed.samples.push_back(sample);
  }

  return parsed;
}

std::string RFOctreePathPlanner::BuildPathVisualModelSdf(
    const std::vector<navigation::PathWaypoint> &_waypoints,
    std::uint64_t _revision) const
{
  std::ostringstream sdf;
  const std::string modelName =
      this->model_name_prefix_ + "_" + std::to_string(_revision);
  sdf << "<sdf version='1.10'>\n";
  sdf << "  <model name='" << modelName << "'>\n";
  sdf << "    <static>true</static>\n";
  sdf << "    <self_collide>false</self_collide>\n";
  sdf << "    <pose>0 0 0 0 0 0</pose>\n";
  sdf << "    <link name='path_body'>\n";
  sdf << "      <pose>0 0 0 0 0 0</pose>\n";
  sdf << "      <inertial>\n";
  sdf << "        <mass>0.001</mass>\n";
  sdf << "        <inertia>\n";
  sdf << "          <ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz>\n";
  sdf << "          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>\n";
  sdf << "        </inertia>\n";
  sdf << "      </inertial>\n";

  for (std::size_t i = 0; i + 1 < _waypoints.size(); ++i)
  {
    navigation::Point3D a = _waypoints[i].position;
    navigation::Point3D b = _waypoints[i + 1].position;
    a.z += static_cast<float>(this->z_offset_);
    b.z += static_cast<float>(this->z_offset_);
    const double length = Distance(a, b);
    if (length < 1e-4)
      continue;

    const gz::math::Vector3d midpoint{
        0.5 * (static_cast<double>(a.x) + static_cast<double>(b.x)),
        0.5 * (static_cast<double>(a.y) + static_cast<double>(b.y)),
        0.5 * (static_cast<double>(a.z) + static_cast<double>(b.z))};
    const gz::math::Vector3d direction{
        (static_cast<double>(b.x) - static_cast<double>(a.x)) / length,
        (static_cast<double>(b.y) - static_cast<double>(a.y)) / length,
        (static_cast<double>(b.z) - static_cast<double>(a.z)) / length};

    gz::math::Quaterniond rotation;
    rotation.SetFrom2Axes(gz::math::Vector3d::UnitZ, direction);
    const gz::math::Vector3d rpy = rotation.Euler();

    sdf << "      <visual name='segment_" << i << "'>\n";
    sdf << "        <pose>" << midpoint.X() << " " << midpoint.Y() << " " << midpoint.Z()
        << " " << rpy.X() << " " << rpy.Y() << " " << rpy.Z() << "</pose>\n";
    sdf << "        <geometry><cylinder><radius>" << this->line_radius_
        << "</radius><length>" << length << "</length></cylinder></geometry>\n";
    sdf << "        <material>\n";
    sdf << "          <ambient>0 0.95 1 1</ambient>\n";
    sdf << "          <diffuse>0 0.95 1 1</diffuse>\n";
    sdf << "          <emissive>0 0.35 0.45 1</emissive>\n";
    sdf << "        </material>\n";
    sdf << "      </visual>\n";
  }

  sdf << "    </link>\n";
  sdf << "  </model>\n";
  sdf << "</sdf>\n";
  return sdf.str();
}

void RFOctreePathPlanner::ReplacePathVisual(gz::sim::EntityComponentManager &_ecm)
{
  if (!this->event_mgr_)
    return;

  std::vector<navigation::PathWaypoint> waypoints;
  std::uint64_t revision = 0;
  {
    std::lock_guard<std::mutex> lock(this->path_mutex_);
    waypoints = this->pending_waypoints_;
    revision = this->pending_revision_;
  }
  if (waypoints.size() < 2)
    return;

  gz::sim::SdfEntityCreator creator(_ecm, *this->event_mgr_);
  if (this->path_model_entity_ != gz::sim::kNullEntity)
  {
    creator.RequestRemoveEntity(this->path_model_entity_, true);
    this->path_model_entity_ = gz::sim::kNullEntity;
  }

  sdf::Root root;
  const sdf::Errors errors = root.LoadSdfString(this->BuildPathVisualModelSdf(waypoints, revision));
  if (!errors.empty() || root.Model() == nullptr)
  {
    std::cerr << "[RFOctreePathPlanner] Failed to build path visual SDF\n";
    return;
  }

  this->path_model_entity_ = creator.CreateEntities(root.Model());
  if (this->path_model_entity_ == gz::sim::kNullEntity)
  {
    std::cerr << "[RFOctreePathPlanner] Failed to create path visual entity\n";
    return;
  }

  if (this->world_entity_ != gz::sim::kNullEntity)
  {
    creator.SetParent(this->path_model_entity_, this->world_entity_);
  }

  std::cerr << "[RFOctreePathPlanner] rendered path visual revision="
            << revision
            << " segments=" << (waypoints.size() - 1)
            << " entity=" << this->path_model_entity_
            << " parent_world=" << this->world_entity_ << "\n";
}

void RFOctreePathPlanner::ClearPathVisual(gz::sim::EntityComponentManager &_ecm)
{
  if (!this->event_mgr_ || this->path_model_entity_ == gz::sim::kNullEntity)
    return;

  gz::sim::SdfEntityCreator creator(_ecm, *this->event_mgr_);
  creator.RequestRemoveEntity(this->path_model_entity_, true);
  this->path_model_entity_ = gz::sim::kNullEntity;
}

navigation::VoxelLabel RFOctreePathPlanner::ToVoxelLabel(std::uint32_t _label)
{
  if (_label == static_cast<std::uint32_t>(navigation::VoxelLabel::Obstacle))
    return navigation::VoxelLabel::Obstacle;
  if (_label == static_cast<std::uint32_t>(navigation::VoxelLabel::Stair))
    return navigation::VoxelLabel::Stair;
  return navigation::VoxelLabel::Free;
}

float RFOctreePathPlanner::ClampProbability(float _value)
{
  if (!std::isfinite(_value))
    return 0.0f;
  return std::max(0.0f, std::min(1.0f, _value));
}

} // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::RFOctreePathPlanner,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::RFOctreePathPlanner,
                    "RFOctreePathPlanner")
