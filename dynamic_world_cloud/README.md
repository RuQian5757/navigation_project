# DynamicWorldCloud

Gazebo Sim Harmonic (`gz-sim8`) system plugin that generates a dynamic
ground-truth point cloud from world collision geometry.

## Features

- Implements `ISystemConfigure` and `ISystemPostUpdate`
- Traverses the world as Model -> Link -> Collision
- Supports box, cylinder, sphere, mesh, and finite plane collision geometry
- Samples each local collision cloud once, then only transforms cached points
- Uses `gz::common::MeshManager` for mesh loading and triangle-surface sampling
- Publishes `gz::msgs::PointCloudPacked` on `/world/dynamic_cloud`
- Publishes semantic fields per point: `label`, `obstacle_probability`, `entity_id`
- Saves binary PCD snapshots every configured interval
- Detects spawned and deleted entities during simulation
- Preserves full XYZ coordinates for multi-floor navigation
- Exposes `GetCurrentPointCloud()` as `const pcl::PointCloud<pcl::PointXYZ>&`
- Companion `RFOctreePathPlanner` plugin builds an RF Octree, runs A*, and
  renders the path as visual-only cylinders

## Dependencies

Ubuntu 24.04 / Gazebo Harmonic:

```bash
sudo apt install libgz-sim8-dev libgz-common5-dev libgz-plugin2-dev libpcl-dev
```

## Build With CMake

From the repository root:

```bash
mkdir -p build
cd build
cmake ..
cmake --build . --target dynamic_world_cloud_plugins
```

The plugin target builds:

- `build/dynamic_world_cloud/libDynamicWorldCloud.so`
- `build/dynamic_world_cloud/libRFOctreePathPlanner.so`

## Build With colcon

From the workspace root containing this package:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## SDF Loading Example

```xml
<plugin filename="DynamicWorldCloud" name="DynamicWorldCloud">
  <point_spacing>0.25</point_spacing>
  <update_rate>10</update_rate>
  <pcd_save_interval>0.0</pcd_save_interval>
  <publish_enabled>true</publish_enabled>
  <max_points_per_publish>50000</max_points_per_publish>
  <pcd_directory>./pcd</pcd_directory>
  <transport_topic>/world/dynamic_cloud</transport_topic>
</plugin>
```

Predict world path planning plugin example:

```xml
<plugin filename="RFOctreePathPlanner" name="RFOctreePathPlanner">
  <transport_topic>/world/dynamic_cloud</transport_topic>
  <rf_model>models/random_forest_voxel_model.rf.txt</rf_model>
  <start>0,-5,1.2</start>
  <goal>0,0,9.2</goal>
</plugin>
```

## Semantic Fields

The published `PointCloudPacked` contains:

- `xyz` as packed `FLOAT32` coordinates
- `label` as `UINT32`: `0=free`, `1=obstacle`, `2=stair`
- `obstacle_probability` as `FLOAT32`
- `entity_id` as `UINT32`

Current label inference is rule based:

- scoped collision name containing `floor` / `ground`, or plane geometry -> free
- otherwise, scoped collision name containing `stair` -> stair
- all other collision geometry -> obstacle

Floor semantics intentionally take priority over stair semantics, so a floor
model that describes stair access is not labeled as a stair surface.

When launching from this repository without installation, use:

```bash
cmake --build build --target dynamic_world_cloud_plugins
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf
```
