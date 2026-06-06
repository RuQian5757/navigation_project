# DynamicWorldCloud

Gazebo Sim Harmonic (`gz-sim8`) system plugin that generates a dynamic
ground-truth point cloud from world collision geometry.

## Features

- Implements `ISystemConfigure` and `ISystemPostUpdate`
- Traverses the world as Model -> Link -> Collision
- Supports box, cylinder, sphere, mesh, and finite plane collision geometry
- Samples each local collision cloud once, then only transforms cached points
- Uses `gz::common::MeshManager` for mesh loading and vertex extraction
- Publishes `gz::msgs::PointCloudPacked` on `/world/dynamic_cloud`
- Saves binary PCD snapshots every configured interval
- Detects spawned and deleted entities during simulation
- Preserves full XYZ coordinates for multi-floor navigation
- Exposes `GetCurrentPointCloud()` as `const pcl::PointCloud<pcl::PointXYZ>&`

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
cmake --build . --target dynamic_world_cloud
```

The plugin target builds `build/dynamic_world_cloud/libDynamicWorldCloud.so`.

## Build With colcon

From the workspace root containing this package:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## SDF Loading Example

```xml
<plugin filename="DynamicWorldCloud" name="DynamicWorldCloud">
  <point_spacing>0.05</point_spacing>
  <update_rate>10.0</update_rate>
  <pcd_save_interval>5.0</pcd_save_interval>
  <pcd_directory>./pcd</pcd_directory>
  <transport_topic>/world/dynamic_cloud</transport_topic>
</plugin>
```

When launching from this repository without installation, use:

```bash
cmake --build build --target dynamic_world_cloud
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf
```
