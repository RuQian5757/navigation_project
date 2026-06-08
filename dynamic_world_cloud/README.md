# DynamicWorldCloud Gazebo Plugins

本資料夾包含兩個 Gazebo Sim system plugin：

- `DynamicWorldCloud`：從 world collision geometry 產生帶語義欄位的 point cloud。
- `RFOctreePathPlanner`：訂閱 point cloud，建立 RF predicted Octree，執行 A*，並在 Gazebo 中畫出路徑。

## Build

從專案根目錄：

```bash
cmake -S . -B build
cmake --build build --target dynamic_world_cloud_plugins
cmake --build build --target rf_octree_path_planner_gazebo
```

輸出：

```text
build/dynamic_world_cloud/libDynamicWorldCloud.so
build/dynamic_world_cloud/libRFOctreePathPlanner.so
build/rf_octree_path_planner_gazebo
```

啟動 world 時建議使用：

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -r -v 2
./scripts/run_gazebo.sh gazebo/maps/warehouse_predict_world.sdf -r -v 2
```

`run_gazebo.sh` 會設定 `GZ_PLUGIN_PATH`、`GZ_SIM_SYSTEM_PLUGIN_PATH`、`GZ_SIM_RESOURCE_PATH` 與本地模型搜尋路徑。

## DynamicWorldCloud

SDF 範例：

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

功能：

- 掃描 world 的 Model -> Link -> Collision。
- 支援 box、cylinder、sphere、finite plane、mesh collision。
- Mesh 使用 `gz::common::MeshManager` 讀取 triangle surface 並依 `point_spacing` 取樣。
- 每個 collision 的 local cloud 只建立一次並 cache。
- `PostUpdate()` 時根據 entity pose 轉換到 world coordinates。
- 支援 simulation 中新增 / 刪除 entity 時更新 cache。
- 發布 `gz::msgs::PointCloudPacked` 到 `/world/dynamic_cloud`。

PointCloudPacked 欄位：

```text
xyz: FLOAT32 x 3
label: UINT32
obstacle_probability: FLOAT32
entity_id: UINT32
point_step: 24 bytes
frame: world
```

semantic label：

```text
0 = free
1 = obstacle
2 = stair
```

目前 rule-based semantic 規則：

- 名稱含 `floor` / `ground`，或 geometry 是 plane：free。
- 否則，名稱含 `stair`：stair。
- 其他 collision：obstacle。

floor 判斷優先於 stair，避免帶有 stair 字樣的樓板模型被整片標成樓梯。

重要參數：

| 參數 | 說明 |
| --- | --- |
| `point_spacing` | collision surface 取樣間距，越小越細 |
| `update_rate` | 發布 Hz |
| `max_points_per_publish` | topic 每包最多點數，0 代表不限制 |
| `pcd_save_interval` | PCD 輸出週期，0 代表不輸出 |
| `transport_topic` | 發布 topic，預設 `/world/dynamic_cloud` |

## RFOctreePathPlanner

SDF 範例：

```xml
<plugin filename="RFOctreePathPlanner" name="RFOctreePathPlanner">
  <transport_topic>/world/dynamic_cloud</transport_topic>
  <rf_model>models/random_forest_voxel_model.rf.txt</rf_model>
  <start>6,-2,1.2</start>
  <goal>0,0,9.2</goal>
  <max_depth>9</max_depth>
  <max_points>50000</max_points>
  <plan_hz>0.1</plan_hz>
  <line_radius>0.06</line_radius>
  <z_offset>0.10</z_offset>
  <block_probability>0.92</block_probability>
  <probability_weight>6</probability_weight>
  <vertical_weight>0.75</vertical_weight>
  <allow_cross_floor>true</allow_cross_floor>
  <stair_connection_radius>1.25</stair_connection_radius>
  <constrain_stair_transitions>true</constrain_stair_transitions>
  <constrain_stair_direction>true</constrain_stair_direction>
  <max_non_stair_vertical_step>0.35</max_non_stair_vertical_step>
  <constrain_stair_exits_to_floor_levels>true</constrain_stair_exits_to_floor_levels>
  <stair_floor_exit_tolerance>0.45</stair_floor_exit_tolerance>
  <floor_z>0</floor_z>
  <story_height>4</story_height>
  <floor_surface_offset>1</floor_surface_offset>
</plugin>
```

資料流：

```text
/world/dynamic_cloud
  -> parse PointCloudPacked
  -> OctreeManager + RandomForestVoxelPredictor
  -> AStarPlanner
  -> visual-only cylinder path model in Gazebo
```

這個 plugin 不讀 CSV。RF 模型來自：

```text
models/random_forest_voxel_model.rf.txt
```

該模型由：

```bash
venv/bin/python3 python/train_model.py
```

產生。

Path visual 是 Gazebo model / visual cylinder，不依賴 marker GUI plugin。若要看見路徑，必須使用 Gazebo GUI，不要使用 `-s` server-only。

### A* 重要參數

| 參數 | 說明 |
| --- | --- |
| `start` / `goal` | world coordinate 起點 / 終點 |
| `max_points` | 每次規劃最多使用點數，目前 predict world 為 50000 |
| `plan_hz` | 最多重建 RF Octree 與規劃頻率 |
| `block_probability` | leaf obstacle probability 阻擋門檻 |
| `stair_connection_radius` | stair virtual connector 半徑 |
| `constrain_stair_transitions` | 只允許在樓梯端點進出 |
| `constrain_stair_direction` | 只允許從樓梯推估方向進出 |
| `max_non_stair_vertical_step` | 非樓梯 leaf 最大 Z 跳躍 |
| `constrain_stair_exits_to_floor_levels` | 樓梯進出必須靠近已知樓層表面 |
| `stair_floor_exit_tolerance` | 樓層表面高度容忍度 |
| `floor_z` / `story_height` / `floor_surface_offset` | 多樓層高度模型 |

目前樓層表面公式：

```text
floor_z + floor_surface_offset + n * story_height
```

predict world 預設為：

```text
0 + 1 + n * 4 => z = 1, 5, 9, ...
```

這些限制是為了避免路徑從樓梯中段離開，或從一般地板 voxel 垂直穿過樓板。

## Debug

確認 topic：

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -l
GZ_PARTITION=dynamic_cloud_test gz topic -i -t /world/dynamic_cloud
```

外部 planner debug：

```bash
./scripts/run_path_planning.sh
```

常見調整：

```bash
PATH_STAIR_FLOOR_EXIT_TOLERANCE=0.55 ./scripts/run_path_planning.sh
PATH_MAX_NON_STAIR_VERTICAL_STEP=0.25 ./scripts/run_path_planning.sh
```
