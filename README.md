# navigation_project

本專案是「基於 Octree 與機器學習的室內多樓層 3D 點雲導航優化」的大學專題實作。核心流程是從 Gazebo 產生帶語義標籤的 3D 點雲，建立導航用 adaptive Octree，輸出 Random Forest 訓練資料，再將 RF 模型接回 Octree 與 A*，最後在 Gazebo 中顯示跨樓層路徑。

## Demo Video

[Watch Demo Video]([https://www.youtube.com/watch?v=xxxxxxxxxxx](https://drive.google.com/file/d/1-dRxy0XI1xdLRo6YUCb_PYV-Iks3zpl-/view?usp=sharing))

## 快速開始

建置主程式、Gazebo plugin、feature exporter、A* planner：

```bash
mkdir -p build
cmake -S . -B build
cmake --build build --target dynamic_world_cloud_plugins
cmake --build build --target leaf_feature_exporter_gazebo
cmake --build build --target rf_octree_path_planner_gazebo
cmake --build build --target test_octree
```

建置 Octree / pointcloud viewer：

```bash
cmake -S scripts -B build/octree_viewer
cmake --build build/octree_viewer
```

啟動 Gazebo：

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -r -v 2
```

另一個 terminal 顯示 Octree：

```bash
./scripts/run_visualization.sh octree
```

訓練資料輸出：

```bash
FEATURE_TRAIN=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
venv/bin/python3 python/train_model.py
```

啟動 predict world 並在 Gazebo 中畫 RF Octree A* 路徑：

```bash
./scripts/run_predict_with_path.sh
```

詳細流程請看 [docs/project_workflow.md](docs/project_workflow.md)。Octree 與 A* 設計請看 [docs/octree_design.md](docs/octree_design.md)。

## 系統資料流

```text
Gazebo world / collision geometry
  -> DynamicWorldCloud plugin
  -> /world/dynamic_cloud (PointCloudPacked)
     fields: xyz, label, obstacle_probability, entity_id

資料集產生:
  /world/dynamic_cloud
  -> gazebo_leaf_feature_exporter
  -> OctreeManager
  -> leaf_feature_exporter
  -> data/train_leaf_features.csv 或 data/leaf_features.csv

模型訓練:
  data/train_*.csv
  -> python/train_model.py
  -> models/random_forest_voxel_model.pkl
  -> models/random_forest_voxel_model.rf.txt

RF 推論 / 導航:
  /world/dynamic_cloud
  -> OctreeManager + RandomForestVoxelPredictor
  -> RF predicted leaf label / obstacle_probability
  -> AStarPlanner
  -> Gazebo visual-only path cylinders

視覺化:
  /world/dynamic_cloud
  -> visualize_octree_gazebo
  -> Gazebo semantic Octree 或 RF predicted Octree
  -> PCLVisualizer
```

## 主要檔案

```text
include/octree_manager.h                 Octree API 與節點資料結構
src/octree_manager.cpp                   Linear adaptive Octree 實作
include/astar_planner.h                  A* 參數與 path waypoint API
src/astar_planner.cpp                    RF probability / stair aware A*
include/leaf_feature_exporter.h          Leaf CSV feature export API
src/leaf_feature_exporter.cpp            leaf -> Random Forest feature CSV
src/gazebo_leaf_feature_exporter.cpp     /world/dynamic_cloud -> Octree -> CSV
include/random_forest_voxel_predictor.h  C++ RF text model predictor
src/random_forest_voxel_predictor.cpp    .rf.txt model loading / inference
python/train_model.py                    Random Forest 訓練與報告輸出
dynamic_world_cloud/DynamicWorldCloud.cc Gazebo 點雲與 semantic publisher
dynamic_world_cloud/RFOctreePathPlanner.cc Gazebo 內 RF Octree + A* + path visual
scripts/visualize_octree_gazebo.cpp      PCL 即時 Octree viewer
scripts/visualize_pointcloud_realtime.py Python 原始點雲 viewer
src/octree_performance_benchmark.cpp     semantic Octree vs RF Octree 效能量測
gazebo/maps/warehouse_world.sdf          訓練 / semantic 測試世界
gazebo/maps/warehouse_predict_world.sdf  RF predict 與 A* 展示世界
```

## 常用腳本

`scripts/run_gazebo.sh`

啟動 Gazebo 並設定 plugin / model path。預設 `GZ_PARTITION=dynamic_cloud_test`。

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -r -v 2
./scripts/run_gazebo.sh gazebo/maps/warehouse_predict_world.sdf -r -v 2
```

`scripts/run_visualization.sh`

共用 viewer 入口：

```bash
./scripts/run_visualization.sh octree      # Gazebo semantic Octree
./scripts/run_visualization.sh octree-rf   # RF predicted Octree
./scripts/run_visualization.sh pointcloud  # 原始點雲
```

展示時常用輕量參數：

```bash
OCTREE_VOXEL_MODE=center-boxes \
OCTREE_HIDE_POINTS=1 \
OCTREE_MAX_VOXELS=3000 \
OCTREE_MAX_RENDER_POINTS=40000 \
OCTREE_REBUILD_HZ=0.5 \
./scripts/run_visualization.sh octree
```

`scripts/run_feature_export.sh`

底層 CSV exporter。常用 wrapper：

```bash
FEATURE_TRAIN=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
./scripts/run_feature_export_predict.sh
./scripts/run_feature_export_rf.sh
```

`scripts/run_predict_with_path.sh`

啟動 `warehouse_predict_world.sdf`。該 world 內已載入 `RFOctreePathPlanner` plugin，會自動建 RF Octree、執行 A*，並在 Gazebo GUI 畫 cyan 路徑線。

```bash
./scripts/run_predict_with_path.sh
```

`scripts/run_path_planning.sh`

外部 debug planner，功能和 Gazebo plugin 類似，但路徑用 Gazebo marker topic 發布。主要用來看 console log 與快速調 PATH_* 參數。

```bash
./scripts/run_path_planning.sh
```

`scripts/run_performance_benchmark.sh`

訂閱同一個 Gazebo point cloud frame，依序比較 semantic Octree + A* 與 RF Octree + A* 的建構時間、規劃時間、expanded nodes、path cost 與 speedup。

```bash
./scripts/run_performance_benchmark.sh
venv/bin/python3 python/analyze_performance.py
```

輸出：

```text
data/performance_benchmark.csv
data/performance_summary.json
data/performance_summary.md
data/performance_summary.png
data/performance_improvement_table.png
data/poster_performance_chart.png
data/poster_performance_table.png
```

`performance_summary.md` 會產生可直接放進報告的表格，包含 RF 相對 semantic / baseline Octree 的總時間、建構時間、A* 規劃時間、expanded nodes、path cost、latency saved 與 speedup。

海報建議優先使用 `poster_performance_chart.png` 和 `poster_performance_table.png`。前者是純長條圖版，後者是高對比表格版。

```bash
venv/bin/python3 python/analyze_performance.py
```

## 重要參數

### DynamicWorldCloud

位置：`gazebo/maps/*.sdf`

- `point_spacing`：collision geometry 取樣間距，越小點越密。
- `update_rate`：點雲發布 Hz。
- `max_points_per_publish`：每包 topic 最多點數。目前展示世界回到 `50000`。
- `transport_topic`：預設 `/world/dynamic_cloud`。
- `pcd_save_interval`：大於 0 時定期輸出 PCD。

### Feature Exporter

- `FEATURE_TRAIN=1`：輸出檔名前綴 `train_`。
- `FEATURE_ONCE=1`：收到第一包 cloud 後輸出一次就結束。
- `FEATURE_MAX_POINTS`：每次輸出最多使用點數。
- `FEATURE_RF_MODEL`：載入 `.rf.txt`，輸出 RF predicted label / probability。
- `FEATURE_STORY_HEIGHT=4`、`FEATURE_FLOOR_SURFACE_OFFSET=1`：多樓層高度模型。

### Octree Viewer

- `OCTREE_VOXEL_MODE`：`centers`、`boxes`、`hybrid`、`center-boxes`。
- `OCTREE_HIDE_POINTS=1`：只看 voxel，不顯示白色原始點雲。
- `OCTREE_COLOR_MODE=probability`：依 obstacle probability 上色。
- `OCTREE_RF_MODEL`：載入 RF 模型顯示 predicted Octree。
- Stair / cross-floor voxel 會優先顯示亮紫紅色，方便辨識樓梯。

### A* / Path Planning

位置：`warehouse_predict_world.sdf` 或 `scripts/run_path_planning.sh` 的 `PATH_*`。

- `start` / `goal` 或 `PATH_START` / `PATH_GOAL`：世界座標。
- `block_probability`：高於此 obstacle probability 視為不可通行。
- `probability_weight`：越高越避開高風險 voxel。
- `stair_connection_radius`：樓梯 virtual connector 半徑。
- `constrain_stair_transitions`：限制只能在樓梯端點進出。
- `constrain_stair_direction`：限制從樓梯正確面向進出，降低側邊上樓。
- `max_non_stair_vertical_step`：非樓梯 voxel 可允許的最大 Z 差，避免穿地板。
- `constrain_stair_exits_to_floor_levels`：樓梯只能在已知樓層高度附近進出。
- `stair_floor_exit_tolerance`：距離樓層高度多少以內可進出樓梯。

### Performance Benchmark

- `BENCHMARK_FRAMES`：量測幾包 point cloud。
- `BENCHMARK_MAX_POINTS`：每包最多使用點數，預設 `50000`。
- `BENCHMARK_OUTPUT`：CSV 輸出路徑。
- `BENCHMARK_RF_MODEL`：C++ `.rf.txt` 模型。
- `PATH_*`：benchmark 會沿用 path planner 的起終點與 A* 參數。

CSV 會有兩種 mode：

```text
semantic_octree_astar
rf_octree_astar
```

報告時建議比較：

- `build_ms`
- `plan_ms`
- `total_ms`
- `expanded_nodes`
- `path_cost`
- `planning_hz`
- `total_speedup`
- `plan_speedup`
- `latency_saved_ms`
- `path_success`

目前多樓層高度模型：

```text
floor_z = 0
story_height = 4
floor_surface_offset = 1
合法樓層表面約為 z = 1, 5, 9, ...
```

## Random Forest 訓練與推論

1. 啟動 Gazebo。
2. 輸出訓練 CSV：

```bash
FEATURE_TRAIN=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
```

3. 可另外輸出 predict reference CSV：

```bash
PREDICT_FEATURE_ONCE=1 ./scripts/run_feature_export_predict.sh
```

4. 訓練模型：

```bash
venv/bin/python3 python/train_model.py
```

`train_model.py` 會讀取：

- `data/train_*.csv` 作為訓練資料。
- `data/` 中非 `train_`、非 `predicted_` 開頭的 CSV 作為 predict / evaluation 資料。

輸出：

```text
models/random_forest_voxel_model.pkl
models/random_forest_voxel_model.rf.txt
models/random_forest_voxel_report.json
data/predicted_*.csv
```

`.pkl` 給 Python 使用，`.rf.txt` 給 C++ viewer / exporter / planner 使用。

## 依賴

Ubuntu 24.04 / Gazebo Harmonic 建議安裝：

```bash
sudo apt install \
  libgz-sim8-dev \
  libgz-common5-dev \
  libgz-plugin2-dev \
  libgz-transport13-dev \
  libgz-msgs10-dev \
  libpcl-dev
```

Python 訓練與 pointcloud viewer 依照 `python/train_model.py` 與 `scripts/visualize_pointcloud_realtime.py` 使用的套件安裝到 `venv`。

## 驗證

```bash
./build/test_octree
gz sdf --check gazebo/maps/warehouse_predict_world.sdf
```

Gazebo 的 inertia warning 來自部分 mesh model 的慣性設定，world 仍可載入；若只做靜態導航展示，不影響點雲 topic 與 Octree pipeline。
