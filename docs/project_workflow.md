# 專案完整流程與操作手冊

這份文件說明目前專案完成版的實際資料流、執行順序、CSV / RF / Octree / A* 的關係，以及常用參數如何調整。

## 1. 一句話總覽

Gazebo world 內的 collision geometry 會被 `DynamicWorldCloud` 取樣成帶語義欄位的 `/world/dynamic_cloud`。所有後續流程都訂閱這個 topic：feature exporter 產生 Random Forest CSV，viewer 即時顯示 Octree，RF predictor 將模型接回 Octree，A* planner 使用 RF predicted leaf label / obstacle probability 搜尋跨樓層路徑。

```text
Gazebo collision geometry
  -> DynamicWorldCloud
  -> /world/dynamic_cloud
  -> OctreeManager
     -> leaf feature CSV
     -> PCL Octree viewer
     -> RandomForestVoxelPredictor
     -> AStarPlanner
     -> Gazebo path visual
```

海報用系統架構圖：

```text
docs/assets/system_architecture_poster.svg
```

這張圖將資料產生、Random Forest 訓練、RF Octree 推論與 A* 導航流程整合在同一張 poster-ready 架構圖中。

## 2. 資料流細節

### Gazebo topic 來源

`DynamicWorldCloud` 會掃描 Gazebo world 中的 collision entities，依 `point_spacing` 對 box / cylinder / sphere / plane / mesh surface 取樣。每個 collision 的 local cloud 會被 cache，simulation 進行時只需根據 entity pose 轉成 world coordinates。

發布 topic：

```text
/world/dynamic_cloud
```

message type：

```cpp
gz::msgs::PointCloudPacked
```

每個 point 的欄位：

```text
xyz: FLOAT32 x 3
label: UINT32
obstacle_probability: FLOAT32
entity_id: UINT32
```

label 定義：

```text
0 = free / walkable
1 = obstacle
2 = stair
```

semantic 來源目前是 Gazebo plugin 的 rule-based ground truth：

- collision scoped name 含 `floor` / `ground`，或 geometry 是 plane：free，probability 約 0.05。
- 名稱含 `stair`：stair，probability 約 0.25。
- 其他 collision：obstacle，probability 約 0.95。

`entity_id` 會在 Octree leaf 中聚合成 `dominant_entity_id`，方便知道 leaf 主要來自哪個 Gazebo collision。

### Octree 建構資料流

所有 C++ 消費者都會把 `PointCloudPacked` 解析成：

```cpp
std::vector<navigation::PointCloudSample>
```

再呼叫：

```cpp
OctreeManager octree(config);
octree.initialize(samples);
```

若有載入 RF 模型：

```cpp
octree.setMLPredictor(rf_predictor.asMLPredictor());
octree.initialize(samples);
```

此時 leaf 幾何特徵先由點雲計算，接著 RF 模型覆寫 leaf 的 `label` 與 `obstacle_probability`。

### 三種 CSV 的差異

`train_*.csv`

訓練資料。由 Gazebo semantic label 聚合而來，檔名前綴 `train_`，會被 `python/train_model.py` 當作訓練集。

`leaf_features.csv`

predict / evaluation reference。label 仍然是 Gazebo semantic ground truth，不是 RF 預測。訓練腳本會用它和模型輸出比較 accuracy / probability error。

`predicted_rf_leaf_features.csv`

已套用 RF 模型後的快照。這份 CSV 方便離線檢查模型推論結果，但即時 RF viewer 和 Gazebo A* plugin 不讀這份 CSV，它們會直接訂閱 `/world/dynamic_cloud` 並即時建 RF Octree。

## 3. 建置

主專案：

```bash
mkdir -p build
cmake -S . -B build
cmake --build build --target dynamic_world_cloud_plugins
cmake --build build --target leaf_feature_exporter_gazebo
cmake --build build --target rf_octree_path_planner_gazebo
cmake --build build --target test_octree
```

Octree viewer：

```bash
cmake -S scripts -B build/octree_viewer
cmake --build build/octree_viewer
```

驗證：

```bash
./build/test_octree
gz sdf --check gazebo/maps/warehouse_world.sdf
gz sdf --check gazebo/maps/warehouse_predict_world.sdf
```

## 4. 啟動 Gazebo

訓練 / semantic 展示世界：

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -r -v 2
```

RF A* path 展示世界：

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_predict_world.sdf -r -v 2
```

或：

```bash
./scripts/run_predict_with_path.sh
```

注意：若要在 Gazebo GUI 看到 path cylinders，不要加 `-s` server-only。

`scripts/run_gazebo.sh` 會自動設定：

- `LD_LIBRARY_PATH`
- `GZ_PLUGIN_PATH`
- `GZ_SIM_SYSTEM_PLUGIN_PATH`
- `GZ_SIM_RESOURCE_PATH`
- `GZ_FILE_PATH`
- `GZ_PARTITION`

預設 partition：

```text
dynamic_cloud_test
```

檢查 topic：

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -l
GZ_PARTITION=dynamic_cloud_test gz topic -i -t /world/dynamic_cloud
```

## 5. 收集資料與訓練 RF

### 輸出訓練資料

啟動 Gazebo 後，在另一個 terminal：

```bash
FEATURE_TRAIN=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
```

輸出：

```text
data/train_leaf_features.csv
```

若想連續輸出多 frame：

```bash
FEATURE_TRAIN=1 FEATURE_TIMESTAMPED=1 FEATURE_EXPORT_HZ=1 ./scripts/run_feature_export.sh
```

### 輸出 evaluation reference

```bash
PREDICT_FEATURE_ONCE=1 ./scripts/run_feature_export_predict.sh
```

輸出：

```text
data/leaf_features.csv
```

這份資料的 `label` / `obstacle_probability` 仍是 Gazebo semantic ground truth。

### 訓練模型

```bash
venv/bin/python3 python/train_model.py
```

讀取規則：

- `data/train_*.csv`：訓練資料。
- `data/` 中非 `train_`、非 `predicted_` 的 CSV：predict / evaluation 資料。

輸出：

```text
models/random_forest_voxel_model.pkl
models/random_forest_voxel_model.rf.txt
models/random_forest_voxel_report.json
data/predicted_*.csv
```

`.pkl` 給 Python 使用；`.rf.txt` 是 C++17 viewer / exporter / A* plugin 使用的文字模型格式。

### 輸出 RF predicted CSV 快照

```bash
RF_FEATURE_ONCE=1 ./scripts/run_feature_export_rf.sh
```

輸出：

```text
data/predicted_rf_leaf_features.csv
```

## 6. 3D 視覺化

### Gazebo semantic Octree

```bash
./scripts/run_visualization.sh octree
```

資料流：

```text
/world/dynamic_cloud
  -> visualize_octree_gazebo
  -> OctreeManager
  -> PCLVisualizer
```

### RF predicted Octree

```bash
./scripts/run_visualization.sh octree-rf
```

或：

```bash
./scripts/run_visualization_rf.sh
```

資料流：

```text
/world/dynamic_cloud
  -> visualize_octree_gazebo
  -> OctreeManager + RandomForestVoxelPredictor
  -> PCLVisualizer
```

### 原始 pointcloud

```bash
./scripts/run_visualization.sh pointcloud
```

### 常用 viewer 調整

只看 voxel，不顯示白色原始點雲：

```bash
OCTREE_HIDE_POINTS=1 ./scripts/run_visualization.sh octree
```

顯示較少 voxel，提升 FPS：

```bash
OCTREE_MAX_VOXELS=2000 OCTREE_REBUILD_HZ=0.5 ./scripts/run_visualization.sh octree
```

只顯示 voxel center，最快：

```bash
OCTREE_VOXEL_MODE=centers ./scripts/run_visualization.sh octree
```

顯示 center 加 voxel 邊框，適合展示：

```bash
OCTREE_VOXEL_MODE=center-boxes ./scripts/run_visualization.sh octree
```

依 obstacle probability 上色：

```bash
OCTREE_COLOR_MODE=probability ./scripts/run_visualization.sh octree
```

依 label 上色：

```bash
OCTREE_COLOR_MODE=label ./scripts/run_visualization.sh octree
```

樓梯 / cross-floor voxel 會優先顯示亮紫紅色，避免被 probability color 淡化。

## 7. RF Octree A* 路徑

`warehouse_predict_world.sdf` 已載入：

```xml
<plugin name='RFOctreePathPlanner' filename='RFOctreePathPlanner'>
```

plugin 會：

1. 訂閱 `/world/dynamic_cloud`。
2. 解析 semantic point cloud。
3. 用 RF 模型建立 RF predicted Octree。
4. 使用 `AStarPlanner` 搜尋 `start` 到 `goal`。
5. 在 Gazebo 中建立 visual-only cylinder path model。

目前預設：

```text
start = 6, -2, 1.2
goal  = 0,  0, 9.2
```

world 內的 start / goal marker 只有 visual，沒有 collision，因此不會被點雲取樣成障礙物。

### A* cost 與硬限制

不可通行：

- `label == obstacle`
- `obstacle_probability >= block_probability`
- 不同 room 且不是樓梯跨層連通
- 非樓梯 voxel 之間 Z 差超過 `max_non_stair_vertical_step`
- free <-> stair 轉換不在樓梯端點
- stair exit 不靠近已知樓層高度

成本項：

- 幾何距離
- obstacle probability 風險
- stair cost
- cross-floor cost
- vertical movement cost
- obstacle clearance cost

### 樓梯防穿地板參數

地圖高度模型：

```xml
<floor_z>0</floor_z>
<story_height>4</story_height>
<floor_surface_offset>1</floor_surface_offset>
```

因此合法樓層表面約為：

```text
z = 1, 5, 9, ...
```

防止樓梯中段直接穿過地板的參數：

```xml
<max_non_stair_vertical_step>0.35</max_non_stair_vertical_step>
<constrain_stair_exits_to_floor_levels>true</constrain_stair_exits_to_floor_levels>
<stair_floor_exit_tolerance>0.45</stair_floor_exit_tolerance>
```

調整建議：

- 若仍從樓梯中段離開：把 `stair_floor_exit_tolerance` 降到 `0.30` 到 `0.40`。
- 若完全找不到樓梯出口：放寬到 `0.55`。
- 若一般地板仍垂直跳層：把 `max_non_stair_vertical_step` 降到 `0.20` 到 `0.30`。

外部 debug planner 可用：

```bash
PATH_STAIR_FLOOR_EXIT_TOLERANCE=0.55 ./scripts/run_path_planning.sh
PATH_MAX_NON_STAIR_VERTICAL_STEP=0.25 ./scripts/run_path_planning.sh
```

## 8. 參數總表

### DynamicWorldCloud SDF

| 參數 | 預設 / 目前值 | 說明 |
| --- | --- | --- |
| `point_spacing` | `0.25` | collision surface 取樣間距 |
| `update_rate` | `10` | topic 發布 Hz |
| `max_points_per_publish` | `50000` | 每包最多點數 |
| `publish_enabled` | `true` | 是否發布 topic |
| `transport_topic` | `/world/dynamic_cloud` | Gazebo Transport topic |
| `pcd_save_interval` | `0.0` | PCD 存檔週期，0 不存 |

### Feature Export

| 環境變數 | 說明 |
| --- | --- |
| `FEATURE_OUTPUT` | CSV 輸出路徑 |
| `FEATURE_TRAIN` | `1` 時檔名前綴 `train_` |
| `FEATURE_ONCE` | `1` 時輸出第一包後結束 |
| `FEATURE_TIMESTAMPED` | `1` 時每 frame 保留獨立 CSV |
| `FEATURE_MAX_DEPTH` | Octree 最大深度 |
| `FEATURE_MAX_POINTS` | 每 frame 最多輸入點數 |
| `FEATURE_EXPORT_HZ` | CSV 輸出頻率 |
| `FEATURE_RF_MODEL` | C++ `.rf.txt` 模型 |
| `FEATURE_STORY_HEIGHT` | 樓層高度週期 |
| `FEATURE_FLOOR_SURFACE_OFFSET` | 每層可通行地板面高度 |

### Visualization

| 環境變數 | 說明 |
| --- | --- |
| `OCTREE_MAX_DEPTH` | viewer 端 Octree 深度 |
| `OCTREE_MAX_VOXELS` | 最多顯示 leaf voxel |
| `OCTREE_MAX_RENDER_POINTS` | viewer 最多使用點數 |
| `OCTREE_REBUILD_HZ` | viewer 重建 Hz |
| `OCTREE_VOXEL_MODE` | `centers` / `boxes` / `hybrid` / `center-boxes` |
| `OCTREE_HIDE_POINTS` | `1` 隱藏原始點雲 |
| `OCTREE_COLOR_MODE` | `depth` / `label` / `probability` |
| `OCTREE_RF_MODEL` | RF model path |

### Path Planning

| SDF / PATH 參數 | 說明 |
| --- | --- |
| `start` / `PATH_START` | 起點 world coordinate |
| `goal` / `PATH_GOAL` | 終點 world coordinate |
| `max_points` / `PATH_MAX_POINTS` | A* 每 frame 使用點數 |
| `block_probability` / `PATH_BLOCK_PROBABILITY` | obstacle probability 阻擋門檻 |
| `probability_weight` | 風險成本權重 |
| `vertical_weight` | Z 移動成本 |
| `stair_connection_radius` | 樓梯 virtual edge 半徑 |
| `constrain_stair_transitions` | 限制樓梯只能端點進出 |
| `constrain_stair_direction` | 限制樓梯正向進出 |
| `max_non_stair_vertical_step` | 非樓梯最大 Z 跳躍 |
| `constrain_stair_exits_to_floor_levels` | 樓梯出口必須靠近樓層面 |
| `stair_floor_exit_tolerance` | 樓梯出口高度容忍度 |

## 9. 常見問題

### Gazebo 開了但 viewer 沒資料

確認 partition 一致：

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -l
```

`run_gazebo.sh`、`run_visualization.sh`、`run_feature_export.sh` 預設都使用 `dynamic_cloud_test`。

### Octree viewer 很卡

降低：

- `OCTREE_MAX_RENDER_POINTS`
- `OCTREE_MAX_VOXELS`
- `OCTREE_REBUILD_HZ`

並改用：

```bash
OCTREE_VOXEL_MODE=centers
```

### RF path 找不到

先確認 semantic Octree 是否正常顯示樓梯，再檢查：

- RF 模型是否存在：`models/random_forest_voxel_model.rf.txt`
- `stair` voxel 是否在 RF viewer 中仍顯示為樓梯色
- `stair_floor_exit_tolerance` 是否過嚴
- `block_probability` 是否過低

### 路徑穿地板或樓梯中段離開

優先調：

```bash
PATH_MAX_NON_STAIR_VERTICAL_STEP=0.25
PATH_STAIR_FLOOR_EXIT_TOLERANCE=0.35
```

或修改 `warehouse_predict_world.sdf` 內對應參數。

## 10. 效能量測與報告圖表

本專案提供一個 headless benchmark，用來在不開 PCLVisualizer 的情況下量測導航流程。它會訂閱 `/world/dynamic_cloud`，對同一包點雲執行兩種 pipeline：

```text
semantic_octree_astar:
  Gazebo semantic label
  -> OctreeManager
  -> AStarPlanner

rf_octree_astar:
  Gazebo point cloud
  -> OctreeManager + RandomForestVoxelPredictor
  -> AStarPlanner
```

這個比較可以回答兩個問題：

1. 加入 RF 後，Octree build time / total time 增加多少。
2. RF probability 是否讓 A* 搜尋更有效率，例如 expanded nodes 或 plan time 是否降低。

### 執行方式

Terminal 1：啟動 predict world。

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_predict_world.sdf -r -v 2
```

Terminal 2：執行 benchmark。

```bash
./scripts/run_performance_benchmark.sh
```

預設輸出：

```text
data/performance_benchmark.csv
```

CSV 欄位：

| 欄位 | 說明 |
| --- | --- |
| `frame` | 第幾包 point cloud |
| `mode` | `semantic_octree_astar` 或 `rf_octree_astar` |
| `source_points` | topic 原始點數 |
| `used_points` | benchmark 實際使用點數 |
| `parse_ms` | PointCloudPacked 解析時間 |
| `build_ms` | Octree 建構時間；RF mode 也包含 leaf RF 推論 |
| `plan_ms` | A* 搜尋時間 |
| `total_ms` | parse + build + plan |
| `node_count` / `leaf_count` | Octree 節點數 / leaf 數 |
| `estimated_node_memory_kb` | Octree nodes 估計記憶體 |
| `rss_kb_after` | 該 mode 完成後 process RSS |
| `peak_rss_kb` | process peak RSS |
| `path_success` | 是否找到路徑 |
| `expanded_nodes` | A* 展開節點數 |
| `waypoints` | 輸出路徑 waypoint 數 |
| `path_cost` | A* path cost |

### 產生 summary 與圖表

```bash
venv/bin/python3 python/analyze_performance.py
```

輸出：

```text
data/performance_summary.json
data/performance_summary.md
data/performance_summary.png
data/performance_improvement_table.png
data/poster_performance_chart.png
data/poster_performance_table.png
```

`performance_summary.md` 會輸出可直接放進報告的 Markdown 表格；`performance_improvement_table.png` 則會輸出同樣內容的圖片版，方便放進簡報。內容包含：

- semantic / baseline Octree 與 RF Octree 的平均 `build_ms`、`plan_ms`、`total_ms`。
- RF 相對 baseline 的百分比改善，例如 total time、A* plan time、expanded nodes、path cost。
- poster-ready 衍生指標，例如 total speedup、A* planning speedup、latency saved、expanded nodes saved。

海報版建議使用：

- `poster_performance_chart.png`：海報用純長條圖，包含 total time、A* planning time、expanded nodes、path cost。
- `poster_performance_table.png`：高對比表格，列出 baseline、RF Octree 與 improvement。

若環境沒有 `matplotlib`，script 仍會輸出 JSON 與 Markdown table，只會略過 PNG。

### 建議使用的效能指標

目前不把 memory 放入主要海報評比，因為 RF predictor 主要改變 leaf 的 `label` 與 `obstacle_probability`，通常不會改變 Octree topology。更有意義的效能指標是：

| 指標 | 說明 |
| --- | --- |
| `total_ms` | 從解析點雲、建 Octree 到 A* 完成的總延遲 |
| `build_ms` | Octree 建構時間；RF mode 包含 RF 推論 overhead |
| `plan_ms` | A* 實際搜尋時間 |
| `expanded_nodes` | A* 展開多少節點，代表搜尋空間大小 |
| `planning_hz` | `1000 / plan_ms`，代表規劃頻率 |
| `path_cost` | A* 成本函數結果，包含距離、風險、樓梯與垂直移動懲罰 |
| `total_speedup` | baseline total time / RF total time |
| `plan_speedup` | baseline plan time / RF plan time |
| `latency_saved_ms` | baseline 與 RF 的延遲差 |
| `build_overhead_ms` | RF 推論造成的額外建構成本 |

### 常用調整

量測 10 個 frame：

```bash
BENCHMARK_FRAMES=10 ./scripts/run_performance_benchmark.sh
```

限制點數，觀察不同點雲量的效能：

```bash
BENCHMARK_MAX_POINTS=30000 ./scripts/run_performance_benchmark.sh
BENCHMARK_MAX_POINTS=50000 ./scripts/run_performance_benchmark.sh
```

改變起終點：

```bash
PATH_START=6,-2,1.2 PATH_GOAL=0,0,9.2 ./scripts/run_performance_benchmark.sh
```

調整樓梯防穿越參數並觀察成功率與規劃時間：

```bash
PATH_STAIR_FLOOR_EXIT_TOLERANCE=0.55 ./scripts/run_performance_benchmark.sh
PATH_MAX_NON_STAIR_VERTICAL_STEP=0.25 ./scripts/run_performance_benchmark.sh
```

### 報告建議呈現方式

建議至少放一張表和一張圖：

```text
Mode                  Build ms   Plan ms   Total ms   Expanded nodes   Path cost
semantic_octree_astar ...
rf_octree_astar       ...
```

也可以直接使用：

```text
data/performance_summary.md
```

其中會有類似：

```text
Metric          Baseline Mean   RF Mean   RF Improvement
Total time      ... ms          ... ms    ... %
A* plan time    ... ms          ... ms    ... %
Expanded nodes  ... nodes       ... nodes ... %
Path cost       ...             ...       ... %
Total speedup   1.00x           ...x      ...
```

圖表建議：

- `build_ms` bar chart
- `plan_ms` bar chart
- `expanded_nodes` bar chart
- `path_cost` bar chart

解讀時請分開討論：

- RF mode 的 `build_ms` 通常會因模型推論而增加。
- 若 RF probability 讓 `expanded_nodes` 或 `plan_ms` 降低，代表模型對搜尋方向有幫助。
- 若 RF 的 `path_cost` 較高，不一定是壞事，可能代表它選擇較安全、較避障或包含跨樓層限制的路徑。
- 若 `total_ms` 沒有下降，也可以主張 RF 提供更安全的風險感知路徑，而不只是追求最快計算。
