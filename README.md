# navigation_project

本專案是「基於 Octree 與機器學習的室內多樓層 3D 點雲導航優化」的大學專題實作。

目前主線功能是：

- Gazebo Sim 內產生室內場景 ground-truth 3D 點雲
- 透過 Gazebo Transport 發布 `/world/dynamic_cloud`
- C++ `OctreeManager` 建立導航用 Linear Octree
- Headless feature exporter 輸出 leaf voxel CSV 給 Random Forest 訓練
- PCLVisualizer 即時顯示 Octree voxel 切割結果
- Gazebo topic 內已包含 rule-based semantic label、obstacle probability 與 entity id
- 後續可接訓練好的 ML 模型、room id 自動標記與 A* / Hybrid A* planner

完整流程與參數說明請看：

- [docs/project_workflow.md](docs/project_workflow.md)
- [docs/octree_design.md](docs/octree_design.md)
- [dynamic_world_cloud/README.md](dynamic_world_cloud/README.md)

## 專案結構

```text
include/octree_manager.h              Octree API 與資料結構
include/leaf_feature_exporter.h       Leaf feature CSV export API
src/octree_manager.cpp                Octree 核心實作
src/leaf_feature_exporter.cpp         Random Forest leaf feature 輸出
src/gazebo_leaf_feature_exporter.cpp  Gazebo topic -> Octree -> CSV
tests/test_octree.cpp                 Octree 單元測試
dynamic_world_cloud/                  Gazebo system plugin
gazebo/maps/warehouse_world.sdf       測試用室內倉庫世界
scripts/visualize_octree_gazebo.cpp   即時 Gazebo Octree viewer
scripts/visualize_pointcloud_realtime.py Python 即時點雲 viewer
scripts/run_gazebo.sh                 Gazebo simulation 啟動腳本
scripts/run_visualization.sh          Octree / pointcloud 共用啟動腳本
scripts/run_feature_export.sh         Gazebo leaf feature CSV 輸出腳本
docs/                                 設計與操作文件
```

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

## 建置

主專案與 Gazebo plugin：

```bash
mkdir -p build
cmake -S . -B build
cmake --build build --target navigation_octree
cmake --build build --target test_octree
cmake --build build --target dynamic_world_cloud
cmake --build build --target leaf_feature_exporter_gazebo
```

Octree viewer：

```bash
cmake -S scripts -B build/octree_viewer
cmake --build build/octree_viewer
```

測試：

```bash
./build/test_octree
```

## 快速執行

Terminal 1：啟動 Gazebo simulation。

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -s -r -v 2
```

Terminal 2：啟動即時 Octree 視覺化。

```bash
./scripts/run_visualization.sh octree
```

Terminal 2 也可以改成輸出 Random Forest 訓練 CSV，不需要開 viewer：

```bash
FEATURE_TRAIN=1 ./scripts/run_feature_export.sh
```

預設會訂閱 `/world/dynamic_cloud`，每秒最多重建一次 Octree，並輸出：

```text
data/train_leaf_features.csv
```

訓練 Random Forest model：

```bash
venv/bin/python3 python/train_model.py
```

預設會讀取 `data/train_*.csv` 作為訓練資料，並自動將 `data/` 中非 `train_`、非 `predicted_` 開頭的 CSV 當作待預測資料，例如 `data/leaf_features.csv`。輸出：

```text
models/random_forest_voxel_model.pkl
models/random_forest_voxel_report.json
data/predicted_leaf_features.csv
```

如果畫面太卡，改用較輕量模式：

```bash
OCTREE_VOXEL_MODE=centers \
OCTREE_MAX_VOXELS=3000 \
OCTREE_MAX_RENDER_POINTS=40000 \
OCTREE_REBUILD_HZ=0.5 \
./scripts/run_visualization.sh octree
```

若只想看原始 pointcloud：

```bash
./scripts/run_visualization.sh pointcloud
```

## Gazebo 點雲資料流

目前 `warehouse_world.sdf` 會載入 `DynamicWorldCloud` plugin。

Plugin 會：

1. 掃描 world 內的 collision geometry。
2. 依照 `point_spacing` 取樣成 local point cloud；mesh 會沿 triangle surface 取樣，不只取 vertices。
3. 將 local cloud cache 起來。
4. 在 `PostUpdate()` 依照 `update_rate` 轉成 world coordinates。
5. 發布帶語義欄位的 `gz::msgs::PointCloudPacked` 到 `/world/dynamic_cloud`。
6. `leaf_feature_exporter_gazebo` 可訂閱 topic，轉成 `std::vector<navigation::PointCloudSample>`。
7. `OctreeManager` 建立 Octree，計算 leaf PCA / normal / density 等特徵。
8. `leaf_feature_exporter` 將 leaf features 輸出成 CSV。
9. Viewer 也可同時訂閱同一個 topic，獨立負責 PCLVisualizer 顯示。

## 常用視覺化模式

只看 voxel center，速度最快：

```bash
--no-points --voxel-mode centers
```

看 voxel center 加上體積邊框，適合展示：

```bash
--no-points --voxel-mode center-boxes --max-voxels 2000
```

保留原始點雲形狀，同時顯示少量 voxel box：

```bash
--voxel-mode hybrid
```

完整 wireframe voxel，最直觀但較卡：

```bash
--voxel-mode boxes --max-voxels 1000
```

## 重要參數

Gazebo plugin 參數在 [gazebo/maps/warehouse_world.sdf](gazebo/maps/warehouse_world.sdf)：

- `point_spacing`：collision 幾何取樣密度。
- `update_rate`：點雲發布頻率。
- `max_points_per_publish`：每次最多發布點數，設為 `0` 代表不限制。
- `transport_topic`：目前為 `/world/dynamic_cloud`。
- `pcd_save_interval`：是否定期輸出 PCD。

Viewer 參數：

- `--partition`：Gazebo Transport partition，需和 Gazebo 相同。
- `--topic`：訂閱 topic，預設 `/world/dynamic_cloud`。
- `--max-depth`：Octree 最大深度。
- `--max-voxels`：最多顯示多少 leaf voxel。
- `--max-render-points`：viewer 端最多使用多少點建 Octree。
- `--rebuild-hz`：每秒最多重建與刷新幾次。
- `--voxel-mode`：`centers`、`boxes`、`hybrid`、`center-boxes`。
- `--no-points`：不顯示原始白色點雲。
- `--color-mode`：`depth`、`label`、`probability`，預設由 `scripts/run_visualization.sh` 設為 `probability`。
- `--probability-color`：依 `obstacle_probability` 上色，越接近 1 越醒目。
- `--label-color`：用語義 label 上色，而不是 Octree depth。

Stair / cross-floor voxel 在所有 color mode 下都會優先顯示為亮紫紅色，避免樓梯因低 obstacle probability 而不明顯。

Feature exporter 參數：

- `--partition`：Gazebo Transport partition，需和 Gazebo 相同。
- `--topic`：訂閱 topic，預設 `/world/dynamic_cloud`。
- `--output`：CSV 輸出路徑，預設 `data/leaf_features.csv`。
- `--max-depth`：特徵輸出使用的 Octree 最大深度。
- `--max-points`：每次輸出最多使用多少收到的點。
- `--export-hz`：每秒最多重建 Octree 並輸出 CSV 幾次。
- `--weak-labels`：不用 Gazebo semantic 欄位，改用 exporter 端規則覆寫 `label` 與 `obstacle_probability`。
- `--train`：輸出 CSV 檔名自動加上 `train_` 前綴，例如 `train_leaf_features.csv`。
- `--floor-z`：第 0 層樓的 z 原點。
- `--story-height`：樓層週期高度，預設 `4`。
- `--floor-surface-offset`：每層樓內可通行地板面的局部 z offset。
- `--ceiling-offset`：每層樓內天花板局部 z offset，腳本預設 `4`。
- `--once`：收到第一包點雲後輸出一次就結束。
- `--timestamped`：每次輸出成獨立檔案，不覆蓋前一份 CSV。

一般操作建議優先改 [scripts/run_visualization.sh](scripts/run_visualization.sh) 上方的參數設定區，或用環境變數覆寫，例如：

```bash
OCTREE_MAX_VOXELS=1000 ./scripts/run_visualization.sh octree
POINTCLOUD_POINT_SIZE=2 ./scripts/run_visualization.sh pointcloud
FEATURE_EXPORT_HZ=0.5 FEATURE_TIMESTAMPED=1 ./scripts/run_feature_export.sh
FEATURE_TRAIN=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
FEATURE_WEAK_LABELS=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
```

## 狀態與後續工作

已完成：

- Gazebo collision geometry ground-truth cloud plugin
- `/world/dynamic_cloud` Gazebo Transport 資料流
- Linear Octree 與 adaptive voxel sizing
- Leaf PCA / avg_normal / Random Forest feature CSV export
- Gazebo topic headless feature exporter
- PCL 即時 Octree 視覺化
- Gazebo semantic point fields：`label`、`obstacle_probability`、`entity_id`
- 樓梯 voxel 高亮與 cross-floor flag
- Random Forest 訓練腳本 `python/train_model.py`

後續可擴充：

- 將 ML 模型輸出接到 `PointCloudSample` / `MLResult`
- 自動 room id 標記
- 3D A* 或 Hybrid A* path planner
- 將 planner cost 與 `computeTraversalInfo()` 串接
