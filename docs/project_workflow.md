# 專案整體流程與操作文件

本文件整理目前專案的實際工作流程，讓後續開發者可以從文件理解：

- Gazebo plugin 如何產生點雲
- 點雲資料如何透過 Gazebo Transport 傳遞
- Octree 如何接收並切割資料
- 如何輸出 Random Forest 訓練用 leaf feature CSV
- 如何執行即時 3D Octree 視覺化
- 常用參數要如何調整

目前主要資料流分成「訓練資料輸出」與「視覺化展示」兩條，兩者都訂閱同一個 Gazebo topic，互不依賴：

```text
warehouse_world.sdf
  -> DynamicWorldCloud Gazebo system plugin
  -> /world/dynamic_cloud, gz::msgs::PointCloudPacked

訓練資料管線:
  -> src/gazebo_leaf_feature_exporter.cpp
  -> OctreeManager
  -> leaf_feature_exporter
  -> data/leaf_features.csv

視覺化管線:
  -> scripts/visualize_octree_gazebo.cpp
  -> OctreeManager
  -> PCLVisualizer 即時顯示 Octree voxel
```

## 1. 主要元件

### DynamicWorldCloud plugin

位置：

- `dynamic_world_cloud/DynamicWorldCloud.cc`
- `dynamic_world_cloud/DynamicWorldCloud.hh`
- `dynamic_world_cloud/CMakeLists.txt`

功能：

- 作為 Gazebo Sim system plugin 載入世界。
- 掃描 world 內所有 collision geometry。
- 支援 box、cylinder、sphere、mesh、plane。
- 對每個 collision 取樣成 local point cloud。
- Mesh collision 會沿 triangle surface 依 `point_spacing` 取樣，不只使用 mesh vertices，因此自製 STL 牆面與樓梯也能產生足夠密度的點雲。
- 將 local cloud cache 起來，避免每個 simulation tick 重新取樣。
- 每次發布時只根據目前 entity pose 把 local points 轉成 world coordinates。
- 將全世界點雲發布成 `gz::msgs::PointCloudPacked`。
- 每個點除了 `x/y/z`，也會帶 `label`、`obstacle_probability`、`entity_id`。
- 可選擇定期輸出 `.pcd` 檔案。
- 支援 simulation 中新增或刪除 entity 時更新 cache。

### OctreeManager

位置：

- `include/octree_manager.h`
- `src/octree_manager.cpp`

功能：

- 將點雲建立成 Linear Octree。
- 用 `std::vector<OctreeNode>` 儲存節點。
- 每個節點保存 `morton_code`、`depth`、`bounds`、`children`、`orthogonal_neighbors`。
- 支援 adaptive voxel sizing。
- 支援 leaf voxel label、obstacle probability、room id、cross-floor flag。
- 支援 6 方向正交鄰居。
- 提供 A* 可用的 traversal cost 資訊。
- 建構 leaf 時會根據 leaf 內點集合計算 centroid、avg_normal、covariance eigenvalues、linearity、flatness、roughness、curvature。

### Leaf feature exporter

位置：

- `include/leaf_feature_exporter.h`
- `src/leaf_feature_exporter.cpp`
- `src/gazebo_leaf_feature_exporter.cpp`
- `scripts/run_feature_export.sh`
- `scripts/run_feature_export_predict.sh`
- `scripts/run_feature_export_rf.sh`

功能：

- `leaf_feature_exporter.cpp` 將 `OctreeNode` leaf 轉成 Random Forest 訓練用特徵列。
- `gazebo_leaf_feature_exporter.cpp` 是 headless 資料管線節點，直接訂閱 Gazebo `/world/dynamic_cloud`。
- 每次收到點雲後，依照 `--export-hz` 節流，將最新點雲轉成 `std::vector<navigation::PointCloudSample>`。
- 若 topic 內有 semantic 欄位，直接填入 `label`、`obstacle_probability`、`entity_id`。
- 使用 `OctreeManager::initialize(samples)` 建立當前 frame 的 Octree。
- Octree 會把 leaf 內 semantic points 聚合成 leaf label 與 dominant entity。
- 呼叫 `exportOctreeLeafFeaturesToCSV(octree.nodes(), output_path)` 輸出 CSV。
- 可以用固定檔名覆蓋輸出，也可以用 `--timestamped` 保留每一個 frame 的 CSV。
- 若使用 `--train`，輸出檔名會加上 `train_` 前綴，方便區分訓練資料與未來要送模型預測的 feature CSV。

這個 exporter 不開啟 PCLVisualizer，也不依賴 Octree 3D 顯示工具。這是目前推薦的資料產生方式，因為訓練資料輸出不應該被 rendering FPS 或視窗互動影響。

### 即時 Octree viewer

位置：

- `scripts/visualize_octree_gazebo.cpp`
- `scripts/run_visualization.sh`
- `scripts/CMakeLists.txt`

功能：

- 直接訂閱 Gazebo Transport topic `/world/dynamic_cloud`。
- 解析 `gz::msgs::PointCloudPacked` 的 `xyz` float32 packed data，以及 `label`、`obstacle_probability`、`entity_id` semantic 欄位。
- 依照設定節流解析頻率，避免 viewer 拖慢 Gazebo。
- 使用收到的點雲建立 Octree。
- 用 PCLVisualizer 顯示 Octree leaf voxel。
- 可選擇顯示原始點雲、voxel center、voxel wireframe box。
- 預設使用 probability color mode；stair / cross-floor voxel 在所有 color mode 下都會優先顯示為亮紫紅色。

### 共用視覺化啟動腳本

位置：

- `scripts/run_visualization.sh`

功能：

- 用同一個入口啟動 Octree 3D 顯示或 pointcloud 3D 顯示。
- 腳本上方集中放置常用參數，並用註解說明每個參數用途。
- 支援用環境變數臨時覆寫參數，不需要直接修改 C++ 或 Python 程式。

### Gazebo simulation 啟動腳本

位置：

- `scripts/run_gazebo.sh`

功能：

- 設定 `LD_LIBRARY_PATH`、`GZ_PLUGIN_PATH`、`GZ_SIM_SYSTEM_PLUGIN_PATH`。
- 設定 `GZ_SIM_RESOURCE_PATH` 與 `GZ_FILE_PATH`，讓 Gazebo 能找到 `gazebo/maps` 內的 world，以及 `gazebo/maps/models` 內的本地模型資源。
- 設定預設 `GZ_PARTITION=dynamic_cloud_test`。
- 從專案根目錄或 `gazebo/maps` 自動解析 world 檔案路徑。

## 2. Gazebo plugin 載入方式

目前 `gazebo/maps/warehouse_world.sdf` 內已經載入 plugin：

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

### Plugin 參數

`point_spacing`

控制 collision geometry 取樣密度，單位是 meter。數值越小點越密，Octree 顯示更細，但 CPU 和記憶體成本更高。

建議：

- 快速 demo：`0.25` 到 `0.40`
- 細緻地圖：`0.05` 到 `0.15`

`update_rate`

控制 plugin 發布點雲的頻率，單位是 Hz。即使 Gazebo simulation tick 很快，plugin 也只會在到達發布時間時重建 global cloud。

建議：

- demo：`1` 到 `5`
- 即時動態測試：`10`

`pcd_save_interval`

控制是否定期儲存 PCD。`0.0` 代表不存檔。若設成 `5.0`，代表每 5 秒輸出一次。

`publish_enabled`

是否發布 `/world/dynamic_cloud`。視覺化與 Octree pipeline 需要設為 `true`。

`max_points_per_publish`

限制每次 topic 發布的最大點數。若完整 cloud 大於此數量，plugin 會等距抽樣後發布。設為 `0` 代表不限制發布點數。

建議：

- 順暢 demo：`30000` 到 `80000`
- 高細節測試：`100000` 以上

`pcd_directory`

PCD 輸出資料夾。只有 `pcd_save_interval > 0` 時會使用。

`transport_topic`

點雲發布 topic，目前預設為：

```text
/world/dynamic_cloud
```

## 3. Plugin 內部流程

### Configure 階段

Gazebo 載入 world 時會呼叫 `DynamicWorldCloud::Configure()`。

流程：

1. 從 SDF 讀取 plugin 參數。
2. 建立 Gazebo Transport publisher。
3. 掃描目前 world 中已存在的 collision entities。
4. 對每個 collision geometry 建立 local point cloud cache。

### Local cloud 建立

Plugin 會用 collision geometry 類型決定取樣方式：

- Box：在表面用固定 spacing 取樣。
- Cylinder：取樣側面與上下圓面。
- Sphere：取樣球面。
- Mesh：讀 mesh triangle index / vertices，沿 triangle surface 依 `point_spacing` 取樣；若 mesh 缺少 triangle index，才退回 sequential triangle 或 vertex fallback。
- Plane：依有限平面大小取樣。

每個 collision 的 local cloud 只建一次並 cache，後續不重複取樣。

### PostUpdate 階段

Gazebo simulation 每 tick 會呼叫 `PostUpdate()`，但 plugin 會先檢查是否到了發布或存檔時間。

流程：

1. 若 simulation paused，直接跳過。
2. 根據 `update_rate` 判斷是否需要 publish。
3. 根據 `pcd_save_interval` 判斷是否需要存檔。
4. 更新 entity cache，偵測新增或刪除的 collision。
5. 將每個 collision 的 local cloud 透過目前 world pose 轉成 world cloud。
6. 發布 `gz::msgs::PointCloudPacked`。
7. 如果啟用 PCD，寫出 binary PCD。

## 4. 資料傳遞格式

Plugin 發布的是 Gazebo Transport message：

```cpp
gz::msgs::PointCloudPacked
```

目前欄位格式是：

```text
field: "xyz"
datatype: FLOAT32
count: 3 floats at offset 0
field: "label"
datatype: UINT32
field: "obstacle_probability"
datatype: FLOAT32
field: "entity_id"
datatype: UINT32
point_step: 24 bytes
每個點: float x, float y, float z, uint32 label, float probability, uint32 entity_id
frame: "world"
```

`DynamicWorldCloud` 目前會根據 collision scoped name 與 geometry 做 semantic 推論：

- 名稱包含 `floor` 或 `ground`，或 collision geometry 是 plane：`label=0`，代表可通行地板。
- 否則，名稱包含 `stair`：`label=2`，代表樓梯。
- 其他 collision：`label=1`，代表障礙物。

這個順序是刻意的：樓板模型應視為 floor；若模型名稱同時描述 floor 與 stair access，也會優先使用 floor semantics，避免整片樓板被誤標成 stair。

`entity_id` 來自 Gazebo collision entity id，會被 exporter 聚合成 leaf 的 dominant entity，方便後續回查 leaf 主要來自哪個物件。

資料傳遞方式：

```text
DynamicWorldCloud::PublishPointCloud()
  -> BuildPointCloudMessage()
  -> cloud_pub_.Publish(msg)
  -> /world/dynamic_cloud

訓練資料:
  -> gazebo_leaf_feature_exporter.cpp callback
  -> parsePointCloudPacked()
  -> std::vector<navigation::PointCloudSample>
  -> OctreeManager::initialize(samples)
  -> exportOctreeLeafFeaturesToCSV()
  -> data/leaf_features.csv

視覺化:
  -> visualize_octree_gazebo.cpp callback
  -> parsePointCloudPacked()
  -> std::vector<navigation::PointCloudSample>
  -> OctreeManager::initialize(samples)
  -> PCLVisualizer
```

Viewer 會根據 `--max-render-points` 對收到的點雲再做一次抽樣，避免大量點雲造成 Octree 重建與 PCL rendering 過慢。

Feature exporter 會根據 `--max-points` 對收到的點雲抽樣，避免訓練資料輸出拖慢 simulation。若點雲含 semantic 欄位，抽樣後仍會保留每個 sample 的 label、probability 與 entity id。

## 5. 建置方式

### 安裝依賴

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

### 建置主專案與 plugin

```bash
mkdir -p build
cmake -S . -B build
cmake --build build --target navigation_octree
cmake --build build --target test_octree
cmake --build build --target dynamic_world_cloud
cmake --build build --target leaf_feature_exporter_gazebo
```

測試 Octree：

```bash
./build/test_octree
```

### 建置 Octree viewer

Viewer 使用獨立的 CMake 設定，避免主專案 target 與展示工具互相干擾。

```bash
cmake -S scripts -B build/octree_viewer
cmake --build build/octree_viewer
```

會產生：

```text
build/octree_viewer/visualize_octree_gazebo
```

確認共用啟動腳本可執行：

```bash
chmod +x scripts/run_visualization.sh
chmod +x scripts/run_feature_export.sh
```

## 6. 執行流程

### Terminal 1：啟動 Gazebo simulation

```bash
./scripts/run_gazebo.sh gazebo/maps/warehouse_world.sdf -s -r -v 2
```

`scripts/run_gazebo.sh` 會自動設定：

- `LD_LIBRARY_PATH`
- `GZ_PLUGIN_PATH`
- `GZ_SIM_SYSTEM_PLUGIN_PATH`
- `GZ_SIM_RESOURCE_PATH`
- `GZ_FILE_PATH`
- `GZ_PARTITION`

預設 partition 是：

```text
dynamic_cloud_test
```

### Terminal 2：確認 topic

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -l
```

應該要看到：

```text
/world/dynamic_cloud
```

查看 topic metadata：

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -i -t /world/dynamic_cloud
```

### Terminal 3：輸出 Random Forest leaf feature CSV

推薦讓資料輸出獨立於 viewer 執行。若要輸出一份「predict / accuracy 評估用」資料，使用：

```bash
./scripts/run_feature_export_predict.sh
```

這份 CSV 預設輸出到：

```text
data/leaf_features.csv
```

它的 `label` 與 `obstacle_probability` 來自 Gazebo semantic 欄位聚合，適合給 `python/train_model.py` 作為非 `train_` 的 predict CSV，讓模型輸出與原始 semantic label 做 accuracy / MAE 比對。

只輸出第一包點雲並結束：

```bash
PREDICT_FEATURE_ONCE=1 ./scripts/run_feature_export_predict.sh
```

每一秒輸出一份帶 frame 編號的 predict CSV：

```bash
PREDICT_FEATURE_TIMESTAMPED=1 PREDICT_FEATURE_EXPORT_HZ=1 ./scripts/run_feature_export_predict.sh
```

輸出訓練資料並在檔名前加上 `train_`：

```bash
FEATURE_TRAIN=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
```

若要直接套用 RF 模型，輸出已經由模型推論過的 leaf label / probability：

```bash
./scripts/run_feature_export_rf.sh
```

這份 CSV 預設輸出到：

```text
data/predicted_rf_leaf_features.csv
```

降低負載：

```bash
PREDICT_FEATURE_MAX_POINTS=50000 PREDICT_FEATURE_EXPORT_HZ=0.5 ./scripts/run_feature_export_predict.sh
```

### Terminal 4：啟動即時 Octree 視覺化

推薦展示模式，使用 Gazebo semantic label / probability：

```bash
./scripts/run_visualization.sh octree
```

如果要顯示 RF 預測後的 Octree：

```bash
./scripts/run_visualization_rf.sh
```

若要比較理想 semantic 與 RF 預測，開兩個 terminal：

```bash
./scripts/run_visualization.sh octree
./scripts/run_visualization_rf.sh
```

兩個 viewer 會使用不同視窗標題，方便比較顏色與 voxel 分布差異。

若覺得邊框太卡，可以改成只顯示 voxel center：

```bash
OCTREE_VOXEL_MODE=centers \
OCTREE_MAX_VOXELS=3000 \
OCTREE_MAX_RENDER_POINTS=40000 \
OCTREE_REBUILD_HZ=0.5 \
./scripts/run_visualization.sh octree
```

若想保留地圖原始形狀，可把 `OCTREE_HIDE_POINTS` 設成 `0`。

```bash
OCTREE_HIDE_POINTS=0 ./scripts/run_visualization.sh octree
```

### Terminal 4 可替代方案：只顯示原始 pointcloud

```bash
./scripts/run_visualization.sh pointcloud
```

## 7. 視覺化參數

日常操作建議優先使用 `scripts/run_visualization.sh` 上方的參數設定區。常用環境變數如下：

- `GZ_PARTITION_VALUE`：Gazebo Transport partition。
- `GZ_POINTCLOUD_TOPIC`：點雲 topic。
- `OCTREE_MAX_DEPTH`：Octree 最大深度。
- `OCTREE_MAX_VOXELS`：Octree viewer 最多顯示的 leaf voxel 數。
- `OCTREE_MAX_RENDER_POINTS`：Octree viewer 最多使用多少點重建 Octree。
- `OCTREE_REBUILD_HZ`：Octree viewer 每秒最多更新次數。
- `OCTREE_VOXEL_MODE`：`centers`、`boxes`、`hybrid`、`center-boxes`。
- `OCTREE_HIDE_POINTS`：`1` 隱藏原始點雲，`0` 顯示原始點雲。
- `OCTREE_COLOR_MODE`：`depth`、`label`、`probability`，腳本預設為 `probability`。
- `OCTREE_WINDOW_TITLE`：viewer 視窗標題，適合同時開 ideal / RF predicted 視窗時區分。
- `OCTREE_RF_MODEL`：指定 `models/random_forest_voxel_model.rf.txt` 時，viewer 端 Octree 會使用 RF 模型推論 leaf label / probability。
- `POINTCLOUD_POINT_SIZE`：Python pointcloud viewer 點大小。
- `POINTCLOUD_MAX_RENDER_POINTS`：Python pointcloud viewer 最多顯示點數。

底層 `visualize_octree_gazebo` 仍支援下列 CLI 參數，方便需要直接呼叫 binary 時使用。

`--partition`

Gazebo Transport partition。必須和 Gazebo simulation 使用相同 partition。`scripts/run_gazebo.sh` 預設是 `dynamic_cloud_test`。

`--topic`

訂閱的 point cloud topic。預設是 `/world/dynamic_cloud`。

`--max-depth`

Octree 最大深度。越大代表 voxel 可以切得越細，但 leaf 數量也可能增加。

`--max-voxels`

最多顯示多少個 leaf voxel。這是 PCLVisualizer 順暢度最重要的參數。

建議：

- 很順：`1000` 到 `3000`
- 一般展示：`3000` 到 `6000`
- 高細節：`10000` 以上

`--max-render-points`

Viewer 最多拿多少點來建立 Octree。這不會改變 Gazebo plugin 發布資料，只是 viewer 端抽樣。

`--rebuild-hz`

Viewer 每秒最多解析、重建 Octree、刷新畫面的次數。預設是 `1`，也就是每秒更新一次。若只是展示 Octree 切割，建議維持 `1` 或 `0.5`。

`--no-points`

不顯示原始白色點雲，只顯示 Octree voxel。

`--no-voxels`

只顯示原始點雲，不顯示 Octree。

`--voxel-mode`

控制 voxel 顯示方式：

- `centers`：只顯示 leaf voxel 中心點，最快。
- `boxes`：每個 leaf voxel 都畫 wireframe box，最直觀但最卡。
- `hybrid`：顯示所有 sampled centers，並畫少量 box 作為示意。
- `center-boxes`：顯示中心點，同時替同一批 sampled voxel 畫完整 wireframe box。

`--label-color`

改用語義顏色，而不是 probability / depth 顏色：

- 綠色：Free
- 紅色：Obstacle
- 亮紫紅色：Stair 或 cross-floor

`--probability-color`

依 `obstacle_probability` 使用連續色階。低概率 voxel 會偏淡藍、較不醒目；中間概率偏黃；接近 `1.0` 的 voxel 會變成亮紅，適合檢查障礙物風險分布。

`--rf-model`

載入 `python/train_model.py` 產生的 C++ Random Forest text model。viewer 建構 Octree 並計算 leaf 幾何特徵後，會用模型推論 `label` 與 `obstacle_probability`，再依目前 color mode 顯示。

`--title`

設定 viewer 視窗標題前綴。`scripts/run_visualization.sh octree` 預設是 `Gazebo Semantic Navigation Octree`，`scripts/run_visualization_rf.sh` 預設是 `RF Predicted Navigation Octree`。

Stair / cross-floor voxel 在所有 color mode 下都會固定顯示為亮紫紅色，不會因樓梯的低障礙概率或 depth 色階而變淡。

目前 Gazebo plugin 發出的點雲已包含 semantic label 與 `obstacle_probability`，因此 `label` 與 `probability` color mode 都可以直接使用。

## 8. Leaf Feature CSV 輸出參數

日常操作建議優先使用兩個語意明確的腳本：

- `scripts/run_feature_export_predict.sh`：輸出模型評估用 predict CSV，保留 Gazebo semantic label。
- `scripts/run_feature_export_rf.sh`：載入 RF 模型，輸出已推論的 CSV。

`scripts/run_feature_export.sh` 是共用底層入口，仍可用於訓練資料、弱標註或手動組合參數。常用環境變數如下：

- `GZ_PARTITION_VALUE`：Gazebo Transport partition。
- `GZ_POINTCLOUD_TOPIC`：點雲 topic。
- `FEATURE_OUTPUT`：CSV 輸出路徑。
- `FEATURE_MAX_DEPTH`：建構 Octree 使用的最大深度。
- `FEATURE_MAX_POINTS`：每次輸出最多使用多少收到的點。
- `FEATURE_EXPORT_HZ`：每秒最多重建 Octree 並輸出 CSV 幾次。
- `FEATURE_ONCE`：設為 `1` 時只輸出第一包點雲。
- `FEATURE_TIMESTAMPED`：設為 `1` 時每次輸出獨立 CSV。
- `FEATURE_TRAIN`：設為 `1` 時輸出 CSV 檔名自動加上 `train_` 前綴。
- `FEATURE_WEAK_LABELS`：設為 `1` 時用規則填入 `label` 與 `obstacle_probability`。
- `FEATURE_RF_MODEL`：指定 `models/random_forest_voxel_model.rf.txt` 時，exporter 端 Octree 會使用 RF 模型推論 leaf label / probability。
- `FEATURE_FLOOR_Z`：第 0 層樓的 z 原點。
- `FEATURE_STORY_HEIGHT`：樓層週期高度，預設 `4`。
- `FEATURE_FLOOR_SURFACE_OFFSET`：每層樓內可通行地板面的局部 z offset。
- `FEATURE_CEILING_OFFSET`：每層樓內天花板局部 z offset，腳本預設 `4`。
- `FEATURE_NEAR_FLOOR`：判斷接近地板的距離帶。
- `FEATURE_NEAR_CEILING`：判斷接近天花板的距離帶。

底層 `leaf_feature_exporter_gazebo` CLI 參數如下：

`--partition`

Gazebo Transport partition。必須和 Gazebo simulation 使用相同 partition。

`--topic`

訂閱的 `gz::msgs::PointCloudPacked` topic。預設是 `/world/dynamic_cloud`。

`--output`

CSV 輸出路徑。未使用 `--timestamped` 時會覆蓋同一個檔案。

`--max-depth`

Octree 最大深度。這會影響 leaf voxel 大小，也會影響輸出的訓練樣本數。

`--max-points`

每次輸出最多使用多少點。若 Gazebo 發布的點數更多，exporter 會等距抽樣。

`--export-hz`

控制 CSV 輸出頻率。建議先用 `1`，也就是每秒最多輸出一次。

`--weak-labels`

不用 Gazebo topic 內的 semantic 欄位，改用 exporter 端 rule-based weak labeling 覆寫 `label` 與 `obstacle_probability`。一般情況建議使用 Gazebo plugin 發出的 semantic labels；`--weak-labels` 適合舊點雲或沒有 semantic 欄位的資料。

`--floor-z`

第 0 層樓的 z 原點。預設是 `0`。

`--story-height`

樓層週期高度。若地板厚度 1m、牆高 3m，則 floor-to-floor 高度為 `4`。

`--floor-surface-offset`

每層樓內可通行地板面的局部 z offset。若你的座標系把每層地板面放在樓層起點，使用預設 `0`；若地板模型厚度 1m 且可通行面在樓層起點上方 1m，可設為 `1`。

`--ceiling-offset`

每層樓內天花板局部 z offset。若地板厚度 1m、牆高 3m，且你想用樓層週期頂端作為 ceiling reference，可使用腳本預設 `4`；若要表示單純室內淨高，則可改成 `3`。

`--near-floor`

判斷接近地板面的距離帶。預設 `0.4`。

`--near-ceiling`

判斷接近天花板的距離帶。預設 `0.4`。

`--ceiling-z`

舊參數，現在作為 `--ceiling-offset` 的 alias。

`--once`

輸出第一包收到的點雲後結束。

`--timestamped`

不覆蓋原 CSV，而是輸出：

```text
leaf_features_frame000001.csv
leaf_features_frame000002.csv
...
```

`--train`

在輸出 CSV 檔名前加上 `train_`，方便區分「訓練資料」與「待預測資料」。例如：

```text
data/leaf_features.csv -> data/train_leaf_features.csv
data/leaf_features_frame000001.csv -> data/train_leaf_features_frame000001.csv
```

如果原本檔名已經是 `train_` 開頭，exporter 不會重複加前綴。

## 9. Random Forest 模型訓練

訓練腳本位於：

```text
python/train_model.py
```

預設會讀取 `data/` 中所有檔名符合 `train_*.csv` 的訓練資料，並訓練兩個 Random Forest：

- `RandomForestClassifier`：預測 `label`，其中 `0=free`、`1=obstacle`、`2=stair`。
- `RandomForestRegressor`：預測 `obstacle_probability`。

建議使用專案 venv 執行：

```bash
venv/bin/python3 python/train_model.py
```

預設輸出：

```text
models/random_forest_voxel_model.pkl
models/random_forest_voxel_model.rf.txt
models/random_forest_voxel_report.json
```

其中 `.pkl` 保留給 Python 分析或後續實驗使用；`.rf.txt` 是 C++17 可直接讀取的輕量 Random Forest 模型格式，供 `OctreeManager::setMLPredictor()`、feature exporter 與 Octree viewer 使用。

同時，若 `data/` 中有非訓練資料，例如 `data/leaf_features.csv`，腳本會自動略過 `train_*.csv` 與 `predicted_*.csv`，將這些一般 feature CSV 進行預測，並輸出成：

```text
data/predicted_leaf_features.csv
```

若系統 Python 缺少套件，請安裝或改用已有 `scikit-learn` 的 venv：

```bash
python3 -m pip install numpy scikit-learn
```

訓練腳本會自動檢查資料是否足夠。若沒有找到 `train_*.csv`、缺少 `label` / `obstacle_probability` 欄位、只有單一 label class，或某些 class 樣本數過少，會停止並印出原因。

若想指定某一份 predict CSV 或指定輸出位置，也可以手動傳參數：

```bash
venv/bin/python3 python/train_model.py \
  --predict data/leaf_features.csv \
  --prediction-output data/predicted_leaf_features.csv
```

預測輸出會保留原 CSV 欄位，並額外加入：

- `predicted_label`
- `predicted_obstacle_probability`

若只想訓練模型、不自動預測任何 CSV，可以加上：

```bash
venv/bin/python3 python/train_model.py --no-auto-predict
```

### 將模型套回 Octree

`OctreeManager` 已提供 `setMLPredictor()`。目前專案內建的 C++ predictor 會讀取 `python/train_model.py` 產生的 `.rf.txt`，並在 Octree leaf 建構完成、avg normal / PCA / density 等特徵計算完成後，對每個 leaf 推論：

- `label`
- `obstacle_probability`

headless exporter 啟用方式：

```bash
./scripts/run_feature_export_rf.sh
```

這時輸出的 CSV 仍會保留同樣欄位，但 `label` 與 `obstacle_probability` 會是模型推論結果，而不是 Gazebo semantic 欄位的聚合結果。預設輸出檔案是：

```text
data/predicted_rf_leaf_features.csv
```

若要修改模型或輸出路徑：

```bash
RF_FEATURE_MODEL=models/random_forest_voxel_model.rf.txt \
RF_FEATURE_OUTPUT=data/predicted_custom_leaf_features.csv \
./scripts/run_feature_export_rf.sh
```

Octree 3D viewer 啟用方式：

```bash
OCTREE_RF_MODEL=models/random_forest_voxel_model.rf.txt ./scripts/run_visualization.sh octree
```

這時 viewer 的 probability / label 顏色會反映模型推論後的 leaf 狀態。

底層 binary 也可以直接指定：

```bash
./build/leaf_feature_exporter_gazebo --rf-model models/random_forest_voxel_model.rf.txt
./build/octree_viewer/visualize_octree_gazebo --rf-model models/random_forest_voxel_model.rf.txt
```

注意：`FEATURE_WEAK_LABELS=1` 是規則弱標註輸出模式，會在 export 時覆寫 label。若目標是看 RF 模型結果，請不要同時啟用 `FEATURE_WEAK_LABELS=1`。

### CSV 欄位重點

每一列代表一個 Octree leaf voxel。主要特徵包含：

- 追蹤定位：`node_index`、`entity_id`、`morton_code`、`min_x`、`min_y`、`min_z`、`max_x`、`max_y`、`max_z`
- 基本幾何：`num_points`、`voxel_volume`、`density`、`voxel_size`、`depth`
- 位置：`center_x`、`center_y`、`center_z`、`story_index`、`story_local_z`、`height_ratio`
- Normal：`avg_normal_x`、`avg_normal_y`、`avg_normal_z`
- 方向：`verticality`、`horizontality`、`slope_angle_rad`
- PCA：`eigenvalue_0`、`eigenvalue_1`、`eigenvalue_2`、`pca_linearity`、`pca_flatness`、`pca_roughness`、`pca_curvature`
- 標籤欄位：最後兩欄固定為 `label`、`obstacle_probability`

`node_index` 是同一次 Octree 建構中的節點索引，適合用來在當前 process 內回查 `octree.nodes()[node_index]`。`entity_id` 是 leaf 內 semantic points 投票最多的 Gazebo collision entity id。若 leaf 同時包含地板與障礙物邊界點，`obstacle_probability` 會接近這些 semantic points 的平均障礙概率，因此可近似反映 leaf 與障礙物的重疊程度。

如果跨 frame、改變 `max_depth`、改變點雲抽樣數，Octree 可能重新切割，`node_index` 不保證穩定。要做跨檔案追蹤時，請同時使用 `entity_id`、`morton_code`、`depth`、AABB bounds 與 center 來比對。

目前 Gazebo plugin 發出的 point cloud 已包含 rule-based semantic labels，因此 CSV 中的 `label` 與 `obstacle_probability` 預設會由 Gazebo collision source 聚合而來。這比只看 leaf 幾何特徵更適合產生 Random Forest 初版訓練資料。

如果想先建立可訓練的初版資料，可以啟用弱標註：

```bash
FEATURE_WEAK_LABELS=1 FEATURE_ONCE=1 ./scripts/run_feature_export.sh
```

弱標註會根據 leaf 高度、normal 方向、斜率與 PCA flatness 粗略填入：

- `0`：接近地板、接近平面且法向量朝上的 free voxel。
- `1`：高於地板、較像牆面或物體表面的 obstacle voxel。
- `2`：有一定高度、表面斜率落在樓梯範圍內的 stair voxel。

這些標籤適合拿來 bootstrap 或人工校正，不建議直接視為最終 ground truth。

## 10. 畫面顏色與意義

目前 `scripts/run_visualization.sh` 預設使用 probability color mode：

- 淡藍：`obstacle_probability` 接近 `0`。
- 黃色：中間風險。
- 亮紅：`obstacle_probability` 接近 `1`。
- 亮紫紅色：Stair 或 cross-floor，所有 color mode 都會優先高亮。

若切換成 depth color mode：

- 白色點：原始點雲，只有未使用 `--no-points` 時才顯示。
- 彩色點：Octree leaf voxel center。
- 彩色 wireframe box：leaf voxel 的體積邊框。
- 偏藍或青色：較淺層 depth，通常代表較大的 voxel。
- 偏綠色：中間 depth。
- 偏黃或紅色：較深層 depth，通常代表較小的 voxel。

PCL 的 point size 是螢幕像素大小，不是真實世界尺寸。因此要看 voxel 實際體積，請使用 `--voxel-mode boxes` 或 `--voxel-mode center-boxes`。

## 11. 效能調整建議

如果 Gazebo 或 viewer 很卡，依序調整：

1. 降低 viewer 的 `--max-voxels`。
2. 降低 viewer 的 `--max-render-points`。
3. 降低 viewer 的 `--rebuild-hz`。
4. 將 viewer 改成 `--voxel-mode centers`。
5. 在 SDF 中降低 `max_points_per_publish`。
6. 在 SDF 中增大 `point_spacing`。
7. 在 SDF 中降低 `update_rate`。

如果 feature exporter 很吃 CPU 或 CSV 太大，依序調整：

1. 降低 `FEATURE_MAX_POINTS`。
2. 降低 `FEATURE_EXPORT_HZ`。
3. 降低 `FEATURE_MAX_DEPTH`。
4. 在 SDF 中降低 `max_points_per_publish`。
5. 在 SDF 中增大 `point_spacing`。

範例，低負載展示：

```bash
OCTREE_VOXEL_MODE=centers \
OCTREE_MAX_VOXELS=3000 \
OCTREE_MAX_RENDER_POINTS=40000 \
OCTREE_REBUILD_HZ=0.5 \
./scripts/run_visualization.sh octree
```

## 12. 常見問題

### 看不到 `/world/dynamic_cloud`

確認 plugin 已建置：

```bash
cmake --build build --target dynamic_world_cloud
```

確認用 `scripts/run_gazebo.sh` 啟動，讓 plugin path 正確設定。

### Viewer 開了但一直 waiting

通常是 partition 不一致。請確認 Gazebo 與 viewer 使用相同 partition：

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -l
```

Viewer 啟動時也要指定：

```bash
--partition dynamic_cloud_test
```

### 只有白色點，沒有 voxel

確認沒有使用 `--no-voxels`，並降低 `--max-depth` 或 `--max-render-points` 測試。

### voxel 邊框太卡

改用：

```bash
--voxel-mode centers
```

或降低：

```bash
--max-voxels 1000
```

### 想只看 voxel，不看原始點雲

加入：

```bash
--no-points
```

推薦：

```bash
OCTREE_HIDE_POINTS=1 OCTREE_VOXEL_MODE=center-boxes OCTREE_MAX_VOXELS=2000 ./scripts/run_visualization.sh octree
```

### 沒有產生 `data/leaf_features.csv`

確認 exporter 已建置：

```bash
cmake --build build --target leaf_feature_exporter_gazebo
```

確認 Gazebo 正在發布 topic：

```bash
GZ_PARTITION=dynamic_cloud_test gz topic -l
```

確認 exporter 使用相同 partition：

```bash
GZ_PARTITION_VALUE=dynamic_cloud_test ./scripts/run_feature_export.sh
```
