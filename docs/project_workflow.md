# 專案整體流程與操作文件

本文件整理目前專案的實際工作流程，讓後續開發者可以從文件理解：

- Gazebo plugin 如何產生點雲
- 點雲資料如何透過 Gazebo Transport 傳遞
- Octree 如何接收並切割資料
- 如何執行即時 3D Octree 視覺化
- 常用參數要如何調整

目前主要資料流如下：

```text
warehouse_world.sdf
  -> DynamicWorldCloud Gazebo system plugin
  -> /world/dynamic_cloud, gz::msgs::PointCloudPacked
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
- 將 local cloud cache 起來，避免每個 simulation tick 重新取樣。
- 每次發布時只根據目前 entity pose 把 local points 轉成 world coordinates。
- 將全世界點雲發布成 `gz::msgs::PointCloudPacked`。
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

### 即時 Octree viewer

位置：

- `scripts/visualize_octree_gazebo.cpp`
- `scripts/run_visualization.sh`
- `scripts/CMakeLists.txt`

功能：

- 直接訂閱 Gazebo Transport topic `/world/dynamic_cloud`。
- 解析 `gz::msgs::PointCloudPacked` 的 `xyz` float32 packed data。
- 依照設定節流解析頻率，避免 viewer 拖慢 Gazebo。
- 使用收到的點雲建立 Octree。
- 用 PCLVisualizer 顯示 Octree leaf voxel。
- 可選擇顯示原始點雲、voxel center、voxel wireframe box。

### 共用視覺化啟動腳本

位置：

- `scripts/run_visualization.sh`

功能：

- 用同一個入口啟動 Octree 3D 顯示或 pointcloud 3D 顯示。
- 腳本上方集中放置常用參數，並用註解說明每個參數用途。
- 支援用環境變數臨時覆寫參數，不需要直接修改 C++ 或 Python 程式。

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

限制每次 topic 發布的最大點數。若完整 cloud 大於此數量，plugin 會等距抽樣後發布。

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
- Mesh：讀 mesh vertices。
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
point_step: 12 bytes
每個點: float x, float y, float z
frame: "world"
```

資料傳遞方式：

```text
DynamicWorldCloud::PublishPointCloud()
  -> BuildPointCloudMessage()
  -> cloud_pub_.Publish(msg)
  -> /world/dynamic_cloud
  -> visualize_octree_gazebo.cpp callback
  -> parsePointCloudPacked()
  -> std::vector<navigation::Point3D>
  -> OctreeManager::initialize(points)
```

Viewer 會根據 `--max-render-points` 對收到的點雲再做一次抽樣，避免大量點雲造成 Octree 重建與 PCL rendering 過慢。

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
```

## 6. 執行流程

### Terminal 1：啟動 Gazebo simulation

```bash
./run_gazebo.sh gazebo/maps/warehouse_world.sdf -s -r -v 2
```

`run_gazebo.sh` 會自動設定：

- `LD_LIBRARY_PATH`
- `GZ_PLUGIN_PATH`
- `GZ_SIM_SYSTEM_PLUGIN_PATH`
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

### Terminal 3：啟動即時 Octree 視覺化

推薦展示模式：

```bash
./scripts/run_visualization.sh octree
```

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

### Terminal 3 可替代方案：只顯示原始 pointcloud

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
- `POINTCLOUD_POINT_SIZE`：Python pointcloud viewer 點大小。
- `POINTCLOUD_MAX_RENDER_POINTS`：Python pointcloud viewer 最多顯示點數。

底層 `visualize_octree_gazebo` 仍支援下列 CLI 參數，方便需要直接呼叫 binary 時使用。

`--partition`

Gazebo Transport partition。必須和 Gazebo simulation 使用相同 partition。`run_gazebo.sh` 預設是 `dynamic_cloud_test`。

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

改用語義顏色，而不是 depth 顏色：

- 綠色：Free
- 紅色：Obstacle
- 藍色：Stair 或 cross-floor

目前 Gazebo plugin 發出的點雲沒有 ML label，因此一般展示 Octree 切割時不建議使用此選項。

## 8. 畫面顏色與意義

在預設 depth color 模式下：

- 白色點：原始點雲，只有未使用 `--no-points` 時才顯示。
- 彩色點：Octree leaf voxel center。
- 彩色 wireframe box：leaf voxel 的體積邊框。
- 偏藍或青色：較淺層 depth，通常代表較大的 voxel。
- 偏綠色：中間 depth。
- 偏黃或紅色：較深層 depth，通常代表較小的 voxel。

PCL 的 point size 是螢幕像素大小，不是真實世界尺寸。因此要看 voxel 實際體積，請使用 `--voxel-mode boxes` 或 `--voxel-mode center-boxes`。

## 9. 效能調整建議

如果 Gazebo 或 viewer 很卡，依序調整：

1. 降低 viewer 的 `--max-voxels`。
2. 降低 viewer 的 `--max-render-points`。
3. 降低 viewer 的 `--rebuild-hz`。
4. 將 viewer 改成 `--voxel-mode centers`。
5. 在 SDF 中降低 `max_points_per_publish`。
6. 在 SDF 中增大 `point_spacing`。
7. 在 SDF 中降低 `update_rate`。

範例，低負載展示：

```bash
OCTREE_VOXEL_MODE=centers \
OCTREE_MAX_VOXELS=3000 \
OCTREE_MAX_RENDER_POINTS=40000 \
OCTREE_REBUILD_HZ=0.5 \
./scripts/run_visualization.sh octree
```

## 10. 常見問題

### 看不到 `/world/dynamic_cloud`

確認 plugin 已建置：

```bash
cmake --build build --target dynamic_world_cloud
```

確認用 `run_gazebo.sh` 啟動，讓 plugin path 正確設定。

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
