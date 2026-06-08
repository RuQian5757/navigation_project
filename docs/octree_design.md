# Octree 導航結構設計說明

此專案使用一個專為室內多樓層機器人導航設計的混合 Octree 結構，結合以下功能：

- 自適應體素分割
- 多樓層與樓梯跨層連通
- 機器學習語義標籤
- 動態更新與增量處理
- 房間封閉性約束
- 高效鄰居查找
- 稀疏點與長形結構處理
- Linear Octree 優化
- 與路徑規劃的深度整合

## 核心設計

### 自適應體素分割

- 使用 `buildLinearOctree()` 根據點雲密度與體素大小進行遞迴分割。
- 低密度區域（走廊）保留較大 voxel：0.2m ~ 0.5m。
- 高密度區域（樓梯、牆、角落）採用較小 voxel：0.05m ~ 0.1m。
- 分割決策依據：
  - `point_count`
  - `voxel_size`
  - `density`
  - `depth`
  - `max_depth`
- 目前 `OctreeConfig::min_points_to_split` 預設為 6。點數低於此值時不繼續分割，避免稀疏點導致過多葉節點。

### 多樓層與樓梯跨層連通性

- 每個 leaf voxel 支援 `is_cross_floor` 標記。
- 標記樓梯 voxel 為 `VoxelLabel::Stair`，並且可直接透過 `isCrossFloorConnected()` 判斷。
- 即使相鄰 voxel 深度不同，也會沿著 voxel face 取多個 probe point，再從 root 回查 containing leaf，處理深度不一致情況。

### 機器學習語義標籤

- 每個 leaf voxel 儲存：
  - `label`（0=可通行、1=障礙、2=樓梯）
  - `obstacle_probability`
  - `dominant_entity_id`（leaf 內 semantic points 主要來源的 Gazebo collision entity）
  - `room_id`
  - `is_cross_floor`
- 透過 `assignLeafLabel()` 將 ML 標籤寫入 leaf。
- `floodFillLabelPropagation()` 會將標籤與障礙概率擴散到鄰近同房間節點。

### Leaf 幾何統計與 PCA 特徵

Octree 建構時，每個 leaf 會保留該 leaf 內部點集合的幾何統計，供 Random Forest 或其他 ML 模型使用。

計算流程：

1. 對 leaf 內所有點計算 centroid。
2. 以 centroid 為中心建立 3x3 covariance matrix。
3. 對 covariance matrix 做 Jacobi eigen decomposition。
4. 將 eigenvalue 由小到大排序為 `lambda0 <= lambda1 <= lambda2`。
5. 最小 eigenvalue 對應的 eigenvector 作為 `avg_normal`，並統一讓 normal 的 z 分量朝上。

衍生特徵：

- `avg_normal`：leaf 內點雲估計出的主要表面法向量。
- `covariance_eigenvalues`：點雲在三個主方向上的分散程度。
- `linearity = (lambda2 - lambda1) / lambda2`：越接近 1 表示點更像線狀結構。
- `flatness = (lambda1 - lambda0) / lambda2`：越接近 1 表示點更像平面，例如地板或牆。
- `roughness = lambda0 / lambda2`：越大表示厚度或雜訊較高。
- `curvature = lambda0 / (lambda0 + lambda1 + lambda2)`：局部曲率/粗糙度近似。

因此 RF 不再只看到 voxel 的位置與密度，也能看到 voxel 內部點分布是平面、線狀、雜亂障礙，或可能的樓梯斜面。

這些特徵會被 `src/leaf_feature_exporter.cpp` 輸出到 CSV。Gazebo simulation 執行時，`src/gazebo_leaf_feature_exporter.cpp` 會直接訂閱 `/world/dynamic_cloud`，由 exporter 建立當前 frame 的 Octree 並輸出 CSV。日常腳本分成三種用途：`run_feature_export.sh` 產生訓練/弱標註或自訂輸出，`run_feature_export_predict.sh` 產生模型評估用 reference CSV，`run_feature_export_rf.sh` 產生 RF predicted CSV。Octree 3D viewer 則直接訂閱同一個 topic 即時顯示；`run_visualization_rf.sh` 不讀 CSV，而是在 viewer 內建構 Octree 後套用 RF model。

### 動態更新與增量處理

- `updateFromPointCloud()` 支援每 0.1 秒動態點雲更新。
- 只更新受影響的 leaf node，並重新計算鄰居連結與標籤傳播。
- 透過 `point_count` 判斷是否需要進一步分割或標記。

### 房間封閉性約束

- 每個 voxel 儲存 `room_id`。
- 鄰居搜尋時若 `room_id` 不同，視為不可通行。
- 這可避免規劃時穿牆或越過不同房間的錯誤連通。

### 高效鄰居查找

- 建立 6 個正交鄰居索引：`POS_X`、`NEG_X`、`POS_Y`、`NEG_Y`、`POS_Z`、`NEG_Z`。
- 鄰居查找會沿 voxel face 建立多個 probe point，再從 root 回查 containing leaf，因此可以處理相鄰 leaf 深度不一致的情況。
- 目前尚未提供完整 26-neighbor API；若 planner 需要斜向鄰居，可在 6 方向鄰居基礎上擴充。
- Morton code（Z-order）目前用於 leaf identity / CSV 對應與 linear octree 編碼；實際包含點查找仍以 root traversal 為主。

### 稀疏點與長形結構處理

- 若點數低於 `min_points_to_split`，則保留 leaf，避免因稀疏點產生過多無用子節點。
- 對走廊等長形低密度區域採用更大 voxel，以節省計算與記憶體。

### Linear Octree 特性

- 節點資料儲存在 `std::vector<OctreeNode>` 中，降低動態指標成本。
- 每個節點仍保留 8 個 child 索引與 6 個 orthogonal neighbor 索引。
- Morton code 用於節點分辨與快速定位。

### 與路徑規劃整合

- 每個 voxel 提供路徑成本資訊：
  - `label`
  - `obstacle_probability`
  - `is_cross_floor`
  - `depth`
- `computeTraversalCost()` 提供 加權成本，適合 A* 或 Hybrid A*。
- 障礙節點成本極高，樓梯與跨層節點成本也能調整。
- `AStarPlanner` 會使用 6 方向 leaf neighbor graph 搜尋路徑，並輸出 `node_indices` 與 `PathWaypoint` polyline。
- A* 的硬限制來自 `computeTraversalInfo()`：Obstacle、過高 obstacle probability、不同房間且非樓梯跨層都不可通行。
- A* 的 edge cost 會再加入 probability、stair、cross-floor、vertical movement 權重，讓 RF predicted 風險直接影響路徑。
- Heuristic 使用 Euclidean distance，並乘上目前 leaf 的 `obstacle_probability` 風險倍率；若跨層但目前不在 stair/cross-floor leaf，也會加入額外 bias，引導搜尋更快靠近樓梯連通區。

### A* 樓梯與樓板約束

目前 A* 不只依賴 label 成本，也加入數個硬限制，避免跨樓層展示時出現「穿地板」或「樓梯走到一半就離開」：

- `max_non_stair_vertical_step`：兩個非樓梯 leaf 之間若 Z 高度差超過此值，edge 直接不可通行。這用來防止 free voxel 直接垂直跳到另一層樓。
- `constrain_stair_transitions`：free <-> stair 轉換只能發生在 stair component 的低端或高端，避免從樓梯側邊或中段進出。
- `constrain_stair_direction`：free <-> stair 的移動方向必須符合樓梯推估出的 ascent direction，避免從側面爬上樓梯。
- `constrain_stair_exits_to_floor_levels`：樓梯進出點必須靠近已知樓層表面高度。

樓層表面高度計算方式：

```text
floor_surface_z = floor_z + floor_surface_offset + n * story_height
```

以目前 predict world 為例：

```text
floor_z = 0
story_height = 4
floor_surface_offset = 1
合法樓層表面約為 z = 1, 5, 9, ...
```

因此即使 RF 將樓梯切成多個小 stair component，中間 component 的局部 min/max 也不會被誤當成真正樓層出口。只有當 stair leaf 與 free leaf 都接近已知樓層高度時，才允許離開或進入樓梯。

### Path smoothing

`AStarPlanner::smoothWaypoints()` 會移除同方向冗餘 waypoint，但目前保留以下關鍵點：

- label 改變處，例如 free -> stair。
- `is_cross_floor` 狀態改變處。
- `room_id` 改變處。
- 明顯 Z 高度變化處。
- 任何包含 stair / cross-floor 的三點片段。

這是為了避免 A* 實際節點合法，但顯示 polyline 被過度簡化成一條穿過樓板或牆面的長直線。

## 文件結構

- `include/octree_manager.h`：Octree API 與資料結構定義。
- `src/octree_manager.cpp`：Octree 核心實作。
- `src/main.cpp`：簡單執行範例。
- `src/leaf_feature_exporter.cpp`：Leaf 特徵 CSV 輸出。
- `src/gazebo_leaf_feature_exporter.cpp`：訂閱 Gazebo topic 並輸出 leaf feature CSV。
- `src/gazebo_rf_path_planner.cpp`：訂閱 Gazebo topic、建立 RF Octree、執行 A* 並發布 Gazebo marker。
- `include/random_forest_voxel_predictor.h`：C++ Random Forest leaf 推論 API。
- `src/random_forest_voxel_predictor.cpp`：讀取 `.rf.txt` 並推論 `label` / `obstacle_probability`。
- `include/astar_planner.h`：A* planner API 與 path waypoint 格式。
- `src/astar_planner.cpp`：使用 Octree neighbor graph、label、probability 與 cross-floor 資訊搜尋路徑。
- `python/train_model.py`：由 `data/train_*.csv` 訓練 Random Forest，並輸出 Python `.pkl` 與 C++ `.rf.txt`。
- `tests/test_octree.cpp`：功能驗證測試。
- `scripts/visualize_octree_gazebo.cpp`：訂閱 Gazebo `/world/dynamic_cloud` 的即時 Octree viewer。
- `scripts/run_feature_export.sh`：Gazebo leaf feature CSV 共用底層腳本。
- `scripts/run_feature_export_predict.sh`：輸出模型評估用 predict feature CSV。
- `scripts/run_feature_export_rf.sh`：載入 RF 模型並輸出已推論 feature CSV。
- `scripts/run_path_planning.sh`：外部 RF Octree + A* debug planner；展示主流程使用 `RFOctreePathPlanner` plugin。
- `scripts/run_predict_with_path.sh`：一次啟動 predict world 與 RF A* 路徑規劃。
- `docs/project_workflow.md`：目前專案完整操作流程。

## 建議後續擴充

1. 增加 Gazebo start / goal marker model 讀取模式，讓展示時可直接拖曳起終點。
2. 增加模型版本資訊與 feature schema 驗證，避免訓練與 C++ 推論欄位不一致。
3. 實作 Hybrid A* path planner。
4. 增加跨深度 26 鄰居搜尋。
5. 加入房間 ID 標記自動化與樓層區分。
