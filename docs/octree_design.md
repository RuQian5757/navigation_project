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
- 若 `point_count < 5`，將不繼續分割，避免稀疏點導致過多葉節點。

### 多樓層與樓梯跨層連通性

- 每個 leaf voxel 支援 `is_cross_floor` 標記。
- 標記樓梯 voxel 為 `VoxelLabel::Stair`，並且可直接透過 `isCrossFloorConnected()` 判斷。
- 即使相鄰 voxel 深度不同，也會透過中心點查找取得上下層 leaf，處理深度不一致情況。

### 機器學習語義標籤

- 每個 leaf voxel 儲存：
  - `label`（0=可通行、1=障礙、2=樓梯）
  - `obstacle_probability`
  - `room_id`
  - `is_cross_floor`
- 透過 `assignLeafLabel()` 將 ML 標籤寫入 leaf。
- `floodFillLabelPropagation()` 會將標籤與障礙概率擴散到鄰近同房間節點。

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
- 26 鄰居支援可透過正交鄰居的 Backtracking 展開，進行更廣泛搜尋。
- 使用 Morton code（Z-order）加速定位，並將節點儲存在連續向量中。

### 稀疏點與長形結構處理

- 若 `num_points < 5`，則保留 leaf，避免因稀疏點產生過多無用子節點。
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

## 文件結構

- `include/octree_manager.h`：Octree API 與資料結構定義。
- `src/octree_manager.cpp`：Octree 核心實作。
- `src/main.cpp`：簡單執行範例。
- `tests/test_octree.cpp`：功能驗證測試。

## 建議後續擴充

1. 導入真實 Gazebo 點雲輸入。
2. 加入 ONNX Runtime 與 Random Forest 標籤推理。
3. 實作 3D A* / Hybrid A* 路徑規劃。
4. 增加跨深度 26 鄰居 Backtracking 搜尋。
5. 加入房間 ID 標記自動化與樓層區分。
