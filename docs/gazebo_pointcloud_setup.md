# Gazebo Point Cloud 取得指南

## 修改內容

已在 `gazebo/maps/warehouse_world.sdf` 加入高解析度深度相機 sensor，可產生 **~1,048,576 點** 的點雲（1024×1024 解析度）。

### 主要參數

- **解析度**：1024×1024 像素
- **視野 (FoV)**：1.047 radians (60°) 水平
- **深度範圍**：0.1m ~ 25m
- **更新頻率**：10 Hz
- **位置**：(10, 10, 2.5) 且傾斜 -0.3 rad (pitch) 與 -0.785 rad (yaw) 以看向倉庫中心

---

## 方法 1：純 Gazebo 方式（最簡單）

### 啟動指令

```bash
# 使用新版 Gazebo (Fortress/Garden)
gz sim warehouse_world.sdf
```

### 確認 Gazebo UI

1. Gazebo 視窗開啟後，應該能看到倉庫場景
2. 在 3D 視圖中應該會看到 `depth_camera_frame` 模型（帶相機連結）
3. 點選相機模型，右邊 Inspector 應該會顯示 sensor 資訊

### 取得點雲資料

#### 方式 A：從 Gazebo Transport Topic 讀取

```bash
# 查看所有 Gazebo topic
gz topic -l | grep points

# 監控深度相機資料
gz topic -e -t "/world/warehouse_world/model/depth_camera_frame/link/camera_link/sensor/depth_camera/depth" | head -20
```

#### 方式 B：使用 Python 訂閱

```python
#!/usr/bin/env python3
import sys
import gz.transport13 as transport

def callback(msg):
    print(f"Received point cloud with {msg.count} points")

# 建立訂閱
topic = "/world/warehouse_world/model/depth_camera_frame/link/camera_link/sensor/depth_camera/depth"
node = transport.Node()
node.Subscribe(topic, callback)

# 持續監聽
input("Press Enter to exit...\n")
```

---

## 方法 2：新版 Gazebo + ROS2（建議如果要用 ROS）

### 前置要求

```bash
# 安裝 ROS2 Gazebo 橋接（以 Humble 為例）
sudo apt install ros-humble-gz-ros2-control
sudo apt install ros-humble-ros2-gz
```

### 啟動

```bash
# 終端 1：啟動 Gazebo
gz sim warehouse_world.sdf

# 終端 2：啟動 ROS2 橋接
ros2 run ros2_gz /world/warehouse_world/model/depth_camera_frame/link/camera_link/sensor/depth_camera/depth

# 終端 3：檢查 ROS topic
ros2 topic list | grep depth
ros2 topic echo /robot/camera/depth/points
```

---

## 方法 3：Gazebo Classic + ROS1（舊版本）

如果你要用 ROS1，需要使用舊版 Gazebo：

### 安裝

```bash
sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros
```

### 編輯 SDF 使用 ROS plugin

需要恢復之前的 plugin 定義，修改 warehouse_world.sdf 中的 `<plugin>` 部分。

### 啟動

```bash
roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/gazebo/maps/warehouse_world.sdf
```

---

## 在 C++ 中讀取點雲

### 方法 A：Gazebo 原生 Transport（推薦）

```cpp
#include <gz/transport.hh>
#include <gz/msgs.hh>
#include "octree_manager.h"

class PointCloudSubscriber {
private:
    navigation::OctreeManager* octree_;

public:
    PointCloudSubscriber(navigation::OctreeManager* octree) 
        : octree_(octree) {
        gz::transport::Node node;
        std::string topic = "/world/warehouse_world/model/depth_camera_frame/link/camera_link/sensor/depth_camera/depth";
        
        node.Subscribe(topic, &PointCloudSubscriber::onPointCloud, this);
        std::cout << "Subscribed to: " << topic << std::endl;
    }

    void onPointCloud(const gz::msgs::PointCloudPacked& msg) {
        std::vector<navigation::Point3D> points;
        
        // 解析 PointCloudPacked 資訊
        for (int i = 0; i < msg.data().size(); i += 12) {  // 每點 12 bytes (x,y,z as float)
            float x, y, z;
            std::memcpy(&x, msg.data().data() + i, 4);
            std::memcpy(&y, msg.data().data() + i + 4, 4);
            std::memcpy(&z, msg.data().data() + i + 8, 4);
            points.push_back({x, y, z});
        }

        // 更新 Octree
        octree_->updateFromPointCloud(points);
    }
};
```

### 方法 B：PCL + ROS2（如果用 ROS2 橋接）

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "octree_manager.h"

class PointCloudSubscriber : public rclcpp::Node {
private:
    navigation::OctreeManager* octree_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;

public:
    PointCloudSubscriber(navigation::OctreeManager* octree) 
        : Node("pointcloud_subscriber"), octree_(octree) {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/points", 
            10,
            std::bind(&PointCloudSubscriber::onPointCloud, this, std::placeholders::_1)
        );
    }

    void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        std::vector<navigation::Point3D> points;
        points.reserve(cloud.size());
        for (const auto& pt : cloud.points) {
            points.push_back({pt.x, pt.y, pt.z});
        }

        octree_->updateFromPointCloud(points);
    }
};
```

---

## 現在的設定對應

SDF 中的深度相機設定：

```xml
<sensor name="depth_camera" type="depth">
  <camera>
    <horizontal_fov>1.047</horizontal_fov>
    <image>
      <width>1024</width>
      <height>1024</height>
    </image>
    <clip>
      <near>0.1</near>
      <far>25</far>
    </clip>
  </camera>
  <visualize>true</visualize>
  <always_on>true</always_on>
  <update_rate>10</update_rate>
</sensor>
```

- **visualize**：在 Gazebo UI 中顯示感測器視景
- **always_on**：始終更新（不暫停）
- **update_rate**：10 Hz 更新頻率

---

## 調整參數

### 增加點數

改為 2048×2048：
```xml
<image>
  <width>2048</width>
  <height>2048</height>
</image>
```

### 降低計算負擔

改為 512×512：
```xml
<image>
  <width>512</width>
  <height>512</height>
</image>
```

### 改變感測器位置

編輯 `<pose>`：
```xml
<pose>10 10 2.5 -0.3 0 -0.785</pose>
<!-- X   Y  Z  Roll Pitch Yaw -->
```

### 改變深度範圍

```xml
<clip>
  <near>0.05</near>  <!-- 最近距離 -->
  <far>50</far>      <!-- 最遠距離 -->
</clip>
```

---

## 常見問題

### 1. 無法找到 warehouse_world.sdf

**解決**：
```bash
cd ~/project/navigation_project
gz sim gazebo/maps/warehouse_world.sdf
```

### 2. 無法讀取深度相機資料

**檢查**：
```bash
gz topic -l | grep depth_camera
```

如果沒有看到 topic，可能是：
- Gazebo 版本過舊（需要 Fortress 或更新）
- SDF 檔案語法錯誤

### 3. 點雲資料很稀疏或發黑

**原因**：相機位置對不到物體

**解決**：
- 調整 `<pose>` 確保相機指向倉庫中心
- 增加 `<clip><far>` 距離
- 在 Gazebo UI 檢查相機可視化

### 4. Gazebo 運行緩慢

**原因**：1024×1024 解析度較高

**解決**：
- 降低到 512×512 或 640×480
- 降低 update_rate 到 5 或 1 Hz
- 用無頭模式：`gz sim -s warehouse_world.sdf`

---

## 下一步

1. **測試點雲獲取**：使用上面任一方法確認能取到點雲
2. **編寫 C++ bridge**：將 Gazebo sensor 資料轉為 `Point3D` 向量
3. **整合 Octree**：將點雲餵入 `OctreeManager`
4. **ML 訓練**：準備點雲標籤資料集
5. **路徑規劃**：實裝 3D A* planner

---

## 參考資源

- [Gazebo Sim 文件](https://gazebosim.org/docs)
- [ROS2 + Gazebo 整合](https://github.com/gazebosim/ros2_gz)
- [PCL 教程](http://pointclouds.org/)
