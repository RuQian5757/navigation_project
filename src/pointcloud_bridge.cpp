#include <gz/transport.hh>
#include <gz/msgs.hh>
#include "octree_manager.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <cmath>

class PointCloudBridge {
private:
    navigation::OctreeManager octree_;
    int frame_count_ = 0;
    int total_points_ = 0;

public:
    PointCloudBridge() {
        // 初始化 Octree：先建立空的根節點
        std::vector<navigation::Point3D> init_points = {
            {10.0f, 10.0f, 2.5f}  // 深度相機中心位置
        };
        octree_.initialize(init_points);
        std::cout << "✓ Octree 初始化完成" << std::endl;
    }

    void onPointCloud(const gz::msgs::PointCloudPacked& msg) {
        frame_count_++;
        std::vector<navigation::Point3D> points;
        
        // Debug: 查看訊息結構
        if (frame_count_ == 1) {
            std::cout << "\n[DEBUG] PointCloudPacked 結構:" << std::endl;
            std::cout << "  - Data size: " << msg.data().size() << " bytes" << std::endl;
            std::cout << "  - Width: " << msg.width() << std::endl;
            std::cout << "  - Height: " << msg.height() << std::endl;
            std::cout << "  - Point step: " << msg.point_step() << std::endl;
            std::cout << "  - Row step: " << msg.row_step() << std::endl;
            std::cout << "  - Fields count: " << msg.field_size() << std::endl;
            for (int i = 0; i < msg.field_size(); ++i) {
                std::cout << "    Field " << i << ": " << msg.field(i).name() 
                         << " offset=" << msg.field(i).offset() 
                         << " datatype=" << msg.field(i).datatype() << std::endl;
            }
        }
        
        // 使用 width * height 計算點數
        size_t num_points = msg.width() * msg.height();
        if (num_points == 0 || msg.point_step() == 0) {
            if (frame_count_ <= 3) {
                std::cout << "[Frame " << frame_count_ << "] 空或無效點雲，跳過 (width=" 
                         << msg.width() << ", height=" << msg.height() 
                         << ", point_step=" << msg.point_step() << ")" << std::endl;
            }
            return;
        }
        
        points.reserve(num_points);
        
        // 解析點雲資料 - 根據實際 point_step 和欄位位置
        int x_offset = 0, y_offset = 4, z_offset = 8;
        
        // 尋找欄位位置
        for (int i = 0; i < msg.field_size(); ++i) {
            if (msg.field(i).name() == "x") x_offset = msg.field(i).offset();
            if (msg.field(i).name() == "y") y_offset = msg.field(i).offset();
            if (msg.field(i).name() == "z") z_offset = msg.field(i).offset();
        }
        
        for (size_t i = 0; i < num_points; ++i) {
            float x = 0, y = 0, z = 0;
            const char* data = msg.data().data();
            size_t offset = i * msg.point_step();
            
            std::memcpy(&x, data + offset + x_offset, sizeof(float));
            std::memcpy(&y, data + offset + y_offset, sizeof(float));
            std::memcpy(&z, data + offset + z_offset, sizeof(float));
            
            // 篩選有效點（不是 NaN 或無限遠）
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                points.push_back({x, y, z});
            }
        }

        if (points.empty()) {
            std::cout << "[Frame " << frame_count_ << "] 解析後無有效點" << std::endl;
            return;
        }

        // Debug: 顯示點的座標範圍
        if (frame_count_ == 1) {
            float min_x = points[0].x, max_x = points[0].x;
            float min_y = points[0].y, max_y = points[0].y;
            float min_z = points[0].z, max_z = points[0].z;
            
            for (const auto& p : points) {
                min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
                min_y = std::min(min_y, p.y); max_y = std::max(max_y, p.y);
                min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
            }
            
            std::cout << "\n[DEBUG] 點雲座標範圍:" << std::endl;
            std::cout << "  X: [" << min_x << ", " << max_x << "]" << std::endl;
            std::cout << "  Y: [" << min_y << ", " << max_y << "]" << std::endl;
            std::cout << "  Z: [" << min_z << ", " << max_z << "]" << std::endl;
        }

        // 限制單幀點數以避免卡住（採樣）
        const size_t MAX_POINTS_PER_FRAME = 500000;  // 增加到 50 萬點
        if (points.size() > MAX_POINTS_PER_FRAME) {
            std::cout << "[Frame " << frame_count_ << "] 點數過多 (" << points.size() 
                     << ")，進行採樣到 " << MAX_POINTS_PER_FRAME << std::endl;
            std::vector<navigation::Point3D> sampled;
            sampled.reserve(MAX_POINTS_PER_FRAME);
            float step = (float)points.size() / MAX_POINTS_PER_FRAME;
            for (size_t i = 0; i < MAX_POINTS_PER_FRAME; ++i) {
                sampled.push_back(points[(size_t)(i * step)]);
            }
            points = sampled;
        }

        // 更新 Octree
        octree_.updateFromPointCloud(points);
        total_points_ += points.size();
        
        std::cout << "[Frame " << frame_count_ << "] 已處理 " << points.size() 
                  << " 個點 | Octree 節點數: " << octree_.getNodeCount() 
                  << " | 葉節點: " << octree_.getLeafCount()
                  << " | 累計點數: " << total_points_ << std::endl;
        
        // 每 10 幀顯示一次統計
        if (frame_count_ % 10 == 0) {
            std::cout << "━━━ 第 " << frame_count_ << " 幀統計 ━━━" << std::endl;
        }
    }

    void run() {
        gz::transport::Node node;
        std::string topic = "/world/warehouse_world/model/depth_camera_frame/link/camera_link/sensor/depth_camera/depth_image/points";
        
        // 訂閱點雲 topic
        if (!node.Subscribe(topic, &PointCloudBridge::onPointCloud, this)) {
            std::cerr << "✗ 無法訂閱 topic: " << topic << std::endl;
            return;
        }
        
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "✓ 已訂閱: " << topic << std::endl;
        std::cout << "✓ 等待點雲資料... (按 Ctrl+C 停止)" << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        
        // 持續執行
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main(int argc, char* argv[]) {
    try {
        PointCloudBridge bridge;
        bridge.run();
    } catch (const std::exception& e) {
        std::cerr << "✗ 錯誤: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
