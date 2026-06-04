#include <gz/transport.hh>
#include <gz/msgs.hh>
#include "octree_manager.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <mutex>

class PointCloudBridge {
private:
    navigation::OctreeManager octree_;
    int frame_count_ = 0;
    int total_points_ = 0;

    int server_fd_ = -1;
    int client_fd_ = -1;
    std::mutex socket_mutex_;
    std::thread server_thread_;
    bool stop_server_ = false;

public:
    PointCloudBridge() {
        std::vector<navigation::Point3D> init_points = {{10.0f, 10.0f, 2.5f}};
        octree_.initialize(init_points);
        std::cout << "✓ Octree 初始化完成" << std::endl;
        server_thread_ = std::thread([this]() { this->serverLoop(); });
    }

    ~PointCloudBridge() {
        stop_server_ = true;
        if (server_thread_.joinable()) {
            shutdown(server_fd_, SHUT_RDWR);
            close(server_fd_);
            server_thread_.join();
        }
        std::lock_guard<std::mutex> lk(socket_mutex_);
        if (client_fd_ != -1) close(client_fd_);
    }

    void serverLoop() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ < 0) {
            std::cerr << "✗ 無法建立 socket" << std::endl;
            return;
        }
        int opt = 1;
        setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(9000);
        if (bind(server_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "✗ 無法 bind localhost:9000" << std::endl;
            return;
        }
        if (listen(server_fd_, 1) < 0) {
            std::cerr << "✗ listen 失敗" << std::endl;
            return;
        }
        std::cout << "✓ TCP pointcloud bridge listening on 127.0.0.1:9000" << std::endl;

        while (!stop_server_) {
            struct sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int fd = accept(server_fd_, (struct sockaddr*)&client_addr, &client_len);
            if (fd < 0) {
                if (stop_server_) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            {
                std::lock_guard<std::mutex> lk(socket_mutex_);
                if (client_fd_ != -1) close(client_fd_);
                client_fd_ = fd;
            }
            std::cout << "✓ Python client connected to pointcloud stream" << std::endl;
            while (!stop_server_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                std::lock_guard<std::mutex> lk(socket_mutex_);
                if (client_fd_ == -1) break;
            }
            std::lock_guard<std::mutex> lk(socket_mutex_);
            if (client_fd_ != -1) {
                close(client_fd_);
                client_fd_ = -1;
            }
        }
    }

    void sendPointsToClient(const std::vector<navigation::Point3D>& points) {
        std::lock_guard<std::mutex> lk(socket_mutex_);
        if (client_fd_ == -1) return;
        uint32_t count = static_cast<uint32_t>(points.size());
        uint32_t count_net = htonl(count);
        if (send(client_fd_, &count_net, sizeof(count_net), MSG_NOSIGNAL) != sizeof(count_net)) {
            close(client_fd_);
            client_fd_ = -1;
            return;
        }
        std::vector<float> buffer;
        buffer.reserve(count * 3);
        for (const auto& p : points) {
            buffer.push_back(p.x);
            buffer.push_back(p.y);
            buffer.push_back(p.z);
        }
        ssize_t bytes = send(client_fd_, reinterpret_cast<const char*>(buffer.data()), buffer.size() * sizeof(float), MSG_NOSIGNAL);
        if (bytes < 0) {
            close(client_fd_);
            client_fd_ = -1;
        }
    }

    void onPointCloud(const gz::msgs::PointCloudPacked& msg) {
        frame_count_++;
        std::vector<navigation::Point3D> points;
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
        int x_offset = 0, y_offset = 4, z_offset = 8;
        for (int i = 0; i < msg.field_size(); ++i) {
            const auto& field = msg.field(i);
            if (field.name() == "x") x_offset = field.offset();
            if (field.name() == "y") y_offset = field.offset();
            if (field.name() == "z") z_offset = field.offset();
        }
        const char* data = msg.data().data();
        for (size_t i = 0; i < num_points; ++i) {
            float x = 0, y = 0, z = 0;
            size_t offset = i * msg.point_step();
            std::memcpy(&x, data + offset + x_offset, sizeof(float));
            std::memcpy(&y, data + offset + y_offset, sizeof(float));
            std::memcpy(&z, data + offset + z_offset, sizeof(float));
            if (std::isfinite(x) && std::isfinite(y) && std::isfinite(z)) {
                points.push_back({x, y, z});
            }
        }
        if (points.empty()) {
            std::cout << "[Frame " << frame_count_ << "] 解析後無有效點" << std::endl;
            return;
        }
        const size_t MAX_POINTS_PER_FRAME = 200000;
        if (points.size() > MAX_POINTS_PER_FRAME) {
            std::cout << "[Frame " << frame_count_ << "] 點數過多 (" << points.size()
                      << ")，採樣到 " << MAX_POINTS_PER_FRAME << std::endl;
            std::vector<navigation::Point3D> sampled;
            sampled.reserve(MAX_POINTS_PER_FRAME);
            float step = static_cast<float>(points.size()) / MAX_POINTS_PER_FRAME;
            for (size_t i = 0; i < MAX_POINTS_PER_FRAME; ++i) {
                sampled.push_back(points[static_cast<size_t>(i * step)]);
            }
            points.swap(sampled);
        }
        sendPointsToClient(points);
        octree_.updateFromPointCloud(points);
        total_points_ += points.size();
        std::cout << "[Frame " << frame_count_ << "] 已處理 " << points.size()
                  << " 個點 | Octree 節點數: " << octree_.getNodeCount()
                  << " | 葉節點: " << octree_.getLeafCount()
                  << " | 累計點數: " << total_points_ << std::endl;
        if (frame_count_ % 10 == 0) {
            std::cout << "━━━ 第 " << frame_count_ << " 幀統計 ━━━" << std::endl;
        }
    }

    void run() {
        gz::transport::Node node;
        std::string topic = "/world/warehouse_world/model/depth_camera_frame/link/camera_link/sensor/depth_camera/depth_image/points";
        if (!node.Subscribe(topic, &PointCloudBridge::onPointCloud, this)) {
            std::cerr << "✗ 無法訂閱 topic: " << topic << std::endl;
            return;
        }
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
        std::cout << "✓ 已訂閱: " << topic << std::endl;
        std::cout << "✓ 等待點雲資料... (按 Ctrl+C 停止)" << std::endl;
        std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << std::endl;
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
