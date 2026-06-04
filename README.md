# navigation_project

## 專案簡介

本專案為「基於 Octree 與機器學習的室內多樓層 3D 點雲導航優化」初始工程骨架。
目標建立 C++ 與 Python 混合專案，C++ 負責 Octree 結構、3D 規劃與 Gazebo 整合，Python 負責 ML 模型訓練與 ONNX 匯出。

## 目前專案結構

- `CMakeLists.txt`：C++ 專案設定與編譯規則。
- `src/`：C++ 原始碼。
- `include/`：C++ 標頭檔。
- `python/`：Python 訓練與資料處理程式。
- `.gitignore`：常見忽略檔案。

## 開發建議

1. 先實作基本 Octree 結構與 3D A* 路徑規劃。
2. 再整合 Gazebo 點雲輸入與增量更新。
3. 最後加入 Python Random Forest 訓練、ONNX 匯出與 C++ 推理。

## 初始編譯指令

```bash
mkdir -p build
cd build
cmake ..
make
```

編譯完成後，可執行：

```bash
./navigation_node
```

## 即時 3D 點雲顯示

專案已提供一個簡單的即時 PyVista 3D viewer，結合 C++ bridge 與 Python client，將 Gazebo 點雲直接顯示為彩色 3D 點雲。

1. 在 build 目錄啟動 C++ bridge：

```bash
cd build
./pointcloud_bridge
```

2. 在專案根目錄啟動 Python viewer：

```bash
cd /home/ruilun/project/navigation_project
source venv/bin/activate
python3 scripts/visualize_pointcloud_realtime.py
```

3. 若尚未安裝 PyVista：

```bash
pip install pyvista numpy
```

如果 Gazebo sensor 正常輸出 point cloud，這個 viewer 應該會即時呈現當前場景的 3D 點雲分佈。