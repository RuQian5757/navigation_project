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
