# Aurora 点云提取工具

Aurora 点云提取工具是一个用于从 STCM 文件中提取点云数据并进行处理的工具集。该工具集包括点云提取、点云处理与建筑元素检测功能。

## 功能特点

- **点云提取**：从 STCM 文件中提取彩色点云数据
- **点云处理**：对点云进行降噪、平面分割和特征提取
- **建筑元素检测（可选）**：门/窗/地面/天花板等检测与着色输出（PLY）

## 系统要求

- Ubuntu 22.04（Jetson Nano / ARM64）
- CMake 3.10+
- PCL（libpcl-dev）
- Python 3（用于 GUI/脚本）

## 目录结构

```
aurora_project/
├── bin/                # 编译后的可执行文件
├── build/              # CMake构建目录
├── data/               # 数据文件
│   ├── 1.stcm          # 原始STCM文件
│   ├── colored_point_cloud.ply  # 提取的PLY格式点云
│   ├── colored_point_cloud.xyz  # 提取的XYZ格式点云
│   ├── walls_enhanced_cloud.ply  # 墙体增强后的点云
│   └── building_elements_detected.ply # 建筑元素检测输出（可选）
├── scripts/            # 运行脚本
│   ├── extract_point_cloud.sh          # 点云提取（Bash）
│   ├── process_point_cloud_walls.sh    # 墙体增强（Bash）
│   └── advanced_building_detection.sh  # 建筑元素检测（Bash，可选）
└── src/                # 源代码
    ├── cloud_processor.h/cpp  # 点云处理模块
    ├── cloud_visualizer.h/cpp # 点云可视化模块
    ├── cloud_processor.cpp    # 点云处理工具
    └── colored_point_extractor.cpp  # 彩色点云提取工具
```

## 使用方法

### 1. 点云提取

从 STCM 文件中提取彩色点云数据：

```bash
cd scripts
./extract_point_cloud.sh
```

这将从`data/1.stcm`文件中提取点云数据，并保存为`data/colored_point_cloud.ply`和`data/colored_point_cloud.xyz`。

### 2. 点云处理

处理和优化点云数据：

```bash
cd scripts
./process_point_cloud_walls.sh
```

此脚本将应用以下处理：

- 降噪：去除噪声点（使用优化后的参数）
- 下采样：减少点云密度，但保留必要细节
- 平面分割：提取主要平面

处理后的点云将保存到 `data/walls_enhanced_cloud.ply`（或脚本指定的输出目录）。

### 3. 说明：CAD/DXF 转换已移除

本仓库已移除 “Convert to CAD / DXF” 第3步点云处理功能。真景建图等功能请使用官方上位机：`/home/jetson/ros2_ws/aurora_remote-release-2.1.0-rc2`。

## 点云处理流程

1. **加载点云数据**：从 PLY 或 XYZ 文件中加载点云数据
2. **降噪**：使用统计离群点滤波去除噪声点
   - 使用优化后的参数（stddev=0.8, meanK=25）
3. **下采样**：使用体素网格下采样减少点云密度
   - 使用合适的叶子大小（0.005）确保保留足够细节
4. **平面分割**：使用 RANSAC 算法提取平面
5. **墙体提取**：从垂直平面中提取墙体线段

### 处理后点云的保存

所有的点云处理结果都会自动保存到指定文件。如果在命令行中未指定输出文件，系统将自动生成一个固定的输出文件名。

### 优化点云提取功能

我们对点云处理算法进行了多项优化，以确保不会过滤掉重要的点：

1. **增加搜索半径**：从 0.1 增加到 0.15，确保找到足够的邻居点
2. **增加考虑的邻居点数量**：从 10 个增加到 20 个，提高统计稳定性
3. **智能过滤保护**：如果过滤掉超过 50%的点，会自动恢复使用原始点云
4. **增加阈值系数**：将标准差阈值乘以 1.5，减少过滤强度

## 注意事项

- 点云提取工具需要 STCM 文件作为输入，确保`data`目录中有有效的 STCM 文件。
- 点云处理工具需要先运行点云提取工具生成点云数据。
- 对于大型点云数据，下采样功能可能会导致内存溢出，请根据需要调整参数。

## 未来计划

- 添加更多点云处理功能，如点云配准和重建
- 改进墙体提取算法，提高准确性
- 添加门窗检测功能
- 与官方上位机（真景建图）能力联动与集成
