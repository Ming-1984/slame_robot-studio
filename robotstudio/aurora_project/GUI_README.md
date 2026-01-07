# Aurora Point Cloud Processing GUI

## 概述

Aurora Point Cloud Processing GUI 是一个图形化界面工具，用于处理来自雷达系统的点云数据。该工具提供了一个用户友好的界面来执行点云提取、处理和转换操作。

## 功能特性

### 主要功能
1. **点云提取** - 从STCM文件中提取点云数据，生成PLY和XYZ格式文件
2. **点云处理** - 对点云进行墙体增强处理，优化建筑结构细节
3. **高级检测（可选）** - 检测门/窗/地面/天花板等并输出着色点云（PLY）

### 界面特性
- 直观的图形用户界面
- 可选择自定义输出目录
- 实时处理日志显示
- 进度条显示处理状态
- 批量处理功能（一键执行所有步骤）
- 快速打开输出文件夹

## 系统要求

### 必需软件
- Ubuntu 22.04（Jetson Nano / ARM64）
- Python 3.7+
- CMake 3.15+
- PCL（libpcl-dev）

### Python依赖
- tkinter（Python标准库，无需额外安装）
- pathlib（Python标准库）

## 安装和使用

### 方法1：直接运行Python脚本

1. 确保已安装Python 3.7+
2. 在命令行中运行：
   ```
   python3 aurora_gui.py
   ```

### 方法2：构建可执行文件

1. 安装PyInstaller：
   ```
   pip install pyinstaller
   ```

2. 运行构建脚本：
   ```
   python3 build_gui.py
   ```

3. 构建完成后，在 `dist/AuroraGUI_Package/` 目录中找到可执行文件

## 使用说明

### 基本操作流程

1. **启动应用程序**
   - 运行 `python3 aurora_gui.py`（或运行构建的可执行文件）

2. **选择输出目录**
   - 点击"Browse"按钮选择输出文件的保存位置
   - 默认使用项目的data目录

3. **执行处理步骤**
   
   **单步执行：**
   - 点击"1. Extract Point Cloud from Radar"提取点云数据
   - 点击"2. Process Point Cloud (Wall Enhancement)"处理点云
   
   **批量执行：**
   - 点击"Run All Steps"自动执行提取与墙体增强两步（并可选高级检测）

4. **监控处理过程**
   - 查看日志区域了解处理进度
   - 观察进度条显示当前状态
   - 状态栏显示当前操作状态

5. **查看结果**
   - 点击"Open Output Folder"打开输出目录
   - 检查生成的文件

### 输出文件说明

处理完成后，输出目录将包含以下文件：

- `colored_point_cloud.ply` - 原始提取的点云数据（PLY格式）
- `colored_point_cloud.xyz` - 原始提取的点云数据（XYZ格式）
- `walls_enhanced_cloud.ply` - 墙体增强处理后的点云数据
- `building_elements_detected.ply` - 建筑元素检测输出（可选）

## 界面说明

### 主要控件

1. **输出目录选择**
   - 文本框显示当前输出目录
   - "Browse"按钮用于选择新的输出目录

2. **功能按钮**
   - "Extract Point Cloud from Radar" - 执行点云提取
   - "Process Point Cloud (Wall Enhancement)" - 执行点云处理
   - "Run All Steps" - 批量执行所有步骤

3. **控制按钮**
   - "Clear Log" - 清空日志显示
   - "Stop Process" - 停止当前正在运行的进程
   - "Open Output Folder" - 打开输出文件夹

4. **信息显示**
   - 进度条 - 显示处理进度
   - 日志区域 - 显示详细的处理信息
   - 状态栏 - 显示当前状态

## 故障排除

### 常见问题

1. **Python未找到**
   - 确保Python已正确安装并添加到系统PATH
   - 重新安装Python并勾选"Add to PATH"选项

2. **CMake错误**
   - 确保已安装CMake 3.15或更高版本
   - 确保CMake已添加到系统PATH

3. **Visual Studio构建工具错误**
   - （Linux）确保已安装 CMake / PCL 等依赖

4. **文件权限错误**
   - 确保对输出目录有写入权限
   - 确认当前用户对目录具有写权限

### 日志信息

程序会在日志区域显示详细的处理信息，包括：
- 处理步骤的开始和完成
- 文件路径和大小信息
- 错误和警告消息
- 处理参数和配置

## 技术细节

### 处理流程

1. **点云提取阶段**
   - 读取STCM格式的雷达数据文件
   - 使用colored_extractor提取点云数据
   - 生成PLY和XYZ格式的输出文件

2. **点云处理阶段**
   - 使用cloud_processor_tool处理点云
   - 应用墙体增强算法
   - 优化建筑结构细节保留

3. **说明：CAD/DXF 转换已移除**

如需真景建图等能力，请使用官方上位机：`/home/jetson/ros2_ws/aurora_remote-release-2.1.0-rc2`。

### 配置参数

程序使用优化的参数配置：
- 噪声过滤：轻度过滤，保留更多细节
- 高度过滤：几乎不过滤，保留完整结构
- 墙体识别：增强算法，提高墙体点保留率
- 密度过滤：轻度过滤，保留低密度特征点

## 支持和反馈

如有问题或建议，请参考项目文档或联系开发团队。
