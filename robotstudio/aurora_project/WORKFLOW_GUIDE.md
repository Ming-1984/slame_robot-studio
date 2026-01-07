# Aurora Point Cloud Processing - 详细工作流程

## 📋 两个按钮 + 可选高级检测

### 🎯 **正确的使用流程**

#### 第一步：设置目录

1. **选择输入目录** - 点击"Input Directory"旁的"Browse"按钮，选择包含`.stcm`文件的文件夹
2. **选择输出目录** - 点击"Output Directory"旁的"Browse"按钮，选择处理结果的保存位置

#### 第二步：执行处理

可以选择单步执行或批量执行：

**单步执行：**

1. 点击"1. Extract Point Cloud from Radar"
2. 等待完成后，点击"2. Process Point Cloud (Wall Enhancement)"

**批量执行：**

- 直接点击"Run All Steps"，系统会自动依次执行提取与墙体增强两步（并可选执行高级检测）

---

## 🔄 **详细处理流程**

### 按钮 1：Extract Point Cloud from Radar

**功能：** 从雷达数据中提取点云

**输入：**

- 输入目录中的`.stcm`文件（雷达扫描数据）

**处理过程：**

1. 自动扫描输入目录，查找所有`.stcm`文件
2. 如果找到多个文件，使用第一个文件进行处理
3. 使用`colored_extractor`程序提取点云数据
4. 编译并构建必要的 C++程序（如果需要）

**输出文件：**

- `colored_point_cloud.ply` - PLY 格式的彩色点云数据
- `colored_point_cloud.xyz` - XYZ 格式的点云数据

**日志信息：**

```
Found 1 STCM file(s) in input directory
STCM file: example.stcm
Using STCM file: C:\path\to\input\example.stcm
Extracting point cloud data (XYZ format)...
Extracting point cloud data (PLY format)...
XYZ file generated: colored_point_cloud.xyz (Size: 1234.56 KB)
PLY file generated: colored_point_cloud.ply (Size: 2345.67 KB)
```

---

### 按钮 2：Process Point Cloud (Wall Enhancement)

**功能：** 对点云进行墙体增强处理

**输入：**

- 第一步生成的`colored_point_cloud.ply`文件

**处理过程：**

1. 检查输出目录中是否存在点云文件
2. 使用`cloud_processor_tool`进行墙体增强处理
3. 应用优化的参数配置：
   - 轻度噪声过滤（保留更多细节）
   - 几乎不进行高度过滤
   - 增强墙体点识别算法
   - 轻度密度过滤

**输出文件：**

- `walls_enhanced_cloud.ply` - 墙体增强后的点云数据

**处理参数：**

```
- 远距离点过滤: distantFactor=3.0 (保留更多点)
- 轻度去噪: meanK=6, stdDev=3.0 (保留更多点)
- 宽泛高度过滤: minPct=0.0001, maxPct=0.9999 (几乎全部保留)
- 地面/天花板处理: angle=5.0, dist=0.03 (精确识别)
- 增强墙体识别: normalK=10, verticalThreshold=0.8 (保留更多墙体)
- 轻度密度过滤: radius=0.1, minNeighbors=1 (保留低密度点)
```

**日志信息：**

```
Using processed point cloud data: walls_enhanced_cloud.ply
Wall enhancement processing complete!
Preservation ratio: 85.3%
```

---

### 说明：CAD/DXF 转换已移除

本仓库已移除 “Convert to CAD / DXF” 第3步点云处理功能；如需真景建图等能力，请使用官方上位机：`/home/jetson/ros2_ws/aurora_remote-release-2.1.0-rc2`。

---

## 📁 **文件流转图**

```
输入目录/
├── example.stcm          ← 用户提供的雷达数据
└── other_data.stcm

                ↓ 按钮1：Extract Point Cloud

输出目录/
├── colored_point_cloud.ply    ← 提取的PLY格式点云
└── colored_point_cloud.xyz    ← 提取的XYZ格式点云

                ↓ 按钮2：Process Point Cloud

输出目录/
├── colored_point_cloud.ply
├── colored_point_cloud.xyz
└── walls_enhanced_cloud.ply   ← 墙体增强后的点云
```

---

## ⚠️ **注意事项**

### 输入要求

1. **STCM 文件格式**：确保输入目录包含有效的`.stcm`格式雷达数据文件
2. **文件权限**：确保对输入和输出目录都有读写权限
3. **磁盘空间**：确保输出目录有足够的磁盘空间

### 处理顺序

1. **必须按顺序执行**：每个步骤都依赖于前一步的输出
2. **可以重复执行**：如果某步失败，可以重新执行该步骤
3. **批量处理**：使用"Run All Steps"可以自动按顺序执行

### 错误处理

1. **缺少 STCM 文件**：系统会提示选择新的输入目录
2. **缺少依赖文件**：系统会尝试查找备选文件
3. **构建失败**：检查 CMake / PCL 等依赖是否正确安装

---

## 🎯 **最佳实践**

### 目录组织建议

```
项目根目录/
├── 输入数据/
│   ├── 扫描数据1.stcm
│   ├── 扫描数据2.stcm
│   └── ...
├── 输出结果/
│   ├── 项目A/
│   ├── 项目B/
│   └── ...
```

### 工作流程建议

1. **准备阶段**：整理好所有 STCM 文件到一个输入目录
2. **设置阶段**：在 GUI 中设置正确的输入和输出目录
3. **处理阶段**：使用批量处理或逐步处理
4. **验证阶段**：检查输出文件的完整性和质量
5. **归档阶段**：将结果文件整理归档

### 质量控制

1. **检查文件大小**：确保生成的文件大小合理
2. **查看日志信息**：关注处理过程中的警告和错误

---

## 🔧 **故障排除**

### 常见问题

1. **找不到 STCM 文件**

   - 检查输入目录路径是否正确
   - 确认文件扩展名为`.stcm`

2. **构建失败**

   - 确保已安装 CMake / PCL 等依赖
   - Jetson/Ubuntu 可参考 `README_JETSON_NANO.md`

3. **输出文件为空**

   - 检查输入文件是否有效
   - 查看详细日志信息

4. **权限错误**
   - 以管理员身份运行程序
   - 检查目录权限设置
