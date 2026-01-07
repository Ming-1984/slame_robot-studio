#!/bin/bash

# Aurora Point Cloud Complete Processing
# 完整的两步处理流程：检查/提取、墙体增强处理

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

echo -e "${PURPLE}🚀 Aurora Point Cloud Complete Processing${NC}"
echo -e "${PURPLE}=========================================${NC}"
echo ""

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BIN_DIR="/home/jetson/ros2_ws/build/bin"
DATA_DIR="$PROJECT_DIR/data"

# 创建数据目录
mkdir -p "$DATA_DIR"

echo -e "${BLUE}📋 Aurora点云处理系统${NC}"
echo -e "${BLUE}功能：从Aurora摄像头提取点云 → 墙体增强处理${NC}"
echo ""

# 检查工具可用性
echo -e "${BLUE}🔧 检查处理工具...${NC}"
tools_ok=true

if [ ! -f "$BIN_DIR/cloud_processor_tool" ]; then
    echo -e "${RED}❌ cloud_processor_tool 未找到${NC}"
    tools_ok=false
else
    echo -e "${GREEN}✅ cloud_processor_tool 可用${NC}"
fi

if [ "$tools_ok" = false ]; then
    echo -e "${RED}请先构建项目: ./build_project.sh${NC}"
    exit 1
fi

echo ""

# 步骤1：检查/提取点云数据
echo -e "${BLUE}📊 步骤1: 点云数据检查与提取${NC}"
echo "----------------------------------------"

if [ -f "$DATA_DIR/colored_point_cloud.ply" ]; then
    file_size=$(du -h "$DATA_DIR/colored_point_cloud.ply" | cut -f1)
    echo -e "${GREEN}✅ 发现现有点云数据: colored_point_cloud.ply (${file_size})${NC}"
    
    # 检查文件格式
    file_type=$(file "$DATA_DIR/colored_point_cloud.ply" | cut -d: -f2)
    echo -e "${BLUE}📄 文件格式: ${file_type}${NC}"
    
    # 快速统计
    if grep -q "element vertex" "$DATA_DIR/colored_point_cloud.ply"; then
        vertex_count=$(grep "element vertex" "$DATA_DIR/colored_point_cloud.ply" | awk '{print $3}')
        echo -e "${BLUE}📈 顶点数量: ${vertex_count}${NC}"
    fi
else
    echo -e "${YELLOW}⚠️ 未找到点云数据文件${NC}"
    echo -e "${BLUE}💡 提示：${NC}"
    echo "   - 连接Aurora设备并运行数据采集"
    echo "   - 或将现有的PLY文件复制到 data/ 目录"
    echo "   - 文件名应为: colored_point_cloud.ply"
fi

echo ""

# 步骤2：点云处理（墙体增强）
echo -e "${BLUE}🔄 步骤2: 点云处理（墙体增强）${NC}"
echo "----------------------------------------"

if [ -f "$DATA_DIR/colored_point_cloud.ply" ]; then
    cd "$DATA_DIR"
    echo "正在进行墙体增强处理..."
    
    if "$BIN_DIR/cloud_processor_tool" colored_point_cloud.ply walls_enhanced_cloud.ply > /tmp/processing.log 2>&1; then
        if [ -f "walls_enhanced_cloud.ply" ]; then
            file_size=$(du -h walls_enhanced_cloud.ply | cut -f1)
            echo -e "${GREEN}✅ 墙体增强处理完成${NC}"
            echo -e "${BLUE}📁 输出文件: walls_enhanced_cloud.ply (${file_size})${NC}"
            
            # 检查处理结果
            if grep -q "element vertex" walls_enhanced_cloud.ply 2>/dev/null; then
                vertex_count=$(grep "element vertex" walls_enhanced_cloud.ply | awk '{print $3}')
                echo -e "${BLUE}📈 处理后顶点数: ${vertex_count}${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️ 处理完成但未生成输出文件${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️ 墙体增强处理失败，将使用原始数据${NC}"
        if [ -f /tmp/processing.log ]; then
            echo -e "${BLUE}错误详情:${NC}"
            head -5 /tmp/processing.log
        fi
    fi
    cd "$PROJECT_DIR"
else
    echo -e "${YELLOW}⚠️ 跳过处理步骤（无输入数据）${NC}"
fi

echo ""

# 清理临时文件
rm -f /tmp/processing.log

echo ""
echo -e "${PURPLE}🎉 处理流程完成！${NC}"
echo -e "${PURPLE}==================${NC}"
echo ""

# 显示最终结果
echo -e "${BLUE}📁 输出文件总览:${NC}"
echo "位置: $DATA_DIR"
echo ""

cd "$DATA_DIR"
echo -e "${BLUE}点云文件 (.ply):${NC}"
ls -la *.ply 2>/dev/null | while read line; do
    echo "  $line"
done || echo "  无点云文件"

echo ""
echo -e "${BLUE}其他文件:${NC}"
ls -la *.xyz *.txt 2>/dev/null | while read line; do
    echo "  $line"
done || echo "  无其他文件"

echo ""
echo -e "${GREEN}✅ Aurora点云处理系统运行完成${NC}"
echo -e "${BLUE}💡 提示：${NC}"
echo "   - PLY文件可用于3D可视化和进一步处理"
echo "   - 墙体增强处理有助于提取建筑结构特征"

echo ""
echo -e "${PURPLE}感谢使用Aurora点云处理系统！${NC}"
