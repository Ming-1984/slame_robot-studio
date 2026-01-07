#!/bin/bash

# Aurora Point Cloud Processing - 简化版本
# 执行两个处理步骤：检查/提取、墙体增强处理

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "🚀 Aurora Point Cloud Processing - 简化版本"
echo "=============================================="
echo ""

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BIN_DIR="$PROJECT_DIR/bin/Release"
DATA_DIR="$PROJECT_DIR/data"

# 创建数据目录
mkdir -p "$DATA_DIR"

echo -e "${BLUE}🔄 运行所有处理步骤${NC}"
echo ""

# 步骤1：检查点云数据
echo -e "${BLUE}步骤1: 检查点云数据${NC}"
if [ -f "$DATA_DIR/colored_point_cloud.ply" ]; then
    echo -e "${GREEN}✅ 发现现有点云数据${NC}"
    file_size=$(du -h "$DATA_DIR/colored_point_cloud.ply" | cut -f1)
    echo -e "${BLUE}📊 文件大小: ${file_size}${NC}"
else
    echo -e "${YELLOW}⚠️ 未找到点云数据，跳过提取步骤${NC}"
fi
echo ""

# 步骤2：处理点云
echo -e "${BLUE}步骤2: 处理点云数据${NC}"
if [ -f "$DATA_DIR/colored_point_cloud.ply" ] && [ -f "$BIN_DIR/cloud_processor_tool" ]; then
    cd "$DATA_DIR"
    echo "正在处理点云数据..."
    if "$BIN_DIR/cloud_processor_tool" colored_point_cloud.ply walls_enhanced_cloud.ply > /dev/null 2>&1; then
        echo -e "${GREEN}✅ 点云处理完成${NC}"
        if [ -f "walls_enhanced_cloud.ply" ]; then
            file_size=$(du -h walls_enhanced_cloud.ply | cut -f1)
            echo -e "${BLUE}📊 输出文件: walls_enhanced_cloud.ply (${file_size})${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️ 点云处理失败，使用原始数据${NC}"
    fi
else
    echo -e "${YELLOW}⚠️ 跳过点云处理（缺少输入数据或工具）${NC}"
fi
echo ""

echo -e "${GREEN}🎉 处理完成！${NC}"
echo ""
echo -e "${BLUE}📁 输出文件位置: $DATA_DIR${NC}"
echo "可用文件："
cd "$DATA_DIR"
ls -la *.ply *.xyz 2>/dev/null || echo "无输出文件"

echo ""
echo -e "${BLUE}📋 处理总结：${NC}"
echo "1. ✅ 点云数据检查完成"
echo "2. ✅ 点云处理完成（如果有数据）"
echo ""
echo -e "${GREEN}所有步骤已完成！${NC}"
