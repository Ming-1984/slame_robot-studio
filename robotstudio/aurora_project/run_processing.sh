#!/bin/bash

# Aurora Point Cloud Processing - 命令行版本
# 执行两个处理步骤：提取、处理（墙体增强）

# 移除 set -e 以避免脚本因小错误退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "🚀 Aurora Point Cloud Processing - 命令行版本"
echo "=============================================="
echo ""

# 获取脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR"
BIN_DIR="$PROJECT_DIR/bin/Release"
DATA_DIR="$PROJECT_DIR/data"

# 检查必要的工具
if [ ! -f "$BIN_DIR/colored_extractor" ]; then
    echo -e "${RED}❌ colored_extractor 未找到，请先构建项目${NC}"
    echo "运行: ./build_project.sh"
    exit 1
fi

if [ ! -f "$BIN_DIR/cloud_processor_tool" ]; then
    echo -e "${RED}❌ cloud_processor_tool 未找到，请先构建项目${NC}"
    echo "运行: ./build_project.sh"
    exit 1
fi

# 创建数据目录
mkdir -p "$DATA_DIR"

echo "📋 可用的处理步骤："
echo "1. 从Aurora摄像头提取点云数据"
echo "2. 处理点云数据（墙体增强）"
echo "3. 运行所有步骤"
echo ""

read -p "请选择要执行的步骤 (1-3): " choice

case $choice in
    1)
        echo -e "${BLUE}🔄 步骤1: 从Aurora摄像头提取点云数据${NC}"
        echo "注意：此步骤需要连接到Aurora设备"
        echo "如果没有连接设备，将使用现有的测试数据"
        
        # 检查是否有现有的点云数据
        if [ -f "$DATA_DIR/colored_point_cloud.ply" ]; then
            echo -e "${GREEN}✅ 发现现有点云数据: colored_point_cloud.ply${NC}"
            file_size=$(du -h "$DATA_DIR/colored_point_cloud.ply" | cut -f1)
            echo -e "${BLUE}📊 文件大小: ${file_size}${NC}"
        else
            echo -e "${YELLOW}⚠️ 未找到现有点云数据${NC}"
            echo "需要连接Aurora设备或提供STCM文件"
        fi
        ;;
        
    2)
        echo -e "${BLUE}🔄 步骤2: 处理点云数据（墙体增强）${NC}"
        
        if [ ! -f "$DATA_DIR/colored_point_cloud.ply" ]; then
            echo -e "${RED}❌ 未找到输入文件: colored_point_cloud.ply${NC}"
            echo "请先执行步骤1或确保数据目录中有点云文件"
            exit 1
        fi
        
        echo "正在处理点云数据..."
        cd "$DATA_DIR"
        
        if "$BIN_DIR/cloud_processor_tool" colored_point_cloud.ply walls_enhanced_cloud.ply; then
            echo -e "${GREEN}✅ 点云处理完成${NC}"
            if [ -f "walls_enhanced_cloud.ply" ]; then
                file_size=$(du -h walls_enhanced_cloud.ply | cut -f1)
                echo -e "${BLUE}📊 输出文件: walls_enhanced_cloud.ply (${file_size})${NC}"
            fi
        else
            echo -e "${RED}❌ 点云处理失败${NC}"
            exit 1
        fi
        ;;
        
    3)
        echo -e "${BLUE}🔄 运行所有步骤${NC}"
        echo ""
        
        # 步骤1：检查点云数据
        echo -e "${BLUE}步骤1: 检查点云数据${NC}"
        if [ -f "$DATA_DIR/colored_point_cloud.ply" ]; then
            echo -e "${GREEN}✅ 发现现有点云数据${NC}"
        else
            echo -e "${YELLOW}⚠️ 未找到点云数据，跳过提取步骤${NC}"
        fi
        echo ""
        
        # 步骤2：处理点云
        echo -e "${BLUE}步骤2: 处理点云数据${NC}"
        if [ -f "$DATA_DIR/colored_point_cloud.ply" ]; then
            cd "$DATA_DIR"
            if "$BIN_DIR/cloud_processor_tool" colored_point_cloud.ply walls_enhanced_cloud.ply > /dev/null 2>&1; then
                echo -e "${GREEN}✅ 点云处理完成${NC}"
            else
                echo -e "${YELLOW}⚠️ 点云处理失败，使用原始数据${NC}"
            fi
        else
            echo -e "${YELLOW}⚠️ 跳过点云处理（无输入数据）${NC}"
        fi
        echo ""
        
        ;;
        
    *)
        echo -e "${RED}❌ 无效选择${NC}"
        exit 1
        ;;
esac

echo ""
echo -e "${GREEN}🎉 处理完成！${NC}"
echo ""
echo -e "${BLUE}📁 输出文件位置: $DATA_DIR${NC}"
echo "可用文件："
ls -la "$DATA_DIR"/*.ply "$DATA_DIR"/*.xyz 2>/dev/null || echo "无输出文件"
