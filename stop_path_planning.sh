#!/bin/bash

# 停止路径规划系统脚本
# 安全地停止所有ROS2路径规划相关进程

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🛑 停止路径规划系统${NC}"
echo "======================="
echo ""

# 停止所有相关进程
echo -e "${YELLOW}正在停止ROS2进程...${NC}"

# 停止探索节点
pkill -f "robust_explore_node.py" > /dev/null 2>&1 && echo "✅ 探索节点已停止"

# 停止Nav2相关进程
pkill -f "bt_navigator" > /dev/null 2>&1 && echo "✅ 行为树导航器已停止"
pkill -f "nav2" > /dev/null 2>&1 && echo "✅ Nav2系统已停止"

# 停止SLAM
pkill -f "slam_toolbox" > /dev/null 2>&1 && echo "✅ SLAM工具已停止"

# 停止硬件接口
pkill -f "hardware_interface" > /dev/null 2>&1 && echo "✅ 硬件接口已停止"

# 停止控制器
pkill -f "ackermann_controller" > /dev/null 2>&1 && echo "✅ 阿克曼控制器已停止"
pkill -f "unified_ackermann_controller" > /dev/null 2>&1 && echo "✅ 统一控制器已停止"

# 停止话题中继
pkill -f "topic_relay_node.py" > /dev/null 2>&1 && echo "✅ 话题中继已停止"

# 停止机器人状态发布器
pkill -f "robot_state_publisher" > /dev/null 2>&1 && echo "✅ 机器人状态发布器已停止"
pkill -f "joint_state_publisher" > /dev/null 2>&1 && echo "✅ 关节状态发布器已停止"

# 停止Aurora SDK
pkill -f "slamware_ros_sdk_server_node" > /dev/null 2>&1 && echo "✅ Aurora SDK已停止"

# 停止监控节点
pkill -f "motor_monitor" > /dev/null 2>&1 && echo "✅ 电机监控已停止"

# 停止RViz
pkill -f "rviz" > /dev/null 2>&1 && echo "✅ RViz已停止"

# 停止所有静态TF发布器
pkill -f "static_transform_publisher" > /dev/null 2>&1 && echo "✅ 静态TF发布器已停止"

# 等待进程结束
sleep 2

echo ""
echo -e "${GREEN}🎉 路径规划系统已完全停止${NC}"

# 检查是否还有残留进程
remaining=$(pgrep -f "nav2\|slam\|aurora\|ackermann" 2>/dev/null | wc -l)
if [ $remaining -gt 0 ]; then
    echo -e "${YELLOW}⚠️ 检测到 $remaining 个残留进程${NC}"
else
    echo -e "${GREEN}✅ 所有进程已清理完毕${NC}"
fi

echo ""
echo -e "${BLUE}💡 提示: 可以通过Web界面重新启动路径规划系统${NC}"