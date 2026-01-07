#!/bin/bash

# Aurora激光雷达路径规划系统启动脚本 (Acamana模型) - 稳定探索版
# 集成MPPI控制器 + Hybrid A* + Reeds-Shepp规划器 + 稳定探索系统
# 优化配置：20Hz控制频率，2000采样轨迹，智能避障，容错探索

# 错误处理
set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 系统配置
export ROS_DOMAIN_ID=99
export LD_LIBRARY_PATH=/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
AURORA_IP="192.168.11.1"
LOG_DIR="logs"

# 运行模式
# - 默认：交互模式（终端运行，可在脚本内输入命令保存地图/查看状态）
# - 后台/非交互模式：适用于 Web/API 调用（不进入 read 循环，避免 stdin 非 TTY 时死循环刷屏）
DAEMON_MODE=false
NO_RVIZ=false

for arg in "$@"; do
    case "$arg" in
        --daemon|--non-interactive)
            DAEMON_MODE=true
            ;;
        --no-rviz)
            NO_RVIZ=true
            ;;
    esac
done

# 若 stdin 不是 TTY，自动切到后台模式（典型场景：Web/API/服务启动）
if [ ! -t 0 ]; then
    DAEMON_MODE=true
fi

# 创建日志目录
mkdir -p $LOG_DIR

# 等待指定ROS2节点出现或超时退出
wait_for_node() {
    local NODE_NAME=$1
    local TIMEOUT=${2:-30}   # 默认超时30秒 (增加稳定性)
    local INTERVAL=0.5
    local ELAPSED=0
    while true; do
        if ros2 node list 2>/dev/null | grep -q "$NODE_NAME"; then
            return 0
        fi
        sleep $INTERVAL
        ELAPSED=$(echo "$ELAPSED + $INTERVAL" | bc)
        if (( $(echo "$ELAPSED >= $TIMEOUT" | bc -l) )); then
            echo -e "${RED}等待节点 $NODE_NAME 超时${NC}"
            return 1
        fi
    done
}

# 等待某个话题可用
wait_for_topic() {
    local TOPIC_NAME=$1
    local TIMEOUT=${2:-30}   # 默认超时30秒 (增加稳定性)
    local INTERVAL=0.5
    local ELAPSED=0
    while true; do
        if ros2 topic list 2>/dev/null | grep -q "$TOPIC_NAME"; then
            return 0
        fi
        sleep $INTERVAL
        ELAPSED=$(echo "$ELAPSED + $INTERVAL" | bc)
        if (( $(echo "$ELAPSED >= $TIMEOUT" | bc -l) )); then
            echo -e "${RED}等待话题 $TOPIC_NAME 超时${NC}"
            return 1
        fi
    done
}

# 清理函数
cleanup() {
    echo -e "\n${YELLOW}🧹 正在清理系统进程...${NC}"
    
    # 终止所有相关进程
    pkill -f "slamware_ros_sdk_server_node" > /dev/null 2>&1 || true
    pkill -f "robot_state_publisher" > /dev/null 2>&1 || true
    pkill -f "joint_state_publisher" > /dev/null 2>&1 || true
    pkill -f "topic_relay_node.py" > /dev/null 2>&1 || true
    pkill -f "hardware_interface" > /dev/null 2>&1 || true
    pkill -f "twist_to_ackermann" > /dev/null 2>&1 || true
    pkill -f "serial_bridge" > /dev/null 2>&1 || true
    pkill -f "motor_monitor" > /dev/null 2>&1 || true
    pkill -f "nav2" > /dev/null 2>&1 || true
    pkill -f "bt_navigator" > /dev/null 2>&1 || true

    pkill -f "robust_explore_node.py" > /dev/null 2>&1 || true
    pkill -f "rviz" > /dev/null 2>&1 || true
    
    # 清理所有静态TF发布器
    pkill -f "static_transform_publisher" > /dev/null 2>&1 || true
    pkill -f "minimal_tf_publisher" > /dev/null 2>&1 || true
    pkill -f "odom2map" > /dev/null 2>&1 || true
    pkill -f "laser2base" > /dev/null 2>&1 || true
    pkill -f "leftcam2base" > /dev/null 2>&1 || true
    pkill -f "rightcam2Leftcam" > /dev/null 2>&1 || true
    pkill -f "imu2Leftcam" > /dev/null 2>&1 || true
    
    sleep 2
    echo -e "${GREEN}✅ 系统清理完成${NC}"
}

# 注册退出处理（仅在脚本结束时清理）
trap cleanup INT TERM

# 显示启动信息
echo "================================================"
echo "🚀 启动Aurora激光雷达路径规划系统 (稳定探索版)"
echo "================================================"
echo "📊 优化配置："
echo "   🎯 MPPI控制器 - 最新MPC变体"
echo "   🗺️ Hybrid A* + Reeds-Shepp规划器"
echo "   ⚡ 20Hz控制频率 - 超快响应"
echo "   🔄 2000采样轨迹 - 实时优化"
echo "   🛡️ 智能避障 - 多层次成本函数"
echo "   🌳 稳定探索系统 - 行为树+状态机架构"
echo "================================================"

# 设置ROS2环境
echo "🔧 设置ROS2环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash
sleep 1

# 添加ROS_DOMAIN_ID设置
export ROS_DOMAIN_ID=99

# 创建日志目录
mkdir -p logs

# 彻底清理系统（直接执行清理命令，不调用cleanup函数）
echo "🧹 清理残留进程..."
pkill -f "slamware_ros_sdk_server_node" > /dev/null 2>&1 || true
pkill -f "robot_state_publisher" > /dev/null 2>&1 || true
pkill -f "joint_state_publisher" > /dev/null 2>&1 || true
pkill -f "topic_relay_node.py" > /dev/null 2>&1 || true
pkill -f "hardware_interface" > /dev/null 2>&1 || true
pkill -f "nav2" > /dev/null 2>&1 || true
pkill -f "bt_navigator" > /dev/null 2>&1 || true
pkill -f "robust_explore_node.py" > /dev/null 2>&1 || true
echo -e "${GREEN}✅ 系统清理完成${NC}"
sleep 1

# 重启ROS2 daemon
echo "🔄 重启ROS2 daemon..."
ros2 daemon stop > /dev/null 2>&1 || true
sleep 1
ros2 daemon start > /dev/null 2>&1
sleep 1

echo "================================================"
echo "🎯 正在按序列启动核心组件..."
echo "================================================"

# 1️⃣ 启动Aurora SDK服务器
echo "1️⃣ 启动Aurora SDK服务器..."
export LD_LIBRARY_PATH=/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
ros2 launch slamware_ros_sdk slamware_ros_sdk_server_node.py ip_address:=$AURORA_IP > logs/aurora_sdk.log 2>&1 &
# 等待 /scan 话题出现表明 SDK 正常工作，最长 10s
wait_for_topic "/scan" 10 || { echo -e "${RED}Aurora SDK 启动失败${NC}"; exit 1; }

# ✨ 新增：启动时清除旧地图
echo "✨ 清除所有历史地图数据..."
ros2 topic pub --once /slamware_ros_sdk_server_node/clear_map slamware_ros_sdk/msg/ClearMapRequest "{}" > /dev/null 2>&1
sleep 1

# 2️⃣ 启动机器人描述发布器
echo "2️⃣ 启动机器人描述发布器..."
ros2 launch acamana robot_static_tf.launch.py use_sim_time:=false > logs/robot_model.log 2>&1 &
# 等待 TF 树就绪（map->odom）
wait_for_topic "/tf" 5

# 3️⃣ 启动统一阿克曼控制器 (优化版本)
echo "3️⃣ 启动统一阿克曼控制器..."
python3 src/ackermann_controller/scripts/unified_ackermann_controller.py > logs/ackermann_controller.log 2>&1 &

# 3️⃣-2 启动话题中继节点
echo "3️⃣-2 启动话题中继节点..."
python3 src/acamana/scripts/topic_relay_node.py > logs/topic_relay.log 2>&1 &

# 4️⃣ 启动底盘硬件接口 (仅串口通信，控制器已优化)
echo "4️⃣ 启动底盘硬件接口..."
ros2 launch hardware_interface hardware_interface.launch.py min_speed:=0.0 > logs/hardware_interface.log 2>&1 &
# 等待 /odom 就绪，最长 6s
wait_for_topic "/odom" 6

# 5️⃣ 启动Nav2导航服务器 (MPPI + Hybrid A*)
echo "5️⃣ 启动高性能Nav2导航服务器 (MPPI控制器 + Hybrid A*规划器)..."
ros2 launch nav2_configs ackermann_slam_nav2_launch.py use_sim_time:=false autostart:=true > logs/nav2.log 2>&1 &
# 等待 nav2 lifecycle 节点激活
wait_for_node "/controller_server" 10 || echo -e "${YELLOW}⚠️  Nav2启动完成${NC}"

# 6️⃣ 启动增强的智能探索系统 (全面优化版)
echo "6️⃣ 启动增强的智能探索系统..."
echo "   🛡️ 行为树+状态机架构 (稳定性)"
echo "   🎯 增强TAD算法前沿评估 (智能性)"
echo "   🏠 房间感知探索策略 (结构化探索)"
echo "   🔄 自适应参数管理 (动态优化)"
echo "   🚀 批量并行计算 (性能提升)"
echo "   🌟 多维度完成判断 (精确评估)"
echo "   🔍 多尺度前沿点检测 (全面覆盖)"
echo "   📊 实时性能监控 (可观测性)"

# 检查依赖
check_explorer_dependencies() {
    local missing_deps=0

    # 检查基础依赖
    if ! python3 -c "import numpy, cv2, sklearn" 2>/dev/null; then
        echo -e "${RED}❌ 缺少基础依赖，请安装: pip3 install numpy opencv-python scikit-learn${NC}"
        missing_deps=1
    fi

    # 检查增强功能依赖
    if ! python3 -c "import threading, concurrent.futures, queue, scipy" 2>/dev/null; then
        echo -e "${RED}❌ 缺少增强功能依赖，请安装: pip3 install scipy${NC}"
        missing_deps=1
    fi

    # 检查房间感知依赖
    if ! python3 -c "import cv2; cv2.connectedComponents" 2>/dev/null; then
        echo -e "${YELLOW}⚠️ OpenCV版本可能过低，房间感知功能可能受限${NC}"
    fi

    # 检查可选的地图优化功能
    if python3 -c "import cv2; print(cv2.cuda.getCudaEnabledDeviceCount())" 2>/dev/null | grep -q "^[1-9]"; then
        echo -e "${GREEN}✅ 检测到GPU支持，地图优化功能可用${NC}"
    else
        echo -e "${YELLOW}⚠️ 未检测到GPU支持，将使用CPU模式进行地图优化${NC}"
    fi

    # 检查系统资源
    local cpu_cores=$(nproc)
    local available_memory=$(free -m | awk 'NR==2{printf "%.1f", $7/1024}')
    echo -e "${BLUE}💻 系统资源: ${cpu_cores}核CPU, ${available_memory}GB可用内存${NC}"

    if [ "$cpu_cores" -lt 4 ]; then
        echo -e "${YELLOW}⚠️ CPU核心数较少，建议降低并行计算线程数${NC}"
    fi

    return $missing_deps
}

# 检查依赖
if check_explorer_dependencies; then
    echo "🛡️ 启动增强的智能探索系统..."
    echo "   ✅ 集成增强TAD算法智能前沿评估"
    echo "   ✅ 集成房间感知探索策略"
    echo "   ✅ 集成自适应参数管理"
    echo "   ✅ 集成批量并行计算"
    echo "   ✅ 集成多维度完成判断"
    echo "   ✅ 集成多尺度前沿点检测"
    echo "   ✅ 集成实时性能监控"

    # 启动优化的稳定探索系统，使用配置文件
    python3 src/aurora_explorer/scripts/robust_explore_node.py \
        --ros-args \
        --params-file src/aurora_explorer/config/optimized_robust_params.yaml \
        > logs/optimized_robust_explore.log 2>&1 &

    # 等待探索节点启动
    sleep 3

    # 可选：启动并行计算性能测试（调试模式）
    if [[ "${DEBUG_MODE:-false}" == "true" ]]; then
        echo "🧪 启动并行计算性能测试..."
        python3 src/aurora_explorer/scripts/test_parallel_computation.py \
            > logs/parallel_computation_test.log 2>&1 &
    fi
else
    echo -e "${RED}❌ 依赖检查失败，请先安装必要的依赖包${NC}"
    exit 1
fi
# Explorer 不依赖 RViz，可立即继续

# 7️⃣ 启动RViz可视化
if [ "$NO_RVIZ" = true ] || [ "$DAEMON_MODE" = true ]; then
    echo "7️⃣ 跳过RViz可视化（后台/无界面模式）..."
else
    echo "7️⃣ 启动RViz可视化..."
    rviz2 -d src/aurora_explorer/config/explore.rviz > logs/rviz.log 2>&1 &
fi

# 8️⃣ 启动底盘监控节点 
echo "8️⃣ 启动底盘监控节点..."
python3 src/hardware_interface/scripts/motor_monitor_node.py > logs/motor_monitor.log 2>&1 &

echo "================================================"
echo -e "${GREEN}✅ 稳定导航探索系统启动完成！${NC}"
echo "================================================"
echo ""
echo -e "${BLUE}🎯 系统特性：${NC}"
echo "   • MPPI控制器 - 最先进的模型预测控制"
echo "   • Hybrid A* - 支持倒车的智能路径规划"
echo "   • 20Hz控制频率 - 超快响应速度"
echo "   • 实时轨迹优化 - 2000个采样轨迹"
echo "   • 智能避障 - 多层次成本函数"
echo "   • Aurora SLAM - 高精度建图定位"
echo "   • 稳定探索系统 - 行为树+状态机容错架构"
echo "   • 多层异常恢复 - 3层防护机制"
echo "   • 模块化设计 - 高度可扩展和可维护"
echo ""
echo -e "${GREEN}🌟 增强的智能探索特性：${NC}"
echo "   • 房间感知探索 - 基于房间结构的智能策略"
echo "   • 自适应参数管理 - 根据环境动态调整参数"
echo "   • 批量并行计算 - 多线程前沿点评估和信息增益计算"
echo "   • 多维度完成判断 - 全局覆盖度、前沿点密度、探索效率综合评估"
echo "   • 多尺度前沿点检测 - 不同尺度下的前沿点发现"
echo "   • 增强TAD算法 - 多层次探索优先级和全局排序"
echo "   • 智能局部目标调整 - 动态路径优化和安全位置确保"
echo ""
echo -e "${YELLOW}--- 🗺️  手动地图保存与系统监控 ---${NC}"
echo "当您对探索的地图满意后，可随时在此处进行保存。"
echo "您也可以查看系统性能统计和并行计算状态。"
echo ""
echo "可用命令："
echo "  's' - 保存地图"
echo "  'p' - 查看性能统计"
echo "  'c' - 查看完成度分析"
echo "  'r' - 查看房间探索状态"
echo "  'a' - 查看自适应参数状态"
echo "  'l' - 查看最新日志"
echo "  'h' - 显示帮助"
echo "  Ctrl+C - 退出系统"

# 后台模式不进入交互循环，直接返回
if [ "$DAEMON_MODE" = true ]; then
    echo ""
    echo -e "${GREEN}✅ 已以后台模式启动（非交互）${NC}"
    echo -e "${CYAN}   可在 Web 界面执行：保存2D地图 / 清除地图 / 停止探索 / 停止导航${NC}"
    exit 0
fi

# 等待用户中断
while true; do
    read -p ">> 请输入命令: " user_input
    case "$user_input" in
        "s"|"S")
            # 保存地图 - 简化版本，直接使用默认名称
            map_name="map_$(date +%Y-%m-%d_%H-%M-%S)"
            echo -e "${BLUE}   💾 正在保存地图: ${map_name}${NC}"

            # 创建maps目录（如果不存在）
            mkdir -p ~/maps

            # 直接使用简单地图保存器保存
            if python3 simple_map_saver.py ~/maps/${map_name} --timeout 10 > /dev/null 2>&1; then
                echo -e "${GREEN}   ✅ 地图保存成功！${NC}"
                echo -e "${GREEN}   📁 位置: ~/maps/${map_name}.yaml 和 ~/maps/${map_name}.pgm${NC}"

                # 显示文件大小
                if [ -f ~/maps/${map_name}.yaml ] && [ -f ~/maps/${map_name}.pgm ]; then
                    yaml_size=$(du -h ~/maps/${map_name}.yaml | cut -f1)
                    pgm_size=$(du -h ~/maps/${map_name}.pgm | cut -f1)
                    echo -e "${BLUE}   📊 大小: YAML ${yaml_size}, PGM ${pgm_size}${NC}"
                fi
            else
                echo -e "${RED}   ❌ 地图保存失败，请稍后重试${NC}"
            fi
            ;;
        "p"|"P")
            # 查看性能统计
            echo -e "${BLUE}📊 系统性能统计：${NC}"
            if [ -f "logs/optimized_robust_explore.log" ]; then
                echo "最近的性能日志："
                tail -30 logs/optimized_robust_explore.log | grep -E "(🚀|🎯|⚡|📊|🔄)" || echo "暂无性能统计数据"
            else
                echo "日志文件不存在"
            fi
            ;;
        "c"|"C")
            # 查看完成度分析
            echo -e "${BLUE}🌟 探索完成度分析：${NC}"
            if [ -f "logs/optimized_robust_explore.log" ]; then
                echo "最近的完成度分析："
                tail -20 logs/optimized_robust_explore.log | grep -E "(📊.*探索进度|🌟.*完成|✅.*完成)" || echo "暂无完成度数据"
            else
                echo "日志文件不存在"
            fi
            ;;
        "r"|"R")
            # 查看房间探索状态
            echo -e "${BLUE}🏠 房间探索状态：${NC}"
            if [ -f "logs/optimized_robust_explore.log" ]; then
                echo "最近的房间探索日志："
                tail -20 logs/optimized_robust_explore.log | grep -E "(🏠|房间|room)" || echo "暂无房间探索数据"
            else
                echo "日志文件不存在"
            fi
            ;;
        "a"|"A")
            # 查看自适应参数状态
            echo -e "${BLUE}🔄 自适应参数状态：${NC}"
            if [ -f "logs/optimized_robust_explore.log" ]; then
                echo "最近的参数调整日志："
                tail -20 logs/optimized_robust_explore.log | grep -E "(🔄.*自适应|参数.*更新)" || echo "暂无参数调整数据"
            else
                echo "日志文件不存在"
            fi
            ;;
        "l"|"L")
            # 查看最新日志
            echo -e "${BLUE}📋 最新系统日志：${NC}"
            if [ -f "logs/optimized_robust_explore.log" ]; then
                tail -10 logs/optimized_robust_explore.log
            else
                echo "日志文件不存在"
            fi
            ;;
        "h"|"H")
            # 显示帮助
            echo -e "${BLUE}📖 可用命令：${NC}"
            echo "  's' - 保存当前地图"
            echo "  'p' - 查看系统性能统计"
            echo "  'c' - 查看探索完成度分析"
            echo "  'r' - 查看房间探索状态"
            echo "  'a' - 查看自适应参数状态"
            echo "  'l' - 查看最新系统日志"
            echo "  'h' - 显示此帮助信息"
            echo "  Ctrl+C - 安全退出系统"
            ;;
        *)
            echo -e "${YELLOW}   无效输入。请输入 's', 'p', 'c', 'r', 'a', 'l', 'h' 或按 Ctrl+C 退出。${NC}"
            ;;
    esac
    echo ""
done
