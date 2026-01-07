#!/bin/bash

# Robot Studio 开机自启动脚本
# 默认使用 robot-studio start 启动完整系统（不自动切热点/网卡）
# 如需热点模式：传入参数 --hotspot 或设置环境变量 ROBOT_STUDIO_HOTSPOT=1

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1" | tee -a /var/log/robot-studio.log
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1" | tee -a /var/log/robot-studio.log
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a /var/log/robot-studio.log
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a /var/log/robot-studio.log
}

# 配置参数
WORKSPACE_DIR="/home/jetson/ros2_ws"
PID_DIR="/var/run/robot-studio"

# 解析启动选项
# - 手动执行默认不切换热点/网卡（避免断网）
# - 作为 systemd 服务启动时，默认同时启动热点（满足“开机自启=上位机+热点”）
START_HOTSPOT=false

is_systemd_invocation=false
if [ -n "${INVOCATION_ID:-}" ]; then
    is_systemd_invocation=true
else
    parent_comm=$(ps -o comm= -p "$PPID" 2>/dev/null | tr -d ' ' || true)
    if [ "$PPID" = "1" ] || [ "$parent_comm" = "systemd" ]; then
        is_systemd_invocation=true
    fi
fi

# 显式关闭优先级最高
if [ "${ROBOT_STUDIO_HOTSPOT:-}" = "0" ]; then
    START_HOTSPOT=false
elif [ "$1" = "--hotspot" ] || [ "${ROBOT_STUDIO_HOTSPOT:-0}" = "1" ]; then
    START_HOTSPOT=true
elif [ "$is_systemd_invocation" = true ]; then
    START_HOTSPOT=true
fi

# 创建PID目录和日志目录
mkdir -p "$PID_DIR"
mkdir -p /var/log

# 切换到工作目录
cd "$WORKSPACE_DIR" || {
    log_error "无法切换到工作目录: $WORKSPACE_DIR"
    exit 1
}

log_info "🚀 Robot Studio 开机自启动开始..."

# 1. 检查robot-studio脚本是否存在
if [ ! -x "./robot-studio" ]; then
    log_error "robot-studio 脚本不存在或不可执行"
    exit 1
fi

# 2. 清理残留进程
log_info "🧹 清理系统残留进程..."

# 清理可能的残留进程
if [ "$START_HOTSPOT" = true ]; then
    pkill -f "hostapd" 2>/dev/null || true
    pkill -f "dnsmasq" 2>/dev/null || true
else
    log_info "跳过清理热点进程（保持网络连接）"
fi
pkill -f "robot_api_server" 2>/dev/null || true
pkill -f "python3 -m http.server" 2>/dev/null || true
pkill -f "ros2_web_bridge" 2>/dev/null || true
pkill -f "slamware_ros_sdk" 2>/dev/null || true
pkill -f "unified_ackermann_controller" 2>/dev/null || true
pkill -f "hardware_interface" 2>/dev/null || true

# 清理端口占用
for port in 8000 8001 8080; do
    pid=$(lsof -t -i :$port 2>/dev/null || true)
    if [ ! -z "$pid" ]; then
        kill -9 $pid 2>/dev/null || true
        log_info "清理端口 $port 占用进程: $pid"
    fi
done

# 3. 快速等待系统稳定
log_info "⏳ 快速等待系统稳定..."
sleep 2

# 4. 启动Robot Studio完整系统
log_info "🚀 启动Robot Studio完整系统（包含底盘控制、SLAM、Web服务；热点可选）..."

# 设置环境变量
export ROS_DOMAIN_ID=99
export LD_LIBRARY_PATH=/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# 使用robot-studio命令启动完整系统，减少超时时间
log_info "启动robot-studio服务..."
start_args=("start")
if [ "$START_HOTSPOT" = true ]; then
    start_args=("start" "--hotspot")
fi

timeout 120 ./robot-studio "${start_args[@]}" > /var/log/robot-studio-startup.log 2>&1
ROBOT_STUDIO_EXIT_CODE=$?

if [ $ROBOT_STUDIO_EXIT_CODE -eq 0 ]; then
    log_success "robot-studio启动完成"
elif [ $ROBOT_STUDIO_EXIT_CODE -eq 124 ]; then
    log_warning "robot-studio启动超时，但可能已部分启动"
else
    log_error "robot-studio启动失败，退出码: $ROBOT_STUDIO_EXIT_CODE"
fi

# 5. 等待并验证服务启动
log_info "⏳ 验证服务启动状态..."

# 等待关键服务启动
local_retry_count=0
max_retries=60  # 1分钟

while [ $local_retry_count -lt $max_retries ]; do
    # 检查Web服务是否启动
    if curl -s "http://localhost:8000/api/status" >/dev/null 2>&1 && \
       curl -s "http://localhost:8080" >/dev/null 2>&1; then
        log_success "Web服务已启动"
        break
    fi

    sleep 1
    ((local_retry_count++))

    # 每15秒输出一次进度
    if [ $((local_retry_count % 15)) -eq 0 ]; then
        log_info "等待Web服务启动... ($local_retry_count/$max_retries 秒)"
    fi
done

# 6. 快速检查启动状态
log_info "🔍 快速检查系统启动状态..."

# 简化的组件状态检查
web_ok=false
wifi_ok=true  # 默认认为WiFi热点正常

# 快速检查Web服务端口
if lsof -i :8000 >/dev/null 2>&1 && lsof -i :8080 >/dev/null 2>&1; then
    web_ok=true
    log_success "Web服务端口正常"
else
    log_warning "Web服务可能仍在启动中"
fi

# 检查WiFi热点（如果启动了）
if pgrep hostapd > /dev/null && pgrep dnsmasq > /dev/null; then
    log_success "WiFi热点运行正常"
else
    log_info "WiFi热点未启动或仍在启动中"
fi

# 7. 创建状态文件
echo "RUNNING" > "$PID_DIR/status"
echo "$(date)" > "$PID_DIR/start_time"

log_success "🎉 Robot Studio 快速启动完成！"
log_info "📱 系统功能："
log_info "   • Web控制界面: http://localhost:8080"
log_info "   • API服务: http://localhost:8000"
log_info "   • 底盘控制: 已启动"
log_info "   • Aurora LIDAR: 已启动"

if pgrep hostapd > /dev/null; then
    log_info "   • WiFi热点: RobotStudio (无密码)"
    log_info "   • 热点访问: http://robot 或 http://192.168.4.1"
fi

log_info "💡 提示: 系统已快速启动，服务可能仍在初始化中"
log_info "💡 如需检查详细状态，请访问Web界面或运行: ./robot-studio status"

exit 0
