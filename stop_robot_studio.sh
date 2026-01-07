#!/bin/bash

# Robot Studio 停止脚本
# 停止robot-studio启动的所有服务

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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

# 配置参数
WORKSPACE_DIR="/home/jetson/ros2_ws"
PID_DIR="/var/run/robot-studio"

# 默认不改热点/网卡，避免断网；如需彻底清理网络：传入 --network-cleanup
NETWORK_CLEANUP=false
if [ "$1" = "--network-cleanup" ]; then
    NETWORK_CLEANUP=true
fi

# 切换到工作目录
cd "$WORKSPACE_DIR" || {
    log_warning "无法切换到工作目录: $WORKSPACE_DIR"
}

log_info "🛑 停止Robot Studio完整系统..."

# 1. 使用robot-studio命令停止服务
if [ -x "./robot-studio" ]; then
    log_info "使用robot-studio命令停止服务..."
    ./robot-studio stop > /var/log/robot-studio-shutdown.log 2>&1
    log_success "robot-studio stop 命令已执行"
else
    log_warning "robot-studio脚本不存在，使用手动清理方式"
fi

# 2. 强制清理所有相关进程
log_info "强制清理所有相关进程..."

# Web服务进程
pkill -f "robot_api_server" 2>/dev/null || true
pkill -f "python3 -m http.server" 2>/dev/null || true
pkill -f "web_interface_server.py" 2>/dev/null || true
pkill -f "ros2_web_bridge" 2>/dev/null || true

# ROS2和机器人控制进程
pkill -f "slamware_ros_sdk" 2>/dev/null || true
pkill -f "unified_ackermann_controller" 2>/dev/null || true
pkill -f "topic_relay_node" 2>/dev/null || true
pkill -f "robot_static_tf.launch.py" 2>/dev/null || true
pkill -f "hardware_interface.launch.py" 2>/dev/null || true

# WiFi热点进程（可选）
if [ "$NETWORK_CLEANUP" = true ]; then
    pkill -f "hostapd" 2>/dev/null || true
    pkill -f "dnsmasq" 2>/dev/null || true
else
    log_info "跳过停止热点进程（保持网络连接）"
fi

log_success "所有相关进程已清理"

# 3. 清理PID文件
log_info "清理PID文件..."

if [ -f "$PID_DIR/robot_studio.pid" ]; then
    ROBOT_STUDIO_PID=$(cat "$PID_DIR/robot_studio.pid")
    if kill -0 $ROBOT_STUDIO_PID 2>/dev/null; then
        kill $ROBOT_STUDIO_PID 2>/dev/null || true
        log_success "主进程已停止 (PID: $ROBOT_STUDIO_PID)"
    fi
    rm -f "$PID_DIR/robot_studio.pid"
fi

# 清理其他PID文件
for pid_file in "$PID_DIR"/*.pid; do
    if [ -f "$pid_file" ]; then
        rm -f "$pid_file"
    fi
done

# 4. 清理网络配置
if [ "$NETWORK_CLEANUP" = true ]; then
    log_info "清理网络配置..."

    # 清理WiFi接口配置
    WIFI_INTERFACE="wlP1p1s0"
    if ip link show "$WIFI_INTERFACE" > /dev/null 2>&1; then
        ip addr flush dev $WIFI_INTERFACE 2>/dev/null || true
        ip link set $WIFI_INTERFACE down 2>/dev/null || true
        log_success "WiFi接口已清理"
    fi

    # 清理iptables规则
    iptables -t nat -F 2>/dev/null || true
    iptables -F FORWARD 2>/dev/null || true
    log_success "防火墙规则已清理"

    # 禁用IP转发
    echo 0 > /proc/sys/net/ipv4/ip_forward
else
    log_info "跳过网络清理（保持当前连接）"
fi

# 5. 清理状态文件
rm -f "$PID_DIR/status"
rm -f "$PID_DIR/start_time"

# 6. 恢复NetworkManager（可选）
if [ "$NETWORK_CLEANUP" = true ]; then
    log_info "恢复NetworkManager..."
    systemctl start NetworkManager 2>/dev/null || true
fi

# 7. 等待进程完全停止
log_info "等待进程完全停止..."
sleep 3

# 8. 验证停止状态
log_info "验证停止状态..."

# 检查端口是否已释放
ports_in_use=0
for port in 8000 8001 8080; do
    if lsof -i :$port > /dev/null 2>&1; then
        ((ports_in_use++))
        log_warning "端口 $port 仍在使用中"
    fi
done

if [ $ports_in_use -eq 0 ]; then
    log_success "所有端口已释放"
else
    log_warning "$ports_in_use 个端口仍在使用中"
fi

# 检查关键进程是否已停止
processes_running=0
key_processes=("robot_api_server" "slamware_ros_sdk")
if [ "$NETWORK_CLEANUP" = true ]; then
    key_processes+=("hostapd" "dnsmasq")
fi

for process in "${key_processes[@]}"; do
    if pgrep -f "$process" > /dev/null; then
        ((processes_running++))
        log_warning "进程 $process 仍在运行"
    fi
done

if [ $processes_running -eq 0 ]; then
    log_success "所有关键进程已停止"
else
    log_warning "$processes_running 个关键进程仍在运行"
fi

log_success "🎉 Robot Studio已完全停止"

# 9. 显示最终状态
if [ "$NETWORK_CLEANUP" = true ]; then
    if systemctl is-active --quiet NetworkManager; then
        log_success "NetworkManager已恢复"
    else
        log_warning "NetworkManager未启动（可能不需要）"
    fi
fi

log_info "系统已恢复到停止前状态"
