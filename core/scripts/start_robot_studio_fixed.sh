#!/bin/bash

# Robot Studio 修复版启动脚本
# 解决端口冲突问题，确保系统正确运行

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置参数
WORKSPACE_DIR="/home/jetson/ros2_ws"
WEB_INTERFACE_DIR="$WORKSPACE_DIR/web_interface"
API_PORT=8000
WEBSOCKET_PORT=8001
WEB_PORT=8080
HOTSPOT_SSID="RobotStudio-AP"
HOTSPOT_IP="192.168.4.1"

echo -e "${PURPLE}🚀 Robot Studio 系统启动器 (修复版)${NC}"
echo -e "${PURPLE}========================================${NC}"
echo ""

# 检查是否以root权限运行
if [ "$EUID" -eq 0 ]; then
    echo -e "${YELLOW}⚠️ 检测到root权限，将用于WiFi热点配置${NC}"
    IS_ROOT=true
else
    echo -e "${BLUE}ℹ️ 非root权限，跳过WiFi热点配置${NC}"
    IS_ROOT=false
fi

# 切换到工作目录
cd "$WORKSPACE_DIR"
echo -e "${BLUE}📁 工作目录: $WORKSPACE_DIR${NC}"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo -e "${YELLOW}🧹 正在清理系统进程...${NC}"
    
    # 停止后台进程
    if [ ! -z "$API_PID" ]; then
        kill $API_PID 2>/dev/null || true
        echo "  - API服务器已停止"
    fi
    
    if [ ! -z "$WEBSOCKET_PID" ]; then
        kill $WEBSOCKET_PID 2>/dev/null || true
        echo "  - WebSocket服务已停止"
    fi
    
    if [ ! -z "$WEB_SERVER_PID" ]; then
        kill $WEB_SERVER_PID 2>/dev/null || true
        echo "  - Web服务器已停止"
    fi
    
    # 清理端口占用
    for port in $API_PORT $WEBSOCKET_PORT $WEB_PORT; do
        local pid=$(lsof -t -i :$port 2>/dev/null || true)
        if [ ! -z "$pid" ]; then
            kill -9 $pid 2>/dev/null || true
        fi
    done
    
    echo -e "${GREEN}✅ 系统清理完成${NC}"
    exit 0
}

# 设置信号处理
trap cleanup SIGINT SIGTERM

# 清理端口占用函数
clear_port() {
    local port=$1
    local service_name=$2
    
    if lsof -i :$port > /dev/null 2>&1; then
        echo -e "${YELLOW}⚠️ 端口 $port 被占用，清理中...${NC}"
        local pid=$(lsof -t -i :$port 2>/dev/null || true)
        if [ ! -z "$pid" ]; then
            kill -9 $pid 2>/dev/null || true
            sleep 1
            echo -e "${GREEN}✅ 端口 $port 已清理${NC}"
        fi
    fi
}

# 检查依赖
check_dependencies() {
    echo -e "${BLUE}🔍 检查系统依赖...${NC}"
    
    local missing_deps=()
    
    # 检查Python依赖
    if ! python3 -c "import fastapi, uvicorn, websockets" 2>/dev/null; then
        missing_deps+=("Python Web依赖")
    fi
    
    # 检查文件存在性
    if [ ! -f "robot_api_server.py" ]; then
        missing_deps+=("API服务器脚本")
    fi
    
    if [ ! -f "ros2_web_bridge.py" ]; then
        missing_deps+=("WebSocket服务脚本")
    fi
    
    if [ ! -d "web_interface" ]; then
        missing_deps+=("Web界面目录")
    fi
    
    if [ ${#missing_deps[@]} -gt 0 ]; then
        echo -e "${RED}❌ 缺少依赖: ${missing_deps[*]}${NC}"
        exit 1
    fi
    
    echo -e "${GREEN}✅ 所有依赖检查通过${NC}"
}

# 设置ROS2环境
setup_ros2_environment() {
    echo -e "${BLUE}🤖 设置ROS2环境...${NC}"
    
    # 设置ROS2环境
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source install/setup.bash 2>/dev/null || true
    
    echo -e "${GREEN}✅ ROS2环境已设置${NC}"
}

# 启动Web服务器
start_web_server() {
    echo -e "${BLUE}🌐 启动Web服务器...${NC}"
    
    clear_port $WEB_PORT "Web服务器"
    
    if [ -d "$WEB_INTERFACE_DIR" ]; then
        cd "$WEB_INTERFACE_DIR"
        python3 -m http.server $WEB_PORT > /dev/null 2>&1 &
        WEB_SERVER_PID=$!
        cd "$WORKSPACE_DIR"
        
        sleep 2
        
        if kill -0 $WEB_SERVER_PID 2>/dev/null; then
            echo -e "${GREEN}✅ Web服务器已启动 (端口: $WEB_PORT)${NC}"
            echo -e "${CYAN}   访问地址: http://$HOTSPOT_IP:$WEB_PORT${NC}"
        else
            echo -e "${RED}❌ Web服务器启动失败${NC}"
        fi
    else
        echo -e "${YELLOW}⚠️ Web界面目录不存在: $WEB_INTERFACE_DIR${NC}"
    fi
}

# 启动API服务器
start_api_server() {
    echo -e "${BLUE}🔌 启动API服务器...${NC}"
    
    clear_port $API_PORT "API服务器"
    
    python3 robot_api_server.py > /tmp/api_server.log 2>&1 &
    API_PID=$!
    
    sleep 3
    
    if kill -0 $API_PID 2>/dev/null; then
        echo -e "${GREEN}✅ API服务器已启动 (端口: $API_PORT)${NC}"
        echo -e "${CYAN}   API地址: http://$HOTSPOT_IP:$API_PORT${NC}"
        
        # 测试API接口
        if curl -s http://localhost:$API_PORT/ > /dev/null; then
            echo -e "${GREEN}✅ API接口响应正常${NC}"
        else
            echo -e "${YELLOW}⚠️ API接口暂时无响应${NC}"
        fi
    else
        echo -e "${RED}❌ API服务器启动失败${NC}"
        if [ -f /tmp/api_server.log ]; then
            echo "错误日志:"
            tail -5 /tmp/api_server.log
        fi
    fi
}

# 启动WebSocket服务
start_websocket_service() {
    echo -e "${BLUE}🔗 启动WebSocket服务...${NC}"
    
    clear_port $WEBSOCKET_PORT "WebSocket服务"
    
    python3 ros2_web_bridge.py > /tmp/websocket.log 2>&1 &
    WEBSOCKET_PID=$!
    
    sleep 3
    
    if kill -0 $WEBSOCKET_PID 2>/dev/null; then
        echo -e "${GREEN}✅ WebSocket服务已启动 (端口: $WEBSOCKET_PORT)${NC}"
        echo -e "${CYAN}   WebSocket地址: ws://$HOTSPOT_IP:$WEBSOCKET_PORT${NC}"
        
        # 检查端口监听
        if lsof -i :$WEBSOCKET_PORT > /dev/null 2>&1; then
            echo -e "${GREEN}✅ WebSocket端口监听正常${NC}"
        else
            echo -e "${YELLOW}⚠️ WebSocket端口未监听${NC}"
        fi
    else
        echo -e "${RED}❌ WebSocket服务启动失败${NC}"
        if [ -f /tmp/websocket.log ]; then
            echo "错误日志:"
            tail -5 /tmp/websocket.log
        fi
    fi
}

# 显示系统信息
show_system_info() {
    echo ""
    echo -e "${PURPLE}🎉 Robot Studio 系统启动完成！${NC}"
    echo -e "${PURPLE}================================${NC}"
    echo ""
    echo -e "${CYAN}📱 连接信息：${NC}"
    echo -e "   WiFi网络: $HOTSPOT_SSID"
    echo -e "   网关地址: $HOTSPOT_IP"
    echo ""
    echo -e "${CYAN}🌐 Web服务：${NC}"
    echo -e "   控制界面: http://$HOTSPOT_IP:$WEB_PORT"
    echo -e "   API服务:  http://$HOTSPOT_IP:$API_PORT"
    echo -e "   WebSocket: ws://$HOTSPOT_IP:$WEBSOCKET_PORT"
    echo ""
    echo -e "${CYAN}🤖 ROS2服务：${NC}"
    echo -e "   路径规划: ./start_path_planning.sh"
    echo -e "   点云处理: ./robotstudio/aurora_project/complete_processing.sh"
    echo ""
    echo -e "${CYAN}📋 可用功能：${NC}"
    echo -e "   ✅ 机器人远程控制"
    echo -e "   ✅ 实时地图显示"
    echo -e "   ✅ 激光雷达可视化"
    echo -e "   ✅ 点云数据处理"
    echo -e "   ✅ 地图保存和导出"
    echo -e "   ✅ 文件管理"
    echo ""
    echo -e "${YELLOW}💡 使用说明：${NC}"
    echo -e "   1. 连接WiFi: $HOTSPOT_SSID (如果已配置)"
    echo -e "   2. 打开浏览器访问: http://$HOTSPOT_IP:$WEB_PORT"
    echo -e "   3. 或使用Android应用连接到: $HOTSPOT_IP"
    echo -e "   4. 按 Ctrl+C 停止所有服务"
    echo ""
}

# 监控系统状态
monitor_system() {
    echo -e "${BLUE}📊 系统监控已启动...${NC}"
    echo -e "${YELLOW}按 Ctrl+C 停止系统${NC}"
    echo ""
    
    while true; do
        sleep 10
        
        # 检查关键进程
        if [ ! -z "$API_PID" ] && ! kill -0 $API_PID 2>/dev/null; then
            echo -e "${RED}⚠️ API服务器进程已停止${NC}"
            API_PID=""
        fi
        
        if [ ! -z "$WEBSOCKET_PID" ] && ! kill -0 $WEBSOCKET_PID 2>/dev/null; then
            echo -e "${RED}⚠️ WebSocket服务进程已停止${NC}"
            WEBSOCKET_PID=""
        fi
        
        if [ ! -z "$WEB_SERVER_PID" ] && ! kill -0 $WEB_SERVER_PID 2>/dev/null; then
            echo -e "${RED}⚠️ Web服务器进程已停止${NC}"
            WEB_SERVER_PID=""
        fi
    done
}

# 主执行流程
main() {
    echo -e "${BLUE}开始启动 Robot Studio 系统...${NC}"
    echo ""
    
    # 1. 检查依赖
    check_dependencies
    echo ""
    
    # 2. 设置ROS2环境
    setup_ros2_environment
    echo ""
    
    # 3. 启动Web服务器
    start_web_server
    echo ""
    
    # 4. 启动API服务器
    start_api_server
    echo ""
    
    # 5. 启动WebSocket服务
    start_websocket_service
    echo ""
    
    # 6. 显示系统信息
    show_system_info
    
    # 7. 监控系统
    monitor_system
}

# 运行主程序
main
