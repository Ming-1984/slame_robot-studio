#!/bin/bash

# Robot Studio 开机自启安装脚本
# 安装WiFi热点和Robot Studio为系统服务

set -e

# 系统密码（用于无人值守环境；如需修改请同步更新）
SYSTEM_PASSWORD="yahboom"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
NC='\033[0m'

echo -e "${PURPLE}📦 Robot Studio 开机自启安装器${NC}"
echo -e "${PURPLE}==============================${NC}"
echo ""

# 检查root权限，如果没有则自动获取
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}⚠️ 需要root权限安装系统服务，正在获取权限...${NC}"
    echo "$SYSTEM_PASSWORD" | sudo -S "$0" "$@"
    exit $?
fi

WORKSPACE_DIR="/home/jetson/ros2_ws"
ORIGINAL_USER="jetson"

echo -e "${BLUE}📋 安装信息：${NC}"
echo "  工作目录: $WORKSPACE_DIR"
echo "  用户: $ORIGINAL_USER"
echo ""

# 1. 安装WiFi热点修复脚本
echo -e "${BLUE}1️⃣ 安装WiFi热点修复脚本...${NC}"
cp "$WORKSPACE_DIR/fix_wifi_hotspot.sh" /usr/local/bin/
chmod +x /usr/local/bin/fix_wifi_hotspot.sh
echo -e "${GREEN}✅ WiFi热点修复脚本已安装${NC}"

# 2. 创建WiFi热点服务
echo -e "${BLUE}2️⃣ 创建WiFi热点服务...${NC}"
cat > /etc/systemd/system/robot-wifi-hotspot.service << EOF
[Unit]
Description=Robot Studio WiFi Hotspot
After=network.target
Wants=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/fix_wifi_hotspot.sh
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# 3. 创建Robot Studio启动脚本
echo -e "${BLUE}3️⃣ 创建Robot Studio启动脚本...${NC}"
cat > /usr/local/bin/start-robot-studio-services.sh << EOF
#!/bin/bash

# Robot Studio 服务启动脚本
WORKSPACE_DIR="$WORKSPACE_DIR"
API_PORT=8000
WEBSOCKET_PORT=8001
WEB_PORT=8080

# 日志函数
log_message() {
    echo "[\$(date '+%Y-%m-%d %H:%M:%S')] \$1" | tee -a /var/log/robot-studio.log
}

log_message "启动Robot Studio服务"

# 切换到工作目录
cd "\$WORKSPACE_DIR"

# 设置ROS2环境
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash 2>/dev/null || true
source install/setup.bash 2>/dev/null || true

# 清理端口占用
for port in \$API_PORT \$WEBSOCKET_PORT \$WEB_PORT; do
    pid=\$(lsof -t -i :\$port 2>/dev/null || true)
    if [ ! -z "\$pid" ]; then
        kill -9 \$pid 2>/dev/null || true
        log_message "清理端口 \$port (PID: \$pid)"
    fi
done

# 等待网络就绪
sleep 5

# 启动Web服务器
cd web_interface
sudo -u $ORIGINAL_USER python3 -m http.server \$WEB_PORT > /dev/null 2>&1 &
WEB_PID=\$!
cd ..
echo \$WEB_PID > /tmp/robot_studio_web.pid
log_message "Web服务器启动 (PID: \$WEB_PID)"

# 启动API服务器
sudo -u $ORIGINAL_USER python3 robot_api_server.py > /tmp/api_server.log 2>&1 &
API_PID=\$!
echo \$API_PID > /tmp/robot_studio_api.pid
log_message "API服务器启动 (PID: \$API_PID)"

# 启动WebSocket服务
sudo -u $ORIGINAL_USER python3 ros2_web_bridge.py > /tmp/websocket.log 2>&1 &
WS_PID=\$!
echo \$WS_PID > /tmp/robot_studio_ws.pid
log_message "WebSocket服务启动 (PID: \$WS_PID)"

# 等待服务启动
sleep 5

# 检查服务状态
services_ok=0
if kill -0 \$WEB_PID 2>/dev/null; then
    log_message "Web服务器运行正常"
    ((services_ok++))
fi

if kill -0 \$API_PID 2>/dev/null; then
    log_message "API服务器运行正常"
    ((services_ok++))
fi

if kill -0 \$WS_PID 2>/dev/null; then
    log_message "WebSocket服务运行正常"
    ((services_ok++))
fi

log_message "Robot Studio服务启动完成 (\$services_ok/3 服务正常)"

# 显示访问信息
log_message "访问地址: http://192.168.4.1:8080"
EOF

chmod +x /usr/local/bin/start-robot-studio-services.sh

# 4. 创建Robot Studio服务
echo -e "${BLUE}4️⃣ 创建Robot Studio服务...${NC}"
cat > /etc/systemd/system/robot-studio.service << EOF
[Unit]
Description=Robot Studio Services
After=robot-wifi-hotspot.service network.target
Wants=robot-wifi-hotspot.service network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/start-robot-studio-services.sh
RemainAfterExit=yes
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# 5. 创建监控服务
echo -e "${BLUE}5️⃣ 创建监控服务...${NC}"
cat > /usr/local/bin/robot-studio-monitor.sh << 'EOF'
#!/bin/bash

# Robot Studio 监控脚本
while true; do
    # 检查WiFi热点
    if ! pgrep hostapd > /dev/null; then
        echo "$(date): hostapd停止，重启WiFi热点服务"
        systemctl restart robot-wifi-hotspot
    fi
    
    # 检查Robot Studio服务
    if [ -f /tmp/robot_studio_api.pid ]; then
        api_pid=$(cat /tmp/robot_studio_api.pid)
        if ! kill -0 $api_pid 2>/dev/null; then
            echo "$(date): API服务器停止，重启Robot Studio服务"
            systemctl restart robot-studio
        fi
    fi
    
    sleep 30
done
EOF

chmod +x /usr/local/bin/robot-studio-monitor.sh

cat > /etc/systemd/system/robot-studio-monitor.service << EOF
[Unit]
Description=Robot Studio Monitor
After=robot-studio.service
Wants=robot-studio.service

[Service]
Type=simple
ExecStart=/usr/local/bin/robot-studio-monitor.sh
Restart=always
RestartSec=10
StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# 6. 重新加载systemd并启用服务
echo -e "${BLUE}6️⃣ 启用系统服务...${NC}"
systemctl daemon-reload

# 启用服务
systemctl enable robot-wifi-hotspot.service
systemctl enable robot-studio.service
systemctl enable robot-studio-monitor.service

echo -e "${GREEN}✅ 所有服务已启用${NC}"

# 7. 创建管理脚本
echo -e "${BLUE}7️⃣ 创建管理脚本...${NC}"
cat > /usr/local/bin/robot-studio-control << 'EOF'
#!/bin/bash

# Robot Studio 控制脚本
case "$1" in
    start)
        echo "启动Robot Studio系统..."
        systemctl start robot-wifi-hotspot
        sleep 5
        systemctl start robot-studio
        systemctl start robot-studio-monitor
        echo "系统启动完成"
        ;;
    stop)
        echo "停止Robot Studio系统..."
        systemctl stop robot-studio-monitor
        systemctl stop robot-studio
        systemctl stop robot-wifi-hotspot
        echo "系统已停止"
        ;;
    restart)
        echo "重启Robot Studio系统..."
        $0 stop
        sleep 3
        $0 start
        ;;
    status)
        echo "=== Robot Studio 系统状态 ==="
        echo "WiFi热点服务:"
        systemctl is-active robot-wifi-hotspot || echo "未运行"
        echo "Robot Studio服务:"
        systemctl is-active robot-studio || echo "未运行"
        echo "监控服务:"
        systemctl is-active robot-studio-monitor || echo "未运行"
        echo ""
        echo "=== 进程状态 ==="
        echo "hostapd: $(pgrep hostapd > /dev/null && echo "运行中" || echo "未运行")"
        echo "dnsmasq: $(pgrep dnsmasq > /dev/null && echo "运行中" || echo "未运行")"
        echo "API服务器: $(pgrep -f robot_api_server > /dev/null && echo "运行中" || echo "未运行")"
        echo "WebSocket: $(pgrep -f ros2_web_bridge > /dev/null && echo "运行中" || echo "未运行")"
        echo ""
        echo "=== 网络状态 ==="
        if ip addr show wlP1p1s0 | grep -q "192.168.4.1"; then
            echo "WiFi接口: 已配置"
        else
            echo "WiFi接口: 未配置"
        fi
        ;;
    logs)
        echo "=== 系统日志 ==="
        journalctl -u robot-wifi-hotspot -u robot-studio -u robot-studio-monitor --no-pager -n 20
        ;;
    *)
        echo "Robot Studio 控制脚本"
        echo ""
        echo "用法: $0 {start|stop|restart|status|logs}"
        echo ""
        echo "命令:"
        echo "  start   - 启动所有服务"
        echo "  stop    - 停止所有服务"
        echo "  restart - 重启所有服务"
        echo "  status  - 查看系统状态"
        echo "  logs    - 查看系统日志"
        echo ""
        exit 1
        ;;
esac
EOF

chmod +x /usr/local/bin/robot-studio-control

# 8. 显示安装结果
echo ""
echo -e "${PURPLE}🎉 Robot Studio 开机自启安装完成！${NC}"
echo -e "${PURPLE}================================${NC}"
echo ""
echo -e "${CYAN}📋 已安装的服务：${NC}"
echo -e "   ✅ robot-wifi-hotspot.service - WiFi热点服务"
echo -e "   ✅ robot-studio.service - Robot Studio主服务"
echo -e "   ✅ robot-studio-monitor.service - 监控服务"
echo ""
echo -e "${CYAN}🎮 管理命令：${NC}"
echo -e "   启动系统: robot-studio-control start"
echo -e "   停止系统: robot-studio-control stop"
echo -e "   重启系统: robot-studio-control restart"
echo -e "   查看状态: robot-studio-control status"
echo -e "   查看日志: robot-studio-control logs"
echo ""
echo -e "${CYAN}📱 连接信息：${NC}"
echo -e "   WiFi名称: RobotStudio-AP"
echo -e "   WiFi密码: robotstudio123"
echo -e "   控制界面: http://192.168.4.1:8080"
echo ""
echo -e "${YELLOW}💡 下一步操作：${NC}"
echo -e "   1. 重启系统: sudo reboot"
echo -e "   2. 或立即启动: robot-studio-control start"
echo -e "   3. 检查状态: robot-studio-control status"
echo ""
echo -e "${GREEN}安装完成！系统将在下次重启后自动启动。${NC}"
