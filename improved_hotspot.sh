#!/bin/bash

# 改进的WiFi热点管理脚本
# 需要 sudo 权限（不再在脚本中硬编码密码）

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 配置参数
WIFI_INTERFACE="wlP1p1s0"
WIRED_INTERFACE="enP8p1s0"
HOTSPOT_SSID="RobotStudio"
HOTSPOT_IP="192.168.4.1"
HOTSPOT_NETWORK="192.168.4.0/24"
DHCP_RANGE="192.168.4.10,192.168.4.50"

IPTABLES_INPUT_CHAIN="RS_HOTSPOT_INPUT"
IPTABLES_FORWARD_CHAIN="RS_HOTSPOT_FORWARD"
IPTABLES_OUTPUT_CHAIN="RS_HOTSPOT_OUTPUT"
IPTABLES_NAT_CHAIN="RS_HOTSPOT_NAT"

# 日志函数
log() {
    echo -e "[$(date '+%Y-%m-%d %H:%M:%S')] $1"
}

# 执行sudo命令函数
run_sudo() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
        return $?
    fi

    if sudo -n true >/dev/null 2>&1; then
        sudo -n "$@"
        return $?
    fi

    if [ -t 0 ]; then
        sudo "$@"
        return $?
    fi

    echo -e "${RED}❌ 需要 sudo 权限: $*${NC}" >&2
    return 1
}

nm_is_active() {
    systemctl is-active --quiet NetworkManager 2>/dev/null
}

# 清理 RobotStudio 自己添加的 iptables 规则（不影响其他服务规则）
cleanup_hotspot_firewall_rules() {
    # 删除跳转规则（可能重复添加，因此循环删除直到不存在）
    while run_sudo iptables -D INPUT -j "$IPTABLES_INPUT_CHAIN" 2>/dev/null; do :; done
    while run_sudo iptables -D OUTPUT -j "$IPTABLES_OUTPUT_CHAIN" 2>/dev/null; do :; done
    while run_sudo iptables -D FORWARD -j "$IPTABLES_FORWARD_CHAIN" 2>/dev/null; do :; done
    while run_sudo iptables -t nat -D POSTROUTING -j "$IPTABLES_NAT_CHAIN" 2>/dev/null; do :; done

    # 清理并删除自定义链
    run_sudo iptables -F "$IPTABLES_INPUT_CHAIN" 2>/dev/null || true
    run_sudo iptables -X "$IPTABLES_INPUT_CHAIN" 2>/dev/null || true
    run_sudo iptables -F "$IPTABLES_OUTPUT_CHAIN" 2>/dev/null || true
    run_sudo iptables -X "$IPTABLES_OUTPUT_CHAIN" 2>/dev/null || true
    run_sudo iptables -F "$IPTABLES_FORWARD_CHAIN" 2>/dev/null || true
    run_sudo iptables -X "$IPTABLES_FORWARD_CHAIN" 2>/dev/null || true
    run_sudo iptables -t nat -F "$IPTABLES_NAT_CHAIN" 2>/dev/null || true
    run_sudo iptables -t nat -X "$IPTABLES_NAT_CHAIN" 2>/dev/null || true
}

# 配置热点的 NAT/转发规则：让热点侧(192.168.4.0/24)可以访问 Aurora 网段(192.168.11.0/24)
setup_hotspot_firewall_rules() {
    # 启用 IP 转发
    run_sudo sh -c "echo 1 > /proc/sys/net/ipv4/ip_forward"

    # 先清理旧规则，避免叠加
    cleanup_hotspot_firewall_rules

    # 创建自定义链
    run_sudo iptables -N "$IPTABLES_INPUT_CHAIN" 2>/dev/null || true
    run_sudo iptables -N "$IPTABLES_OUTPUT_CHAIN" 2>/dev/null || true
    run_sudo iptables -N "$IPTABLES_FORWARD_CHAIN" 2>/dev/null || true
    run_sudo iptables -t nat -N "$IPTABLES_NAT_CHAIN" 2>/dev/null || true

    # 将自定义链挂载到内置链最前面（避免被默认 DROP/其他规则拦截）
    run_sudo iptables -I INPUT 1 -j "$IPTABLES_INPUT_CHAIN"
    run_sudo iptables -I OUTPUT 1 -j "$IPTABLES_OUTPUT_CHAIN"
    run_sudo iptables -I FORWARD 1 -j "$IPTABLES_FORWARD_CHAIN"
    run_sudo iptables -t nat -I POSTROUTING 1 -j "$IPTABLES_NAT_CHAIN"

    # 本机访问放行（热点侧访问 Jetson 上的 Web/API/SSH 等）
    run_sudo iptables -A "$IPTABLES_INPUT_CHAIN" -i "$WIFI_INTERFACE" -s "$HOTSPOT_NETWORK" -j ACCEPT 2>/dev/null || true
    run_sudo iptables -A "$IPTABLES_OUTPUT_CHAIN" -o "$WIFI_INTERFACE" -d "$HOTSPOT_NETWORK" -j ACCEPT 2>/dev/null || true

    # 热点 -> 有线（Aurora）转发
    if [ "$WIRED_CONNECTED" = true ]; then
        # NAT：让 Aurora 看到的源地址是 192.168.11.100（Jetson），无需改 Aurora 路由
        run_sudo iptables -t nat -A "$IPTABLES_NAT_CHAIN" -s "$HOTSPOT_NETWORK" -o "$WIRED_INTERFACE" -j MASQUERADE 2>/dev/null || true
        # 转发放行
        run_sudo iptables -A "$IPTABLES_FORWARD_CHAIN" -i "$WIFI_INTERFACE" -o "$WIRED_INTERFACE" -s "$HOTSPOT_NETWORK" -j ACCEPT 2>/dev/null || true
        run_sudo iptables -A "$IPTABLES_FORWARD_CHAIN" -i "$WIRED_INTERFACE" -o "$WIFI_INTERFACE" -d "$HOTSPOT_NETWORK" -m conntrack --ctstate RELATED,ESTABLISHED -j ACCEPT 2>/dev/null || true
    fi
}

# 检查网络状态
check_network_status() {
    log "${BLUE}🔍 检查当前网络状态...${NC}"
    
    # 检查有线连接
    WIRED_CONNECTED=false
    if ip link show $WIRED_INTERFACE >/dev/null 2>&1; then
        if ip route | grep -q "dev $WIRED_INTERFACE"; then
            WIRED_CONNECTED=true
            WIRED_IP=$(ip addr show $WIRED_INTERFACE | grep "inet " | awk '{print $2}' | cut -d/ -f1)
            log "${GREEN}✅ 有线网络已连接: $WIRED_IP${NC}"
        fi
    fi
    
    # 检查WiFi状态
    WIFI_CONNECTED=false
    WIFI_MODE="disconnected"
    if ip link show $WIFI_INTERFACE >/dev/null 2>&1; then
        if iwconfig $WIFI_INTERFACE 2>/dev/null | grep -q "Mode:Managed"; then
            if ip route | grep -q "dev $WIFI_INTERFACE"; then
                WIFI_CONNECTED=true
                WIFI_MODE="client"
                WIFI_IP=$(ip addr show $WIFI_INTERFACE | grep "inet " | awk '{print $2}' | cut -d/ -f1)
                CURRENT_SSID=$(iwconfig $WIFI_INTERFACE | grep -o 'ESSID:"[^"]*"' | cut -d'"' -f2)
                log "${GREEN}✅ WiFi客户端已连接: $WIFI_IP (SSID: $CURRENT_SSID)${NC}"
            fi
        elif iwconfig $WIFI_INTERFACE 2>/dev/null | grep -q "Mode:Master"; then
            WIFI_MODE="hotspot"
            log "${GREEN}✅ WiFi热点模式已启用${NC}"
        fi
    fi
}

# 启用WiFi热点
enable_wifi_hotspot() {
    log "${PURPLE}📡 启用WiFi热点模式...${NC}"
    
    # 1. 停止冲突服务
    log "${YELLOW}1. 停止冲突服务...${NC}"
    # 关键点：不要直接停 NetworkManager（它也管理有线 enP8p1s0，停了会导致 Aurora 网段丢IP）
    # 这里改为让 NetworkManager 仅“放弃管理 WiFi 接口”，避免它自动切回客户端模式覆盖热点
    if command -v nmcli >/dev/null 2>&1 && nm_is_active; then
        run_sudo nmcli dev disconnect "$WIFI_INTERFACE" 2>/dev/null || true
        run_sudo nmcli dev set "$WIFI_INTERFACE" managed no 2>/dev/null || true
    else
        # 兜底：如果 nmcli 不可用或 NetworkManager 不在运行，则停掉 NetworkManager 避免干扰
        run_sudo systemctl stop NetworkManager 2>/dev/null || true
        run_sudo systemctl mask NetworkManager 2>/dev/null || true
    fi

    run_sudo pkill -f hostapd 2>/dev/null || true
    run_sudo pkill -f dnsmasq 2>/dev/null || true
    sleep 2
    
    # 2. 断开WiFi客户端连接（如果有）
    if [ "$WIFI_CONNECTED" = true ]; then
        log "${YELLOW}2. 断开WiFi客户端连接...${NC}"
        run_sudo wpa_cli -i $WIFI_INTERFACE disconnect 2>/dev/null || true
        sleep 2
    fi
    
    # 3. 配置WiFi接口为热点模式
    log "${YELLOW}3. 配置WiFi接口...${NC}"
    run_sudo ip link set $WIFI_INTERFACE down
    sleep 1
    run_sudo ip link set $WIFI_INTERFACE up
    run_sudo ip addr flush dev $WIFI_INTERFACE
    run_sudo ip addr add $HOTSPOT_IP/24 dev $WIFI_INTERFACE
    
    # 等待接口稳定
    sleep 2
    
    # 4. 配置hostapd（简化配置以提高兼容性）
    log "${YELLOW}4. 配置hostapd...${NC}"
    run_sudo tee /etc/hostapd/hostapd.conf > /dev/null << EOF
interface=$WIFI_INTERFACE
driver=nl80211
ssid=$HOTSPOT_SSID
hw_mode=g
channel=6
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=0
EOF

    # 5. 配置dnsmasq
    log "${YELLOW}5. 配置dnsmasq...${NC}"
    run_sudo tee /etc/dnsmasq.conf > /dev/null << EOF
# WiFi热点DHCP配置
interface=$WIFI_INTERFACE
dhcp-range=$DHCP_RANGE,255.255.255.0,24h
dhcp-option=1,255.255.255.0
dhcp-option=28,192.168.4.255
# 兼容 Windows：下发 Aurora 网段路由（即使客户端子网掩码异常也能走网关）
dhcp-option=option:classless-static-route,192.168.11.0/24,$HOTSPOT_IP
dhcp-option=249,192.168.11.0/24,$HOTSPOT_IP
server=8.8.8.8
server=8.8.4.4
bind-interfaces
dhcp-option=3,$HOTSPOT_IP
dhcp-option=6,$HOTSPOT_IP

# DNS解析
address=/robot/$HOTSPOT_IP
address=/robot.local/$HOTSPOT_IP
address=/robotstudio/$HOTSPOT_IP
address=/robotstudio.local/$HOTSPOT_IP

# 日志
log-queries
log-dhcp
EOF

    # 6. 启动hostapd
    log "${YELLOW}6. 启动hostapd...${NC}"
    run_sudo hostapd /etc/hostapd/hostapd.conf -B
    sleep 3
    
    if ! pgrep hostapd > /dev/null; then
        log "${RED}❌ hostapd启动失败${NC}"
        return 1
    fi
    
    # 7. 启动dnsmasq
    log "${YELLOW}7. 启动dnsmasq...${NC}"
    run_sudo dnsmasq --conf-file=/etc/dnsmasq.conf
    sleep 2
    
    # 8. 配置防火墙和网络转发
    log "${YELLOW}8. 配置网络转发和防火墙...${NC}"
    setup_hotspot_firewall_rules
    
    # 9. 验证服务状态
    log "${YELLOW}9. 验证服务状态...${NC}"
    sleep 3
    
    # 确保热点IP地址正确配置（防止被NetworkManager覆盖）
    run_sudo ip addr add $HOTSPOT_IP/24 dev $WIFI_INTERFACE 2>/dev/null || true
    
    if pgrep hostapd > /dev/null && pgrep dnsmasq > /dev/null; then
        log "${GREEN}✅ WiFi热点启动成功！${NC}"
        log "${GREEN}   SSID: $HOTSPOT_SSID (无密码)${NC}"
        log "${GREEN}   IP: $HOTSPOT_IP${NC}"
        log "${GREEN}   网络: $HOTSPOT_NETWORK${NC}"
        log "${GREEN}   Web访问: http://robot 或 http://$HOTSPOT_IP:8080${NC}"
        log "${GREEN}   API访问: http://$HOTSPOT_IP:8000${NC}"
        log "${GREEN}   WebSocket: ws://$HOTSPOT_IP:8001${NC}"
        
        # 显示当前IP配置确认
        CURRENT_IP=$(ip addr show $WIFI_INTERFACE | grep "inet 192.168.4" | awk '{print $2}' | cut -d/ -f1)
        if [ "$CURRENT_IP" = "$HOTSPOT_IP" ]; then
            log "${GREEN}   ✅ 热点IP地址配置正确: $CURRENT_IP${NC}"
        else
            log "${YELLOW}   ⚠️  IP地址可能有问题，当前: $CURRENT_IP${NC}"
        fi
        
        return 0
    else
        log "${RED}❌ WiFi热点启动失败${NC}"
        return 1
    fi
}

# 停止WiFi热点
stop_wifi_hotspot() {
    log "${PURPLE}🛑 停止WiFi热点...${NC}"
    
    # 停止服务
    run_sudo pkill -f hostapd 2>/dev/null || true
    run_sudo pkill -f dnsmasq 2>/dev/null || true
    
    # 清理网络配置
    run_sudo ip addr flush dev $WIFI_INTERFACE 2>/dev/null || true
    cleanup_hotspot_firewall_rules
    
    # 恢复NetworkManager
    if command -v nmcli >/dev/null 2>&1; then
        # 如果 NetworkManager 仍在运行，则恢复对 WiFi 的管理
        if nm_is_active; then
            run_sudo nmcli dev set "$WIFI_INTERFACE" managed yes 2>/dev/null || true
            run_sudo nmcli dev connect "$WIFI_INTERFACE" 2>/dev/null || true
        else
            run_sudo systemctl unmask NetworkManager 2>/dev/null || true
            run_sudo systemctl start NetworkManager 2>/dev/null || true
        fi
    else
        run_sudo systemctl unmask NetworkManager 2>/dev/null || true
        run_sudo systemctl start NetworkManager 2>/dev/null || true
    fi
    
    log "${GREEN}✅ WiFi热点已停止${NC}"
}

# 检查热点状态
check_hotspot_status() {
    if pgrep hostapd > /dev/null && pgrep dnsmasq > /dev/null; then
        log "${GREEN}✅ WiFi热点运行中${NC}"

        # 额外一致性检查：避免出现 hostapd/dnsmasq 在跑但 WiFi 又被 NetworkManager 切回“已管理/已连接”导致热点被覆盖
        if command -v nmcli >/dev/null 2>&1 && nm_is_active; then
            NM_DEV_STATE=$(nmcli -t -f DEVICE,STATE device status 2>/dev/null | awk -F: -v dev="$WIFI_INTERFACE" '$1==dev {print $2}' | head -n 1)
            if [ -n "$NM_DEV_STATE" ] && [ "$NM_DEV_STATE" != "unmanaged" ]; then
                log "${YELLOW}⚠️  NetworkManager 正在管理 $WIFI_INTERFACE (state=$NM_DEV_STATE)，可能会覆盖热点模式/热点IP，导致客户端无法访问 Aurora(192.168.11.1)${NC}"
                log "${YELLOW}   建议执行: sudo ./improved_hotspot.sh fix-routing（不重启热点，仅修复路由/NAT）${NC}"
            fi
        fi

        CURRENT_HOTSPOT_IP=$(ip addr show $WIFI_INTERFACE 2>/dev/null | grep "inet " | awk '{print $2}' | cut -d/ -f1 | grep -E "^192\\.168\\.4\\." | head -n 1)
        if [ "$WIFI_MODE" != "hotspot" ] || [ "$CURRENT_HOTSPOT_IP" != "$HOTSPOT_IP" ]; then
            log "${YELLOW}⚠️  热点状态不一致：当前 $WIFI_INTERFACE 模式=$WIFI_MODE，IP=${CURRENT_HOTSPOT_IP:-none}（期望：模式=hotspot，IP=$HOTSPOT_IP）${NC}"
            log "${YELLOW}   建议执行: sudo ./improved_hotspot.sh restart（会断开当前WiFi客户端连接）${NC}"
        fi
        
        # 显示连接的客户端
        CLIENTS=$(run_sudo iw dev $WIFI_INTERFACE station dump 2>/dev/null | grep Station | wc -l)
        log "${CYAN}📱 已连接客户端: $CLIENTS${NC}"
        
        return 0
    else
        log "${RED}❌ WiFi热点未运行${NC}"
        return 1
    fi
}

# 主函数
main() {
    case "$1" in
        "start")
            check_network_status
            enable_wifi_hotspot
            ;;
        "stop")
            stop_wifi_hotspot
            ;;
        "fix-routing")
            check_network_status
            if pgrep hostapd > /dev/null && pgrep dnsmasq > /dev/null; then
                log "${PURPLE}🔧 修复热点到Aurora的路由/NAT（不中断热点）...${NC}"
                setup_hotspot_firewall_rules
                log "${GREEN}✅ 路由/NAT修复完成${NC}"
            else
                log "${RED}❌ 当前未检测到热点进程(hostapd/dnsmasq)，请先启动热点再执行 fix-routing${NC}"
                exit 1
            fi
            ;;
        "status")
            check_network_status
            check_hotspot_status
            ;;
        "restart")
            stop_wifi_hotspot
            sleep 3
            check_network_status
            enable_wifi_hotspot
            ;;
        *)
            echo "使用方法: $0 {start|stop|status|restart}"
            echo "          $0 {fix-routing}"
            echo ""
            echo "命令："
            echo "  start   - 启动WiFi热点"
            echo "  stop    - 停止WiFi热点"
            echo "  status  - 检查热点状态"
            echo "  restart - 重启WiFi热点"
            echo "  fix-routing - 修复热点路由/NAT（不中断热点）"
            exit 1
            ;;
    esac
}

# 运行主函数
main "$@"
