#!/bin/bash

# WiFi热点修复脚本
# 专门解决当前的WiFi热点问题

set -e

# 系统密码（用于无人值守环境；如需修改请同步更新）
SYSTEM_PASSWORD="yahboom"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🔧 WiFi热点修复工具${NC}"
echo "======================"
echo ""

# 检查root权限，如果没有则自动获取
if [ "$EUID" -ne 0 ]; then
    echo -e "${YELLOW}⚠️ 需要root权限，正在获取权限...${NC}"
    echo "$SYSTEM_PASSWORD" | sudo -S "$0" "$@"
    exit $?
fi

# 配置参数
WIFI_INTERFACE="wlP1p1s0"
HOTSPOT_SSID="RobotStudio-AP"
HOTSPOT_PASSWORD="robotstudio123"
HOTSPOT_IP="192.168.4.1"

echo -e "${BLUE}📋 配置信息：${NC}"
echo "  WiFi接口: $WIFI_INTERFACE"
echo "  热点名称: $HOTSPOT_SSID"
echo "  热点密码: $HOTSPOT_PASSWORD"
echo "  热点IP: $HOTSPOT_IP"
echo ""

# 1. 停止所有相关服务
echo -e "${BLUE}⏹️ 停止相关服务...${NC}"
systemctl stop hostapd 2>/dev/null || true
systemctl stop dnsmasq 2>/dev/null || true
systemctl stop NetworkManager 2>/dev/null || true
systemctl stop wpa_supplicant 2>/dev/null || true

# 2. 解除hostapd屏蔽
echo -e "${BLUE}🔓 解除hostapd屏蔽...${NC}"
systemctl unmask hostapd 2>/dev/null || true

# 3. 修复hostapd配置
echo -e "${BLUE}📝 修复hostapd配置...${NC}"
cat > /etc/hostapd/hostapd.conf << EOF
interface=$WIFI_INTERFACE
driver=nl80211
ssid=$HOTSPOT_SSID
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_passphrase=$HOTSPOT_PASSWORD
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
ieee80211n=1
ht_capab=[HT40][SHORT-GI-20][DSSS_CCK-40]
EOF

# 4. 修复dnsmasq配置
echo -e "${BLUE}🌐 修复dnsmasq配置...${NC}"
cat > /etc/dnsmasq.conf << EOF
interface=$WIFI_INTERFACE
dhcp-range=192.168.4.10,192.168.4.50,255.255.255.0,24h
server=8.8.8.8
server=8.8.4.4
domain-needed
bogus-priv
no-resolv
bind-interfaces
EOF

# 5. 配置网络接口
echo -e "${BLUE}🔧 配置网络接口...${NC}"
ip link set $WIFI_INTERFACE down 2>/dev/null || true
sleep 1
ip link set $WIFI_INTERFACE up
ip addr flush dev $WIFI_INTERFACE 2>/dev/null || true
ip addr add $HOTSPOT_IP/24 dev $WIFI_INTERFACE

# 6. 配置IP转发
echo -e "${BLUE}🔀 配置IP转发...${NC}"
echo 1 > /proc/sys/net/ipv4/ip_forward
echo 'net.ipv4.ip_forward=1' > /etc/sysctl.d/99-robot-studio.conf

# 7. 配置iptables
echo -e "${BLUE}🛡️ 配置防火墙...${NC}"
iptables -t nat -F 2>/dev/null || true
iptables -F FORWARD 2>/dev/null || true
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
iptables -A FORWARD -i eth0 -o $WIFI_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i $WIFI_INTERFACE -o eth0 -j ACCEPT

# 保存iptables规则
mkdir -p /etc/iptables
iptables-save > /etc/iptables/rules.v4

# 8. 手动启动hostapd
echo -e "${BLUE}📡 启动hostapd...${NC}"
# 先尝试systemd方式
if systemctl start hostapd 2>/dev/null; then
    echo -e "${GREEN}✅ hostapd通过systemd启动成功${NC}"
    HOSTAPD_METHOD="systemd"
else
    echo -e "${YELLOW}⚠️ systemd启动失败，尝试手动启动...${NC}"
    # 手动启动
    hostapd /etc/hostapd/hostapd.conf -B 2>/dev/null && {
        echo -e "${GREEN}✅ hostapd手动启动成功${NC}"
        HOSTAPD_METHOD="manual"
    } || {
        echo -e "${RED}❌ hostapd启动失败${NC}"
        HOSTAPD_METHOD="failed"
    }
fi

# 9. 启动dnsmasq
echo -e "${BLUE}🌐 启动dnsmasq...${NC}"
if systemctl start dnsmasq 2>/dev/null; then
    echo -e "${GREEN}✅ dnsmasq通过systemd启动成功${NC}"
    DNSMASQ_METHOD="systemd"
else
    echo -e "${YELLOW}⚠️ systemd启动失败，尝试手动启动...${NC}"
    # 手动启动
    dnsmasq --conf-file=/etc/dnsmasq.conf 2>/dev/null && {
        echo -e "${GREEN}✅ dnsmasq手动启动成功${NC}"
        DNSMASQ_METHOD="manual"
    } || {
        echo -e "${RED}❌ dnsmasq启动失败${NC}"
        DNSMASQ_METHOD="failed"
    }
fi

# 10. 检查服务状态
echo ""
echo -e "${BLUE}📊 检查服务状态...${NC}"

# 检查hostapd
if pgrep hostapd > /dev/null; then
    echo -e "${GREEN}✅ hostapd进程运行中${NC}"
else
    echo -e "${RED}❌ hostapd进程未运行${NC}"
fi

# 检查dnsmasq
if pgrep dnsmasq > /dev/null; then
    echo -e "${GREEN}✅ dnsmasq进程运行中${NC}"
else
    echo -e "${RED}❌ dnsmasq进程未运行${NC}"
fi

# 检查网络接口
if ip addr show $WIFI_INTERFACE | grep -q $HOTSPOT_IP; then
    echo -e "${GREEN}✅ 网络接口配置正确${NC}"
else
    echo -e "${RED}❌ 网络接口配置错误${NC}"
fi

# 11. 测试WiFi热点
echo ""
echo -e "${BLUE}🔍 测试WiFi热点...${NC}"
sleep 3

# 检查是否可以扫描到热点
if iwlist scan 2>/dev/null | grep -q "$HOTSPOT_SSID"; then
    echo -e "${GREEN}✅ 可以扫描到WiFi热点${NC}"
else
    echo -e "${YELLOW}⚠️ 暂时无法扫描到WiFi热点（可能需要等待）${NC}"
fi

# 12. 创建监控脚本
echo -e "${BLUE}📝 创建监控脚本...${NC}"
cat > /usr/local/bin/monitor-wifi-hotspot.sh << 'EOF'
#!/bin/bash

# WiFi热点监控脚本
WIFI_INTERFACE="wlP1p1s0"
HOTSPOT_IP="192.168.4.1"

while true; do
    # 检查hostapd
    if ! pgrep hostapd > /dev/null; then
        echo "$(date): hostapd停止，重启中..."
        hostapd /etc/hostapd/hostapd.conf -B 2>/dev/null || true
    fi
    
    # 检查dnsmasq
    if ! pgrep dnsmasq > /dev/null; then
        echo "$(date): dnsmasq停止，重启中..."
        dnsmasq --conf-file=/etc/dnsmasq.conf 2>/dev/null || true
    fi
    
    # 检查网络接口
    if ! ip addr show $WIFI_INTERFACE | grep -q $HOTSPOT_IP; then
        echo "$(date): 网络接口配置丢失，重新配置..."
        ip addr add $HOTSPOT_IP/24 dev $WIFI_INTERFACE 2>/dev/null || true
    fi
    
    sleep 30
done
EOF

chmod +x /usr/local/bin/monitor-wifi-hotspot.sh

# 13. 显示结果
echo ""
echo -e "${PURPLE}🎉 WiFi热点修复完成！${NC}"
echo -e "${PURPLE}====================${NC}"
echo ""
echo -e "${CYAN}📱 连接信息：${NC}"
echo -e "   WiFi名称: $HOTSPOT_SSID"
echo -e "   WiFi密码: $HOTSPOT_PASSWORD"
echo -e "   网关地址: $HOTSPOT_IP"
echo ""
echo -e "${CYAN}📊 服务状态：${NC}"
echo -e "   hostapd: $HOSTAPD_METHOD"
echo -e "   dnsmasq: $DNSMASQ_METHOD"
echo ""
echo -e "${YELLOW}💡 使用说明：${NC}"
echo -e "   1. 等待1-2分钟让热点完全启动"
echo -e "   2. 在手机/电脑上搜索WiFi: $HOTSPOT_SSID"
echo -e "   3. 连接后访问: http://$HOTSPOT_IP:8080"
echo ""
echo -e "${BLUE}🔧 管理命令：${NC}"
echo -e "   启动监控: nohup /usr/local/bin/monitor-wifi-hotspot.sh &"
echo -e "   检查进程: ps aux | grep -E 'hostapd|dnsmasq'"
echo -e "   查看日志: journalctl -u hostapd -u dnsmasq"
echo ""

# 14. 询问是否启动监控
echo -e "${YELLOW}是否启动WiFi热点监控？(y/N): ${NC}"
read -r start_monitor
if [[ "$start_monitor" =~ ^[Yy]$ ]]; then
    nohup /usr/local/bin/monitor-wifi-hotspot.sh > /var/log/wifi-hotspot-monitor.log 2>&1 &
    echo -e "${GREEN}✅ WiFi热点监控已启动${NC}"
    echo -e "   日志文件: /var/log/wifi-hotspot-monitor.log"
fi

echo ""
echo -e "${GREEN}修复完成！请尝试连接WiFi热点。${NC}"
