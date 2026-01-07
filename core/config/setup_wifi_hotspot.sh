#!/bin/bash

# WiFi热点配置脚本 - Jetson Nano
# 创建一个WiFi接入点，供PC和Android设备连接

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🔧 Jetson Nano WiFi热点配置工具${NC}"
echo "========================================"

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}❌ 请以root权限运行此脚本${NC}"
    echo "使用: sudo $0"
    exit 1
fi

# 配置参数
WIFI_INTERFACE="wlP1p1s0"
HOTSPOT_SSID="RobotStudio-AP"
HOTSPOT_PASSWORD="robotstudio123"
HOTSPOT_IP="192.168.4.1"
DHCP_RANGE_START="192.168.4.10"
DHCP_RANGE_END="192.168.4.50"

echo -e "${BLUE}📋 配置参数：${NC}"
echo "  WiFi接口: $WIFI_INTERFACE"
echo "  热点名称: $HOTSPOT_SSID"
echo "  热点密码: $HOTSPOT_PASSWORD"
echo "  热点IP: $HOTSPOT_IP"
echo "  DHCP范围: $DHCP_RANGE_START - $DHCP_RANGE_END"
echo ""

# 检查WiFi接口
echo -e "${BLUE}🔍 检查WiFi接口...${NC}"
if ! ip link show $WIFI_INTERFACE > /dev/null 2>&1; then
    echo -e "${RED}❌ WiFi接口 $WIFI_INTERFACE 不存在${NC}"
    echo "可用的网络接口："
    ip link show | grep -E "^[0-9]+:" | cut -d: -f2 | tr -d ' '
    exit 1
fi
echo -e "${GREEN}✅ WiFi接口 $WIFI_INTERFACE 存在${NC}"

# 检查必要的软件包
echo -e "${BLUE}📦 检查必要的软件包...${NC}"
if ! command -v hostapd &> /dev/null; then
    echo -e "${YELLOW}安装hostapd...${NC}"
    apt install -y hostapd
fi
if ! command -v dnsmasq &> /dev/null; then
    echo -e "${YELLOW}安装dnsmasq...${NC}"
    apt install -y dnsmasq
fi

# 停止服务
echo -e "${BLUE}⏹️ 停止相关服务...${NC}"
systemctl stop hostapd || true
systemctl stop dnsmasq || true
systemctl stop NetworkManager || true

# 配置静态IP
echo -e "${BLUE}🌐 配置静态IP...${NC}"
cat > /etc/dhcpcd.conf.backup << EOF
# 备份原始配置
$(cat /etc/dhcpcd.conf 2>/dev/null || echo "# 原始文件不存在")
EOF

cat >> /etc/dhcpcd.conf << EOF

# WiFi热点静态IP配置
interface $WIFI_INTERFACE
static ip_address=$HOTSPOT_IP/24
nohook wpa_supplicant
EOF

# 配置dnsmasq
echo -e "${BLUE}🔧 配置DHCP服务器 (dnsmasq)...${NC}"
cp /etc/dnsmasq.conf /etc/dnsmasq.conf.backup 2>/dev/null || true

cat > /etc/dnsmasq.conf << EOF
# WiFi热点DHCP配置
interface=$WIFI_INTERFACE
dhcp-range=$DHCP_RANGE_START,$DHCP_RANGE_END,255.255.255.0,24h

# DNS设置
server=8.8.8.8
server=8.8.4.4

# 日志
log-queries
log-dhcp

# 其他设置
domain-needed
bogus-priv
EOF

# 配置hostapd
echo -e "${BLUE}📡 配置WiFi接入点 (hostapd)...${NC}"
cat > /etc/hostapd/hostapd.conf << EOF
# WiFi热点配置
interface=$WIFI_INTERFACE
driver=nl80211

# 网络设置
ssid=$HOTSPOT_SSID
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0

# 安全设置
wpa=2
wpa_passphrase=$HOTSPOT_PASSWORD
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP

# 性能优化
ieee80211n=1
ht_capab=[HT40][SHORT-GI-20][DSSS_CCK-40]
EOF

# 配置hostapd默认文件
echo 'DAEMON_CONF="/etc/hostapd/hostapd.conf"' > /etc/default/hostapd

# 配置IP转发
echo -e "${BLUE}🔀 配置IP转发...${NC}"
echo 'net.ipv4.ip_forward=1' >> /etc/sysctl.conf

# 配置iptables规则
echo -e "${BLUE}🛡️ 配置防火墙规则...${NC}"
iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
iptables -A FORWARD -i eth0 -o $WIFI_INTERFACE -m state --state RELATED,ESTABLISHED -j ACCEPT
iptables -A FORWARD -i $WIFI_INTERFACE -o eth0 -j ACCEPT

# 保存iptables规则
iptables-save > /etc/iptables/rules.v4

# 创建启动脚本
echo -e "${BLUE}📝 创建启动脚本...${NC}"
cat > /usr/local/bin/start-hotspot.sh << 'EOF'
#!/bin/bash

# WiFi热点启动脚本
WIFI_INTERFACE="wlan0"

echo "启动WiFi热点..."

# 停止NetworkManager对WiFi接口的管理
nmcli radio wifi off 2>/dev/null || true
rfkill unblock wlan 2>/dev/null || true

# 启动接口
ip link set $WIFI_INTERFACE up

# 启动服务
systemctl start dnsmasq
systemctl start hostapd

# 应用iptables规则
iptables-restore < /etc/iptables/rules.v4

echo "WiFi热点已启动"
echo "SSID: RobotStudio-AP"
echo "密码: robotstudio123"
echo "网关: 192.168.4.1"
EOF

chmod +x /usr/local/bin/start-hotspot.sh

# 创建停止脚本
cat > /usr/local/bin/stop-hotspot.sh << 'EOF'
#!/bin/bash

# WiFi热点停止脚本
echo "停止WiFi热点..."

systemctl stop hostapd
systemctl stop dnsmasq

echo "WiFi热点已停止"
EOF

chmod +x /usr/local/bin/stop-hotspot.sh

# 创建systemd服务
echo -e "${BLUE}⚙️ 创建systemd服务...${NC}"
cat > /etc/systemd/system/wifi-hotspot.service << EOF
[Unit]
Description=WiFi Hotspot Service
After=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/bin/start-hotspot.sh
ExecStop=/usr/local/bin/stop-hotspot.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

# 重新加载systemd
systemctl daemon-reload

# 启用服务（可选）
echo -e "${YELLOW}是否要启用WiFi热点自动启动？(y/N): ${NC}"
read -r enable_autostart
if [[ "$enable_autostart" =~ ^[Yy]$ ]]; then
    systemctl enable wifi-hotspot
    echo -e "${GREEN}✅ WiFi热点已设置为开机自启${NC}"
fi

echo ""
echo -e "${GREEN}🎉 WiFi热点配置完成！${NC}"
echo ""
echo -e "${BLUE}📋 使用说明：${NC}"
echo "  启动热点: sudo systemctl start wifi-hotspot"
echo "  停止热点: sudo systemctl stop wifi-hotspot"
echo "  或者使用: sudo /usr/local/bin/start-hotspot.sh"
echo "           sudo /usr/local/bin/stop-hotspot.sh"
echo ""
echo -e "${BLUE}📱 连接信息：${NC}"
echo "  WiFi名称: $HOTSPOT_SSID"
echo "  WiFi密码: $HOTSPOT_PASSWORD"
echo "  网关地址: $HOTSPOT_IP"
echo "  Web服务: http://$HOTSPOT_IP:8000"
echo ""
echo -e "${YELLOW}⚠️ 注意：${NC}"
echo "  1. 重启后配置才会完全生效"
echo "  2. 确保eth0网络连接正常以提供互联网访问"
echo "  3. 如需修改配置，请编辑 /etc/hostapd/hostapd.conf"
echo ""

# 询问是否立即启动
echo -e "${YELLOW}是否要立即启动WiFi热点？(y/N): ${NC}"
read -r start_now
if [[ "$start_now" =~ ^[Yy]$ ]]; then
    echo -e "${BLUE}🚀 启动WiFi热点...${NC}"
    /usr/local/bin/start-hotspot.sh
    
    # 检查状态
    sleep 3
    if systemctl is-active --quiet hostapd && systemctl is-active --quiet dnsmasq; then
        echo -e "${GREEN}✅ WiFi热点启动成功！${NC}"
        echo -e "${BLUE}📊 状态检查：${NC}"
        echo "  hostapd: $(systemctl is-active hostapd)"
        echo "  dnsmasq: $(systemctl is-active dnsmasq)"
    else
        echo -e "${RED}❌ WiFi热点启动失败${NC}"
        echo "请检查日志: journalctl -u hostapd -u dnsmasq"
    fi
fi

echo ""
echo -e "${GREEN}配置完成！${NC}"
