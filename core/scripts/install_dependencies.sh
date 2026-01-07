#!/bin/bash

# Robot Studio 依赖安装脚本
# 安装所有必要的依赖包和库

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🔧 Robot Studio 依赖安装器${NC}"
echo "=================================="
echo ""

# 检查是否以root权限运行
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}❌ 请以root权限运行此脚本${NC}"
    echo "使用: sudo $0"
    exit 1
fi

# 获取原始用户信息
ORIGINAL_USER=${SUDO_USER:-$USER}
ORIGINAL_HOME=$(eval echo ~$ORIGINAL_USER)

echo -e "${BLUE}📋 安装信息：${NC}"
echo "  原始用户: $ORIGINAL_USER"
echo "  用户目录: $ORIGINAL_HOME"
echo "  工作目录: $(pwd)"
echo ""

# 更新系统包
echo -e "${BLUE}📦 更新系统包...${NC}"
apt update

# 安装基础依赖
echo -e "${BLUE}🔨 安装基础依赖...${NC}"
apt install -y \
    curl \
    wget \
    git \
    build-essential \
    cmake \
    pkg-config \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv

# 安装网络工具
echo -e "${BLUE}🌐 安装网络工具...${NC}"
apt install -y \
    hostapd \
    dnsmasq \
    iptables-persistent \
    net-tools \
    wireless-tools

# 安装Python Web依赖
echo -e "${BLUE}🐍 安装Python Web依赖...${NC}"
pip3 install --upgrade pip

# 为原始用户安装Python包
sudo -u $ORIGINAL_USER pip3 install --user \
    fastapi \
    uvicorn \
    websockets \
    requests \
    aiofiles \
    python-multipart \
    pydantic

# 安装点云处理依赖
echo -e "${BLUE}☁️ 安装点云处理依赖...${NC}"
apt install -y \
    libpcl-dev \
    pcl-tools \
    libeigen3-dev \
    libflann-dev \
    libvtk9-dev \
    libboost-all-dev \
    libqhull-dev \
    libusb-1.0-0-dev \
    freeglut3-dev

# 检查ROS2安装
echo -e "${BLUE}🤖 检查ROS2安装...${NC}"
if ! command -v ros2 &> /dev/null; then
    echo -e "${YELLOW}⚠️ 未检测到ROS2，请手动安装ROS2 Humble${NC}"
    echo "参考: https://docs.ros.org/en/humble/Installation.html"
else
    echo -e "${GREEN}✅ ROS2 已安装${NC}"
    
    # 安装额外的ROS2包
    echo -e "${BLUE}📦 安装ROS2额外包...${NC}"
    apt install -y \
        ros-humble-nav2-bringup \
        ros-humble-nav2-map-server \
        ros-humble-slam-toolbox \
        ros-humble-robot-localization \
        ros-humble-tf2-tools \
        ros-humble-rviz2
fi

# 设置权限
echo -e "${BLUE}🔐 设置文件权限...${NC}"
if [ -f "$ORIGINAL_HOME/ros2_ws/start_robot_studio.sh" ]; then
    chmod +x "$ORIGINAL_HOME/ros2_ws/start_robot_studio.sh"
    chown $ORIGINAL_USER:$ORIGINAL_USER "$ORIGINAL_HOME/ros2_ws/start_robot_studio.sh"
fi

if [ -f "$ORIGINAL_HOME/ros2_ws/setup_wifi_hotspot.sh" ]; then
    chmod +x "$ORIGINAL_HOME/ros2_ws/setup_wifi_hotspot.sh"
    chown $ORIGINAL_USER:$ORIGINAL_USER "$ORIGINAL_HOME/ros2_ws/setup_wifi_hotspot.sh"
fi

# 创建必要的目录
echo -e "${BLUE}📁 创建必要目录...${NC}"
mkdir -p "$ORIGINAL_HOME/ros2_ws/maps"
mkdir -p "$ORIGINAL_HOME/ros2_ws/logs"
chown -R $ORIGINAL_USER:$ORIGINAL_USER "$ORIGINAL_HOME/ros2_ws/maps"
chown -R $ORIGINAL_USER:$ORIGINAL_USER "$ORIGINAL_HOME/ros2_ws/logs"

# 配置防火墙（如果存在）
echo -e "${BLUE}🛡️ 配置防火墙...${NC}"
if command -v ufw &> /dev/null; then
    ufw allow 8000/tcp  # API服务器
    ufw allow 8001/tcp  # WebSocket
    ufw allow 8080/tcp  # Web服务器
    echo -e "${GREEN}✅ 防火墙规则已添加${NC}"
fi

# 检查WiFi接口
echo -e "${BLUE}📡 检查WiFi接口...${NC}"
wifi_interfaces=$(ls /sys/class/net/ | grep -E '^wl|^wlan' || true)
if [ -n "$wifi_interfaces" ]; then
    echo -e "${GREEN}✅ 发现WiFi接口: $wifi_interfaces${NC}"
else
    echo -e "${YELLOW}⚠️ 未发现WiFi接口，WiFi热点功能可能不可用${NC}"
fi

# 验证安装
echo -e "${BLUE}✅ 验证安装...${NC}"

# 检查Python包
echo "检查Python包..."
sudo -u $ORIGINAL_USER python3 -c "
try:
    import fastapi, uvicorn, websockets, requests
    print('✅ Python Web依赖正常')
except ImportError as e:
    print(f'❌ Python依赖缺失: {e}')
"

# 检查ROS2
if command -v ros2 &> /dev/null; then
    echo "✅ ROS2命令可用"
else
    echo "❌ ROS2命令不可用"
fi

# 检查网络工具
if command -v hostapd &> /dev/null && command -v dnsmasq &> /dev/null; then
    echo "✅ WiFi热点工具可用"
else
    echo "❌ WiFi热点工具不可用"
fi

echo ""
echo -e "${GREEN}🎉 依赖安装完成！${NC}"
echo ""
echo -e "${BLUE}📋 下一步操作：${NC}"
echo "1. 编译ROS2工作空间:"
echo "   cd $ORIGINAL_HOME/ros2_ws"
echo "   colcon build"
echo ""
echo "2. 配置WiFi热点:"
echo "   sudo $ORIGINAL_HOME/ros2_ws/setup_wifi_hotspot.sh"
echo ""
echo "3. 启动Robot Studio系统:"
echo "   sudo $ORIGINAL_HOME/ros2_ws/start_robot_studio.sh"
echo ""
echo -e "${YELLOW}💡 提示：${NC}"
echo "- 首次运行需要配置WiFi热点"
echo "- 确保ROS2工作空间已正确编译"
echo "- 检查所有硬件连接正常"
echo ""

# 创建快速启动脚本
cat > /usr/local/bin/robot-studio << EOF
#!/bin/bash
# Robot Studio 快速启动脚本

cd $ORIGINAL_HOME/ros2_ws
sudo ./start_robot_studio.sh "\$@"
EOF

chmod +x /usr/local/bin/robot-studio
echo -e "${GREEN}✅ 已创建全局命令: robot-studio${NC}"
echo "   现在可以在任何位置运行: robot-studio"

echo ""
echo -e "${GREEN}安装完成！${NC}"
