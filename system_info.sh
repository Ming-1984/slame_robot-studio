#!/bin/bash

# Robot Studio 系统信息脚本
echo -e "\033[0;35m🤖 Robot Studio 系统信息\033[0m"
echo "========================"
echo ""

echo -e "\033[0;36m📁 系统结构:\033[0m"
echo "  ├── start_robot_studio.sh     - 🚀 主启动脚本"
echo "  ├── install_system.sh         - 📦 系统安装脚本"
echo "  ├── fix_wifi.sh               - 📡 WiFi修复脚本"
echo "  ├── system_info.sh            - ℹ️  系统信息脚本"
echo "  ├── robot_api_server.py       - 🔌 API服务器"
echo "  ├── ros2_web_bridge.py        - 🔗 WebSocket桥接"
echo "  ├── simple_map_saver.py       - 💾 地图保存工具"
echo "  ├── web_interface/            - 🌐 Web控制界面"
echo "  ├── robotstudio/              - ☁️  点云处理系统"
echo "  ├── core/                     - 🎯 核心组件"
echo "  │   ├── scripts/              - 📜 核心脚本"
echo "  │   ├── docs/                 - 📚 文档"
echo "  │   └── config/               - ⚙️  配置文件"
echo "  └── data/                     - 📊 数据文件"
echo "      ├── maps/                 - 🗺️  地图文件"
echo "      ├── logs/                 - 📝 日志文件"
echo "      └── exports/              - 📤 导出文件"
echo ""

echo -e "\033[0;36m🚀 快速启动:\033[0m"
echo "  启动系统: ./start.sh (一键启动所有服务)"
echo "  路径规划: ./start_path_planning.sh (仅ROS2系统)"
echo "  查看信息: ./system_info.sh"
echo ""

echo -e "\033[0;36m📱 访问地址:\033[0m"
echo "  简单访问: http://robot (推荐)"
echo "  直接访问: http://192.168.4.1"
echo "  WiFi名称: RobotStudio (无密码)"
echo ""

echo -e "\033[0;36m📊 系统状态:\033[0m"
if command -v robot-studio-control &> /dev/null; then
    robot-studio-control status
else
    echo "  系统服务未安装"
    echo "  运行 ./install_system.sh 安装开机自启服务"
fi

echo ""
echo -e "\033[0;36m📚 文档位置:\033[0m"
echo "  主文档: ./core/docs/ROBOT_STUDIO_README.md"
echo "  WiFi设置: ./core/docs/WIFI_HOTSPOT_SETUP.md"




