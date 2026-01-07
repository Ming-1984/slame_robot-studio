# Robot Studio - 机器人远程控制系统

一个完整的机器人远程控制和监控系统，集成了路径规划、点云处理、实时地图显示和多平台控制界面。

## 🚀 系统特性

### 核心功能
- **🤖 机器人远程控制**: 通过Web界面或Android应用控制机器人移动
- **🗺️ 实时地图显示**: 显示SLAM生成的实时地图和机器人位置
- **📡 激光雷达可视化**: 实时显示激光雷达扫描数据
- **☁️ 点云处理**: Aurora摄像头点云数据提取、墙体增强与可选高级检测（已移除DXF转换）
- **📱 多平台支持**: Web界面和Android原生应用
- **📁 文件管理**: 地图文件的保存、下载和管理

### 网络架构
- **WiFi热点**: Jetson Nano作为接入点 (192.168.4.1)
- **RESTful API**: HTTP接口提供机器人控制和数据访问
- **WebSocket**: 实时数据传输和双向通信
- **Web界面**: 基于HTML5/JavaScript的控制界面

## 📋 系统要求

### 硬件要求
- Jetson Nano 或类似ARM64设备
- WiFi网卡 (支持AP模式)
- Aurora深度摄像头
- 激光雷达传感器
- 机器人底盘

### 软件要求
- Ubuntu 20.04/22.04
- ROS2 Humble
- Python 3.8+
- 网络工具 (hostapd, dnsmasq)

## 🔧 安装和配置

### 1. 快速安装
```bash
# 克隆或下载项目到 /home/jetson/ros2_ws
cd /home/jetson/ros2_ws

# 安装所有依赖
sudo ./install_dependencies.sh

# 编译ROS2工作空间
colcon build

# 配置WiFi热点
sudo ./setup_wifi_hotspot.sh
```

### 2. 手动安装步骤

#### 安装Python依赖
```bash
pip3 install fastapi uvicorn websockets requests aiofiles python-multipart pydantic
```

#### 安装系统依赖
```bash
sudo apt update
sudo apt install -y hostapd dnsmasq iptables-persistent
sudo apt install -y libpcl-dev pcl-tools libeigen3-dev libboost-all-dev
```

#### 安装ROS2包
```bash
sudo apt install -y ros-humble-nav2-bringup ros-humble-slam-toolbox
```

## 🚀 启动系统

### 完整系统启动
```bash
# 启动所有服务 (需要root权限用于WiFi热点)
sudo ./start_robot_studio.sh
```

### 分步启动
```bash
# 1. 启动WiFi热点
sudo systemctl start wifi-hotspot

# 2. 启动ROS2路径规划系统
./start_path_planning.sh

# 3. 启动Web服务 (在新终端)
python3 robot_api_server.py

# 4. 启动WebSocket服务 (在新终端)
python3 ros2_web_bridge.py
```

## 📱 使用方法

### Web界面控制
1. 连接WiFi网络: `RobotStudio-AP` (密码: `robotstudio123`)
2. 打开浏览器访问: `http://192.168.4.1:8080`
3. 使用虚拟摇杆控制机器人移动
4. 查看实时地图和激光雷达数据
5. 执行系统命令和文件管理

### Android应用控制
1. 安装Android应用 (APK在android_app目录)
2. 连接到机器人WiFi网络
3. 在应用中输入服务器IP: `192.168.4.1`
4. 点击连接并使用各项功能

### 点云处理
```bash
# 运行完整的点云处理流程
cd robotstudio/aurora_project
./complete_processing.sh

# 或通过Web界面的"启动点云处理"按钮
```

说明：本仓库已移除 “Convert to CAD / DXF” 第3步点云处理功能；真景建图等能力请使用官方上位机：`/home/jetson/ros2_ws/aurora_remote-release-2.1.0-rc2`。

## 🌐 API接口

### RESTful API (端口 8000)
- `GET /api/status` - 获取系统状态
- `GET /api/robot/pose` - 获取机器人位姿
- `POST /api/robot/velocity` - 设置机器人速度
- `POST /api/robot/goal` - 设置目标点
- `POST /api/system/command` - 执行系统命令
- `GET /api/files/maps` - 列出地图文件
- `GET /api/files/download/{filename}` - 下载文件

### WebSocket接口 (端口 8001)
- 实时地图数据推送
- 机器人位姿更新
- 激光雷达数据流
- 双向命令通信

## 📁 项目结构

```
/home/jetson/ros2_ws/
├── start_robot_studio.sh          # 主启动脚本
├── install_dependencies.sh        # 依赖安装脚本
├── setup_wifi_hotspot.sh         # WiFi热点配置
├── robot_api_server.py            # API服务器
├── ros2_web_bridge.py             # WebSocket桥接
├── start_path_planning.sh         # 路径规划启动脚本
├── simple_map_saver.py            # 地图保存工具
├── web_interface/                 # Web控制界面
│   ├── index.html
│   └── js/
│       ├── robot-control.js
│       ├── map-display.js
│       └── websocket-client.js
├── android_app/                   # Android应用源码
├── robotstudio/aurora_project/    # 点云处理系统
└── maps/                          # 保存的地图文件
```

## 🔧 配置说明

### WiFi热点配置
- SSID: `RobotStudio-AP`
- 密码: `robotstudio123`
- IP范围: `192.168.4.10-192.168.4.50`
- 网关: `192.168.4.1`

### 服务端口
- Web界面: `8080`
- API服务: `8000`
- WebSocket: `8001`

### 文件路径
- 地图文件: `~/maps/`
- 点云数据: `~/ros2_ws/robotstudio/aurora_project/data/`
- 日志文件: `~/ros2_ws/logs/`

## 🚨 故障排除

### 常见问题

#### 1. WiFi热点无法启动
```bash
# 检查WiFi接口
ip link show

# 重启网络服务
sudo systemctl restart hostapd dnsmasq

# 查看日志
sudo journalctl -u hostapd -u dnsmasq
```

#### 2. API服务无法访问
```bash
# 检查端口占用
netstat -tlnp | grep 8000

# 检查防火墙
sudo ufw status

# 查看API日志
tail -f /tmp/api_server.log
```

#### 3. ROS2话题无数据
```bash
# 检查ROS2节点
ros2 node list

# 检查话题
ros2 topic list
ros2 topic echo /map --once

# 重启ROS2 daemon
ros2 daemon stop
ros2 daemon start
```

#### 4. 地图保存失败
```bash
# 使用简单地图保存器
python3 simple_map_saver.py ~/maps/test_map --timeout 10

# 检查地图话题
ros2 topic echo /map --once
```

## 📞 技术支持

### 日志位置
- API服务器: `/tmp/api_server.log`
- WebSocket服务: `/tmp/websocket.log`
- 系统日志: `journalctl -f`

### 调试命令
```bash
# 检查系统状态
curl http://192.168.4.1:8000/api/status

# 测试WebSocket连接
wscat -c ws://192.168.4.1:8001

# 检查ROS2环境
ros2 doctor
```

## 🎯 开发和扩展

### 添加新的API接口
1. 在 `robot_api_server.py` 中添加新的路由
2. 实现对应的ROS2发布者/订阅者
3. 更新Web界面的JavaScript代码

### 自定义地图处理
1. 修改 `simple_map_saver.py` 中的处理逻辑
2. 添加新的文件格式支持
3. 集成到Web界面的文件管理功能

### Android应用定制
1. 修改 `android_app/` 目录下的源码
2. 添加新的活动和功能
3. 重新编译APK

## 📄 许可证

本项目采用MIT许可证，详见LICENSE文件。

## 🤝 贡献

欢迎提交Issue和Pull Request来改进这个项目！

---

**Robot Studio** - 让机器人控制变得简单而强大！
