# 🤖 Robot Studio

基于ROS2的智能机器人控制系统，集成Aurora激光雷达、路径规划、Web控制界面等功能。

## 🚀 快速启动

### 完整系统启动
```bash
sudo ./start.sh
```

### 仅路径规划
```bash
./start_path_planning.sh  # 启动
./stop_path_planning.sh   # 停止
```

### 系统信息
```bash
./system_info.sh         # 查看系统状态
./test_system_status.sh  # 测试系统功能
```

## 📱 访问方式

- **WiFi连接**: `RobotStudio` (无密码)
- **Web界面**: http://robot 或 http://192.168.4.1
- **API服务**: http://192.168.4.1:8000

## 📁 系统结构

- `core/` - 核心组件和配置
- `web_interface/` - Web控制界面
- `robotstudio/` - 点云处理系统
- `data/` - 数据文件（地图、日志等）

## 📚 详细文档

- [完整文档](./core/docs/ROBOT_STUDIO_README.md)
- [WiFi设置指南](./core/docs/WIFI_HOTSPOT_SETUP.md)

## 🎯 主要功能

- 🤖 机器人远程控制
- 🗺️ 实时地图显示
- 📡 激光雷达可视化
- ☁️ 点云数据处理
- 📱 多平台支持（Web + Android）
- 📁 文件管理和导出

---
**Robot Studio** - 让机器人控制变得简单而强大！
