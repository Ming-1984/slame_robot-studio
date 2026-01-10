# 🤖 Robot Studio

基于ROS2的智能机器人控制系统，集成Aurora激光雷达、路径规划、Web控制界面等功能。

## 🚀 快速启动

### 完整系统启动
```bash
./robot-studio start
# 如需同时开启热点：
./robot-studio start --hotspot
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

## ⏻ 安全关机

- **Web界面**：点击页面底部“关机”，按提示输入 `关机` 才会执行关机。
- **API**（支持 dry-run / plan-only）：
  - 仅检查不执行：`{"command":"shutdown_system","parameters":{"plan_only":true}}`
  - 仅停止服务不关机：`{"command":"shutdown_system","parameters":{"confirm":"关机","dry_run":true,"network_cleanup":false}}`
  - 真正关机：`{"command":"shutdown_system","parameters":{"confirm":"关机"}}`

## 🧪 一次性测试模式（重启）

用于调试网络/SSH：执行后会立刻重启一次，并在“下一次开机”进入测试模式（不启热点/不上位机，仅启动 SSH），随后会自动恢复为“下次再开机正常自启（热点+上位机）”。

- **Web界面**：点击页面底部“测试模式”，按提示输入 `测试模式` 确认。
- **API**：
  - 仅检查不执行：`{"command":"enter_test_mode_once","parameters":{"plan_only":true}}`
  - 真正执行（将重启）：`{"command":"enter_test_mode_once","parameters":{"confirm":"测试模式"}}`

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
