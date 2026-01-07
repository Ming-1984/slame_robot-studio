# WiFi 热点与 Aurora 透传（Jetson 作为网关）

本项目使用 Jetson 在 WiFi 热点侧提供控制界面（`192.168.4.1`），同时通过有线网口连接 Aurora（常见为 `192.168.11.1`），实现 PC 只连 Jetson 热点也能访问 Aurora 网段。

## ✅ 目标拓扑

- Jetson WiFi 热点：`RobotStudio`（无密码）
  - Jetson 网关：`192.168.4.1/24`
  - DHCP：`192.168.4.10-192.168.4.50`
- Jetson 有线连接 Aurora：
  - Jetson：常见 `192.168.11.100/24`
  - Aurora：`192.168.11.1`
- 关键能力：热点侧（`192.168.4.0/24`）访问 Aurora 网段（`192.168.11.0/24`）通过 NAT/转发实现，不需要改 Aurora 路由。

## 🚀 启动热点

```bash
sudo ./improved_hotspot.sh start
```

停止热点：

```bash
sudo ./improved_hotspot.sh stop
```

查看状态：

```bash
./improved_hotspot.sh status
```

## 🔧 不重启热点修复透传（推荐）

当 Windows 已连接热点、能 ping 通 `192.168.4.1`，但 ping 不通 `192.168.11.100/192.168.11.1` 时，执行：

```bash
sudo ./improved_hotspot.sh fix-routing
```

该命令只修复 NAT/转发规则，不会重启热点进程（不中断已连接设备）。

## ✅ 验证清单（从 Windows 侧）

1) 连接 WiFi：`RobotStudio`

2) 依次测试：
- `ping 192.168.4.1`（Jetson 热点网关）
- `ping 192.168.11.100`（Jetson 有线口 IP）
- `ping 192.168.11.1`（Aurora）

## 🔍 常见问题排查

### 1) 能 ping 通 192.168.4.1，但 192.168.11.* 都不通
- 先执行：`sudo ./improved_hotspot.sh fix-routing`
- 再在 Jetson 上确认 Aurora 在线：
  ```bash
  ping -c 1 192.168.11.1
  ```

### 2) 热点会被自动切回 WiFi 客户端模式
热点启动时脚本会让 NetworkManager “放弃管理 WiFi 接口”，避免它自动重连覆盖热点；如仍发生，请执行：

```bash
sudo ./improved_hotspot.sh restart
```

（会断开当前热点连接，请谨慎）

### 3) Windows 侧仍无法访问 Aurora（兜底验证）
在 Windows 上执行 `route print`，确认到 `192.168.11.0/24` 的路由走 `192.168.4.1`；必要时可临时添加：

```powershell
route add 192.168.11.0 mask 255.255.255.0 192.168.4.1
```

（正常情况下不需要；脚本已通过 DHCP 下发该路由）
   - 验证IP地址：`ip addr show wlP1p1s0`

---

**选择您的方案并开始使用Robot Studio！** 🚀
