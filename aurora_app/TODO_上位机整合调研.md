# 上位机整合调研（Aurora 官方上位机 + Robot Studio）

> 目标：把“官方 Aurora 上位机（Aurora Remote）”与“自研 Robot Studio 上位机（Web）”整合为一个入口/一个产品形态，避免分别在两个地方使用。

## 1. 当前状态（已实测）

### 1.1 网络与服务拓扑
- **Robot Studio（自研上位机）**：`http://192.168.4.1:8080`（静态 Web 页面，Python `SimpleHTTP/0.6`）
- **Robot Studio API**：`http://192.168.4.1:8000`（`uvicorn` / FastAPI）
- **Robot Studio WebSocket**：`ws://192.168.4.1:8001`（ROS2 Web Bridge，实时推送 map/odom/scan 等）
- **Aurora 设备地址**：`192.168.11.1`

### 1.2 Aurora 设备对外接口（端口探测）
（从当前电脑可直连 `192.168.11.1`，说明热点侧已具备路由/NAT/桥接能力）
- `192.168.11.1:80`：HTTP（Slamtec Device Management Web）
- `192.168.11.1:1445`：TCP（Slamware ROS SDK 默认连接端口，见 `ros2_ws/src/slamware_ros_sdk/src/server/server_params.cpp`）
- `192.168.11.1:1448`：HTTP（返回 404 但带 `Access-Control-Allow-Origin:*`，疑似设备侧另一套 HTTP API，需进一步探测其真实路径）
- `192.168.11.1:7447`：TCP（Aurora Public Remote SDK 默认端口，见 `ros2_ws/src/aurora_remote_public/include/aurora_pubsdk_common_def.h`）

### 1.3 Aurora 设备管理 Web（`192.168.11.1`）
- `http://192.168.11.1/` → 302 到 `/index.html`，页面标题为 **Slamtec Device Management**
- 该 Web 前端调用的 API 主要在 **`/service`** 下（部分接口需要登录 token）
  - 可匿名访问示例：`GET http://192.168.11.1/service/system/product`
  - 需要登录示例：`GET http://192.168.11.1/service/system/lidar/info`（会 302 到登录）

### 1.4 官方 Aurora Remote（本仓库 `aurora_app/`）
- 当前目录 `aurora_app/` 为 Windows 官方上位机发行包（`.exe/.dll/media`），**无源码**。
- 实测进程窗口标题包含版本信息：`SLAMTEC Aurora Remote - 2.1.0-rc2 (SDK: 2.1.0-rc2)`
- 结论：**无法在本仓库直接“改源码合并 UI”**；整合应走「嵌入/代理」或「按协议/SDK复刻关键功能」路线。

### 1.5 自研 Robot Studio（本仓库 `ros2_ws/`）
核心组件（已在仓库内）：
- `ros2_ws/robot_api_server.py`：FastAPI（8000），负责控制与文件/项目等 REST API
- `ros2_ws/ros2_web_bridge.py`：WebSocket（8001），桥接 ROS2 topic 到 Web（map/odom/scan 等）
- `ros2_ws/web_interface/`：Web 前端（8080 静态站点）
- `ros2_ws/src/slamware_ros_sdk`：Aurora ROS2 SDK（默认连 `robot_port=1445`）
- `ros2_ws/src/aurora_remote_public`：Aurora Public Remote SDK（默认端口 `7447`）
- `ros2_ws/setup_aurora_bridge.sh`：网络桥接/NAT/端口转发脚本（热点 `192.168.4.0/24` ↔ Aurora `192.168.11.0/24`）

## 2. “整合”可行性结论

### 2.1 你想要的“整合”可能有两种层级
1) **入口整合（最快见效）**：Robot Studio 作为统一入口，把官方页面/功能“嵌进去/跳转进去”
2) **功能整合（产品级）**：Robot Studio 复刻并覆盖 Aurora Remote 的关键能力（地图管理、重定位、导出等），最终不再依赖 Windows Aurora Remote

### 2.2 受限点（需要你确认）
- 官方 Aurora Remote 是闭源二进制：要做“同一个 App 内原生合并”，通常涉及 **授权/再分发许可** 与 **技术不可控**。
- 设备管理 Web（`192.168.11.1`）可作为“官方能力入口”，但与 Robot Studio 的 UI/鉴权/同源策略存在整合工作量（iframe/反向代理/SSO 等）。

## 3. 方案备选（建议按阶段落地）

### 方案 A：Robot Studio 单入口 + 嵌入官方 Device Management（推荐做为第一步）
做法：
- 在 `ros2_ws/web_interface/` 增加一个页面或一个 Tab：
  - 方式 A1：直接 iframe：`<iframe src="http://192.168.11.1/index.html">`
  - 方式 A2：Robot Studio 侧做反向代理（同源），把 `192.168.11.1` 映射到 `http://192.168.4.1:8000/aurora-devmgr/...`（更利于后续深度整合按钮/状态）

优点：
- 最快形成“一个入口”，几乎不碰协议
- 风险小，可快速验证用户体验

风险/注意：
- 部分浏览器对 iframe 的 cookie/第三方存储更严格（需要实际验证）
- 若要做 A2 反代，需要处理静态资源路径、缓存、以及可能的 WebSocket（若设备管理页使用）

### 方案 B：Robot Studio 逐步覆盖 Aurora Remote 的“必用功能”（产品化路线）
基于现有 `slamware_ros_sdk (1445)` + `aurora_remote_public (7447)`，把常用能力做成 Web UI + API：
- 地图文件（STCM）同步：对接 `slamware_ros_sdk` 的 `SyncGetStcm/SyncSetStcm`（服务已存在）
- 重定位：对接 `RelocalizationRequest` + 状态 topic（服务/话题已存在）
- 关键状态面板：设备在线、建图/定位模式、回环、建图清空等（视 SDK 支持补齐）
- 数据导出：轨迹/点云/地图（优先复用你已有的 `robotstudio/`、`photoreal/` 流程与项目管理器）

优点：
- 最终用户只用 Robot Studio（Web/Android），形成“完整产品”
- 可持续迭代，UI/交互统一

风险/注意：
- Aurora Remote 内含的高级可视化/工具链较多（需要你明确“必须做哪些”）
- 可能存在 SDK 版本差异：Aurora Remote 显示 `SDK 2.1.0-rc2`，仓库内 Public Remote SDK 为 `1.2.0-rc`（需要确认是否要升级/兼容）

### 方案 C：保留 Windows Aurora Remote，但通过 Robot Studio “一键启动/一键下载/连接配置自动化”
做法：
- Robot Studio 提供：
  - 官方 Aurora Remote 下载入口（对应版本包）
  - 一键检测网络桥接、自动给出推荐连接地址（`192.168.11.1:7447` 或 `192.168.4.1:7447`）
  - 常用参数/校验（ping/端口探测/日志）

优点：
- 工期短，且保留官方体验
缺点：
- 仍然是“两套界面”（只是少切换），不满足“单产品”理想态

## 4. TODO（先确认后实施）

### 4.0 你刚确认的目标（本轮结论）
你希望的“整合”是做一个 **Windows 桌面软件（控制中心/启动器）**：
- 软件里直接打开/嵌入 **我们的上位机网页**（Robot Studio Web）
 - 软件里一键启动/管理 **官方上位机**（`aurora_remote.exe`）
- 提供 **统一入口 + 网络自检/快捷入口**（连接操作仍在官方上位机内完成，不做自动连接）

### 4.1 需求确认（你需要回答）
- 你说的“官方上位机”具体指：
  - A) `aurora_remote.exe`（Windows Aurora Remote）  
  - B) `192.168.11.1` 的 Device Management Web  
  - 还是两者都要整合？
- 你希望最终产品形态：
  - 只保留 Web（手机/PC 浏览器）？
  - 还是 Web + Windows 客户端并存？
- 必做功能 Top 5（从 Aurora Remote 里挑）：
  - 地图下载/上传（STCM）/清图/保存？
  - 重定位？
  - 2D Map（comap）导出？
  - 点云/轨迹导出？
  - PhotoReal/稠密建图流程？
  - 设备网络配置/固件升级？
- 是否需要“登录/权限”（比如禁止普通用户进入设备管理/网络配置）？

### 4.1.1 针对“桌面控制中心”的补充确认（建议你直接回答）
- 目标运行环境：是否只做 **Windows 10/11**？
- “一个页面管理”你能接受：
  - A) 控制中心内嵌我们的 Web（一个 Tab），官方 Aurora Remote 作为 **外部窗口**一键启动
  - B) 需要把 Aurora Remote **嵌进同一个窗口**（Win32 SetParent/嵌入窗口，兼容性风险更高）
- 自动连接的期望层级：
  - L1：自动探测并显示推荐连接地址（如 `192.168.11.1:7447` / `192.168.11.1:1445`），点一下打开
  - L2：启动 `aurora_remote.exe` 后，能自动填入 IP/端口并触发连接（依赖官方程序是否支持参数/配置；否则需 UI 自动化，稳定性一般）
  - L3：完全不依赖 `aurora_remote.exe`，我们用 SDK/ROS2 自己实现连接与核心功能（产品化路线）
- 打包分发：是否需要把 `aurora_remote.exe` 一起打进安装包？（这涉及官方许可/再分发授权，需你确认）

> 你已确认：**Windows 10/11**；并且“最好嵌进同一个窗口”；同时“连接能力必须完全使用官方上位机”（本控制台不做自动连接/不做协议实现）。

### 4.2 技术调研补全（我来做）
- 进一步识别 `192.168.11.1:1448` 的 HTTP API 真实路径（若与某些官方功能相关）
- 盘点 `slamware_ros_sdk` 当前已暴露的能力 vs Aurora Remote UI（做差距表）
- 如果你确认要对齐 Aurora Remote 新版本：评估升级 `aurora_remote_public`（1.2.0-rc → 2.x）对现有工程的影响

### 4.3 第一阶段落地（建议先做）
- 在 `ros2_ws/web_interface/` 加一个“官方管理”入口页（iframe 方式）+ 连通性检测（调用 `robot_api_server` 做 ping/端口检测并展示）
- 在 `robot_api_server.py` 增加一个 `/api/aurora/status`：汇总 `192.168.11.1` 的可达性、1445/7447/80/1448 端口状态

### 4.4 如果你确认走“Windows 桌面控制中心”，我建议的实施顺序
1) **做最小可用 Launcher（1~2天量级）**
   - 技术实现：C# `WinForms + WebView2`（体积小、Windows 生态好、便于嵌入外部窗口）
   - 功能：
     - 内嵌 `http://192.168.4.1:8080`（或自动识别网关）显示 Robot Studio
     - 一键启动 `aurora_remote.exe`（默认从 `aurora_app/aurora_remote.exe` 取；也可让用户选择路径）
     - 一键打开 `http://192.168.11.1/index.html`（Device Management）
     - “连接自检”面板：ping + 端口（80/1445/7447/8000/8001）+ API 健康检查
2) **加“自动连接”能力（需要验证官方程序能力）**
   - 优先尝试：官方 Aurora Remote 是否支持命令行参数/配置文件（若支持则稳定）
   - 不支持则备选：UI 自动化（可做，但版本变化会导致脚本失效）
   - 备注：你当前决定先不做自动连接，仅保留为未来选项

- 解决方案：`windows_control_center/RobotStudioControlCenter.sln`

---

## 6. 产品化与打包（本轮新增需求，先确认后实施）

### 6.1 你本轮提出的新目标
- **Aurora Remote 视图顶部控件全部清除**：只保留官方上位机画面（占满区域），不显示“exe 路径/浏览/启动/嵌入/自检/IP 输入”等上方条。
- **打开我们的软件即自动启动 + 自动嵌入**：不再需要手动输入/选择 `aurora_remote.exe` 路径。
- **完整打包**：发布包里包含官方上位机（`aurora_app/` 整个发行目录：exe/dll/media/工具等），做到“拿到一个包就能用”。

### 6.2 建议的产品行为（v1）
- 程序启动后：
  - Tab1：内嵌 Robot Studio Web（仍访问 `http://192.168.4.1:8080/`）
  - **后台自动启动 `aurora_remote.exe` 并嵌入**到 Tab2（无顶部控件）
  - 启动失败时：给出明确错误（缺文件/缺 VC 运行库/缺 WebView2 等）+ 指引处理
- `aurora_remote.exe` 默认定位规则：优先从程序同目录的 `.\aurora_app\aurora_remote.exe` 启动（打包后固定存在）。

### 6.3 打包/分发方案（两种可选）
**方案 A：绿色免安装包（推荐先做，最快落地）**
- `dotnet publish` 生成一个可运行目录（建议 `win-x64` + 自包含，不要求目标电脑装 .NET）
- 把 `aurora_app/` 整目录复制到发布目录
- 最终交付：一个文件夹（可再 zip），双击我们的 `exe` 即可运行

**方案 B：安装包（Inno Setup / WiX / MSIX）**
- 优点：有安装向导、开始菜单快捷方式、卸载等
- 成本：需要引入打包工具链（Inno/WiX/MSIX），并处理权限/签名等问题

> 备注：`aurora_app/` 当前体积约 350MB，整体包体会比较大（属正常预期）。

### 6.4 关键风险/需要你确认的点
- **再分发授权**：我们是否有权把官方 `aurora_remote.exe` 及其依赖一起打包分发给客户？（必须你确认）
- **依赖组件**：
  - WebView2 Runtime：目标电脑未安装时如何处理？（提示用户安装 / 或随包附带离线安装器）
  - VC++ Runtime：官方包里已有 `VC_redist.x64.exe`，是否允许我们在首次启动时提示/引导安装？
- **失败兜底**：如果用户把 `aurora_app` 目录删了/路径不对：
  - A) 直接报错并提示把目录放回正确位置（不提供任何“选择 exe”UI）
  - B) 仍保留一个隐藏的“高级设置/修复”入口（默认不显示，仅用于售后排障）

### 6.5 TODO（你确认后我再开始改）
- [ ] UI：Aurora Remote Tab 改为“纯画面”模式（移除顶部 controls panel，占满）
- [ ] 启动：应用启动时自动启动/嵌入 Aurora Remote（异步，不阻塞 UI）
- [ ] 运行：启动失败提示更友好（缺文件/依赖/权限/窗口句柄获取失败等）
- [ ] 打包：新增 `scripts/package.ps1`（一键 `publish` + 复制 `aurora_app/` + 生成 zip/输出目录）
- [ ] 打包：发布目录结构与默认路径约定（例如：`.\aurora_app\aurora_remote.exe`）
- [ ] 品牌：统一产品命名（窗口标题/Tab 文案/输出 exe 名称/图标）
- [ ] 验收：在“干净电脑”验证（无 .NET / 无 WebView2 / 无 VC++ 的场景分别验证提示与修复路径）

### 6.6 你需要先确认的 5 个问题（你回复编号即可）
1) 最终产品名用什么？（用于窗口标题/EXE 名/安装包名）
2) 交付形式选哪个：A) 绿色免安装 zip；B) 安装包（Inno/WiX/MSIX）？
3) 关闭我们的软件时：是否要 **自动关闭** `aurora_remote.exe`？（是/否）
4) WebView2/VC++ 依赖：是否允许我们在缺失时 **随包附带离线安装器并弹窗引导安装**？
5) 失败兜底选哪个：A) 不提供选择 exe（严格要求目录存在）；B) 保留隐藏“高级设置/修复”入口？

