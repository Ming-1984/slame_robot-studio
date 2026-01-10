# Web 上位机：摇杆映射修复 + 键盘控制 + 调试模式（规划 / TODO）

> 说明：本文档只做规划，不会立即改代码/改系统配置。你确认后我再按本文逐项实现与测试。

## 0. 当前现状（已定位的关键点）

- Web 摇杆逻辑在：`web_interface/js/robot-control.js`
  - 当前把圆形摇杆坐标（-1~1）做了“圆→方”映射，使对角线时线速度与角速度都可能达到最大值（手感容易过激）。
  - 当前角速度为 `-normX * maxAngularSpeed`（右推→负角速度，理论上对应右转；但需结合底盘/控制器确认方向）。
- 速度下发接口：`POST /api/robot/velocity`（后端：`robot_api_server.py`）
- 系统自启与热点：
  - `robot-studio.service` 已在系统中 `enabled`，开机执行 `start_robot_studio.sh`，默认会启动热点（systemd 调用时会走 `--hotspot`）。
  - 热点由 `improved_hotspot.sh` 管理（不是 `hostapd.service` 方式）。

## 1. 目标与非目标

### 目标
1) 修复 Web 摇杆“角度/速度映射不合理”的问题，让操作手感更符合直觉、可控。
2) 增加键盘控制（WASD/方向键）用于电脑端调试与精细操控。
3) Web 界面增加“调试模式”：
   - 开启调试后：**下次开机**自动关闭（禁用）热点与上位机自启。
   - 同时执行：`sudo systemctl start ssh`（自动输入系统密码 `yahboom`）。

### 非目标（本轮不做，除非你追加确认）
- 不改动 ROS2 控制链/底盘控制器本身参数（只先修 Web 输入映射与系统调试开关）。
- 不移除/不加密硬编码密码（按你要求保持现状，但会在文档中标注安全风险）。

## 2. 需要你确认的行为预期（确认后我再实现）

### 2.1 摇杆期望（选择或补充）
- A. 对角线（右上/左上）时：线速度和角速度 **都能到最大值**（更激进）
- B. 对角线时：线速度/角速度按圆形幅值缩放（例如 45° 时各约 70%）**更线性、更稳**（推荐）

### 2.2 转向方向（必须确认）
- 右推 = 右转（顺时针）/ 左推 = 左转（逆时针）是否正确？
  - 如果你现在感觉反了，我会把符号翻转并提供一个“方向反转”开关（默认按你确认值写死）。

### 2.3 键盘按键映射（默认方案）
- `W/↑` 前进，`S/↓` 后退
- `A/←` 左转，`D/→` 右转
- `Space` 立即停止
- `Shift` 按住进入慢速（等价于慢速模式）

## 3. 实现方案（设计）

### 3.1 摇杆映射修复（Web）
计划改造 `web_interface/js/robot-control.js`：
1) 去掉当前“圆→方”映射逻辑，改为更符合直觉的 **圆形幅值映射**：
   - 计算 `x=deltaX/maxRadius`, `y=-deltaY/maxRadius`
   - 加入 deadzone（例如 0.08~0.12）
   - 幅值曲线（可选）：`scaled = ((r-deadzone)/(1-deadzone))^curve`（curve 默认 1.5 左右更细腻）
   - `linear = y_scaled * maxLinearSpeed * slowMultiplier`
   - `angular = (-x_scaled) * maxAngularSpeed * slowMultiplier`
2) 增加“转向灵敏度”系数（不改 maxAngularSpeed 的全局含义）：
   - `angular *= turnGain`（UI 可调，默认 1.0）
3) 增加“调试可视化”（可在调试面板开启）：
   - 显示当前 `x/y/r/linear/angular` 数值与是否触发 deadzone
4) 兼容移动端触控与桌面端鼠标。

**验收标准**
- 推到正上：只前进不转向；推到正右：原地右转（或小转向，取决于你的确认）；
- 推到 45°：不会突然角速度飙升（除非你选择 2.1 的 A 方案）；
- 松手回中：快速归零，机器人停止。

### 3.2 键盘控制（Web）
在 `web_interface/js/robot-control.js` 增加 keyboard driver：
1) `keydown/keyup` 维护按键状态集合（避免 key repeat 导致抖动）。
2) 使用 `setInterval(50ms)` 或复用现有节流逻辑，持续发送速度（按键保持时）。
3) 当页面焦点在输入框/弹窗时，键盘控制不生效（避免误触）。
4) 提供 UI 开关：
   - “键盘控制：开/关”
   - “显示按键提示”

**验收标准**
- 按 `W` 前进、松开立即停；
- `W+D` 前进+右转；
- `Space` 立即停止并清空按键状态。

### 3.3 调试模式（Web + 后端 + 系统）
目标：不靠手工 ssh/手工停自启，在 Web 上一键进入“调试启动形态”。

#### 3.3.1 后端新增命令（API）
在 `robot_api_server.py` 增加系统命令：
- `set_debug_mode`：
  - `parameters.enable: true/false`
  - `parameters.apply_now: false`（默认不立刻停服务/关热点，只影响“下次开机”的自启行为）
  - 内部写入状态文件：`/home/jetson/.robot-studio/debug_mode.json`
- `get_debug_mode`：返回当前状态与系统侧检测结果（robot-studio.service 是否 enabled、ssh 是否 active 等）

#### 3.3.2 开启调试时要做的系统动作（按你要求）
当 enable=true：
1) 关闭自启（影响下次开机）：
   - `systemctl disable robot-studio.service`
   - （若存在其他热点自启服务也一并 disable；当前系统未发现 `robot-wifi-hotspot.service`，但会做兼容判断）
2) 启动 SSH：
   - 执行 `sudo systemctl start ssh`
   - 使用 `echo yahboom | sudo -S ...` 方式自动输入密码（仅当当前进程非 root 且需要 sudo 时）
3) 写日志：`/home/jetson/.robot-studio/logs/debug_mode.log`

当 enable=false（恢复）：
1) `systemctl enable robot-studio.service`
2) （可选）不自动启动服务，只恢复“下次开机自启”（避免你当前网络断开）
3) ssh 不做停止（默认保持开启，除非你要求关闭）

#### 3.3.3 Web 界面交互
在 `web_interface/index.html` 增加“调试面板”：
- Toggle：调试模式（打开时弹窗二次确认）
- 状态展示：
  - robot-studio.service：enabled/disabled
  - ssh：active/inactive
  - 热点：检测 hostapd/dnsmasq 是否在运行（仅展示，不强制操作）
- 按钮：
  - “开启调试（下次开机不自启+启 SSH）”
  - “恢复正常自启”

**安全提示（会在 UI 明示）**
- 开启调试会修改系统服务 enable 状态；如果你依赖热点访问，请确保有替代网络方式再重启。
- 硬编码密码存在安全风险（按你要求保留）。

## 4. 测试与回滚计划

### 4.1 摇杆/键盘测试（不需要重启）
- 在浏览器打开 Web 页，开启“数值调试”，检查线速度/角速度随输入变化是否符合预期。
- 验证 `Space` 急停与松手归零。

### 4.2 调试模式测试（尽量不影响现场网络）
1) 先只调用 `get_debug_mode`（只读）。
2) 再启用 debug（`apply_now=false`）：只改变下次开机自启，不立即停服务/断网。
3) 验证：
   - `systemctl is-enabled robot-studio.service` 变为 disabled
   - `systemctl is-active ssh` 变为 active
4) 回滚：关闭 debug（恢复 enable）。

## 5. 实施顺序（你确认后我按此执行）
1) 摇杆映射重构 + 调试数值显示
2) 键盘控制（含 UI 开关）
3) 后端调试模式 API（get/set）
4) Web 调试面板接入 + 系统动作（disable 自启、start ssh）
5) 逐项现场测试与日志核对

---

## ✅ 请你在此回复确认
1) 2.1 你选 A 还是 B？
2) 右推=右转 是否正确？（2.2）
3) 键盘映射是否接受默认方案？（2.3）
4) 调试模式“恢复正常”是否需要同时 `systemctl stop ssh`？（默认不停止）
