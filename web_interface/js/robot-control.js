/**
 * Robot Control JavaScript
 * 保留：地图显示 + 虚拟摇杆控制 + 状态/日志 + 导航/探索控制
 * 移除：文件管理/点云三级处理等入口
 */

class RobotController {
    constructor() {
        const hostname = window.location.hostname;
        this.apiBaseUrl = `http://${hostname}:8000/api`;

        this.isSlowMode = false;
        this.maxLinearSpeed = 0.3;
        this.maxAngularSpeed = 1.0;

        // 摇杆映射参数（对角线可到双满速 + 死区 + 可控的非线性）
        this.joystickDeadzone = 0.08;
        this.joystickCurve = 1.0; // 1.0=线性；>1 更细腻

        // 阿克曼“原地转向”兼容：仅转向时给一点前进速度，避免出现倒车转向的手感
        this.turnInPlaceLinearBias = 0.05; // m/s
        this.turnInPlaceLinearThreshold = 0.02; // m/s
        this.turnInPlaceAngularThreshold = 0.2; // rad/s

        // 键盘控制
        this.keyboardControlEnabled = true;
        this.keyboardPressedKeys = new Set();
        this.keyboardSendIntervalMs = 100;
        this.keyboardSendTimer = null;

        this.currentLinearVel = 0;
        this.currentAngularVel = 0;

        this.lastCommandTime = 0;
        this.commandInterval = 50;
        this.pendingCommand = null;
        this.isCommandPending = false;

        this.initializeControls();
        this.initializeJoystick();
        this.initializeKeyboardControl();
        this.startStatusUpdates();
    }

    clamp(value, min, max) {
        return Math.max(min, Math.min(max, value));
    }

    // 圆形摇杆坐标(-1~1, r<=1) -> “激进方形映射”(对角线双满速)，同时保持角度方向不变
    mapJoystickCircleToSquare(x, y) {
        const r = Math.hypot(x, y);
        if (r < this.joystickDeadzone) {
            return { x: 0, y: 0, r: 0 };
        }

        const dirX = x / r;
        const dirY = y / r;

        const mag = (r - this.joystickDeadzone) / (1 - this.joystickDeadzone);
        const magCurved = Math.pow(this.clamp(mag, 0, 1), this.joystickCurve);

        const maxDirAbs = Math.max(Math.abs(dirX), Math.abs(dirY));
        const squareFactor = maxDirAbs > 0 ? 1 / maxDirAbs : 0;

        const outX = this.clamp(dirX * squareFactor * magCurved, -1, 1);
        const outY = this.clamp(dirY * squareFactor * magCurved, -1, 1);

        return { x: outX, y: outY, r: magCurved };
    }

    isTextInputFocused() {
        const el = document.activeElement;
        if (!el) return false;
        const tag = (el.tagName || '').toLowerCase();
        if (tag === 'input' || tag === 'textarea' || tag === 'select') return true;
        if (el.isContentEditable) return true;
        return false;
    }

    stopKeyboardControlTimer() {
        if (this.keyboardSendTimer) {
            clearInterval(this.keyboardSendTimer);
            this.keyboardSendTimer = null;
        }
    }

    updateKeyboardVelocity() {
        if (!this.keyboardControlEnabled) return;

        const keys = this.keyboardPressedKeys;
        const forward = keys.has('KeyW') || keys.has('ArrowUp');
        const backward = keys.has('KeyS') || keys.has('ArrowDown');
        const left = keys.has('KeyA') || keys.has('ArrowLeft');
        const right = keys.has('KeyD') || keys.has('ArrowRight');
        const shift = keys.has('ShiftLeft') || keys.has('ShiftRight');

        let axisY = 0;
        if (forward && !backward) axisY = 1;
        else if (backward && !forward) axisY = -1;

        let axisX = 0;
        if (right && !left) axisX = 1;
        else if (left && !right) axisX = -1;

        const speedMultiplier = (this.isSlowMode || shift) ? 0.3 : 1.0;
        let linear = axisY * this.maxLinearSpeed * speedMultiplier;
        let angular = -axisX * this.maxAngularSpeed * speedMultiplier; // 右=负角速度（右转）

        if (Math.abs(linear) < this.turnInPlaceLinearThreshold &&
            Math.abs(angular) > this.turnInPlaceAngularThreshold) {
            linear = this.turnInPlaceLinearBias * speedMultiplier;
        }

        this.currentLinearVel = linear;
        this.currentAngularVel = angular;
        this.updateVelocityDisplay();
        this.sendVelocityCommand(this.currentLinearVel, this.currentAngularVel);
    }

    initializeKeyboardControl() {
        const toggle = document.getElementById('keyboardControlToggle');
        if (toggle) {
            this.keyboardControlEnabled = !!toggle.checked;
            toggle.addEventListener('change', () => {
                this.keyboardControlEnabled = !!toggle.checked;
                if (!this.keyboardControlEnabled) {
                    this.keyboardPressedKeys.clear();
                    this.stopKeyboardControlTimer();
                    this.sendVelocityCommand(0, 0);
                    this.addLog('键盘控制已关闭', 'info');
                } else {
                    this.addLog('键盘控制已开启', 'info');
                }
            });
        }

        const handledCodes = new Set([
            'KeyW', 'KeyA', 'KeyS', 'KeyD',
            'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight',
            'Space', 'ShiftLeft', 'ShiftRight',
        ]);

        window.addEventListener('keydown', (e) => {
            if (!this.keyboardControlEnabled) return;
            if (this.isTextInputFocused()) return;

            const code = e.code || '';
            if (!handledCodes.has(code)) return;

            // 防止方向键滚动页面、空格触发滚动
            if (code.startsWith('Arrow') || code === 'Space') {
                e.preventDefault();
            }

            // 空格：立即停止并清空按键状态
            if (code === 'Space') {
                this.keyboardPressedKeys.clear();
                this.stopKeyboardControlTimer();
                this.currentLinearVel = 0;
                this.currentAngularVel = 0;
                this.updateVelocityDisplay();
                this.sendVelocityCommand(0, 0);
                this.addLog('键盘急停', 'warning');
                return;
            }

            this.keyboardPressedKeys.add(code);

            if (!this.keyboardSendTimer) {
                this.keyboardSendTimer = setInterval(
                    () => this.updateKeyboardVelocity(),
                    this.keyboardSendIntervalMs
                );
            }

            this.updateKeyboardVelocity();
        });

        window.addEventListener('keyup', (e) => {
            const code = e.code || '';
            if (!this.keyboardPressedKeys.has(code)) return;
            this.keyboardPressedKeys.delete(code);

            // 如果没有任何运动相关按键，停止定时器并发送停止
            const hasMoveKeys =
                this.keyboardPressedKeys.has('KeyW') || this.keyboardPressedKeys.has('ArrowUp') ||
                this.keyboardPressedKeys.has('KeyS') || this.keyboardPressedKeys.has('ArrowDown') ||
                this.keyboardPressedKeys.has('KeyA') || this.keyboardPressedKeys.has('ArrowLeft') ||
                this.keyboardPressedKeys.has('KeyD') || this.keyboardPressedKeys.has('ArrowRight');

            if (!hasMoveKeys) {
                this.stopKeyboardControlTimer();
                this.currentLinearVel = 0;
                this.currentAngularVel = 0;
                this.updateVelocityDisplay();
                this.sendVelocityCommand(0, 0);
            } else {
                this.updateKeyboardVelocity();
            }
        });

        window.addEventListener('blur', () => {
            this.keyboardPressedKeys.clear();
            this.stopKeyboardControlTimer();
            this.currentLinearVel = 0;
            this.currentAngularVel = 0;
            this.updateVelocityDisplay();
            this.sendVelocityCommand(0, 0);
        });
    }

    initializeControls() {
        const stopBtn = document.getElementById('stopBtn');
        if (stopBtn) {
            stopBtn.addEventListener('click', () => this.stopRobot());
        }

        const slowModeBtn = document.getElementById('slowModeBtn');
        if (slowModeBtn) {
            slowModeBtn.addEventListener('click', () => this.toggleSlowMode());
        }

        const clearLogBtn = document.getElementById('clearLogBtn');
        if (clearLogBtn) {
            clearLogBtn.addEventListener('click', () => this.clearLog());
        }

        const startNavBtn = document.getElementById('startNavBtn');
        if (startNavBtn) {
            startNavBtn.addEventListener('click', () => this.executeSystemCommand('start_path_planning'));
        }

        const stopNavBtn = document.getElementById('stopNavBtn');
        if (stopNavBtn) {
            stopNavBtn.addEventListener('click', () => {
                if (confirm('确认停止导航系统？机器人将停止自主运动。')) {
                    this.executeSystemCommand('stop_path_planning');
                }
            });
        }

        const startExplorationBtn = document.getElementById('startExplorationBtn');
        if (startExplorationBtn) {
            startExplorationBtn.addEventListener('click', () => this.executeSystemCommand('start_exploration'));
        }

        const stopExplorationBtn = document.getElementById('stopExplorationBtn');
        if (stopExplorationBtn) {
            stopExplorationBtn.addEventListener('click', () => this.executeSystemCommand('stop_exploration'));
        }

        const saveMapBtn = document.getElementById('saveMapBtn');
        if (saveMapBtn) {
            saveMapBtn.addEventListener('click', () => this.executeSystemCommand('save_map'));
        }

        const clearMapBtn = document.getElementById('clearMapBtn');
        if (clearMapBtn) {
            clearMapBtn.addEventListener('click', () => {
                if (confirm('确认清除当前地图？此操作会清除设备当前建图数据。')) {
                    this.executeSystemCommand('clear_map');
                }
            });
        }

        const emergencyStopBtn = document.getElementById('emergencyStopBtn');
        if (emergencyStopBtn) {
            emergencyStopBtn.addEventListener('click', () => {
                if (confirm('确认执行紧急停止？将立即发送多次零速度命令。')) {
                    this.executeSystemCommand('emergency_stop');
                }
            });
        }

        const shutdownBtn = document.getElementById('shutdownBtn');
        if (shutdownBtn) {
            shutdownBtn.addEventListener('click', () => {
                const token = prompt('输入“关机”确认关机（将断开连接）；其它输入将取消。');
                if (token !== '关机') {
                    this.addLog('已取消关机', 'info');
                    return;
                }

                shutdownBtn.disabled = true;
                shutdownBtn.classList.add('disabled');
                this.executeSystemCommand('shutdown_system', { confirm: '关机' });
            });
        }

        const testModeBtn = document.getElementById('testModeBtn');
        if (testModeBtn) {
            testModeBtn.addEventListener('click', () => {
                const warning =
                    '即将进入一次性【测试模式】并重启系统：\n' +
                    '1) 下次开机不会启动热点/上位机（Web 将不可用）\n' +
                    '2) 下次开机会启动 SSH（用于调试）\n' +
                    '3) 仅生效一次：再重启后会恢复正常自启（热点+上位机）\n\n' +
                    '确认继续？';

                if (!confirm(warning)) {
                    this.addLog('已取消进入测试模式', 'info');
                    return;
                }

                const token = prompt('输入“测试模式”确认执行（将立即重启）；其它输入将取消。');
                if (token !== '测试模式') {
                    this.addLog('已取消进入测试模式', 'info');
                    return;
                }

                testModeBtn.disabled = true;
                testModeBtn.classList.add('disabled');
                this.executeSystemCommand('enter_test_mode_once', { confirm: '测试模式' });
                this.addLog('测试模式请求已发送，系统即将重启...', 'warning');
            });
        }
    }

    initializeJoystick() {
        const joystick = document.getElementById('joystick');
        const knob = document.getElementById('joystickKnob');
        if (!joystick || !knob) return;

        let isDragging = false;
        let joystickRect = joystick.getBoundingClientRect();

        const updateJoystickRect = () => {
            joystickRect = joystick.getBoundingClientRect();
        };

        window.addEventListener('resize', updateJoystickRect);

        const handleStart = (e) => {
            isDragging = true;
            updateJoystickRect();
            e.preventDefault();
        };

        const handleMove = (e) => {
            if (!isDragging) return;
            e.preventDefault();

            const clientX = e.touches ? e.touches[0].clientX : e.clientX;
            const clientY = e.touches ? e.touches[0].clientY : e.clientY;

            const centerX = joystickRect.left + joystickRect.width / 2;
            const centerY = joystickRect.top + joystickRect.height / 2;

            let deltaX = clientX - centerX;
            let deltaY = clientY - centerY;

            const maxRadius = joystickRect.width / 2 - 20;
            const distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

            if (distance > maxRadius) {
                deltaX = (deltaX / distance) * maxRadius;
                deltaY = (deltaY / distance) * maxRadius;
            }

            knob.style.transform = `translate(${deltaX - 20}px, ${deltaY - 20}px)`;

            // 圆盘坐标（-1~1, r<=1）
            const rawX = deltaX / maxRadius;
            const rawY = -deltaY / maxRadius;

            // 对角线双满速映射 + 死区/曲线
            const mapped = this.mapJoystickCircleToSquare(rawX, rawY);

            const speedMultiplier = this.isSlowMode ? 0.3 : 1.0;
            let linear = mapped.y * this.maxLinearSpeed * speedMultiplier;
            let angular = -mapped.x * this.maxAngularSpeed * speedMultiplier;

            // 仅转向时给一点前进速度（阿克曼车无法真正原地转向）
            if (Math.abs(linear) < this.turnInPlaceLinearThreshold &&
                Math.abs(angular) > this.turnInPlaceAngularThreshold) {
                linear = this.turnInPlaceLinearBias * speedMultiplier;
            }

            this.currentLinearVel = linear;
            this.currentAngularVel = angular;

            this.updateVelocityDisplay();
            this.sendVelocityCommand(this.currentLinearVel, this.currentAngularVel);
        };

        const handleEnd = (e) => {
            if (!isDragging) return;
            isDragging = false;
            e.preventDefault();

            knob.style.transform = 'translate(-50%, -50%)';
            this.currentLinearVel = 0;
            this.currentAngularVel = 0;
            this.updateVelocityDisplay();
            this.sendVelocityCommand(0, 0);
        };

        joystick.addEventListener('mousedown', handleStart);
        document.addEventListener('mousemove', handleMove);
        document.addEventListener('mouseup', handleEnd);

        joystick.addEventListener('touchstart', handleStart);
        document.addEventListener('touchmove', handleMove);
        document.addEventListener('touchend', handleEnd);
    }

    updateVelocityDisplay() {
        const linearVel = document.getElementById('linearVel');
        const angularVel = document.getElementById('angularVel');
        if (linearVel) linearVel.textContent = this.currentLinearVel.toFixed(2);
        if (angularVel) angularVel.textContent = this.currentAngularVel.toFixed(2);
    }

    async sendVelocityCommand(linear, angular) {
        const now = Date.now();
        if (now - this.lastCommandTime < this.commandInterval) {
            this.pendingCommand = { linear, angular };
            if (!this.isCommandPending) {
                this.isCommandPending = true;
                setTimeout(() => {
                    if (this.pendingCommand) {
                        this.sendVelocityCommandImmediate(
                            this.pendingCommand.linear,
                            this.pendingCommand.angular
                        );
                        this.pendingCommand = null;
                    }
                    this.isCommandPending = false;
                }, this.commandInterval - (now - this.lastCommandTime));
            }
            return;
        }

        this.sendVelocityCommandImmediate(linear, angular);
    }

    async sendVelocityCommandImmediate(linear, angular) {
        try {
            this.lastCommandTime = Date.now();

            const controller = new AbortController();
            const timeoutId = setTimeout(() => controller.abort(), 200);

            const response = await fetch(`${this.apiBaseUrl}/robot/velocity`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    linear_x: linear,
                    linear_y: 0,
                    linear_z: 0,
                    angular_x: 0,
                    angular_y: 0,
                    angular_z: angular
                }),
                signal: controller.signal
            });

            clearTimeout(timeoutId);
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }
        } catch (error) {
            if (error.name === 'AbortError') {
                this.addLog('控制命令超时，请检查网络连接', 'warning');
            } else {
                this.addLog(`发送速度命令失败: ${error.message}`, 'error');
            }
        }
    }

    async stopRobot() {
        try {
            const response = await fetch(`${this.apiBaseUrl}/robot/stop`, { method: 'POST' });
            if (!response.ok) {
                throw new Error(`HTTP error! status: ${response.status}`);
            }

            this.currentLinearVel = 0;
            this.currentAngularVel = 0;
            this.updateVelocityDisplay();
            this.addLog('机器人已停止', 'success');
        } catch (error) {
            this.addLog(`停止机器人失败: ${error.message}`, 'error');
        }
    }

    async executeSystemCommand(command, parameters = null) {
        try {
            this.addLog(`执行命令: ${command}`, 'info');

            const response = await fetch(`${this.apiBaseUrl}/system/command`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ command, parameters })
            });

            const data = await response.json().catch(() => ({}));
            const status = data.status || (response.ok ? 'success' : 'error');

            if (!response.ok || status === 'error') {
                const msg = data.message || `命令执行失败: ${response.status}`;
                const err = data.error ? ` (${data.error})` : '';
                this.addLog(`${msg}${err}`, 'error');
                return;
            }

            if (data.message) {
                this.addLog(data.message, status === 'warning' ? 'warning' : 'success');
            } else {
                this.addLog('命令已执行', 'success');
            }

            if (data.output) {
                const outputText = String(data.output).trim();
                if (outputText) {
                    this.addLog(outputText, 'info');
                }
            }
        } catch (error) {
            this.addLog(`命令请求失败: ${error.message}`, 'error');
        }
    }

    toggleSlowMode() {
        this.isSlowMode = !this.isSlowMode;
        const btn = document.getElementById('slowModeBtn');

        if (btn) {
            if (this.isSlowMode) {
                btn.classList.remove('btn-warning');
                btn.classList.add('btn-success');
                btn.innerHTML = '<i class="bi bi-speedometer2"></i> 慢速模式';
            } else {
                btn.classList.remove('btn-success');
                btn.classList.add('btn-warning');
                btn.innerHTML = '<i class="bi bi-speedometer"></i> 慢速';
            }
        }

        this.addLog(this.isSlowMode ? '已启用慢速模式' : '已禁用慢速模式', 'info');
    }

    async startStatusUpdates() {
        const updateStatus = async () => {
            try {
                const response = await fetch(`${this.apiBaseUrl}/status`);
                const data = await response.json();

                const ros2Status = document.getElementById('ros2Status');
                const mapStatus = document.getElementById('mapStatus');
                if (ros2Status) ros2Status.textContent = data.ros2_status;
                if (mapStatus) mapStatus.textContent = data.data_status.map ? '可用' : '不可用';

                this.updateConnectionStatus(true);
            } catch {
                this.updateConnectionStatus(false);
            }
        };

        updateStatus();
        setInterval(updateStatus, 5000);
    }

    updateConnectionStatus(isConnected) {
        const statusIndicator = document.getElementById('connectionStatus');
        const statusText = document.getElementById('connectionText');

        if (!statusIndicator || !statusText) return;

        if (isConnected) {
            statusIndicator.className = 'status-indicator status-online';
            statusText.textContent = '在线';
        } else {
            statusIndicator.className = 'status-indicator status-offline';
            statusText.textContent = '离线';
        }
    }

    addLog(message, type = 'info') {
        const logContainer = document.getElementById('logContainer');
        if (!logContainer) return;

        const timestamp = new Date().toLocaleTimeString();
        const logEntry = document.createElement('div');
        logEntry.className = `log-entry log-${type}`;

        let color = '#ffffff';
        switch (type) {
            case 'error': color = '#ff6b6b'; break;
            case 'success': color = '#51cf66'; break;
            case 'warning': color = '#ffd43b'; break;
            case 'info': color = '#74c0fc'; break;
        }

        logEntry.innerHTML = `<span style="color: ${color}">[${timestamp}] ${message}</span>`;
        logContainer.appendChild(logEntry);
        logContainer.scrollTop = logContainer.scrollHeight;

        while (logContainer.children.length > 100) {
            logContainer.removeChild(logContainer.firstChild);
        }
    }

    clearLog() {
        const logContainer = document.getElementById('logContainer');
        if (!logContainer) return;
        logContainer.innerHTML = '';
        this.addLog('日志已清除', 'info');
    }
}

document.addEventListener('DOMContentLoaded', () => {
    const controller = new RobotController();
    window.robotController = controller;
    window.robotControl = controller;

    window.mapDisplay = new MapDisplay();
    window.wsClient = new WebSocketClient();
});
