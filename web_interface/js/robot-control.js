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
        this.currentLinearVel = 0;
        this.currentAngularVel = 0;

        this.lastCommandTime = 0;
        this.commandInterval = 50;
        this.pendingCommand = null;
        this.isCommandPending = false;

        this.initializeControls();
        this.initializeJoystick();
        this.startStatusUpdates();
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
                const token = prompt('输入 POWER_OFF 确认关机（将断开连接）；其它输入将取消。');
                if (token !== 'POWER_OFF') {
                    this.addLog('已取消关机', 'info');
                    return;
                }

                shutdownBtn.disabled = true;
                shutdownBtn.classList.add('disabled');
                this.executeSystemCommand('shutdown_system', { confirm: 'POWER_OFF' });
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

            // 圆盘坐标（-1~1）
            let normX = deltaX / maxRadius;
            let normY = -deltaY / maxRadius;

            // 映射到“速度方形”，允许前进+转向同时接近最大值
            const r = Math.sqrt(normX * normX + normY * normY);
            if (r < 0.001) {
                normX = 0;
                normY = 0;
            } else {
                const maxAbs = Math.max(Math.abs(normX), Math.abs(normY));
                if (maxAbs > 0) {
                    const scale = r / maxAbs;
                    normX *= scale;
                    normY *= scale;
                }
            }

            const speedMultiplier = this.isSlowMode ? 0.3 : 1.0;
            this.currentLinearVel = normY * this.maxLinearSpeed * speedMultiplier;
            this.currentAngularVel = -normX * this.maxAngularSpeed * speedMultiplier;

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
