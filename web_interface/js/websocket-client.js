/**
 * WebSocket Client JavaScript
 * WebSocket连接和实时数据处理
 */

class WebSocketClient {
    constructor() {
        // 自动检测正确的WebSocket地址
        const hostname = window.location.hostname;
        this.wsUrl = `ws://${hostname}:8001`;
        this.ws = null;
        this.reconnectInterval = 3000; // 3秒重连间隔
        this.maxReconnectAttempts = 20; // 增加重连次数
        this.reconnectAttempts = 0;
        this.isConnected = false;
        this.reconnectTimer = null;

        this.connect();
    }
    
    connect() {
        try {
            this.ws = new WebSocket(this.wsUrl);
            
            this.ws.onopen = (event) => {
                this.onOpen(event);
            };
            
            this.ws.onmessage = (event) => {
                this.onMessage(event);
            };
            
            this.ws.onclose = (event) => {
                this.onClose(event);
            };
            
            this.ws.onerror = (event) => {
                this.onError(event);
            };
            
        } catch (error) {
            console.error('WebSocket连接失败:', error);
            this.scheduleReconnect();
        }
    }
    
    onOpen(event) {
        console.log('WebSocket连接已建立');
        this.isConnected = true;
        this.reconnectAttempts = 0;

        // 清除重连定时器
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
            this.reconnectTimer = null;
        }

        // 更新连接状态
        this.updateConnectionStatus(true);

        // 添加日志
        if (window.robotController) {
            robotController.addLog('WebSocket连接已建立', 'success');
        }

        // 请求当前数据
        this.requestCurrentData();
    }
    
    onMessage(event) {
        try {
            const message = JSON.parse(event.data);
            this.handleMessage(message);
        } catch (error) {
            // JSON解析失败 - 可能是数据太大或格式错误
            const dataPreview = event.data.substring(0, 500);
            console.error('解析WebSocket消息失败:', error);
            console.error('数据预览（前500字符）:', dataPreview);
            console.error('数据总长度:', event.data.length);

            // 尝试检测是否是地图数据
            if (event.data.includes('"type":"map_data"') || event.data.includes('map_data')) {
                console.warn('⚠️ 地图数据JSON解析失败，可能是数据过大或包含非法字符');
            }
        }
    }
    
    onClose(event) {
        console.log('WebSocket连接已关闭:', event.code, event.reason);
        this.isConnected = false;
        
        // 更新连接状态
        this.updateConnectionStatus(false);
        
        // 添加日志
        if (window.robotController) {
            robotController.addLog(`WebSocket连接已关闭: ${event.reason}`, 'warning');
        }
        
        // 尝试重连
        if (event.code !== 1000) { // 不是正常关闭
            this.scheduleReconnect();
        }
    }
    
    onError(event) {
        console.error('WebSocket错误:', event);
        
        // 添加日志
        if (window.robotController) {
            robotController.addLog('WebSocket连接错误', 'error');
        }
    }
    
    scheduleReconnect() {
        if (this.reconnectTimer) {
            clearTimeout(this.reconnectTimer);
        }

        if (this.reconnectAttempts < this.maxReconnectAttempts) {
            this.reconnectAttempts++;

            console.log(`${this.reconnectInterval / 1000}秒后尝试重连 (${this.reconnectAttempts}/${this.maxReconnectAttempts})`);

            if (window.robotController) {
                robotController.addLog(
                    `${this.reconnectInterval / 1000}秒后尝试重连 (${this.reconnectAttempts}/${this.maxReconnectAttempts})`,
                    'info'
                );
            }

            this.reconnectTimer = setTimeout(() => {
                this.connect();
            }, this.reconnectInterval);
        } else {
            console.error('达到最大重连次数，停止重连');

            if (window.robotController) {
                robotController.addLog('达到最大重连次数，停止重连', 'error');
            }
        }
    }
    
    handleMessage(message) {
        if (message.type === 'topic_data') {
            this.handleTopicData(message.topic, message.data);
        } else if (message.type === 'map_data') {
            // 处理地图数据 - 减少日志输出
            if (window.mapDisplay) {
                // 检查数据是否有效
                if (!message.data || message.data.length === 0) {
                    console.warn('收到的地图数据为空');
                    return;
                }

                // 节流：检查是否与上次数据相同
                const currentSize = `${message.width}x${message.height}`;
                if (!this._lastMapSize) {
                    console.log('🔵 首次收到地图数据:', currentSize);
                    this._lastMapSize = currentSize;
                } else if (this._lastMapSize !== currentSize) {
                    console.log('🔵 地图尺寸变化:', this._lastMapSize, '→', currentSize);
                    this._lastMapSize = currentSize;
                }
                // 尺寸未变时不输出日志

                // 转换为期望的格式
                const mapData = {
                    info: {
                        width: message.width,
                        height: message.height,
                        resolution: message.resolution,
                        origin: {
                            position: message.origin
                        }
                    },
                    data: message.data
                };
                window.mapDisplay.updateMapData(mapData);
            } else {
                console.error('mapDisplay 未初始化');
            }
        } else if (message.type === 'service_response') {
            this.handleServiceResponse(message);
        } else if (message.type === 'status_update') {
            this.handleStatusUpdate(message.data);
        } else if (message.type === 'error') {
            console.error('WebSocket错误:', message.message);
        } else {
            console.log('未知消息类型:', message.type, message);
        }
    }
    
    handleTopicData(topic, data) {
        switch (topic) {
            case 'map':
                // 禁用此路径，统一使用map_data类型处理地图数据
                // 避免双重更新导致地图抖动
                console.log('⚠️ [topic_data/map] 已禁用，使用map_data类型代替');
                break;

            case 'odom':
                if (window.mapDisplay) {
                    window.mapDisplay.updateRobotPose(data.pose.pose);
                }
                break;



            case 'robot_pose':
                if (window.mapDisplay) {
                    window.mapDisplay.updateRobotPose(data);
                }
                break;

            default:
                console.log(`收到话题数据: ${topic}`, data);
        }
    }
    
    handleServiceResponse(message) {
        console.log('服务响应:', message);
        
        if (window.robotController) {
            const status = message.success ? 'success' : 'error';
            const text = message.success ? '服务调用成功' : `服务调用失败: ${message.error}`;
            robotController.addLog(text, status);
        }
    }
    
    handleStatusUpdate(data) {
        console.log('状态更新:', data);
        
        // 可以在这里处理系统状态更新
        if (window.robotController) {
            robotController.addLog(`系统状态更新: ${JSON.stringify(data)}`, 'info');
        }
    }
    
    requestCurrentData() {
        // 请求当前地图数据
        this.sendMessage({
            type: 'request_map'
        });

        // 订阅地图数据更新
        this.sendMessage({
            type: 'subscribe',
            topic: 'map'
        });

        // 请求当前的机器人位姿
        this.sendMessage({
            type: 'subscribe',
            topic: 'odom'
        });


    }
    
    sendMessage(message) {
        if (this.isConnected && this.ws.readyState === WebSocket.OPEN) {
            try {
                this.ws.send(JSON.stringify(message));
                return true;
            } catch (error) {
                console.error('发送WebSocket消息失败:', error);
                return false;
            }
        } else {
            console.warn('WebSocket未连接，无法发送消息');
            return false;
        }
    }
    
    sendVelocityCommand(linear, angular) {
        return this.sendMessage({
            type: 'cmd_vel',
            data: {
                linear: { x: linear, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: angular }
            }
        });
    }
    
    sendGoalPose(x, y, theta = 0, frameId = 'map') {
        // 将角度转换为四元数
        const qz = Math.sin(theta / 2);
        const qw = Math.cos(theta / 2);
        
        return this.sendMessage({
            type: 'goal_pose',
            data: {
                frame_id: frameId,
                pose: {
                    position: { x: x, y: y, z: 0 },
                    orientation: { x: 0, y: 0, z: qz, w: qw }
                }
            }
        });
    }
    
    callService(serviceName, parameters = {}) {
        return this.sendMessage({
            type: 'service_call',
            service: serviceName,
            parameters: parameters
        });
    }
    
    updateConnectionStatus(isConnected) {
        // 更新UI中的连接状态
        const statusIndicator = document.getElementById('connectionStatus');
        const statusText = document.getElementById('connectionText');
        
        if (statusIndicator && statusText) {
            if (isConnected) {
                statusIndicator.className = 'status-indicator status-online';
                statusText.textContent = '在线';
            } else {
                statusIndicator.className = 'status-indicator status-offline';
                statusText.textContent = '离线';
            }
        }
    }
    
    disconnect() {
        if (this.ws) {
            this.ws.close(1000, '用户主动断开连接');
        }
    }
    
    // 重置重连计数器（用于手动重连）
    resetReconnectAttempts() {
        this.reconnectAttempts = 0;
    }
    
    // 手动重连
    reconnect() {
        this.resetReconnectAttempts();
        if (this.ws) {
            this.ws.close();
        }
        setTimeout(() => {
            this.connect();
        }, 1000);
    }
}

// 添加一些全局辅助函数
window.wsReconnect = function() {
    if (window.wsClient) {
        wsClient.reconnect();
        if (window.robotController) {
            robotController.addLog('手动重连WebSocket', 'info');
        }
    }
};

window.wsSendGoal = function(x, y, theta = 0) {
    if (window.wsClient) {
        const success = wsClient.sendGoalPose(x, y, theta);
        if (window.robotController) {
            const message = success ? 
                `发送目标点: (${x.toFixed(2)}, ${y.toFixed(2)}, ${(theta * 180 / Math.PI).toFixed(1)}°)` :
                '发送目标点失败';
            const type = success ? 'success' : 'error';
            robotController.addLog(message, type);
        }
        return success;
    }
    return false;
};

window.wsCallService = function(serviceName) {
    if (window.wsClient) {
        const success = wsClient.callService(serviceName);
        if (window.robotController) {
            const message = success ? 
                `调用服务: ${serviceName}` :
                `调用服务失败: ${serviceName}`;
            const type = success ? 'info' : 'error';
            robotController.addLog(message, type);
        }
        return success;
    }
    return false;
};
