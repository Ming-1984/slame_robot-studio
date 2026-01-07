/**
 * Map Display JavaScript
 * 地图显示和机器人位置可视化
 */

class MapDisplay {
    constructor() {
        this.mapCanvas = document.getElementById('mapCanvas');
        this.mapCtx = this.mapCanvas.getContext('2d');

        // 地图数据
        this.mapData = null;
        this.robotPose = null;

        // 视图参数
        this.mapScale = 1.0;
        this.mapOffsetX = 0;
        this.mapOffsetY = 0;
        this.minScale = 0.1;
        this.maxScale = 10.0;

        // 渲染控制
        this.needsRedraw = true;
        this.lastMapDataTime = 0;

        // 地图缩放控制 - 只缩小不放大策略
        this.initialScale = null;  // 初始缩放比例（第一次设置后不再改变）
        this.hasSetInitialScale = false;  // 是否已设置初始缩放

        // 清除地图标志 - 用于忽略清除后的旧地图数据
        this.isMapCleared = false;
        this.clearMapTime = null;  // 记录清除地图的时间戳
        this.mapUpdateCount = 0;   // 清除后收到的地图更新次数

        // 地图尺寸稳定性过滤器 - 防止SLAM尺寸抖动
        this.sizeStabilityBuffer = [];  // 最近N次的尺寸记录
        this.sizeStabilityThreshold = 3;  // 需要连续N次相同尺寸才接受

        this.initializeCanvas();
        this.setupEventListeners();
        this.startRenderLoop();
    }
    
    initializeCanvas() {
        // 延迟设置画布大小，确保DOM完全加载
        setTimeout(() => {
            this.resizeCanvas();
            this.needsRedraw = true;
        }, 100);

        window.addEventListener('resize', () => {
            setTimeout(() => {
                this.resizeCanvas();
                this.needsRedraw = true;
            }, 100);
        });

        // 重置视图按钮
        document.getElementById('resetMapViewBtn').addEventListener('click', () => {
            this.resetView();
            this.needsRedraw = true;
        });
    }
    
    resizeCanvas() {
        const mapContainer = this.mapCanvas.parentElement;
        const rect = mapContainer.getBoundingClientRect();

        // 确保画布尺寸合理，留出足够的边距给标题和按钮
        const availableWidth = Math.max(300, rect.width - 40);
        const availableHeight = Math.max(200, rect.height - 100); // 增加顶部边距

        this.mapCanvas.width = availableWidth;
        this.mapCanvas.height = availableHeight;

        // 设置CSS样式确保画布正确显示
        this.mapCanvas.style.width = availableWidth + 'px';
        this.mapCanvas.style.height = availableHeight + 'px';
        this.mapCanvas.style.border = '1px solid #ddd';

        console.log(`画布尺寸调整: ${availableWidth}x${availableHeight}`);



        // 如果已有地图数据，重新调整视图
        if (this.mapData) {
            this.resetView();
        }
    }
    
    setupEventListeners() {
        // 地图缩放和拖拽
        this.mapCanvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            this.handleZoom(e);
        });
        
        let isDragging = false;
        let lastMouseX = 0;
        let lastMouseY = 0;
        
        this.mapCanvas.addEventListener('mousedown', (e) => {
            isDragging = true;
            lastMouseX = e.clientX;
            lastMouseY = e.clientY;
            this.mapCanvas.style.cursor = 'grabbing';
        });
        
        this.mapCanvas.addEventListener('mousemove', (e) => {
            if (isDragging) {
                const deltaX = e.clientX - lastMouseX;
                const deltaY = e.clientY - lastMouseY;
                
                this.mapOffsetX += deltaX;
                this.mapOffsetY += deltaY;
                
                lastMouseX = e.clientX;
                lastMouseY = e.clientY;
            }
        });
        
        this.mapCanvas.addEventListener('mouseup', () => {
            isDragging = false;
            this.mapCanvas.style.cursor = 'grab';
        });
        
        this.mapCanvas.addEventListener('mouseleave', () => {
            isDragging = false;
            this.mapCanvas.style.cursor = 'default';
        });
        
        this.mapCanvas.style.cursor = 'grab';
    }
    
    handleZoom(e) {
        const rect = this.mapCanvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;
        
        const zoomFactor = e.deltaY > 0 ? 0.9 : 1.1;
        const newScale = Math.max(this.minScale, Math.min(this.maxScale, this.mapScale * zoomFactor));
        
        if (newScale !== this.mapScale) {
            // 计算缩放中心
            const scaleChange = newScale / this.mapScale;
            
            this.mapOffsetX = mouseX - (mouseX - this.mapOffsetX) * scaleChange;
            this.mapOffsetY = mouseY - (mouseY - this.mapOffsetY) * scaleChange;
            
            this.mapScale = newScale;
        }
    }
    
    resetView() {
        if (this.mapData && this.mapData.info) {
            // 使用像素尺寸计算缩放
            const mapPixelWidth = this.mapData.info.width;
            const mapPixelHeight = this.mapData.info.height;

            // 确保画布尺寸有效
            const canvasWidth = this.mapCanvas.width || 800;
            const canvasHeight = this.mapCanvas.height || 600;

            // 计算适合画布的缩放比例，留出足够边距
            const margin = 80; // 为地图信息和比例尺留出空间
            const availableWidth = canvasWidth - margin;
            const availableHeight = canvasHeight - margin;

            const scaleX = availableWidth / mapPixelWidth;
            const scaleY = availableHeight / mapPixelHeight;
            const calculatedScale = Math.min(scaleX, scaleY, 2.0); // 限制最大缩放

            // 只缩小不放大策略
            if (!this.hasSetInitialScale) {
                // 第一次设置，记录初始缩放
                this.initialScale = calculatedScale;
                this.mapScale = calculatedScale;
                this.hasSetInitialScale = true;
                // 移除日志，提升性能
            } else {
                // 后续更新，只在需要缩小时调整
                if (calculatedScale < this.initialScale) {
                    this.mapScale = calculatedScale;
                    this.initialScale = calculatedScale; // 更新基准
                    // 移除日志，提升性能
                } else {
                    // 保持当前缩放，不放大
                    this.mapScale = this.initialScale;
                }
            }

            // 居中显示地图
            this.mapOffsetX = (canvasWidth - mapPixelWidth * this.mapScale) / 2;
            this.mapOffsetY = (canvasHeight - mapPixelHeight * this.mapScale) / 2;

            // 移除日志，提升性能
        } else {
            this.mapScale = 1.0;
            this.mapOffsetX = 0;
            this.mapOffsetY = 0;
        }
    }
    
    updateMapData(mapData) {
        if (!mapData || !mapData.info) {
            console.error('地图数据无效:', mapData);
            return;
        }

        const currentSize = `${mapData.info.width}x${mapData.info.height}`;

        // 如果地图已被清除，使用时间和次数来判断是否接受新地图
        if (this.isMapCleared) {
            const timeSinceClear = Date.now() - this.clearMapTime;
            this.mapUpdateCount++;

            // 策略：清除后等待3秒或收到10次更新后，接受新地图
            // 这样可以给SLAM足够时间清除旧地图
            if (timeSinceClear > 3000 || this.mapUpdateCount > 10) {
                console.log('🆕 清除后接受新地图:', currentSize,
                    `(等待${(timeSinceClear/1000).toFixed(1)}秒, 收到${this.mapUpdateCount}次更新)`);
                this.isMapCleared = false;
                this.mapUpdateCount = 0;
                // 清除稳定性缓冲区
                this.sizeStabilityBuffer = [];
            } else {
                // 仍在等待期，忽略数据
                if (this.mapUpdateCount === 1 || this.mapUpdateCount % 5 === 0) {
                    // 每5次更新输出一次日志，避免刷屏
                    console.log(`⏭️ 等待SLAM清除地图... (${(timeSinceClear/1000).toFixed(1)}秒, ${this.mapUpdateCount}次更新)`);
                }
                return;
            }
        }

        // 尺寸稳定性检查 - 防止SLAM尺寸抖动导致地图不断重绘
        this.sizeStabilityBuffer.push(currentSize);
        if (this.sizeStabilityBuffer.length > this.sizeStabilityThreshold) {
            this.sizeStabilityBuffer.shift();
        }

        // 检查最近N次尺寸是否一致
        const isStable = this.sizeStabilityBuffer.length === this.sizeStabilityThreshold &&
                        this.sizeStabilityBuffer.every(size => size === currentSize);

        if (!isStable && this.mapData) {
            // 尺寸不稳定，暂时忽略（但记录日志）
            const oldSize = `${this.mapData.info.width}x${this.mapData.info.height}`;
            if (oldSize !== currentSize) {
                console.warn(`⏸️ 地图尺寸不稳定，等待稳定后更新: ${oldSize} → ${currentSize} (缓冲: [${this.sizeStabilityBuffer.join(', ')}])`);
            }
            return;
        }

        // 检查是否是第一次接收地图
        const isFirstMap = !this.mapData;

        // 完全禁用自动缩放调整 - 只在第一次设置
        let needAdjustScale = false;

        if (isFirstMap) {
            // 第一次接收地图 - 设置初始缩放
            needAdjustScale = true;
            console.log('📍 首次地图:', currentSize);
        } else if (this.mapData) {
            // 后续更新 - 检查尺寸是否变化
            const oldSize = `${this.mapData.info.width}x${this.mapData.info.height}`;

            if (oldSize !== currentSize) {
                // 尺寸变化了 - 这是问题的关键！
                const widthDiff = mapData.info.width - this.mapData.info.width;
                const heightDiff = mapData.info.height - this.mapData.info.height;

                console.warn('⚠️ 地图尺寸变化:', oldSize, '→', currentSize,
                    `(Δw=${widthDiff}, Δh=${heightDiff})`);

                // 记录到诊断数组（最多保留最近10次）
                if (!this.sizeChangeHistory) {
                    this.sizeChangeHistory = [];
                }
                this.sizeChangeHistory.push({
                    time: new Date().toLocaleTimeString(),
                    from: oldSize,
                    to: currentSize,
                    diff: {w: widthDiff, h: heightDiff}
                });
                if (this.sizeChangeHistory.length > 10) {
                    this.sizeChangeHistory.shift();
                }
            }

            // 不再自动调整缩放，保持用户视图
            needAdjustScale = false;
        }

        this.mapData = mapData;

        // 只在尺寸真正变化时更新DOM（避免频繁DOM操作）
        const currentResolution = mapData.info.resolution.toFixed(3);

        const mapSizeElement = document.getElementById('mapSize');
        const mapResolutionElement = document.getElementById('mapResolution');

        if (mapSizeElement && mapSizeElement.textContent !== currentSize) {
            mapSizeElement.textContent = currentSize;
        }
        if (mapResolutionElement && mapResolutionElement.textContent !== currentResolution) {
            mapResolutionElement.textContent = currentResolution;
        }

        // 只在需要时调整缩放（第一次或地图变大）
        if (needAdjustScale) {
            this.resetView();
        }

        // 绘制地图（使用节流，避免频繁重绘）
        this.requestMapRedraw();
    }
    
    updateRobotPose(poseData) {
        this.robotPose = poseData;

        // 更新位姿显示
        if (poseData && poseData.position) {
            const robotXElement = document.getElementById('robotX');
            const robotYElement = document.getElementById('robotY');
            const robotThetaElement = document.getElementById('robotTheta');

            if (robotXElement) robotXElement.textContent = poseData.position.x.toFixed(2);
            if (robotYElement) robotYElement.textContent = poseData.position.y.toFixed(2);

            // 计算角度
            if (robotThetaElement && poseData.orientation) {
                const q = poseData.orientation;
                const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
                robotThetaElement.textContent = (theta * 180 / Math.PI).toFixed(1) + '°';
            }
        } else {
            // 调试：如果数据结构不对，输出警告
            console.warn('机器人位姿数据结构不正确:', poseData);
        }

        // 如果有地图数据，请求重绘（使用节流）
        if (this.mapData) {
            this.requestMapRedraw();
        }
    }

    // 节流的地图重绘请求 - 统一使用needsRedraw标志
    requestMapRedraw() {
        // 简单标记需要重绘，由renderLoop统一处理
        this.needsRedraw = true;
    }


    
    startRenderLoop() {
        const render = () => {
            if (this.needsRedraw) {
                this.renderMap();
                this.needsRedraw = false;
            }
            requestAnimationFrame(render);
        };
        render();
    }
    
    renderMap() {
        const ctx = this.mapCtx;
        ctx.clearRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);

        // 绘制背景
        ctx.fillStyle = '#f0f0f0';
        ctx.fillRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);

        if (!this.mapData || !this.mapData.data) {
            // 显示无地图数据提示
            ctx.fillStyle = '#666';
            ctx.font = '16px Arial';
            ctx.textAlign = 'center';
            ctx.fillText('等待地图数据...', this.mapCanvas.width / 2, this.mapCanvas.height / 2);
            return;
        }

        // 移除所有console.log，提升性能

        ctx.save();
        ctx.translate(this.mapOffsetX, this.mapOffsetY);
        ctx.scale(this.mapScale, this.mapScale);

        // 绘制地图
        this.drawOccupancyGrid();

        // 绘制机器人
        if (this.robotPose) {
            this.drawRobot();
        }

        ctx.restore();

        // 绘制比例尺和地图信息
        this.drawScale();
        this.drawMapInfo();
    }

    drawMapInfo() {
        if (!this.mapData || !this.mapData.info) return;

        const ctx = this.mapCtx;
        const mapInfo = this.mapData.info;

        // 在左上角显示地图信息
        ctx.save();
        ctx.fillStyle = 'rgba(255, 255, 255, 0.9)';
        ctx.fillRect(10, 10, 200, 60);
        ctx.strokeStyle = '#ddd';
        ctx.strokeRect(10, 10, 200, 60);

        ctx.fillStyle = '#333';
        ctx.font = '12px Arial';
        ctx.textAlign = 'left';
        ctx.fillText(`尺寸: ${mapInfo.width}×${mapInfo.height}`, 15, 25);
        ctx.fillText(`分辨率: ${mapInfo.resolution.toFixed(3)}m/px`, 15, 40);
        ctx.fillText(`缩放: ${(this.mapScale * 100).toFixed(1)}%`, 15, 55);

        ctx.restore();
    }

    // 直接绘制地图的方法（修复版本 - 使用this.mapScale而不是重新计算）
    drawMapDirect() {
        if (!this.mapData) {
            return;
        }

        const ctx = this.mapCtx;

        // 清空画布
        ctx.fillStyle = '#f0f0f0';
        ctx.fillRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);

        const width = this.mapData.info.width;
        const height = this.mapData.info.height;
        const data = this.mapData.data;

        if (!data || data.length === 0) {
            return;
        }

        // 验证地图数据完整性
        if (width <= 0 || height <= 0 || data.length !== width * height) {
            console.warn('地图数据尺寸不匹配', {
                width: width,
                height: height,
                dataLength: data.length,
                expected: width * height
            });
            return;
        }

        // 使用this.mapScale和this.mapOffsetX/Y，而不是重新计算
        // 这样可以保持用户的缩放和平移状态，避免抽搐
        const scale = this.mapScale;
        const offsetX = this.mapOffsetX;
        const offsetY = this.mapOffsetY;

        // 创建图像数据
        const imageData = ctx.createImageData(width, height);
        const pixels = imageData.data;

        // 填充像素数据
        let unknownCount = 0;
        let freeCount = 0;
        let occupiedCount = 0;

        for (let i = 0; i < data.length; i++) {
            const value = data[i];
            let color;

            if (value === -1) {
                // 未知区域 - 灰色
                color = [128, 128, 128, 255];
                unknownCount++;
            } else if (value === 0) {
                // 自由空间 - 白色
                color = [255, 255, 255, 255];
                freeCount++;
            } else {
                // 障碍物 - 黑色
                color = [0, 0, 0, 255];
                occupiedCount++;
            }

            const pixelIndex = i * 4;
            pixels[pixelIndex] = color[0];     // R
            pixels[pixelIndex + 1] = color[1]; // G
            pixels[pixelIndex + 2] = color[2]; // B
            pixels[pixelIndex + 3] = color[3]; // A
        }

        // 创建临时画布来绘制图像
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = width;
        tempCanvas.height = height;
        const tempCtx = tempCanvas.getContext('2d');
        tempCtx.putImageData(imageData, 0, 0);

        // 绘制到主画布
        ctx.save();
        ctx.translate(offsetX, offsetY);
        ctx.scale(scale, scale);
        ctx.drawImage(tempCanvas, 0, 0);
        ctx.restore();

        // 绘制机器人位置（如果有）
        if (this.robotPose) {
            this.drawRobotDirect(ctx, offsetX, offsetY, scale);
        }
    }

    // 简化的机器人绘制方法
    drawRobotDirect(ctx, offsetX, offsetY, scale) {
        if (!this.robotPose || !this.mapData) return;

        const pose = this.robotPose;
        const mapInfo = this.mapData.info;

        // 正确的坐标转换：世界坐标 -> 地图像素坐标
        const originX = mapInfo.origin.position.x;
        const originY = mapInfo.origin.position.y;
        const resolution = mapInfo.resolution;

        // 转换到地图像素坐标
        const mapPixelX = (pose.position.x - originX) / resolution;
        const mapPixelY = (pose.position.y - originY) / resolution;

        // 转换到画布坐标（注意Y轴翻转）
        const canvasX = offsetX + mapPixelX * scale;
        const canvasY = offsetY + (mapInfo.height - mapPixelY) * scale;

        // 移除console.log，提升性能

        ctx.save();
        ctx.translate(canvasX, canvasY);

        // 绘制机器人圆形
        ctx.fillStyle = '#ff0000';
        ctx.beginPath();
        ctx.arc(0, 0, 8, 0, 2 * Math.PI);
        ctx.fill();

        // 绘制朝向箭头
        const yaw = this.getYawFromQuaternion(pose.orientation);
        ctx.rotate(-yaw); // 注意角度方向
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(15, 0);
        ctx.stroke();

        ctx.restore();
    }



    // 从四元数计算偏航角的辅助方法
    getYawFromQuaternion(q) {
        return Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    }

    drawOccupancyGrid() {
        const ctx = this.mapCtx;
        const mapInfo = this.mapData.info;
        const mapData = this.mapData.data;

        // 移除所有console.log，提升性能

        // 创建图像数据
        const imageData = ctx.createImageData(mapInfo.width, mapInfo.height);
        const data = imageData.data;

        for (let i = 0; i < mapData.length; i++) {
            const value = mapData[i];
            let color;

            if (value === -1) {
                // 未知区域 - 灰色
                color = [128, 128, 128, 255];
            } else if (value === 0) {
                // 自由空间 - 白色
                color = [255, 255, 255, 255];
            } else if (value > 50) {
                // 占用空间 - 黑色 (概率值大于50%)
                color = [0, 0, 0, 255];
            } else {
                // 低概率占用 - 浅灰色
                const intensity = 255 - (value * 2);
                color = [intensity, intensity, intensity, 255];
            }

            const pixelIndex = i * 4;
            data[pixelIndex] = color[0];     // R
            data[pixelIndex + 1] = color[1]; // G
            data[pixelIndex + 2] = color[2]; // B
            data[pixelIndex + 3] = color[3]; // A
        }

        // 创建临时画布来绘制图像
        const tempCanvas = document.createElement('canvas');
        tempCanvas.width = mapInfo.width;
        tempCanvas.height = mapInfo.height;
        const tempCtx = tempCanvas.getContext('2d');

        tempCtx.putImageData(imageData, 0, 0);

        // 简化的绘制方法 - 直接绘制，不进行复杂的坐标变换
        const displayWidth = mapInfo.width;
        const displayHeight = mapInfo.height;

        // 直接绘制地图，不进行Y轴翻转（保持图像正向显示）
        ctx.drawImage(tempCanvas, 0, 0, displayWidth, displayHeight);
    }
    
    drawRobot() {
        const ctx = this.mapCtx;

        if (!this.robotPose || !this.mapData || !this.mapData.info) return;

        const pose = this.robotPose.position;
        const orientation = this.robotPose.orientation;
        const mapInfo = this.mapData.info;

        // 正确的坐标转换：世界坐标 -> 地图像素坐标
        const originX = mapInfo.origin.position.x;
        const originY = mapInfo.origin.position.y;
        const resolution = mapInfo.resolution;

        // 转换到地图像素坐标
        const mapPixelX = (pose.x - originX) / resolution;
        const mapPixelY = (pose.y - originY) / resolution;

        // 应用当前的变换（缩放和偏移）
        const canvasX = mapPixelX * this.mapScale + this.mapOffsetX;
        const canvasY = (mapInfo.height - mapPixelY) * this.mapScale + this.mapOffsetY; // Y轴翻转

        // 计算机器人朝向角度
        const q = orientation;
        const theta = Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));

        ctx.save();
        ctx.translate(canvasX, canvasY);
        ctx.rotate(-theta); // 注意角度方向

        // 绘制机器人主体（圆形）
        ctx.fillStyle = '#007bff';
        ctx.beginPath();
        ctx.arc(0, 0, 8, 0, 2 * Math.PI); // 8像素半径
        ctx.fill();

        // 绘制方向指示器
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 3;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(15, 0); // 15像素长度
        ctx.stroke();

        ctx.restore();
    }
    
    drawScale() {
        const ctx = this.mapCtx;

        // 绘制比例尺
        const scaleLength = 1.0; // 1米
        const pixelLength = scaleLength * this.mapScale;

        if (pixelLength > 20) { // 只有当比例尺足够大时才显示
            ctx.save();
            ctx.strokeStyle = '#333';
            ctx.lineWidth = 2;
            ctx.font = '12px Arial';
            ctx.fillStyle = '#333';

            const x = 20;
            const y = this.mapCanvas.height - 30;

            ctx.beginPath();
            ctx.moveTo(x, y);
            ctx.lineTo(x + pixelLength, y);
            ctx.stroke();

            ctx.fillText('1m', x + pixelLength / 2 - 10, y - 5);
            ctx.restore();
        }
    }

    clearMap() {
        // 设置清除标志，记录清除时间
        this.isMapCleared = true;
        this.clearMapTime = Date.now();
        this.mapUpdateCount = 0;

        // 清除地图数据
        this.mapData = null;
        this.robotPose = null;

        // 重置缩放标志，允许下次重新设置初始缩放
        this.hasSetInitialScale = false;
        this.initialScale = null;

        // 重置视图参数，避免旧地图的缩放影响新地图
        this.mapScale = 1.0;
        this.mapOffsetX = 0;
        this.mapOffsetY = 0;

        // 清除画布（完全清空，包括背景）
        const ctx = this.mapCtx;
        ctx.clearRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);

        // 绘制干净的背景
        ctx.fillStyle = '#f0f0f0';
        ctx.fillRect(0, 0, this.mapCanvas.width, this.mapCanvas.height);

        // 显示提示信息
        ctx.fillStyle = '#666';
        ctx.font = '16px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('地图已清除，等待SLAM清除完成（3秒）...', this.mapCanvas.width / 2, this.mapCanvas.height / 2);

        // 清除DOM显示
        const mapSizeElement = document.getElementById('mapSize');
        const mapResolutionElement = document.getElementById('mapResolution');
        if (mapSizeElement) mapSizeElement.textContent = '-';
        if (mapResolutionElement) mapResolutionElement.textContent = '-';

        console.log('🗑️ 地图显示已清除，等待SLAM清除完成（3秒或10次更新后接受新地图）');
    }

    // 调试方法：查看地图尺寸变化历史
    showSizeChangeHistory() {
        if (!this.sizeChangeHistory || this.sizeChangeHistory.length === 0) {
            console.log('📊 没有记录到地图尺寸变化');
            return;
        }

        console.log('📊 地图尺寸变化历史（最近10次）:');
        console.table(this.sizeChangeHistory);
    }


}
