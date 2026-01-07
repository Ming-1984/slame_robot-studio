#!/bin/bash

# Robot Studio 服务健康监控脚本
# 监控Web服务、API服务、WebSocket服务的健康状态
# 自动重启失败的服务，记录日志，发送通知

# 配置参数
WORKSPACE_DIR="/home/jetson/ros2_ws"
LOG_FILE="/tmp/service_monitor.log"
PID_FILE="/tmp/service_monitor.pid"
CHECK_INTERVAL=30  # 检查间隔（秒）
MAX_RESTART_COUNT=3  # 最大重启次数

# 服务配置
declare -A SERVICES=(
    ["web"]="8080:web_interface_server.py.*--port 8080:."
    ["api"]="8000:robot_api_server.py:."
    ["websocket"]="8001:ros2_web_bridge.py:."
)

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 日志函数
log() {
    local level=$1
    shift
    local message="$*"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    echo "[$timestamp] [$level] $message" | tee -a "$LOG_FILE"
}

log_info() { log "INFO" "$@"; }
log_warn() { log "WARN" "$@"; }
log_error() { log "ERROR" "$@"; }

# 检查服务是否运行
check_service() {
    local service_name=$1
    local service_config=${SERVICES[$service_name]}
    local port=$(echo "$service_config" | cut -d: -f1)
    local process=$(echo "$service_config" | cut -d: -f2)
    local workdir=$(echo "$service_config" | cut -d: -f3)
    
    # 检查端口是否监听
    if lsof -i :$port > /dev/null 2>&1; then
        # 检查进程是否存在
        if pgrep -f "$process" > /dev/null 2>&1; then
            return 0  # 服务正常
        else
            log_warn "$service_name 端口$port在监听但进程不匹配"
            return 1  # 端口被其他进程占用
        fi
    else
        return 1  # 服务未运行
    fi
}

# 启动服务
start_service() {
    local service_name=$1
    local service_config=${SERVICES[$service_name]}
    local port=$(echo "$service_config" | cut -d: -f1)
    local process=$(echo "$service_config" | cut -d: -f2)
    local workdir=$(echo "$service_config" | cut -d: -f3)
    
    log_info "启动 $service_name 服务..."
    
    # 切换到工作目录
    cd "$WORKSPACE_DIR/$workdir" || {
        log_error "无法切换到目录: $WORKSPACE_DIR/$workdir"
        return 1
    }
    
    # 设置环境变量
    export ROS_DOMAIN_ID=99
    source /opt/ros/humble/setup.bash 2>/dev/null || true
    source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true
    export LD_LIBRARY_PATH=/home/jetson/ros2_ws/src/aurora_remote_public/lib/linux_aarch64:$LD_LIBRARY_PATH
    
    # 清理可能存在的端口占用
    local existing_pid=$(lsof -t -i :$port 2>/dev/null)
    if [ ! -z "$existing_pid" ]; then
        log_warn "端口$port被进程$existing_pid占用，正在清理..."
        kill -9 $existing_pid 2>/dev/null || true
        sleep 2
    fi
    
    # 启动服务
    case "$service_name" in
        "web")
            nohup python3 "$WORKSPACE_DIR/web_interface_server.py" --bind 0.0.0.0 --port "$port" --directory "$WORKSPACE_DIR/web_interface" > "/tmp/${service_name}_service.log" 2>&1 &
            echo $! > "/tmp/${service_name}_service.pid"
            ;;
        "api")
            nohup python3 "$WORKSPACE_DIR/robot_api_server.py" > "/tmp/${service_name}_service.log" 2>&1 &
            echo $! > "/tmp/${service_name}_service.pid"
            ;;
        "websocket")
            nohup python3 "$WORKSPACE_DIR/ros2_web_bridge.py" > "/tmp/${service_name}_service.log" 2>&1 &
            echo $! > "/tmp/${service_name}_service.pid"
            ;;
    esac
    
    # 等待服务启动
    sleep 5
    
    # 验证启动是否成功
    if check_service "$service_name"; then
        log_info "$service_name 服务启动成功 (端口: $port)"
        return 0
    else
        log_error "$service_name 服务启动失败"
        return 1
    fi
}

# 停止服务
stop_service() {
    local service_name=$1
    local service_config=${SERVICES[$service_name]}
    local port=$(echo "$service_config" | cut -d: -f1)
    local process=$(echo "$service_config" | cut -d: -f2)
    
    log_info "停止 $service_name 服务..."
    
    # 通过PID文件停止
    local pid_file="/tmp/${service_name}_service.pid"
    if [ -f "$pid_file" ]; then
        local pid=$(cat "$pid_file")
        if [ ! -z "$pid" ] && ps -p "$pid" > /dev/null 2>&1; then
            kill "$pid" 2>/dev/null || true
            sleep 2
            if ps -p "$pid" > /dev/null 2>&1; then
                kill -9 "$pid" 2>/dev/null || true
            fi
        fi
        rm -f "$pid_file"
    fi
    
    # 通过进程名停止
    pkill -f "$process" 2>/dev/null || true
    
    # 通过端口停止
    local pid=$(lsof -t -i :$port 2>/dev/null)
    if [ ! -z "$pid" ]; then
        kill -9 "$pid" 2>/dev/null || true
    fi
    
    sleep 2
    
    if ! check_service "$service_name"; then
        log_info "$service_name 服务已停止"
        return 0
    else
        log_warn "$service_name 服务可能未完全停止"
        return 1
    fi
}

# 重启服务
restart_service() {
    local service_name=$1
    local restart_count_file="/tmp/${service_name}_restart_count"
    
    # 读取重启计数
    local restart_count=0
    if [ -f "$restart_count_file" ]; then
        restart_count=$(cat "$restart_count_file")
    fi
    
    # 检查是否达到最大重启次数
    if [ "$restart_count" -ge "$MAX_RESTART_COUNT" ]; then
        log_error "$service_name 达到最大重启次数 ($MAX_RESTART_COUNT)，停止自动重启"
        return 1
    fi
    
    # 增加重启计数
    restart_count=$((restart_count + 1))
    echo "$restart_count" > "$restart_count_file"
    
    log_warn "$service_name 服务异常，执行第$restart_count次重启..."
    
    # 停止服务
    stop_service "$service_name"
    
    # 等待一段时间后重启
    sleep 5
    
    # 启动服务
    if start_service "$service_name"; then
        log_info "$service_name 服务重启成功"
        # 重启成功，重置计数器
        rm -f "$restart_count_file"
        return 0
    else
        log_error "$service_name 服务重启失败"
        return 1
    fi
}

# 检查系统资源
check_system_resources() {
    # 检查内存使用
    local mem_usage=$(free | grep Mem | awk '{printf("%.1f"), $3/$2 * 100.0}')
    if (( $(echo "$mem_usage > 90.0" | bc -l) )); then
        log_warn "内存使用率过高: ${mem_usage}%"
    fi
    
    # 检查磁盘空间
    local disk_usage=$(df / | tail -1 | awk '{print $5}' | sed 's/%//')
    if [ "$disk_usage" -gt 90 ]; then
        log_warn "磁盘使用率过高: ${disk_usage}%"
    fi
    
    # 检查CPU负载
    local cpu_load=$(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')
    if (( $(echo "$cpu_load > 4.0" | bc -l) )); then
        log_warn "CPU负载过高: $cpu_load"
    fi
}

# 生成状态报告
generate_status_report() {
    local report_file="/tmp/service_status_report.txt"
    local timestamp=$(date '+%Y-%m-%d %H:%M:%S')
    
    cat > "$report_file" << EOF
Robot Studio 服务状态报告
生成时间: $timestamp

=== 服务状态 ===
EOF
    
    for service_name in "${!SERVICES[@]}"; do
        if check_service "$service_name"; then
            echo "$service_name: 运行中 ✅" >> "$report_file"
        else
            echo "$service_name: 已停止 ❌" >> "$report_file"
        fi
    done
    
    cat >> "$report_file" << EOF

=== 系统资源 ===
内存使用: $(free | grep Mem | awk '{printf("%.1f%%"), $3/$2 * 100.0}')
磁盘使用: $(df / | tail -1 | awk '{print $5}')
CPU负载: $(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')

=== 网络状态 ===
监听端口: $(netstat -tlnp | grep -E "(8000|8001|8080)" | wc -l) 个
WiFi状态: $(iwconfig 2>/dev/null | grep -q "Access Point" && echo "热点模式" || echo "客户端模式")

=== 最近日志 ===
EOF
    
    tail -10 "$LOG_FILE" >> "$report_file"
    
    log_info "状态报告已生成: $report_file"
}

# 主监控循环
monitor_services() {
    log_info "服务监控启动，检查间隔: ${CHECK_INTERVAL}秒"
    log_info "监控服务: ${!SERVICES[*]}"
    
    while true; do
        # 检查每个服务
        for service_name in "${!SERVICES[@]}"; do
            if ! check_service "$service_name"; then
                log_warn "$service_name 服务未运行，尝试重启..."
                restart_service "$service_name"
            fi
        done
        
        # 检查系统资源
        check_system_resources
        
        # 生成状态报告（每小时一次）
        local current_minute=$(date +%M)
        if [ "$current_minute" = "00" ]; then
            generate_status_report
        fi
        
        # 等待下次检查
        sleep "$CHECK_INTERVAL"
    done
}

# 启动监控守护进程
start_monitor() {
    if [ -f "$PID_FILE" ] && ps -p $(cat "$PID_FILE") > /dev/null 2>&1; then
        echo "监控服务已在运行 (PID: $(cat $PID_FILE))"
        return 1
    fi
    
    echo "启动服务监控守护进程..."
    
    # 创建日志文件
    touch "$LOG_FILE"
    
    # 后台运行监控
    nohup bash "$0" --monitor > /dev/null 2>&1 &
    echo $! > "$PID_FILE"
    
    echo "监控服务已启动 (PID: $(cat $PID_FILE))"
    echo "日志文件: $LOG_FILE"
    
    log_info "服务监控守护进程已启动"
}

# 停止监控守护进程
stop_monitor() {
    if [ -f "$PID_FILE" ]; then
        local pid=$(cat "$PID_FILE")
        if ps -p "$pid" > /dev/null 2>&1; then
            kill "$pid" 2>/dev/null
            log_info "监控守护进程已停止 (PID: $pid)"
        fi
        rm -f "$PID_FILE"
    fi
    echo "监控服务已停止"
}

# 显示监控状态
show_status() {
    echo -e "${PURPLE}=== Robot Studio 服务监控状态 ===${NC}"
    
    # 监控守护进程状态
    if [ -f "$PID_FILE" ] && ps -p $(cat "$PID_FILE") > /dev/null 2>&1; then
        echo -e "监控守护进程: ${GREEN}运行中${NC} (PID: $(cat $PID_FILE))"
    else
        echo -e "监控守护进程: ${RED}已停止${NC}"
    fi
    
    echo ""
    echo -e "${CYAN}=== 服务状态 ===${NC}"
    
    for service_name in "${!SERVICES[@]}"; do
        local service_config=${SERVICES[$service_name]}
        local port=$(echo "$service_config" | cut -d: -f1)
        
        if check_service "$service_name"; then
            echo -e "$service_name (端口$port): ${GREEN}运行中${NC} ✅"
        else
            echo -e "$service_name (端口$port): ${RED}已停止${NC} ❌"
        fi
    done
    
    echo ""
    echo -e "${CYAN}=== 系统资源 ===${NC}"
    echo "内存使用: $(free | grep Mem | awk '{printf("%.1f%%"), $3/$2 * 100.0}')"
    echo "磁盘使用: $(df / | tail -1 | awk '{print $5}')"
    echo "CPU负载: $(uptime | awk -F'load average:' '{print $2}' | awk '{print $1}' | sed 's/,//')"
    
    if [ -f "$LOG_FILE" ]; then
        echo ""
        echo -e "${CYAN}=== 最近日志 ===${NC}"
        tail -5 "$LOG_FILE"
    fi
}

# 手动重启所有服务
restart_all_services() {
    echo "重启所有服务..."
    for service_name in "${!SERVICES[@]}"; do
        # 清除重启计数
        rm -f "/tmp/${service_name}_restart_count"
        restart_service "$service_name"
    done
    echo "所有服务重启完成"
}

# 主函数
main() {
    cd "$WORKSPACE_DIR" || {
        echo "错误: 无法切换到工作目录 $WORKSPACE_DIR"
        exit 1
    }
    
    case "$1" in
        --monitor)
            monitor_services
            ;;
        start)
            start_monitor
            ;;
        stop)
            stop_monitor
            ;;
        status)
            show_status
            ;;
        restart)
            restart_all_services
            ;;
        *)
            echo "Robot Studio 服务健康监控"
            echo ""
            echo "使用方法: $0 {start|stop|status|restart}"
            echo ""
            echo "命令："
            echo "  start   - 启动监控守护进程"
            echo "  stop    - 停止监控守护进程" 
            echo "  status  - 显示服务和监控状态"
            echo "  restart - 手动重启所有服务"
            echo ""
            exit 1
            ;;
    esac
}

main "$@"
