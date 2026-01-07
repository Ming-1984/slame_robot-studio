#!/bin/bash

# Robot Studio 安全关机脚本
# - 尽量先停止 Robot Studio/ROS2 相关进程，再执行关机
# - 支持 --dry-run 用于验证流程（不真正关机）

set -euo pipefail

WORKSPACE_DIR="/home/jetson/ros2_ws"
LOG_FILE_DEFAULT="/var/log/robot-studio-safe-shutdown.log"
LOG_FILE_FALLBACK="/tmp/robot-studio-safe-shutdown.log"

ORIGINAL_ARGS=("$@")

DELAY_SECONDS=2
DRY_RUN=false
NETWORK_CLEANUP=true
PLAN_ONLY=false

usage() {
    cat << 'EOF'
用法:
  safe_shutdown.sh [--delay <seconds>] [--dry-run] [--plan-only] [--no-network-cleanup]

参数:
  --delay <seconds>          延迟执行（默认 2 秒，便于 API 返回响应）
  --dry-run                  只执行停止流程，不真正关机
  --plan-only                仅输出计划/检查，不停止服务、不关机（用于安全验证）
  --no-network-cleanup       不做网络/热点清理（仅停止服务）
EOF
}

log() {
    local msg="$1"
    local ts
    ts="$(date '+%Y-%m-%d %H:%M:%S')"
    echo "[$ts] $msg" | tee -a "$LOG_FILE" >/dev/null
}

require_root() {
    if [ "$(id -u)" -eq 0 ]; then
        return 0
    fi

    if sudo -n true >/dev/null 2>&1; then
        exec sudo -n bash "$0" "$@"
    fi

    echo "❌ 需要 root 权限执行关机流程（sudo -n 不可用）" >&2
    exit 1
}

while [ $# -gt 0 ]; do
    case "$1" in
        --delay)
            if [ $# -lt 2 ]; then
                echo "❌ --delay 需要一个数字参数" >&2
                exit 2
            fi
            DELAY_SECONDS="$2"
            shift 2
            ;;
        --dry-run)
            DRY_RUN=true
            shift
            ;;
        --plan-only)
            PLAN_ONLY=true
            shift
            ;;
        --no-network-cleanup)
            NETWORK_CLEANUP=false
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "❌ 未知参数: $1" >&2
            usage
            exit 2
            ;;
    esac
done

require_root "${ORIGINAL_ARGS[@]}"

mkdir -p /var/log 2>/dev/null || true
LOG_FILE="$LOG_FILE_DEFAULT"
if ! touch "$LOG_FILE" >/dev/null 2>&1; then
    LOG_FILE="$LOG_FILE_FALLBACK"
    touch "$LOG_FILE" >/dev/null 2>&1 || true
fi

log "🚨 收到安全关机请求 (delay=${DELAY_SECONDS}s, dry_run=${DRY_RUN}, network_cleanup=${NETWORK_CLEANUP})"

if [ "$PLAN_ONLY" = true ]; then
    log "🧾 plan-only：将执行以下步骤（不实际执行）"
    log "  1) 停止路径规划: $WORKSPACE_DIR/stop_path_planning.sh"
    log "  2) 停止 Robot Studio: $WORKSPACE_DIR/stop_robot_studio.sh (network_cleanup=${NETWORK_CLEANUP})"
    log "  3) sync 文件系统"
    log "  4) 关机: systemctl poweroff / shutdown -h now / poweroff (dry-run=${DRY_RUN})"
    exit 0
fi

if [ "$DELAY_SECONDS" -gt 0 ]; then
    log "⏳ 延迟 ${DELAY_SECONDS}s 后执行停止流程..."
    sleep "$DELAY_SECONDS" || true
fi

log "🛑 停止路径规划/探索..."
if [ -x "$WORKSPACE_DIR/stop_path_planning.sh" ]; then
    timeout 20 bash "$WORKSPACE_DIR/stop_path_planning.sh" >/dev/null 2>&1 || true
else
    log "⚠️ 未找到 stop_path_planning.sh，跳过"
fi

log "🛑 停止 Robot Studio 服务..."
if [ -x "$WORKSPACE_DIR/stop_robot_studio.sh" ]; then
    stop_args=()
    if [ "$NETWORK_CLEANUP" = true ]; then
        stop_args+=(--network-cleanup)
    fi
    timeout 60 bash "$WORKSPACE_DIR/stop_robot_studio.sh" "${stop_args[@]}" >/dev/null 2>&1 || true
else
    log "⚠️ 未找到 stop_robot_studio.sh，改用进程清理兜底"
    pkill -TERM -f "ros2_web_bridge" 2>/dev/null || true
    pkill -TERM -f "web_interface_server.py" 2>/dev/null || true
    pkill -TERM -f "slamware_ros_sdk" 2>/dev/null || true
    pkill -TERM -f "nav2" 2>/dev/null || true
    pkill -TERM -f "robust_explore_node" 2>/dev/null || true
    sleep 2
    pkill -KILL -f "ros2_web_bridge" 2>/dev/null || true
    pkill -KILL -f "web_interface_server.py" 2>/dev/null || true
    pkill -KILL -f "slamware_ros_sdk" 2>/dev/null || true
    pkill -KILL -f "nav2" 2>/dev/null || true
    pkill -KILL -f "robust_explore_node" 2>/dev/null || true
fi

log "💾 sync 文件系统..."
sync || true

if [ "$DRY_RUN" = true ]; then
    log "✅ dry-run 模式：已完成停止流程，跳过关机"
    exit 0
fi

log "⏻ 执行关机..."

if command -v systemctl >/dev/null 2>&1; then
    systemctl poweroff -i >/dev/null 2>&1 || true
fi

shutdown -h now >/dev/null 2>&1 || true
poweroff >/dev/null 2>&1 || true

log "⚠️ 关机命令已下发（如仍未关机，请检查 sudo/systemd 权限与日志：$LOG_FILE）"
