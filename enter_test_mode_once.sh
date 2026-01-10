#!/bin/bash

# Robot Studio 一次性“测试模式”入口脚本
# - 写入一次性标志
# - 关闭 robot-studio.service（影响下次开机：不上位机/不启热点）
# - 确保 debug-once service 启用（用于下次开机启动 ssh 并恢复自启）
# - 触发重启（支持 --delay / --plan-only / --dry-run）

set -euo pipefail

# 系统密码（用于无人值守环境；如需修改请同步更新）
SYSTEM_PASSWORD="yahboom"

WORKSPACE_DIR="/home/jetson/ros2_ws"
UNIT_SRC="$WORKSPACE_DIR/robot-studio-debug-once.service"
UNIT_DST="/etc/systemd/system/robot-studio-debug-once.service"

FLAG_DIR="/var/lib/robot-studio"
FLAG_FILE="$FLAG_DIR/test_mode_once.json"
LOG_FILE="/var/log/robot-studio-test-mode.log"

DELAY_SECONDS=3
PLAN_ONLY=false
DRY_RUN=false

usage() {
    cat << 'EOF'
用法:
  enter_test_mode_once.sh [--delay <seconds>] [--plan-only] [--dry-run]

参数:
  --delay <seconds>    延迟重启（默认 3 秒，便于 API 返回响应）
  --plan-only          仅输出计划/检查，不改系统、不重启
  --dry-run            执行写标志/disable/enable，但不重启（谨慎使用）
EOF
}

log() {
    local ts
    ts="$(date '+%Y-%m-%d %H:%M:%S')"
    echo "[$ts] $1" | tee -a "$LOG_FILE" >/dev/null
}

require_root() {
    if [ "$(id -u)" -eq 0 ]; then
        return 0
    fi

    if sudo -n true >/dev/null 2>&1; then
        exec sudo -n bash "$0" "$@"
    fi

    # 无 TTY 场景也能用：自动输入密码
    echo "$SYSTEM_PASSWORD" | sudo -S -p '' bash "$0" "$@"
    exit $?
}

ORIGINAL_ARGS=("$@")
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
        --plan-only)
            PLAN_ONLY=true
            shift
            ;;
        --dry-run)
            DRY_RUN=true
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

mkdir -p "$FLAG_DIR" /var/log 2>/dev/null || true
touch "$LOG_FILE" 2>/dev/null || true

if [ "$PLAN_ONLY" = true ]; then
    log "🧾 plan-only：将执行以下步骤（不实际执行）"
    log "  1) 安装并 enable: robot-studio-debug-once.service（下次开机启动 ssh 并恢复自启）"
    log "  2) 写入标志: $FLAG_FILE"
    log "  3) disable: robot-studio.service（下次开机不上位机/不启热点）"
    log "  4) start: ssh（如果存在）"
    log "  5) 延迟 ${DELAY_SECONDS}s 后 reboot（dry_run=${DRY_RUN}）"
    exit 0
fi

log "🧪 进入一次性测试模式（下次开机不启热点/不上位机，仅启动 ssh；仅生效一次）"

# 1) 安装 debug-once unit 并启用
if [ ! -f "$UNIT_SRC" ]; then
    log "❌ 缺少 unit 源文件: $UNIT_SRC"
    exit 1
fi

cp -f "$UNIT_SRC" "$UNIT_DST"
chmod 0644 "$UNIT_DST"
systemctl daemon-reload
systemctl enable robot-studio-debug-once.service >/dev/null 2>&1 || true
log "✅ robot-studio-debug-once.service 已安装并启用"

# 2) 写入一次性标志
cat > "$FLAG_FILE" << EOF
{
  "enabled": true,
  "created_at": "$(date -Iseconds)"
}
EOF
log "✅ 已写入标志: $FLAG_FILE"

# 3) 禁用 robot-studio 开机自启（仅影响下次开机）
systemctl disable robot-studio.service >/dev/null 2>&1 || true
log "✅ 已执行: systemctl disable robot-studio.service（下次开机不上位机/不启热点）"

# 4) 启动 ssh（按你的要求）
if systemctl list-unit-files | grep -qE '^ssh\\.service'; then
    systemctl start ssh >/dev/null 2>&1 || true
    log "✅ 已执行: systemctl start ssh"
elif systemctl list-unit-files | grep -qE '^sshd\\.service'; then
    systemctl start sshd >/dev/null 2>&1 || true
    log "✅ 已执行: systemctl start sshd"
else
    log "⚠️ 未找到 ssh/sshd unit，跳过"
fi

sync || true

if [ "$DRY_RUN" = true ]; then
    log "✅ dry-run：已完成配置，但不会重启"
    exit 0
fi

log "⏻ 将在 ${DELAY_SECONDS}s 后重启..."
sleep "$DELAY_SECONDS" || true

if command -v systemctl >/dev/null 2>&1; then
    systemctl reboot >/dev/null 2>&1 || true
fi

shutdown -r now >/dev/null 2>&1 || true
