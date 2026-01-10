#!/bin/bash

# Robot Studio 一次性“测试模式”开机辅助
# - 如果检测到测试模式标志：启动 ssh，然后恢复 robot-studio.service 为“下次开机正常自启”
# - 仅执行一次：执行后会删除标志文件

set -euo pipefail

FLAG_DIR="/var/lib/robot-studio"
FLAG_FILE="$FLAG_DIR/test_mode_once.json"
LOG_FILE="/var/log/robot-studio-test-mode.log"

log() {
    local ts
    ts="$(date '+%Y-%m-%d %H:%M:%S')"
    echo "[$ts] $1" >> "$LOG_FILE"
}

mkdir -p "$FLAG_DIR" /var/log 2>/dev/null || true
touch "$LOG_FILE" 2>/dev/null || true

if [ ! -f "$FLAG_FILE" ]; then
    exit 0
fi

log "🧪 检测到一次性测试模式标志，进入调试启动流程..."

# 1) 启动 SSH（按你的要求：仅 start，不做 enable）
if systemctl list-unit-files | grep -qE '^ssh\\.service'; then
    systemctl start ssh 2>/dev/null || true
    log "✅ 已执行: systemctl start ssh"
elif systemctl list-unit-files | grep -qE '^sshd\\.service'; then
    systemctl start sshd 2>/dev/null || true
    log "✅ 已执行: systemctl start sshd"
else
    log "⚠️ 未找到 ssh/sshd unit，跳过"
fi

# 2) 恢复 robot-studio 开机自启（只影响“下一次”开机）
systemctl enable robot-studio.service 2>/dev/null || true
log "✅ 已执行: systemctl enable robot-studio.service（下次开机恢复正常：热点+上位机自启）"

# 3) 清除标志，保证只生效一次
rm -f "$FLAG_FILE" 2>/dev/null || true
sync || true
log "✅ 已清除测试模式标志（本次生效结束）"
