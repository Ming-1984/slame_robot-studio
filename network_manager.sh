#!/bin/bash

# 网络状态管理器
# 智能管理有线网络和WiFi网络的切换，确保系统稳定性

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 网络接口配置
WIFI_INTERFACE="wlP1p1s0"
WIRED_INTERFACE="enP8p1s0"
HOTSPOT_SSID="RobotStudio"
HOTSPOT_IP="192.168.4.1"

# 日志函数
log() {
    echo -e "[$(date '+%H:%M:%S')] $1"
}

# 执行需要sudo的命令（不在脚本中硬编码密码）
run_sudo() {
    if [ "$(id -u)" -eq 0 ]; then
        "$@"
        return $?
    fi

    if sudo -n true >/dev/null 2>&1; then
        sudo -n "$@"
        return $?
    fi

    if [ -t 0 ]; then
        sudo "$@"
        return $?
    fi

    log "${RED}❌ 需要 sudo 权限: $*${NC}"
    return 1
}

# 检测网络连接状态
detect_network_status() {
    log "${BLUE}🔍 检测网络状态...${NC}"
    
    # 检测有线网络
    WIRED_STATUS="disconnected"
    WIRED_IP=""
    if ip link show $WIRED_INTERFACE >/dev/null 2>&1; then
        if ip route | grep -q "dev $WIRED_INTERFACE" && ip addr show $WIRED_INTERFACE | grep -q "inet "; then
            WIRED_STATUS="connected"
            WIRED_IP=$(ip addr show $WIRED_INTERFACE | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)
            log "${GREEN}✅ 有线网络: $WIRED_IP${NC}"
        else
            log "${YELLOW}⚠️  有线接口存在但未连接${NC}"
        fi
    else
        log "${RED}❌ 有线接口不存在${NC}"
    fi
    
    # 检测WiFi网络
    WIFI_STATUS="disconnected"
    WIFI_MODE="unknown"
    WIFI_IP=""
    WIFI_SSID=""
    
    if ip link show $WIFI_INTERFACE >/dev/null 2>&1; then
        # 检查WiFi模式
        if iwconfig $WIFI_INTERFACE 2>/dev/null | grep -q "Mode:Master"; then
            WIFI_MODE="hotspot"
            WIFI_STATUS="hotspot"
            WIFI_IP=$(ip addr show $WIFI_INTERFACE | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)
            log "${GREEN}✅ WiFi热点: $WIFI_IP${NC}"
        elif iwconfig $WIFI_INTERFACE 2>/dev/null | grep -q "Mode:Managed"; then
            WIFI_MODE="client"
            if ip route | grep -q "dev $WIFI_INTERFACE" && ip addr show $WIFI_INTERFACE | grep -q "inet "; then
                WIFI_STATUS="connected"
                WIFI_IP=$(ip addr show $WIFI_INTERFACE | grep "inet " | awk '{print $2}' | cut -d/ -f1 | head -1)
                WIFI_SSID=$(iwconfig $WIFI_INTERFACE 2>/dev/null | grep -o 'ESSID:"[^"]*"' | cut -d'"' -f2)
                log "${GREEN}✅ WiFi客户端: $WIFI_IP (SSID: $WIFI_SSID)${NC}"
            else
                WIFI_STATUS="disconnected"
                log "${YELLOW}⚠️  WiFi已配置但未连接${NC}"
            fi
        fi
    else
        log "${RED}❌ WiFi接口不存在${NC}"
    fi
    
    # 检测当前SSH连接源
    SSH_SOURCE=""
    SSH_VIA_WIFI=false
    SSH_VIA_WIRED=false
    
    if [ ! -z "$SSH_CONNECTION" ]; then
        SSH_SOURCE=$(echo $SSH_CONNECTION | awk '{print $1}')
    elif [ ! -z "$(who am i)" ]; then
        SSH_SOURCE=$(who am i | awk '{print $5}' | tr -d '()')
    fi
    
    if [ ! -z "$SSH_SOURCE" ]; then
        log "${CYAN}📡 SSH连接来自: $SSH_SOURCE${NC}"
        
        # 判断SSH通过哪个网络
        if [ "$WIRED_STATUS" = "connected" ] && [ ! -z "$WIRED_IP" ]; then
            WIRED_SUBNET=$(echo $WIRED_IP | cut -d. -f1-3)
            SSH_SUBNET=$(echo $SSH_SOURCE | cut -d. -f1-3)
            if [ "$WIRED_SUBNET" = "$SSH_SUBNET" ]; then
                SSH_VIA_WIRED=true
                log "${CYAN}🔌 SSH通过有线网络${NC}"
            fi
        fi
        
        if [ "$WIFI_STATUS" = "connected" ] && [ ! -z "$WIFI_IP" ]; then
            WIFI_SUBNET=$(echo $WIFI_IP | cut -d. -f1-3)
            SSH_SUBNET=$(echo $SSH_SOURCE | cut -d. -f1-3)
            if [ "$WIFI_SUBNET" = "$SSH_SUBNET" ]; then
                SSH_VIA_WIFI=true
                log "${CYAN}📶 SSH通过WiFi网络${NC}"
            fi
        fi
    fi
    
    # 检测互联网连接
    INTERNET_STATUS="unknown"
    if ping -c 1 -W 3 8.8.8.8 >/dev/null 2>&1; then
        INTERNET_STATUS="connected"
        log "${GREEN}🌐 互联网连接正常${NC}"
    else
        INTERNET_STATUS="disconnected"
        log "${YELLOW}⚠️  无互联网连接${NC}"
    fi
}

# 获取最佳网络配置建议
get_network_recommendation() {
    detect_network_status
    
    log "${PURPLE}📊 网络配置分析...${NC}"
    
    # 分析当前网络状况
    if [ "$WIRED_STATUS" = "connected" ] && [ "$WIFI_STATUS" = "connected" ]; then
        # 双网卡都连接
        if [ "$SSH_VIA_WIRED" = true ]; then
            RECOMMENDATION="dual_network_wifi_hotspot"
            log "${GREEN}💡 建议: 保持有线SSH，将WiFi切换为热点模式${NC}"
        else
            RECOMMENDATION="dual_network_stable"
            log "${GREEN}💡 建议: 双网卡稳定模式，保持当前配置${NC}"
        fi
    elif [ "$WIRED_STATUS" = "connected" ] && [ "$WIFI_STATUS" != "connected" ]; then
        # 只有有线连接
        RECOMMENDATION="wired_only_wifi_hotspot"
        log "${GREEN}💡 建议: 有线保持SSH，WiFi启用热点${NC}"
    elif [ "$WIRED_STATUS" != "connected" ] && [ "$WIFI_STATUS" = "connected" ]; then
        # 只有WiFi连接
        if [ "$SSH_VIA_WIFI" = true ]; then
            RECOMMENDATION="wifi_only_keep_connection"
            log "${YELLOW}💡 建议: SSH依赖WiFi，保持当前连接，不建议启用热点${NC}"
        else
            RECOMMENDATION="wifi_client_stable"
            log "${GREEN}💡 建议: WiFi客户端模式稳定${NC}"
        fi
    else
        # 无网络连接
        RECOMMENDATION="no_network_emergency"
        log "${RED}💡 建议: 紧急模式，需要手动配置网络${NC}"
    fi
}

# 安全切换到WiFi热点模式
safe_switch_to_hotspot() {
    log "${PURPLE}🔄 安全切换到WiFi热点模式...${NC}"
    
    detect_network_status
    
    # 安全检查
    if [ "$SSH_VIA_WIFI" = true ] && [ "$WIRED_STATUS" != "connected" ]; then
        log "${RED}❌ 危险操作被阻止: SSH通过WiFi连接且无有线备份${NC}"
        log "${YELLOW}   请先连接有线网络或从有线网络SSH登录${NC}"
        return 1
    fi
    
    # 执行切换
    log "${BLUE}🎯 执行WiFi热点切换...${NC}"
    
    # 使用智能热点管理器
    if [ -f "smart_wifi_hotspot.sh" ]; then
        ./smart_wifi_hotspot.sh start
    else
        log "${RED}❌ 智能热点管理器不存在${NC}"
        return 1
    fi
    
    # 验证切换结果
    sleep 5
    detect_network_status
    
    if [ "$WIFI_STATUS" = "hotspot" ]; then
        log "${GREEN}✅ WiFi热点切换成功${NC}"
        return 0
    else
        log "${RED}❌ WiFi热点切换失败${NC}"
        return 1
    fi
}

# 网络故障恢复
network_recovery() {
    log "${YELLOW}🔧 执行网络故障恢复...${NC}"
    
    # 重启网络服务
    log "重启网络管理服务..."
    run_sudo systemctl restart NetworkManager 2>/dev/null || true
    sleep 3
    
    # 重启网络接口
    if ip link show $WIFI_INTERFACE >/dev/null 2>&1; then
        log "重启WiFi接口..."
        run_sudo ip link set $WIFI_INTERFACE down
        sleep 1
        run_sudo ip link set $WIFI_INTERFACE up
        sleep 2
    fi
    
    if ip link show $WIRED_INTERFACE >/dev/null 2>&1; then
        log "重启有线接口..."
        run_sudo ip link set $WIRED_INTERFACE down
        sleep 1
        run_sudo ip link set $WIRED_INTERFACE up
        sleep 2
    fi
    
    # 重新检测
    sleep 5
    detect_network_status
    
    log "${GREEN}网络故障恢复完成${NC}"
}

# 显示网络状态
show_network_status() {
    detect_network_status
    
    echo -e "${PURPLE}=== Robot Studio 网络状态 ===${NC}"
    echo ""
    
    echo -e "${CYAN}网络接口状态:${NC}"
    echo -e "  有线网络 ($WIRED_INTERFACE): $([[ $WIRED_STATUS == "connected" ]] && echo -e "${GREEN}已连接${NC} ($WIRED_IP)" || echo -e "${RED}未连接${NC}")"
    echo -e "  WiFi网络 ($WIFI_INTERFACE): $([[ $WIFI_STATUS == "connected" ]] && echo -e "${GREEN}客户端模式${NC} ($WIFI_IP)" || [[ $WIFI_STATUS == "hotspot" ]] && echo -e "${GREEN}热点模式${NC} ($WIFI_IP)" || echo -e "${RED}未连接${NC}")"
    
    if [ ! -z "$WIFI_SSID" ]; then
        echo -e "  WiFi SSID: $WIFI_SSID"
    fi
    
    echo ""
    echo -e "${CYAN}连接状态:${NC}"
    echo -e "  互联网连接: $([[ $INTERNET_STATUS == "connected" ]] && echo -e "${GREEN}正常${NC}" || echo -e "${RED}断开${NC}")"
    
    if [ ! -z "$SSH_SOURCE" ]; then
        echo -e "  SSH来源: $SSH_SOURCE"
        if [ "$SSH_VIA_WIRED" = true ]; then
            echo -e "  SSH方式: ${GREEN}有线网络${NC} (安全)"
        elif [ "$SSH_VIA_WIFI" = true ]; then
            echo -e "  SSH方式: ${YELLOW}WiFi网络${NC} (注意安全)"
        fi
    fi
    
    echo ""
    get_network_recommendation
}

# 主函数
main() {
    case "$1" in
        "status")
            show_network_status
            ;;
        "hotspot")
            safe_switch_to_hotspot
            ;;
        "recovery")
            network_recovery
            ;;
        "detect")
            detect_network_status
            ;;
        "recommend")
            get_network_recommendation
            ;;
        *)
            echo "Robot Studio 网络状态管理器"
            echo ""
            echo "使用方法: $0 {status|hotspot|recovery|detect|recommend}"
            echo ""
            echo "命令："
            echo "  status    - 显示完整网络状态"
            echo "  hotspot   - 安全切换到WiFi热点模式"
            echo "  recovery  - 执行网络故障恢复"
            echo "  detect    - 检测网络连接状态"
            echo "  recommend - 获取网络配置建议"
            echo ""
            exit 1
            ;;
    esac
}

main "$@"
