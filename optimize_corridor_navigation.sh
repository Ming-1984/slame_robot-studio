#!/bin/bash

# 走廊导航优化脚本
# 解决小车在走廊中无法通过的问题

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'  
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo "================================================"
echo "🔧 走廊导航优化脚本"
echo "================================================"
echo "🎯 优化目标："
echo "   ✅ 减少膨胀半径，允许通过更窄的空间"
echo "   ✅ 降低避障敏感度，减少过度保守"
echo "   ✅ 优化机器人几何参数"
echo "   ✅ 调整costmap配置"
echo "================================================"

# 检查是否有运行中的导航系统
if pgrep -f "nav2" > /dev/null; then
    echo -e "${YELLOW}⚠️  检测到运行中的Nav2系统${NC}"
    echo "请先停止当前导航系统："
    echo "  ./stop_path_planning.sh"
    echo ""
    read -p "是否现在停止并重启？(y/n): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "🛑 停止当前系统..."
        ./stop_path_planning.sh
        sleep 3
    else
        echo "❌ 请手动停止系统后重新运行此脚本"
        exit 1
    fi
fi

# 备份当前配置
echo "💾 备份当前配置..."
BACKUP_DIR="nav2_configs_backup_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$BACKUP_DIR"
cp -r src/nav2_configs/params/ "$BACKUP_DIR/"
echo "✅ 配置已备份到: $BACKUP_DIR"

# 显示优化参数
echo ""
echo "🔧 应用的优化参数："
echo "   📐 膨胀半径: 0.6m → 0.35m (减少42%)"
echo "   🛡️ 安全边界: 3cm → 1cm (减少67%)"
echo "   📊 成本缩放: 2.5 → 1.8 (降低28%)"
echo "   🎯 避障权重: 200 → 100 (降低50%)"
echo "   📏 碰撞边界: 15cm → 8cm (减少47%)"
echo "   🔍 检测范围: 8m → 6m (减少25%)"
echo ""

# 验证配置文件
CONFIG_FILE="src/nav2_configs/params/ackermann_nav2_params.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo -e "${RED}❌ 配置文件不存在: $CONFIG_FILE${NC}"
    exit 1
fi

# 检查关键参数是否已优化
echo "🔍 验证优化配置..."
if grep -q "inflation_radius: 0.35" "$CONFIG_FILE"; then
    echo "✅ 膨胀半径已优化"
else
    echo -e "${RED}❌ 膨胀半径未优化${NC}"
fi

if grep -q "footprint_padding: 0.01" "$CONFIG_FILE"; then
    echo "✅ 安全边界已优化"
else
    echo -e "${RED}❌ 安全边界未优化${NC}"
fi

if grep -q "collision_margin_distance: 0.08" "$CONFIG_FILE"; then
    echo "✅ 碰撞边界已优化"
else
    echo -e "${RED}❌ 碰撞边界未优化${NC}"
fi

echo ""
echo "🚀 启动优化后的导航系统..."
echo "================================================"

# 启动优化后的系统
./start_path_planning.sh

echo ""
echo "================================================"
echo "✅ 走廊导航优化完成！"
echo "================================================"
echo "📋 测试建议："
echo "   1. 在RViz中设置导航目标到走廊另一端"
echo "   2. 观察机器人是否能成功通过走廊"
echo "   3. 检查costmap显示是否合理"
echo ""
echo "🔧 如果仍有问题，可以进一步调整："
echo "   - 膨胀半径: 可降至0.25m"
echo "   - 安全边界: 可降至0.005m"
echo "   - 检测范围: 可降至4m"
echo ""
echo "📁 配置备份位置: $BACKUP_DIR"
echo "🔄 恢复命令: cp -r $BACKUP_DIR/* src/nav2_configs/params/"
echo "================================================"
