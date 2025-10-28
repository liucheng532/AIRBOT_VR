#!/bin/bash

# 简化版启动脚本 - 适配Quest3应用
# 所有组件都在ROS2环境中运行

echo "=========================================="
echo "Quest3 VR机械臂控制系统 - 简化启动"
echo "=========================================="
echo ""

# 设置环境
source /opt/ros/jazzy/setup.bash

# 激活conda环境
source /opt/miniconda/etc/profile.d/conda.sh
conda activate airbotplay_312

# 不需要设置PYTHONPATH，因为conda环境中已经安装了airbot包

# 获取IP地址
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo "📡 本机IP地址: $LOCAL_IP"
echo "   请在Quest3应用中配置此IP地址（端口8000）"
echo ""

# 检查机械臂服务
if ! pgrep -f "airbot_server" > /dev/null; then
    echo "⚠️  机械臂服务未运行，请先在宿主机执行："
    echo "   airbot_server -i can0 -p 50051"
    echo ""
    read -p "按Enter继续（或Ctrl+C取消）..."
fi

echo "=========================================="
echo "启动组件..."
echo "=========================================="
echo ""

# 启动TCP桥接器（后台运行）
echo "🚀 启动TCP到ROS2桥接器..."
python3 tcp_to_ros2_bridge.py &
BRIDGE_PID=$!
sleep 2

# 检查桥接器是否启动成功
if ! kill -0 $BRIDGE_PID 2>/dev/null; then
    echo "❌ TCP桥接器启动失败"
    exit 1
fi
echo "✅ TCP桥接器运行中 (PID: $BRIDGE_PID)"

# 启动VR控制节点（后台运行）
# echo "🚀 启动VR机械臂控制节点..."
# python3 -m airbot_vr.vr_arm_record &
# VR_ARM_PID=$!
# sleep 2


# 启动VR控制节点（后台运行）
echo "🚀 启动VR机械臂控制节点..."
cd /airbot_vr_py
python3 vr_arm_record.py &
VR_ARM_PID=$!
sleep 2


# 检查VR控制节点是否启动成功
if ! kill -0 $VR_ARM_PID 2>/dev/null; then
    echo "❌ VR控制节点启动失败"
    kill $BRIDGE_PID 2>/dev/null
    exit 1
fi
echo "✅ VR控制节点运行中 (PID: $VR_ARM_PID)"

echo ""
echo "=========================================="
echo "✅ 系统启动完成！"
echo "=========================================="
echo ""
echo "📱 Quest3应用配置:"
echo "   IP地址: $LOCAL_IP"
echo "   端口: 8000"
echo ""
echo "🎮 控制说明:"
echo "   右手柄Grip → 开始VR控制"
echo "   左手柄前扳机 → 激活机械臂"
echo "   左手柄Grip → 控制夹爪"
echo "   左手柄X键 → 重置位置"
echo "   左手柄Y键 → 退出"
echo ""
echo "🔍 测试命令（新终端）:"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   ros2 topic echo /vr_controller"
echo "   ros2 topic echo /leftInfo"
echo ""
echo "按 Ctrl+C 停止所有服务"
echo ""

# 清理函数
cleanup() {
    echo ""
    echo "正在停止服务..."
    kill $BRIDGE_PID 2>/dev/null
    kill $VR_ARM_PID 2>/dev/null
    wait $BRIDGE_PID 2>/dev/null
    wait $VR_ARM_PID 2>/dev/null
    echo "✅ 所有服务已停止"
    exit 0
}

trap cleanup INT TERM

# 保持运行
while true; do
    # 检查进程是否还在运行
    if ! kill -0 $BRIDGE_PID 2>/dev/null; then
        echo "❌ TCP桥接器已停止"
        cleanup
    fi
    if ! kill -0 $VR_ARM_PID 2>/dev/null; then
        echo "❌ VR控制节点已停止"
        cleanup
    fi
    sleep 1
done

