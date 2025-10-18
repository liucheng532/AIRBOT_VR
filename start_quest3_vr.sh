#!/bin/bash

echo "=========================================="
echo "启动Quest3 VR机械臂控制系统"
echo "使用原有的ROS2架构 + TCP桥接"
echo "=========================================="

# 获取本机IP
LOCAL_IP=$(hostname -I | awk '{print $1}')
echo "本机IP地址: $LOCAL_IP"
echo "请在Quest3应用中配置此IP地址"
echo ""

# 检查机械臂服务
if ! pgrep -f "airbot_server" > /dev/null; then
    echo "⚠️  警告: 机械臂服务未运行"
    echo "   请在宿主机运行: airbot_server -i can0 -p 50051"
    echo ""
fi

# 设置ROS2环境
source /opt/ros/jazzy/setup.bash

# 设置Python路径
export PYTHONPATH=$PYTHONPATH:$(pwd)/airbot_vr_python_sdk

# 启动函数
start_component() {
    local name=$1
    local command=$2
    echo "🚀 启动 $name..."
    gnome-terminal --title="$name" -- bash -c "
        source /opt/ros/jazzy/setup.bash
        export PYTHONPATH=\$PYTHONPATH:$(pwd)/airbot_vr_python_sdk
        cd $(pwd)
        $command
        exec bash
    " &
    sleep 2
}

# 1. 启动TCP到ROS2桥接器
start_component "TCP-ROS2桥接" "python3 tcp_to_ros2_bridge.py"

# 2. 启动原有的VR机械臂控制节点（也在ROS2环境中运行）
start_component "VR机械臂控制" "python3 -m airbot_vr.vr_arm"

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
echo "   - 右手柄Grip: 开始VR控制"
echo "   - 左手柄前扳机: 激活机械臂控制"
echo "   - 左手柄Grip: 控制夹爪"
echo "   - 左手柄X键: 重置机械臂位置"
echo "   - 左手柄Y键: 退出控制"
echo ""
echo "🔍 测试命令:"
echo "   ros2 topic echo /vr_controller"
echo "   ros2 topic echo /leftInfo"
echo ""
echo "按 Ctrl+C 停止"
echo ""

# 等待用户中断
trap 'echo "正在停止所有服务..."; pkill -f "tcp_to_ros2"; pkill -f "airbot_vr.vr_arm"; exit 0' INT
while true; do
    sleep 1
done

