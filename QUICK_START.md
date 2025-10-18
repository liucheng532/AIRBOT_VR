# Quest3 VR机械臂控制 - 快速开始

## 🚀 最简单的启动方式

### 第一步：启动机械臂服务（宿主机）
```bash
airbot_server -i can0 -p 50051
```

### 第二步：启动VR控制系统
```bash
cd /airbot_vr_py
./start_quest3_vr.sh
```

### 第三步：配置Quest3应用
在`ControllerPoseSender.cs`中设置您的PC IP地址：
```csharp
public string ipAddress = "您的PC_IP";  // 运行启动脚本后会显示
public int port = 8000;
```

### 第四步：运行Quest3应用
启动您的VR应用，它会自动连接到PC

---

## 🔧 手动启动（用于调试）

### 终端1：TCP桥接器
```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=$PYTHONPATH:$(pwd)/airbot_vr_python_sdk
python3 tcp_to_ros2_bridge.py
```

### 终端2：VR机械臂控制（也在ROS2环境中）
```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=$PYTHONPATH:/airbot_vr_py/airbot_vr_python_sdk
python3 -m airbot_vr.vr_arm
```

---

## 🎮 控制说明

| Quest3操作 | 功能 |
|------------|------|
| 右手柄Grip | 开始VR控制 |
| 左手柄前扳机 | 激活机械臂位置控制 |
| 左手柄Grip | 控制夹爪开合 |
| 左手柄X键 | 重置机械臂到初始位置 |
| 左手柄Y键 | 紧急停止/退出控制 |

---

## 🔍 测试连接

### 测试TCP连接
```bash
python3 test_vr_connection.py
```

### 监控ROS2话题
```bash
# 查看按键状态
ros2 topic echo /vr_controller

# 查看左手位置
ros2 topic echo /leftInfo

# 查看右手位置
ros2 topic echo /rightInfo
```

---

## 💡 架构说明

```
Quest3 APP (TCP 8000)
    ↓
tcp_to_ros2_bridge.py (ROS2桥接)
    ↓
airbot_vr.vr_arm (原有VR控制节点)
    ↓
airbot_server (机械臂服务)
    ↓
机械臂硬件
```

**优势**：
- ✅ 使用原有的经过测试的VR控制代码
- ✅ 只需一个轻量级桥接器适配您的Quest3应用
- ✅ 保留所有原有的安全机制和功能

---

## ❗ 常见问题

**Q: 提示找不到rclpy模块？**
A: tcp_to_ros2_bridge.py需要在ROS2环境中运行，确保已执行：
```bash
source /opt/ros/jazzy/setup.bash
```

**Q: VR控制节点找不到airbot模块？**
A: 确保在airbotplay_312 conda环境中运行，并设置PYTHONPATH

**Q: Quest3连接不上？**
A: 检查防火墙、网络连接，确保PC和Quest3在同一网络

---

**准备好了吗？运行 `./start_quest3_vr.sh` 开始体验！** 🎮🤖

