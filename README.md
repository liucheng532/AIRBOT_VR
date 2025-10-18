# Quest3 VR机械臂遥操作系统

基于airbot机械臂的Quest3 VR遥操作系统，支持实时机械臂控制和VR交互。

## 🎯 功能特性

- ✅ **Quest3 VR控制** - 支持Quest3 VR设备实时控制机械臂
- ✅ **动态IP配置** - 支持运行时配置PC IP地址，无需重新编译
- ✅ **实时遥操作** - 250Hz高频控制，低延迟响应
- ✅ **安全机制** - 内置工作空间限制和紧急停止
- ✅ **易于部署** - 一键启动脚本，简化部署流程

## 🏗️ 系统架构

```
Quest3 VR设备
    ↓ TCP连接 (端口8000)
tcp_to_ros2_bridge.py (TCP到ROS2桥接器)
    ↓ ROS2话题
airbot_vr.vr_arm (VR控制节点)
    ↓ gRPC (端口50051)
airbot_server (机械臂驱动)
    ↓ CAN总线
机械臂硬件
```

## 📋 系统要求

### 硬件要求
- **机械臂**：AIRBOT Play系列
- **VR设备**：Meta Quest3
- **PC配置**：
  - 操作系统：Ubuntu 20.04 或更高版本
  - CPU：≥ 4 核（推荐用于实时控制）
  - 显存：≥ 8 GB
  - 存储空间：≥ 5GB可用磁盘空间
  - USB端口：≥ 3个USB端口

### 软件要求
- **Python**：3.9 或更高版本
- **Conda**：用于Python环境管理
- **Docker**：用于机械臂驱动

## 🚀 环境配置

### 第一步：机械臂基础环境配置

**⚠️ 重要：在安装VR功能之前，必须先完成机械臂的基础环境配置！**

请按照官方文档完成机械臂的基础环境配置：

1. **访问官方文档**：[AIRBOT Play 软件安装指南](https://docs.airbots.online/airbot-play/quick-start/software-setup.html)

2. **按照官方文档完成以下步骤**：
   - 安装Docker引擎
   - 安装驱动软件包 (`airbot-configure`)
   - 安装Python SDK包
   - 验证机械臂基础功能正常

3. **验证基础环境**：
   ```bash
   # 检查机械臂驱动是否安装成功
   dpkg -l | grep airbot
   
   # 检查Python SDK是否安装成功
   python3 -m pip list | grep airbot
   ```

### 第二步：VR功能环境配置

完成机械臂基础环境配置后，按照以下步骤安装VR功能：

#### 2.1 创建Conda环境

```bash
# 创建专用的conda环境
conda create -n airbotplay_312 python=3.12
conda activate airbotplay_312
```

#### 2.2 安装VR功能依赖

```bash
# 进入项目目录
cd /airbot_vr_py

# 安装VR功能所需的软件包
# 注意：这些包在zip/文件夹中提供
pip install zip/airbot_py-5.1.6-py3-none-any.whl
pip install zip/mmk2_kdl_py-0.1.3-py3-none-any.whl
pip install zip/airbot_vr_py-0.0.1-py3-none-any.whl

# 安装其他依赖
pip install rclpy scipy numpy
```

#### 2.3 配置ROS2环境

```bash
# 设置ROS2环境
source /opt/ros/jazzy/setup.bash

# 验证ROS2环境
ros2 --version
```

#### 2.4 验证VR环境

```bash
# 激活conda环境
conda activate airbotplay_312

# 验证Python包
python3 -c "from airbot_py.arm import AIRBOTPlay; print('✅ airbot SDK可用')"
python3 -c "from airbot_vr.vr_arm import VRArm; print('✅ VR模块可用')"
python3 -c "import rclpy; print('✅ ROS2可用')"
```

## 🎮 快速开始，需要让主机和quest3连接同一个热点

### 1. 启动机械臂服务

```bash
# 在宿主机运行（需要CAN总线权限）
airbot_server -i can0 -p 50051
```

### 2. 启动VR控制系统

```bash
# 进入项目目录
cd /airbot_vr_py

# 一键启动VR控制系统
./start_simple.sh
```

### 3. 配置Quest3应用（找冯紫嫣）

1. **获取PC IP地址**：启动脚本会显示本机IP地址
2. **配置Quest3应用**：
   - 在Unity项目中修改`ControllerPoseSender.cs`
   - 设置`ipAddress`为PC的IP地址
   - 重新编译并部署到Quest3设备

### 4. 开始VR控制

1. **启动Quest3应用**
2. **首次使用**：在VR界面中输入PC的IP地址
3. **日常使用**：应用会自动使用保存的IP地址

## 🎯 VR控制说明

### 控制操作

| Quest3操作 | 功能 |
|------------|------|
| 右手柄Grip | 开始VR控制 |
| 左手柄前扳机 | 激活机械臂位置控制 |
| 左手柄Grip | 控制夹爪开合 |
| 左手柄X键 | 重置机械臂到初始位置 |
| 左手柄Y键 | 紧急停止/退出控制 |
| 长按右手柄菜单键 | 重新配置IP地址 |

### 操作流程

```
启动Quest3应用
    ↓
首次使用：显示IP配置界面 → 输入PC IP → 连接成功
    ↓
右手柄Grip：开始VR控制
    ↓
左手柄前扳机：激活位置跟踪
    ↓
移动左手：机械臂跟随移动
    ↓
左手柄Grip：控制夹爪开合
```

## 🔧 测试和调试

### 测试VR连接

```bash
# 测试Quest3应用连接
python3 test_vr_connection.py
```

### 监控系统状态

```bash
# 监控ROS2话题
ros2 topic echo /vr_controller
ros2 topic echo /leftInfo
ros2 topic echo /rightInfo

# 检查机械臂服务
pgrep -f airbot_server

# 检查VR控制进程
pgrep -f tcp_to_ros2
```

## 📁 项目结构

```
/airbot_vr_py/
├── Unity project/              # Quest3 VR应用源码
│   └── Assets/Scripts/
│       └── ControllerPoseSender.cs
├── start_simple.sh            # 一键启动脚本
├── tcp_to_ros2_bridge.py     # TCP到ROS2桥接器
├── test_vr_connection.py      # VR连接测试工具
├── QUICK_START.md            # 快速开始指南
├── zip/                      # 软件包文件夹
│   ├── airbot_py-5.1.6-py3-none-any.whl
│   ├── mmk2_kdl_py-0.1.3-py3-none-any.whl
│   ├── airbot_vr_py-0.0.1-py3-none-any.whl
│   ├── airbot-configure_5.1.6-1_all.deb
│   └── airbot_vr部署操作文档.pdf
└── README.md                 # 本文件
```

## 🚨 安全注意事项

1. **机械臂朝向对齐**：确保机械臂朝向与人体朝向完全一致
2. **工作空间限制**：系统会自动限制机械臂在安全工作空间内
3. **紧急停止**：随时可以通过左手柄Y键紧急停止
4. **网络安全**：确保VR设备和PC在可信网络中
5. **操作前检查**：确保机械臂周围无障碍物

## 🔍 故障排除

### 常见问题

**Q: 机械臂服务启动失败**
```bash
# 检查CAN总线权限
sudo ip link show can0
# 检查机械臂驱动
dpkg -l | grep airbot
```

**Q: VR控制节点启动失败**
```bash
# 检查conda环境
conda activate airbotplay_312
# 检查Python包
python3 -c "from airbot_py.arm import AIRBOTPlay"
```

**Q: Quest3连接不上**
- 检查防火墙设置
- 确认PC和Quest3在同一网络
- 验证IP地址配置正确

**Q: 机械臂不跟随VR移动**
- 检查是否按下了正确的按键组合
- 确认机械臂服务正常运行
- 检查ROS2话题数据是否正常

### 调试命令

```bash
# 检查系统状态
./start_simple.sh  # 查看启动日志

# 测试连接
python3 test_vr_connection.py

# 监控数据流
ros2 topic echo /vr_controller
```

## 📞 技术支持

如果遇到问题，请检查：

1. **环境配置**：确保按照官方文档完成机械臂基础环境配置
2. **依赖安装**：确认所有Python包和ROS2环境正确安装
3. **网络连接**：验证PC和Quest3设备网络连通性
4. **权限设置**：检查CAN总线和Docker权限

## 📄 许可证

本项目基于airbot机械臂SDK开发，请遵循相关许可证要求。

## 🤝 贡献

欢迎提交Issue和Pull Request来改进本项目。

---

**开始您的VR机械臂控制之旅！** 🎮🤖

## 📚 参考文档

- [AIRBOT Play 官方文档](https://docs.airbots.online/airbot-play/quick-start/software-setup.html)
- [ROS2 Jazzy 文档](https://docs.ros.org/en/jazzy/)
- [Unity VR 开发指南](https://docs.unity3d.com/Manual/XR.html)
