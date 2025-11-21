# VR机械臂系统安装指南

## 📋 安装前准备

### 系统要求检查

```bash
# 检查Ubuntu版本
lsb_release -a

# 检查Python版本
python3 --version

# 检查Docker是否安装
docker --version

# 检查Conda是否安装
conda --version
```

### 网络要求

- PC和Quest3设备必须在同一网络
- 确保防火墙允许8000端口通信
- 建议使用有线网络连接PC

## 🔧 详细安装步骤

### 第一步：机械臂基础环境配置

**⚠️ 重要：必须先完成此步骤！**

1. **访问官方文档**：
   - 打开浏览器访问：[AIRBOT Play 软件安装指南](https://docs.airbots.online/airbot-play/quick-start/software-setup.html)

2. **按照官方文档完成安装**：
   ```bash
   # 1. 安装Docker引擎（按照官方文档）
   sudo apt-get update
   sudo apt-get install -y ca-certificates curl
   # ... 按照官方文档继续
   
   # 2. 安装驱动软件包
   sudo apt-get install ./zip/airbot-configure_5.1.6-1_all.deb
   
   # 3. 安装Python SDK包
   pip3 install -i https://pypi.tuna.tsinghua.edu.cn/simple ./zip/airbot_py-5.1.6-py3-none-any.whl
   ```

3. **验证基础环境**：
   ```bash
   # 检查驱动安装
   dpkg -l | grep airbot
   
   # 检查Python SDK
   python3 -m pip list | grep airbot
   
   # 测试机械臂连接（可选）
   # airbot_server -i can0 -p 50051
   ```

### 第二步：VR功能环境配置

#### 2.1 创建专用Conda环境

```bash
# 创建VR功能专用环境
conda create -n airbotplay_312 python=3.12 -y
conda activate airbotplay_312

# 验证环境
python --version
which python
```

#### 2.2 安装VR功能软件包

```bash
# 进入项目目录
cd /airbot_vr_py

# 安装VR功能所需的wheel包
pip install zip/airbot_py-5.1.6-py3-none-any.whl
pip install zip/mmk2_kdl_py-0.1.3-py3-none-any.whl
pip install zip/airbot_vr_py-0.0.1-py3-none-any.whl

# 安装其他依赖
pip install rclpy scipy numpy
```

#### 2.3 配置ROS2环境

```bash
# 设置ROS2环境变量
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc

# 验证ROS2安装
ros2 --version
```

#### 2.4 验证VR环境

```bash
# 激活conda环境
conda activate airbotplay_312

# 验证所有依赖
python3 -c "
import sys
print(f'Python版本: {sys.version}')

try:
    from airbot_py.arm import AIRBOTPlay
    print('✅ airbot SDK可用')
except ImportError as e:
    print(f'❌ airbot SDK导入失败: {e}')

try:
    from airbot_vr.vr_arm import VRArm
    print('✅ VR模块可用')
except ImportError as e:
    print(f'❌ VR模块导入失败: {e}')

try:
    import rclpy
    print('✅ ROS2可用')
except ImportError as e:
    print(f'❌ ROS2导入失败: {e}')

try:
    import scipy
    print('✅ SciPy可用')
except ImportError as e:
    print(f'❌ SciPy导入失败: {e}')
"
```

### 第三步：系统测试

#### 3.1 测试机械臂服务

```bash
# 启动机械臂服务（需要CAN总线权限）
sudo airbot_server -i can0 -p 50051
```

#### 3.2 测试VR控制系统

```bash
# 进入项目目录
cd /airbot_vr_py

# 启动VR控制系统
./start_simple.sh
```

#### 3.3 测试Quest3连接

```bash
# 在另一个终端测试连接
python3 test_vr_connection.py
```

## 🔍 环境验证清单

### 基础环境检查

- [ ] Ubuntu 20.04+ 系统
- [ ] Docker引擎安装并运行
- [ ] airbot-configure驱动安装成功
- [ ] airbot_py Python包安装成功
- [ ] 机械臂硬件连接正常

### VR环境检查

- [ ] Conda环境`airbotplay_312`创建成功
- [ ] 所有wheel包安装成功
- [ ] ROS2 Jazzy环境配置正确
- [ ] Python依赖包安装完整
- [ ] 网络连接正常

### 功能测试检查

- [ ] 机械臂服务启动成功
- [ ] VR控制系统启动成功
- [ ] TCP桥接器监听8000端口
- [ ] ROS2话题正常发布
- [ ] Quest3应用连接成功

## 🚨 常见安装问题

### 问题1：Docker权限不足

```bash
# 解决方案
sudo usermod -aG docker $USER
# 重新登录或重启系统
```

### 问题2：Python包安装失败

```bash
# 解决方案
conda activate airbotplay_312
pip install --upgrade pip
pip install zip/airbot_py-5.1.6-py3-none-any.whl
```

### 问题3：ROS2环境未配置

```bash
# 解决方案
source /opt/ros/jazzy/setup.bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
```

### 问题4：CAN总线权限不足

```bash
# 解决方案
sudo ip link set can0 up type can bitrate 1000000
sudo chmod 666 /dev/ttyUSB*  # 如果使用USB转CAN
```

## 📞 技术支持

如果安装过程中遇到问题：

1. **检查官方文档**：确保按照[AIRBOT官方文档](https://docs.airbots.online/airbot-play/quick-start/software-setup.html)完成基础环境配置
2. **查看错误日志**：仔细阅读终端输出的错误信息
3. **验证依赖**：使用上述验证脚本检查所有依赖是否正确安装
4. **网络连接**：确保网络连接正常，可以访问必要的软件源

---

**安装完成后，请参考[README.md](README.md)开始使用VR机械臂控制系统！**
