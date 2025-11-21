# 项目结构整理建议

## 🗂️ **当前项目结构**

```
/airbot_vr_py/
├── airbot_vr_python_sdk/          # ❌ 可以删除（本地复制）
├── Unity project/                  # ✅ 保留（Quest3应用）
├── build/                         # ❌ 可以删除（编译产物）
├── install/                       # ❌ 可以删除（安装产物）
├── src/                           # ❌ 可以删除（源代码，已编译）
├── start_simple.sh                # ✅ 保留（启动脚本）
├── tcp_to_ros2_bridge.py         # ✅ 保留（核心桥接器）
├── test_vr_connection.py         # ✅ 保留（测试工具）
├── QUICK_START.md                # ✅ 保留（使用说明）
└── reciver_tcp.py                # ❌ 可以删除（旧版本）
```

## 🧹 **建议删除的文件/文件夹**

### **可以安全删除**：
1. **`airbot_vr_python_sdk/`** - 本地复制的SDK，使用conda环境中的版本
2. **`build/`** - 编译产物，可以重新生成
3. **`install/`** - 安装产物，可以重新生成  
4. **`src/`** - 源代码，已编译完成
5. **`reciver_tcp.py`** - 旧版本，已被`tcp_to_ros2_bridge.py`替代

### **保留的核心文件**：
1. **`start_simple.sh`** - 启动脚本（已修改为使用conda环境）
2. **`tcp_to_ros2_bridge.py`** - TCP到ROS2桥接器
3. **`test_vr_connection.py`** - 连接测试工具
4. **`QUICK_START.md`** - 使用说明
5. **`Unity project/`** - Quest3应用代码

## 📝 **清理后的最终结构**

```
/airbot_vr_py/
├── Unity project/                 # Quest3 VR应用
│   └── Assets/Scripts/
│       ├── ControllerPoseSender.cs
│       └── (其他Unity脚本)
├── start_simple.sh               # 启动脚本
├── tcp_to_ros2_bridge.py        # TCP桥接器
├── test_vr_connection.py        # 测试工具
├── QUICK_START.md               # 使用说明
└── README.md                    # 项目说明（新增）
```

## 🔧 **环境依赖说明**

### **系统要求**：
- Ubuntu 20.04+ 
- ROS2 Jazzy
- Python 3.12
- Conda环境：`airbotplay_312`

### **conda环境中的包**：
- `airbot_py` - 机械臂控制SDK
- `airbot_vr` - VR控制模块
- `mmk2_kdl_py` - 运动学计算
- `rclpy` - ROS2 Python客户端
- `scipy`, `numpy` - 科学计算库

## 🚀 **GitHub上传建议**

### **1. 创建.gitignore**
```gitignore
# 编译产物
build/
install/
src/

# Python缓存
__pycache__/
*.pyc
*.pyo

# Unity缓存
Unity project/Library/
Unity project/Logs/
Unity project/obj/
Unity project/.vs/

# 系统文件
.DS_Store
Thumbs.db

# 临时文件
*.tmp
*.log
```

### **2. 创建README.md**
```markdown
# Quest3 VR机械臂控制系统

基于airbot机械臂的Quest3 VR遥操作系统。

## 功能特性
- Quest3 VR设备控制
- 实时机械臂遥操作
- 动态IP配置
- 安全控制机制

## 系统要求
- Ubuntu 20.04+
- ROS2 Jazzy
- Conda环境：airbotplay_312
- Quest3 VR设备

## 快速开始
```bash
./start_simple.sh
```

## 详细说明
请参考 [QUICK_START.md](QUICK_START.md)
```

## ✅ **清理步骤**

1. **删除不需要的文件夹**：
```bash
rm -rf airbot_vr_python_sdk/
rm -rf build/
rm -rf install/
rm -rf src/
rm -f reciver_tcp.py
```

2. **测试启动脚本**：
```bash
./start_simple.sh
```

3. **创建.gitignore**：
```bash
# 创建.gitignore文件
cat > .gitignore << 'EOF'
build/
install/
src/
__pycache__/
*.pyc
*.pyo
Unity project/Library/
Unity project/Logs/
Unity project/obj/
Unity project/.vs/
.DS_Store
Thumbs.db
*.tmp
*.log
EOF
```

4. **创建README.md**：
```bash
# 创建项目说明文档
```

## 🎯 **优势**

- ✅ 项目结构清晰
- ✅ 依赖管理规范（使用conda环境）
- ✅ 文件大小最小
- ✅ 易于维护和分享
- ✅ 符合开源项目标准

---

**建议：先测试修改后的启动脚本是否正常工作，然后再进行清理。**
