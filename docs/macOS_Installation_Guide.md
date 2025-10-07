# XLeRobot系统在macOS上原生运行完整指南

## 目录
1. [系统架构和依赖分析](#1-系统架构和依赖分析)
2. [macOS兼容性评估](#2-macos兼容性评估)
3. [安装和配置步骤](#3-安装和配置步骤)
4. [替代方案和解决方法](#4-替代方案和解决方法)
5. [开发环境设置](#5-开发环境设置)
6. [测试和验证方案](#6-测试和验证方案)
7. [故障排除指南](#7-故障排除指南)
8. [推荐配置](#8-推荐配置)

## 1. 系统架构和依赖分析

### 1.1 整体架构
XLeRobot是一个低成本的双臂移动家庭机器人系统，基于以下技术栈：
- **核心框架**: LeRobot (HuggingFace) 0.3.0+
- **编程语言**: Python 3.10+
- **深度学习**: PyTorch 2.2+
- **仿真环境**: ManiSkill + Mujoco
- **硬件控制**: Feetech STS3215舵机
- **视觉系统**: OpenCV + YOLO
- **通信协议**: 串口通信 (pyserial)

### 1.2 主要组件
1. **机械臂控制**: SO-100/SO-101双臂系统
2. **移动底座**: 三全向轮设计
3. **视觉系统**: 支持RGB/RGB-D摄像头
4. **遥控接口**: 键盘/Xbox手柄/Switch Joycon/VR
5. **仿真模块**: ManiSkill和Mujoco仿真环境
6. **Web控制**: FastAPI远程控制界面

### 1.3 核心依赖
```
Python >= 3.10
PyTorch >= 2.2
LeRobot >= 0.3.0
OpenCV >= 4.5
NumPy, SciPy
pyserial
ManiSkill
Mujoco
FastAPI, uvicorn
```

## 2. macOS兼容性评估

### 2.1 硬件兼容性
- ✅ **Apple Silicon (M1/M2/M3/M4)**: 完全支持
- ✅ **内存**: 建议最少16GB，推荐32GB
- ✅ **存储**: 至少50GB可用空间
- ⚠️ **USB端口**: 需要USB转串口适配器

### 2.2 软件兼容性
- ✅ **Python**: 3.10+完全兼容（当前系统3.13.3）
- ✅ **PyTorch**: 支持MPS GPU加速
- ✅ **LeRobot**: 原生支持macOS
- ✅ **OpenCV**: 完全支持，包括摄像头访问
- ⚠️ **pyserial**: 需要额外驱动配置
- ⚠️ **evdev**: Linux专用，需要替代方案

### 2.3 已知限制
1. **串口通信**: macOS对USB串口设备的访问权限较为严格
2. **evdev库**: 仅支持Linux，需要使用pynput替代
3. **摄像头权限**: 需要手动授予终端/IDE摄像头访问权限

## 3. 安装和配置步骤

### 3.1 系统准备
```bash
# 安装Xcode命令行工具
xcode-select --install

# 安装Homebrew（如果未安装）
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# 安装基础依赖
brew install cmake pkg-config ffmpeg portaudio

# 安装uv包管理器
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### 3.2 使用uv创建Python环境
```bash
# 克隆XLeRobot仓库
git clone https://github.com/Vector-Wangel/XLeRobot.git
cd XLeRobot

# 创建Python 3.10项目环境
uv python install 3.10
uv init --python 3.10

# 或直接在项目目录中创建虚拟环境
uv venv --python 3.10
source .venv/bin/activate
```

### 3.3 使用uv安装依赖
```bash
# 安装支持Apple Silicon GPU加速的PyTorch
uv add torch torchvision torchaudio

# 安装LeRobot和所有依赖
uv add 'lerobot[all]'

# 安装项目依赖（开发模式）
uv add -e .

# 安装视觉和串口依赖
uv add opencv-python pyserial pynput

# 安装其他必需依赖
uv add numpy scipy websockets pyyaml

# 安装仿真环境
uv add mujoco mujoco-python-viewer mani-skill gymnasium

# 可选：添加开发依赖
uv add --dev pytest black flake8 mypy
```

### 3.4 使用pyproject.toml管理依赖（推荐）
创建 `pyproject.toml` 文件：

```toml
[project]
name = "xlerobot"
version = "0.1.0"
description = "XLeRobot双臂移动机器人系统"
requires-python = ">=3.10"
dependencies = [
    "torch>=2.2.0",
    "torchvision",
    "torchaudio",
    "lerobot[all]>=0.3.0",
    "opencv-python>=4.5.0",
    "pyserial",
    "pynput",
    "numpy",
    "scipy",
    "websockets",
    "pyyaml",
    "mujoco",
    "mujoco-python-viewer",
    "mani-skill",
    "gymnasium",
    "fastapi",
    "uvicorn",
]

[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "black",
    "flake8",
    "mypy",
    "jupyter",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"
```

然后运行：
```bash
# 安装所有依赖
uv sync

# 安装开发依赖
uv sync --dev
```

## 4. 替代方案和解决方法

### 4.1 串口通信解决方案
```bash
# 安装USB转串口驱动
brew install --cask ch340g-driver
# 或使用CP210x驱动
brew install --cask cp210x-usb-driver

# 验证串口设备
ls /dev/cu.*
```

### 4.2 evdev替代方案
由于evdev仅支持Linux，在macOS上使用pynput：
```python
# 在代码中替换evdev导入
# from evdev import InputDevice, categorize, ecodes
import pynput.keyboard as keyboard
```

### 4.3 摄像头权限配置
```bash
# 授予终端摄像头访问权限
# 系统偏好设置 > 安全性与隐私 > 摄像头 > 添加终端应用
```

### 4.4 虚拟环境替代方案
如果遇到兼容性问题，可以使用Docker或UTM：
```bash
# 使用Docker运行Linux环境
docker pull python:3.10
docker run -it --device /dev/ttyUSB0 python:3.10
```

## 5. 开发环境设置

### 5.1 IDE配置
推荐使用VS Code或PyCharm：
```bash
# 安装VS Code扩展
code --install-extension ms-python.python
code --install-extension ms-python.pylint
code --install-extension ms-toolsai.jupyter
```

### 5.2 环境变量配置
```bash
# 添加到~/.zshrc或~/.bashrc
export PYTHONPATH="/Users/devel0per/Code/XLeRobot:$PYTHONPATH"
export DYLD_LIBRARY_PATH="/opt/homebrew/lib:$DYLD_LIBRARY_PATH"

# uv环境路径（如果使用.venv）
export PATH="/Users/devel0per/Code/XLeRobot/.venv/bin:$PATH"

# 重新加载配置
source ~/.zshrc
```

### 5.3 使用uv测试环境设置
```bash
# 创建测试脚本
cat > test_environment.py << 'EOF'
import torch
import cv2
import serial
import numpy as np

print("PyTorch version:", torch.__version__)
print("MPS available:", torch.backends.mps.is_available())
print("OpenCV version:", cv2.__version__)
print("NumPy version:", np.__version__)

# 测试摄像头
cap = cv2.VideoCapture(0)
if cap.isOpened():
    print("Camera: OK")
    cap.release()
else:
    print("Camera: Not available")

# 测试串口
try:
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:", [port.device for port in ports])
except:
    print("Serial ports: Check permissions")

# 测试关键库是否正确安装
try:
    import lerobot
    print("LeRobot: OK")
except ImportError as e:
    print(f"LeRobot: Error - {e}")

try:
    import mujoco
    print("Mujoco: OK")
except ImportError as e:
    print(f"Mujoco: Error - {e}")
EOF

# 使用uv运行测试脚本
uv run test_environment.py
```

## 6. 测试和验证方案

### 6.1 基础组件测试
```bash
# 测试仿真环境
cd /Users/devel0per/Code/XLeRobot/simulation/mujoco
uv run xlerobot_mujoco.py

# 测试机器人控制（需要硬件）
cd /Users/devel0per/Code/XLeRobot/software/examples
uv run 0_so100_keyboard_joint_control.py

# 测试视觉系统
uv run test_yolo.py
```

### 6.2 分阶段验证
1. **阶段1**: 软件环境安装验证
2. **阶段2**: 仿真环境测试
3. **阶段3**: 硬件连接测试
4. **阶段4**: 完整系统测试

### 6.3 性能基准测试
```python
# 创建性能测试脚本
cat > benchmark.py << 'EOF'
import time
import torch
import numpy as np

def test_mps_performance():
    # 测试MPS性能
    device = torch.device("mps" if torch.backends.mps.is_available() else "cpu")
    print(f"Using device: {device}")

    # 简单矩阵运算测试
    size = 1000
    a = torch.randn(size, size, device=device)
    b = torch.randn(size, size, device=device)

    start_time = time.time()
    c = torch.matmul(a, b)
    end_time = time.time()

    print(f"Matrix multiplication time: {end_time - start_time:.4f}s")

test_mps_performance()
EOF
```

## 7. 故障排除指南

### 7.1 常见问题及解决方案

**问题1: 摄像头无法访问**
```bash
# 解决方案：检查权限设置
sudo tccutil reset Camera
# 然后在系统偏好设置中重新授权
```

**问题2: 串口设备无权限**
```bash
# 解决方案：添加用户到dialout组（如果适用）
sudo usermod -a -G dialout $USER
# 或使用sudo运行程序
```

**问题3: MPS后端不可用**
```python
# 解决方案：强制使用CPU
device = torch.device("cpu")
```

**问题4: 动态链接库错误**
```bash
# 解决方案：重新安装依赖
brew reinstall pkg-config
uv remove opencv-python
uv add opencv-python --refresh
```

**问题5: uv环境激活问题**
```bash
# 解决方案：手动激活环境
source .venv/bin/activate
# 或使用uv run命令直接运行
uv run python script.py
```

## 8. 推荐配置

### 8.1 硬件配置建议
- **最低配置**: M1芯片，16GB内存，50GB存储
- **推荐配置**: M1 Pro/Max芯片，32GB内存，100GB存储
- **必需配件**: USB转串口适配器，摄像头

### 8.2 软件配置建议
- **操作系统**: macOS 12.0+ (Monterey或更新版本)
- **Python版本**: 3.10.x
- **开发工具**: VS Code + Python扩展
- **包管理**: uv + pyproject.toml（推荐）

### 8.3 uv使用最佳实践
```bash
# 查看当前环境信息
uv info

# 列出已安装的包
uv pip list

# 更新所有依赖
uv sync --upgrade

# 清理缓存
uv cache clean

# 导出依赖列表
uv pip freeze > requirements.txt

# 从requirements.txt安装
uv pip install -r requirements.txt
```

## 总结

XLeRobot系统在macOS上原生运行是完全可行的，主要优势包括：
- ✅ 完整的软件栈支持
- ✅ MPS GPU加速提升性能
- ✅ 原生开发环境
- ✅ 良好的兼容性

需要特别注意的方面：
- ⚠️ 串口设备权限配置
- ⚠️ 摄像头访问权限
- ⚠️ Linux专用库的替代方案

通过本指南提供的详细步骤和解决方案，您可以在macOS上成功搭建和运行完整的XLeRobot系统，进行机器人开发和仿真实验。