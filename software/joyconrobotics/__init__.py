"""
JoyCon机器人控制模块

这个模块提供了用于控制Nintendo Switch JoyCon手柄的Python接口，
主要用于机器人控制和姿态检测应用。

主要功能：
- JoyCon设备连接和通信
- 按键状态检测和事件处理
- 陀螺仪和加速度计数据读取
- 机器人姿态控制和移动

作者：Box2AI Robotics 盒桥智能
版权所有 © 2025
"""

# 导入主要的JoyCon机器人控制类
from .joyconrobotics import JoyconRobotics

# 版本号
__version__ = "0.2.4"

# 公开导出的类列表
__all__ = [
    "JoyconRobotics",
]
