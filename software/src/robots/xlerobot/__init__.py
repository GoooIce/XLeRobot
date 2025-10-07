"""
XLeRobot机器人控制核心模块

该模块提供了XLeRobot机器人的核心控制功能，包括：
- 机器人配置管理
- 机器人硬件控制接口
- 客户端-主机通信接口
"""

# 导入XLeRobot配置类，用于定义机器人的硬件参数和控制参数
from .config_xlerobot import XLerobotConfig
# 导入XLeRobot核心控制类，提供机器人硬件的直接控制功能
from .xlerobot import XLerobot
# 导入客户端类，用于远程控制机器人（暂时注释掉）
# from .xlerobot_client import XLerobotClient
# 导入主机相关类，用于运行机器人控制服务（暂时注释掉）
# from .xlerobot_host import XLerobotHost, XLerobotHostConfig

# 定义模块的公共接口，当使用 from xlerobot import * 时导入这些类
__all__ = [
    'XLerobotConfig',  # 配置类
    'XLerobot',        # 核心机器人控制类
]