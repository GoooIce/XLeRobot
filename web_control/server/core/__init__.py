"""
统一远程控制系统的核心模块

该模块为简化架构提供核心组件：
- 配置管理：服务器配置参数的加载和验证
- 通信协议定义：与机器人主机通信的标准协议
- 远程控制核心实现：统一的机器人控制接口

简化架构的设计目标是：
1. 减少代码复杂度
2. 提供统一的接口
3. 支持多种机器人类型
4. 便于维护和扩展
"""

# 导入服务器配置类
from .config import ServerConfig
# 导入机器人通信协议类
from .protocol import RobotProtocol

# 导出的公共接口
__all__ = ['ServerConfig', 'RobotProtocol']