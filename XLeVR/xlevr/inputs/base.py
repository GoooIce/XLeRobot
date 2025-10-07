"""
输入提供者的基础类和数据结构模块。

定义了遥操作系统中所有输入设备的抽象基类、控制模式、控制目标等核心数据结构。
包括VR设备、键盘、手柄等输入设备的通用接口定义。
"""

import asyncio
import numpy as np
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, Literal, Dict, Any
from enum import Enum

class ControlMode(Enum):
    """
    遥操作系统的控制模式枚举。

    定义了系统可以运行的不同控制模式，用于切换机器人的工作状态。
    """
    POSITION_CONTROL = "position"  # 位置控制模式：机器人根据输入进行位置跟踪
    IDLE = "idle"                 # 空闲模式：机器人保持当前位置，不响应输入

@dataclass
class ControlGoal:
    """
    输入提供者发送的高级控制目标消息。

    定义了单个机械臂的控制指令，包括位置、姿态、夹爪状态等。
    这个数据类作为输入设备和机器人控制器之间的通信标准。
    """
    arm: Literal["left", "right"]               # 控制的机械臂：左臂或右臂
    mode: Optional[ControlMode] = None          # 控制模式（None表示不改变模式）
    target_position: Optional[np.ndarray] = None # 3D目标位置（机器人坐标系）
    wrist_roll_deg: Optional[float] = None      # 手腕滚动角度（度）
    wrist_flex_deg: Optional[float] = None      # 手腕俯仰角度（度）
    gripper_closed: Optional[bool] = None       # 夹爪状态（None表示不改变状态）

    # 用于调试和监控的附加数据
    metadata: Optional[Dict[str, Any]] = None   # 额外的元数据信息

class BaseInputProvider(ABC):
    """
    输入提供者的抽象基类。

    定义了所有输入设备（VR、键盘、手柄等）必须实现的通用接口。
    确保不同输入设备能够以统一的方式向系统发送控制指令。
    """

    def __init__(self, command_queue: asyncio.Queue):
        """
        初始化输入提供者。

        Args:
            command_queue: 用于发送控制目标的异步队列
        """
        self.command_queue = command_queue  # 控制指令队列
        self.is_running = False             # 运行状态标志

    @abstractmethod
    async def start(self):
        """
        启动输入提供者。

        子类必须实现此方法，用于开始接收输入数据。
        应该包含设备初始化、连接建立等操作。
        """
        pass

    @abstractmethod
    async def stop(self):
        """
        停止输入提供者。

        子类必须实现此方法，用于停止接收输入数据。
        应该包含设备断开、资源清理等操作。
        """
        pass

    async def send_goal(self, goal: ControlGoal):
        """
        向命令队列发送控制目标。

        Args:
            goal: 要发送的控制目标对象
        """
        try:
            # 将控制目标异步放入队列
            await self.command_queue.put(goal)
        except Exception as e:
            # 处理队列满或其他错误
            # 静默处理错误，避免输入设备异常影响系统稳定性
            pass 