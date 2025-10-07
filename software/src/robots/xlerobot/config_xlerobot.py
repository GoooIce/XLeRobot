# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
XLeRobot配置模块

该模块定义了XLeRobot机器人的各种配置类，包括：
- 相机配置函数
- 机器人硬件配置类
- 主机和客户端配置类
"""

from dataclasses import dataclass, field

# 导入相机相关配置类
from lerobot.cameras.configs import CameraConfig, Cv2Rotation, ColorMode
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCamera, RealSenseCameraConfig

# 导入基础机器人配置类
from ..config import RobotConfig


def xlerobot_cameras_config() -> dict[str, CameraConfig]:
    """
    XLeRobot相机配置函数

    该函数定义了机器人支持的所有相机配置选项。目前所有相机配置都被注释掉，
    用户可以根据实际硬件情况启用相应的相机配置。

    Returns:
        dict[str, CameraConfig]: 包含所有相机配置的字典，键为相机名称，值为相机配置对象

    Note:
        以下是可用的相机配置示例：
        - left_wrist: 左腕相机（USB摄像头）
        - right_wrist: 右腕相机（USB摄像头）
        - head: 头部相机（RealSense深度相机）
    """
    return {
        # 左腕相机配置 - 使用USB摄像头
        # "left_wrist": OpenCVCameraConfig(
        #     index_or_path="/dev/video0",    # USB设备路径
        #     fps=30,                         # 帧率：30帧/秒
        #     width=640,                      # 图像宽度：640像素
        #     height=480,                     # 图像高度：480像素
        #     rotation=Cv2Rotation.NO_ROTATION # 图像旋转：无旋转
        # ),

        # 右腕相机配置 - 使用USB摄像头
        # "right_wrist": OpenCVCameraConfig(
        #     index_or_path="/dev/video2",    # USB设备路径
        #     fps=30,                         # 帧率：30帧/秒
        #     width=640,                      # 图像宽度：640像素
        #     height=480,                     # 图像高度：480像素
        #     rotation=Cv2Rotation.NO_ROTATION # 图像旋转：无旋转
        # ),

        # 头部RGB-D相机配置 - 使用USB摄像头
        # "head(RGDB)": OpenCVCameraConfig(
        #     index_or_path="/dev/video2",    # USB设备路径
        #     fps=30,                         # 帧率：30帧/秒
        #     width=640,                      # 图像宽度：640像素
        #     height=480,                     # 图像高度：480像素
        #     rotation=Cv2Rotation.NO_ROTATION # 图像旋转：无旋转
        # ),

        # 头部RealSense相机配置 - 支持深度信息
        # "head": RealSenseCameraConfig(
        #     serial_number_or_name="125322060037",  # 相机序列号（需要替换为实际序列号）
        #     fps=30,                                 # 帧率：30帧/秒
        #     width=1280,                            # 图像宽度：1280像素
        #     height=720,                            # 图像高度：720像素
        #     color_mode=ColorMode.BGR,              # 颜色模式：BGR格式（OpenCV默认）
        #     rotation=Cv2Rotation.NO_ROTATION,      # 图像旋转：无旋转
        #     use_depth=True                         # 启用深度信息采集
        # ),
    }


@RobotConfig.register_subclass("xlerobot")
@dataclass
class XLerobotConfig(RobotConfig):
    """
    XLeRobot机器人配置类

    该类继承自RobotConfig，定义了XLeRobot机器人的所有硬件参数和控制参数。
    包括串口配置、安全参数、相机配置、遥操作键位配置等。

    Attributes:
        port1 (str): 主控制串口路径，连接so101控制器和头部相机
        port2 (str): 副控制串口路径，连接其他硬件设备
        disable_torque_on_disconnect (bool): 断开连接时是否自动禁用电机扭矩，提高安全性
        max_relative_target (int | None): 相对位置目标向量的最大安全限制，防止过大的运动指令
        cameras (dict[str, CameraConfig]): 相机配置字典，包含所有已配置的相机参数
        use_degrees (bool): 角度单位设置，True表示使用度数，False表示使用弧度
        teleop_keys (dict[str, str]): 遥操作键位映射，定义键盘控制按键
    """

    # 串口配置
    port1: str = "/dev/ttyACM0"  # 主控制串口（连接so101控制器和头部相机）
    port2: str = "/dev/ttyACM1"  # 副控制串口（与其他设备通信）

    # 安全配置
    disable_torque_on_disconnect: bool = True  # 断开连接时自动禁用电机扭矩，防止意外运动

    # 安全限制参数
    # max_relative_target 限制相对位置目标向量的大小以确保安全。
    # 设置为正标量为所有电机提供相同值，或设置为与跟随臂电机数量相同长度的列表。
    max_relative_target: int | None = None  # 相对位置目标的最大安全限制

    # 相机配置
    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)  # 所有相机的配置字典

    # 单位设置
    # 设置为`True`以与之前的策略/数据集保持向后兼容
    use_degrees: bool = False  # False表示使用弧度，True表示使用度数

    # 遥操作键位配置
    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # 底盘移动控制键
            "forward": "i",      # 前进（+x方向）
            "backward": "k",     # 后退（-x方向）
            "left": "j",         # 左移（+y方向）
            "right": "l",        # 右移（-y方向）
            "rotate_left": "u",  # 左转（+z方向）
            "rotate_right": "o", # 右转（-z方向）

            # 速度控制键
            "speed_up": "n",     # 提高移动速度
            "speed_down": "m",   # 降低移动速度

            # 系统控制键
            "quit": "b",         # 退出遥操作模式
        }
    )



@dataclass
class XLerobotHostConfig:
    """
    XLeRobot主机配置类

    该类定义了机器人主机的网络配置和运行参数。主机运行在机器人端，
    负责接收客户端指令并执行，同时向客户端发送传感器观测数据。

    Attributes:
        port_zmq_cmd (int): ZMQ命令接收端口，用于接收客户端发送的控制指令
        port_zmq_observations (int): ZMQ观测数据发送端口，用于向客户端发送传感器数据
        connection_time_s (int): 连接持续时间（秒），超过此时间后主机将自动停止运行
        watchdog_timeout_ms (int): 看门狗超时时间（毫秒），超过此时间未收到指令则停止机器人
        max_loop_freq_hz (int): 最大循环频率（赫兹），控制主循环的运行频率，防止CPU过载
    """

    # 网络通信配置
    port_zmq_cmd: int = 5555           # 命令接收端口（客户端→主机）
    port_zmq_observations: int = 5556   # 观测数据发送端口（主机→客户端）

    # 运行时配置
    connection_time_s: int = 3600       # 程序运行时间限制（秒），1小时后自动停止

    # 安全配置
    # 看门狗机制：如果超过0.5秒没有收到命令，则停止机器人运动，确保安全
    watchdog_timeout_ms: int = 500      # 看门狗超时时间（毫秒）

    # 性能配置
    # 如果机器人运动出现抖动，可以降低此频率并使用`top`命令监控CPU负载
    max_loop_freq_hz: int = 30          # 最大循环频率（赫兹），控制主循环运行速度

@RobotConfig.register_subclass("xlerobot_client")
@dataclass
class XLerobotClientConfig(RobotConfig):
    """
    XLeRobot客户端配置类

    该类定义了机器人客户端的网络配置和运行参数。客户端运行在控制端，
    负责向主机发送控制指令并接收传感器观测数据。

    Attributes:
        remote_ip (str): 主机IP地址，运行在机器人上的主机的网络地址
        port_zmq_cmd (int): ZMQ命令发送端口，用于向主机发送控制指令
        port_zmq_observations (int): ZMQ观测数据接收端口，用于接收主机发送的传感器数据
        teleop_keys (dict[str, str]): 遥操作键位映射，定义键盘控制按键
        cameras (dict[str, CameraConfig]): 相机配置字典，定义客户端期望接收的相机数据格式
        polling_timeout_ms (int): 轮询超时时间（毫秒），等待数据的最大时间
        connect_timeout_s (int): 连接超时时间（秒），建立连接的最大等待时间
    """

    # 网络通信配置
    remote_ip: str  # 主机IP地址（必需参数，机器人端的IP地址）
    port_zmq_cmd: int = 5555           # 命令发送端口（客户端→主机）
    port_zmq_observations: int = 5556   # 观测数据接收端口（主机→客户端）

    # 遥操作键位配置
    teleop_keys: dict[str, str] = field(
        default_factory=lambda: {
            # 底盘移动控制键
            "forward": "i",      # 前进（+x方向）
            "backward": "k",     # 后退（-x方向）
            "left": "j",         # 左移（+y方向）
            "right": "l",        # 右移（-y方向）
            "rotate_left": "u",  # 左转（+z方向）
            "rotate_right": "o", # 右转（-z方向）

            # 速度控制键
            "speed_up": "n",     # 提高移动速度
            "speed_down": "m",   # 降低移动速度

            # 系统控制键
            "quit": "b",         # 退出遥操作模式
        }
    )

    # 相机配置
    cameras: dict[str, CameraConfig] = field(default_factory=xlerobot_cameras_config)  # 客户端期望的相机配置

    # 网络超时配置
    polling_timeout_ms: int = 15   # 轮询超时时间（毫秒），等待数据的最大时间
    connect_timeout_s: int = 5     # 连接超时时间（秒），建立连接的最大等待时间
