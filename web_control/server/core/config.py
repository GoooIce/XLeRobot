"""
统一远程控制的简化配置系统

该配置系统用简洁、最小化的基本设置集替换复杂的多控制器配置。

设计目标：
1. 简化配置管理
2. 减少配置项数量
3. 提供合理的默认值
4. 支持环境变量配置
5. 便于部署和维护
"""

# 导入操作系统相关功能
import os
# 导入数据类装饰器，用于创建配置类
from dataclasses import dataclass
# 导入可选类型提示
from typing import Optional
# 导入路径处理库
from pathlib import Path


@dataclass
class ServerConfig:
    """
    简化的服务器配置类，只包含必要的设置项

    使用数据类装饰器自动生成初始化方法、比较方法等
    """

    # ==================== 机器人类型配置 ====================
    # 决定连接到哪种类型的主机程序
    robot_type: str = "maniskill"  # 支持的类型: maniskill|mujoco|xlerobot

    # ==================== UI服务器配置 ====================
    # Web界面运行的服务器配置
    ui_host: str = "0.0.0.0"      # Web服务器绑定的IP地址 (0.0.0.0表示所有网络接口)
    ui_port: int = 8000           # Web界面的端口号

    # ==================== 机器人主机配置 ====================
    # 机器人/仿真程序运行的远程主机配置
    robot_host: str = "localhost" # 机器人主机程序的IP地址
    robot_port_cmd: int = 5555    # 命令通道端口号 (发送控制指令)
    robot_port_data: int = 5556   # 数据/观察通道端口号 (接收状态数据)

    # ==================== 视频设置 ====================
    # 摄像头流的配置参数
    video_width: int = 640        # 视频宽度 (像素)
    video_height: int = 480       # 视频高度 (像素)
    video_fps: int = 30           # 视频帧率 (每秒帧数)
    video_quality: int = 80       # JPEG图像质量 (1-100, 越高质量越好)

    # ==================== 连接设置 ====================
    # 网络连接相关的超时和轮询配置
    connect_timeout_s: int = 5    # 连接超时时间 (秒)
    polling_timeout_ms: int = 100 # 数据轮询超时时间 (毫秒)

    @classmethod
    def from_env(cls, env_file: Optional[str] = None) -> "ServerConfig":
        """
        从环境变量加载配置

        该方法允许通过环境变量或.env文件覆盖默认配置值，
        便于在不同环境中部署和配置系统。

        Args:
            env_file: 可选的.env文件路径，用于加载环境变量

        Returns:
            包含环境变量值的ServerConfig实例
        """
        # 如果提供了环境变量文件，先加载它
        if env_file:
            cls._load_env_file(env_file)

        # 使用环境变量创建配置实例，如果环境变量不存在则使用默认值
        return cls(
            # ==================== 机器人类型 ====================
            robot_type=os.getenv('ROBOT_TYPE', 'maniskill'),

            # ==================== UI服务器配置 ====================
            ui_host=os.getenv('UI_HOST', '0.0.0.0'),
            ui_port=int(os.getenv('UI_PORT', '8000')),

            # ==================== 机器人主机配置 ====================
            robot_host=os.getenv('ROBOT_HOST', 'localhost'),
            robot_port_cmd=int(os.getenv('ROBOT_PORT_CMD', '5555')),
            robot_port_data=int(os.getenv('ROBOT_PORT_DATA', '5556')),

            # ==================== 视频设置 ====================
            video_width=int(os.getenv('VIDEO_WIDTH', '640')),
            video_height=int(os.getenv('VIDEO_HEIGHT', '480')),
            video_fps=int(os.getenv('VIDEO_FPS', '30')),
            video_quality=int(os.getenv('VIDEO_QUALITY', '80')),

            # ==================== 连接设置 ====================
            connect_timeout_s=int(os.getenv('CONNECT_TIMEOUT_S', '5')),
            polling_timeout_ms=int(os.getenv('POLLING_TIMEOUT_MS', '100')),
        )

    @staticmethod
    def _load_env_file(env_file: str) -> None:
        """
        从.env文件加载环境变量

        解析.env文件中的键值对，并将其设置为当前进程的环境变量。
        支持注释行（以#开头）和空行的忽略。

        Args:
            env_file: .env文件的路径
        """
        env_path = Path(env_file)
        # 检查文件是否存在
        if not env_path.exists():
            print(f"Warning: .env file not found at {env_path}")
            return

        # 逐行读取并解析环境变量
        with open(env_path, 'r') as f:
            for line in f:
                line = line.strip()  # 移除首尾空白字符
                # 跳过空行和注释行，只处理包含等号的行
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)  # 只分割第一个等号
                    os.environ[key.strip()] = value.strip()  # 设置环境变量

    def validate(self) -> None:
        """
        验证配置值的有效性

        检查所有配置参数是否在有效范围内，如果发现无效值则抛出异常。
        这有助于在系统启动时发现配置错误，避免运行时问题。

        Raises:
            ValueError: 当任何配置值无效时抛出
        """
        # 验证机器人类型
        valid_robot_types = ['maniskill', 'mujoco', 'xlerobot']
        if self.robot_type not in valid_robot_types:
            raise ValueError(f"Invalid robot_type: {self.robot_type}. Must be one of {valid_robot_types}")

        # 验证UI端口号 (1-65535是有效的TCP/UDP端口范围)
        if not (1 <= self.ui_port <= 65535):
            raise ValueError(f"Invalid ui_port: {self.ui_port}. Must be between 1-65535")

        # 验证机器人命令端口号
        if not (1 <= self.robot_port_cmd <= 65535):
            raise ValueError(f"Invalid robot_port_cmd: {self.robot_port_cmd}. Must be between 1-65535")

        # 验证机器人数据端口号
        if not (1 <= self.robot_port_data <= 65535):
            raise ValueError(f"Invalid robot_port_data: {self.robot_port_data}. Must be between 1-65535")

        # 验证视频质量 (JPEG质量范围1-100)
        if not (1 <= self.video_quality <= 100):
            raise ValueError(f"Invalid video_quality: {self.video_quality}. Must be between 1-100")

        # 验证视频帧率 (合理的帧率范围1-120)
        if not (1 <= self.video_fps <= 120):
            raise ValueError(f"Invalid video_fps: {self.video_fps}. Must be between 1-120")

    def get_robot_cmd_address(self) -> str:
        """
        获取机器人命令通道的完整ZeroMQ地址

        Returns:
            格式为 "tcp://主机:端口" 的命令通道地址字符串
        """
        return f"tcp://{self.robot_host}:{self.robot_port_cmd}"

    def get_robot_data_address(self) -> str:
        """
        获取机器人数据通道的完整ZeroMQ地址

        Returns:
            格式为 "tcp://主机:端口" 的数据通道地址字符串
        """
        return f"tcp://{self.robot_host}:{self.robot_port_data}"

    def __str__(self) -> str:
        """
        返回配置的字符串表示，用于日志记录

        Returns:
            包含主要配置信息的格式化字符串
        """
        return (
            f"ServerConfig(\n"
            f"  robot_type={self.robot_type}\n"                    # 机器人类型
            f"  ui_server={self.ui_host}:{self.ui_port}\n"       # UI服务器地址
            f"  robot_host={self.robot_host}:{self.robot_port_cmd}/{self.robot_port_data}\n"  # 机器人主机地址
            f"  video={self.video_width}x{self.video_height}@{self.video_fps}fps\n"  # 视频配置
            f")"
        )