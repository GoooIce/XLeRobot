"""
统一遥操作系统的配置模块
从config.yaml文件加载配置，如果文件不存在则使用默认值作为后备方案。
"""

import os
import yaml
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional
import numpy as np
from pathlib import Path

# 默认配置值（如果YAML文件不存在则使用这些值作为后备）
DEFAULT_CONFIG = {
    "network": {  # 网络配置
        "https_port": 8443,        # HTTPS服务器端口号
        "websocket_port": 8442,    # WebSocket服务器端口号
        "host_ip": "0.0.0.0"      # 服务器监听的IP地址
    },
    "ssl": {  # SSL安全证书配置
        "certfile": "cert.pem",    # SSL证书文件路径
        "keyfile": "key.pem"       # SSL私钥文件路径
    },
    "robot": {  # 机器人配置
        "left_arm": {              # 左臂配置
            "name": "Left Arm",    # 左臂名称
            "port": "/dev/ttyACM0", # 左臂串口设备路径
            "enabled": True        # 是否启用左臂
        },
        "right_arm": {             # 右臂配置
            "name": "Right Arm",   # 右臂名称
            "port": "/dev/ttyACM1", # 右臂串口设备路径
            "enabled": True        # 是否启用右臂
        },
        "vr_to_robot_scale": 1.0,  # VR到机器人的缩放比例
        "send_interval": 0.05,     # 发送控制指令的时间间隔（秒）
    },
    "control": {  # 控制参数配置
        "keyboard": {              # 键盘控制配置
            "pos_step": 0.01,      # 键盘控制位置步长（米）
            "angle_step": 5.0      # 键盘控制角度步长（度）
        }
    }
}

def load_config(config_path: str = "config.yaml") -> dict:
    """
    从YAML文件加载配置，如果文件不存在则使用默认值作为后备方案。

    Args:
        config_path: 配置文件的路径，默认为"config.yaml"

    Returns:
        dict: 加载的配置字典
    """
    config = DEFAULT_CONFIG.copy()  # 复制默认配置作为基础

    if os.path.exists(config_path):  # 检查配置文件是否存在
        try:
            with open(config_path, 'r') as f:
                yaml_config = yaml.safe_load(f)  # 安全加载YAML配置
                if yaml_config:
                    # 深度合并YAML配置到默认配置中
                    _deep_merge(config, yaml_config)
        except Exception as e:
            print(f"警告：无法从{config_path}加载配置: {e}")
            print("使用默认配置")
    else:
        print(f"配置文件{config_path}未找到，使用默认配置")

    return config

def save_config(config: dict, config_path: str = "config.yaml"):
    """
    将配置保存到YAML文件。

    Args:
        config: 要保存的配置字典
        config_path: 配置文件的路径，默认为"config.yaml"

    Returns:
        bool: 保存成功返回True，失败返回False
    """
    try:
        with open(config_path, 'w') as f:
            # 将配置字典写入YAML文件，使用美观的格式
            yaml.dump(config, f, default_flow_style=False, indent=2)
        return True
    except Exception as e:
        print(f"保存配置到{config_path}时出错: {e}")
        return False

def _deep_merge(base: dict, update: dict):
    """
    深度合并字典：将update字典的内容合并到base字典中。

    Args:
        base: 基础字典（将被修改）
        update: 要合并的更新字典
    """
    for key, value in update.items():
        # 如果键在基础字典中存在且两边都是字典，则递归合并
        if key in base and isinstance(base[key], dict) and isinstance(value, dict):
            _deep_merge(base[key], value)
        else:
            # 否则直接覆盖或添加键值对
            base[key] = value

# 加载配置数据
_config_data = load_config()

# 为了向后兼容，从配置字典中提取常用值
HTTPS_PORT = _config_data["network"]["https_port"]        # HTTPS端口号
WEBSOCKET_PORT = _config_data["network"]["websocket_port"] # WebSocket端口号
HOST_IP = _config_data["network"]["host_ip"]               # 主机IP地址

CERTFILE = _config_data["ssl"]["certfile"]                 # SSL证书文件
KEYFILE = _config_data["ssl"]["keyfile"]                   # SSL私钥文件

VR_TO_ROBOT_SCALE = _config_data["robot"]["vr_to_robot_scale"] # VR到机器人的缩放比例
SEND_INTERVAL = _config_data["robot"]["send_interval"]          # 发送指令间隔

POS_STEP = _config_data["control"]["keyboard"]["pos_step"]      # 键盘控制位置步长
ANGLE_STEP = _config_data["control"]["keyboard"]["angle_step"]  # 键盘控制角度步长

# 设备端口配置
DEFAULT_FOLLOWER_PORTS = {
    "left": _config_data["robot"]["left_arm"]["port"],   # 左臂端口
    "right": _config_data["robot"]["right_arm"]["port"]  # 右臂端口
}

@dataclass
class XLeVRConfig:
    """
    遥操作系统的主要配置类（仅VR模式）。

    使用数据类装饰器自动生成初始化方法和其他方法。
    包含网络设置、SSL配置、设备端口配置等所有必要的配置参数。
    """
    # 网络设置
    https_port: int = HTTPS_PORT        # HTTPS服务器端口
    websocket_port: int = WEBSOCKET_PORT # WebSocket服务器端口
    host_ip: str = HOST_IP              # 服务器监听IP地址

    # SSL安全设置
    certfile: str = CERTFILE            # SSL证书文件路径
    keyfile: str = KEYFILE              # SSL私钥文件路径

    # 设备端口配置（可选，用于向后兼容）
    follower_ports: Dict[str, str] = None  # 左右臂的串口端口配置

    # 控制标志（仅VR和HTTPS相关）
    enable_vr: bool = True              # 是否启用VR控制
    enable_keyboard: bool = False       # 是否启用键盘控制
    enable_https: bool = True           # 是否启用HTTPS服务
    log_level: str = "warning"          # 日志级别
    vr_to_robot_scale: float = VR_TO_ROBOT_SCALE  # VR到机器人的缩放比例

    # 可选的Web应用目录，如果其他地方使用到
    webapp_dir: str = "webapp"          # Web应用目录路径
    def __post_init__(self):
        """
        在数据类初始化完成后调用的方法。
        用于初始化follower_ports配置。
        """
        if self.follower_ports is None:
            self.follower_ports = {
                "left": _config_data["robot"]["left_arm"]["port"],   # 左臂端口
                "right": _config_data["robot"]["right_arm"]["port"]  # 右臂端口
            }

    @property
    def ssl_files_exist(self) -> bool:
        """
        检查SSL证书和私钥文件是否存在。

        Returns:
            bool: 如果两个文件都存在返回True，否则返回False
        """
        return os.path.exists(self.certfile) and os.path.exists(self.keyfile)

    def ensure_ssl_certificates(self) -> bool:
        """
        确保SSL证书存在，如果不存在则生成自签名证书。

        Returns:
            bool: 证书创建或验证成功返回True，失败返回False
        """
        from .utils import ensure_ssl_certificates
        return ensure_ssl_certificates(self.certfile, self.keyfile)

    @property
    def webapp_exists(self) -> bool:
        """
        检查Web应用目录是否存在。

        Returns:
            bool: 目录存在返回True，否则返回False
        """
        return os.path.exists(self.webapp_dir)

def get_config_data():
    """
    获取当前配置数据的副本。

    Returns:
        dict: 当前配置数据的副本，防止外部修改原始数据
    """
    return _config_data.copy()

def update_config_data(new_config: dict):
    """
    更新配置数据并保存到文件。

    Args:
        new_config: 包含新配置的字典

    Returns:
        bool: 保存成功返回True，失败返回False
    """
    global _config_data
    _deep_merge(_config_data, new_config)  # 深度合并新配置
    return save_config(_config_data)       # 保存到文件

# 全局配置实例，供其他模块使用
config = XLeVRConfig() 
