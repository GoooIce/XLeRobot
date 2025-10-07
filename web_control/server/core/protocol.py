"""
机器人主机的统一通信协议

该模块定义了远程控制核心与机器人主机之间通信的标准消息格式
和编码/解码函数。使用JSON格式进行消息传输，支持多种命令类型。

协议特点：
1. 使用JSON格式，便于调试和扩展
2. 支持Base64编码的二进制数据传输
3. 包含时间戳用于性能监控
4. 统一的错误处理机制
5. 支持多种机器人类型
"""

# 导入JSON处理库
import json
# 导入Base64编码库，用于二进制数据传输
import base64
# 导入时间模块，用于生成时间戳
import time
# 导入类型提示
from typing import Dict, Any, Optional, List
# 导入枚举类
from enum import Enum


class CommandType(Enum):
    """
    机器人控制的标准命令类型枚举

    定义了所有支持的机器人控制命令类型，确保通信双方使用统一的命令标识。
    """

    MOVE = "move"                    # 移动命令
    STOP = "stop"                    # 停止命令
    RESET = "reset"                  # 重置命令
    GET_STATE = "get_state"          # 获取状态命令
    SET_ARM_JOINT = "set_arm_joint"  # 设置机械臂关节角度命令
    SET_CAMERA_POSITION = "set_camera_position"  # 设置摄像头位置命令
    RESET_CAMERA = "reset_camera"    # 重置摄像头命令
    PING = "ping"                    # 连通性测试命令


class ResponseType(Enum):
    """
    机器人主机的标准响应类型枚举

    定义了所有支持的响应类型，用于区分不同类型的返回数据。
    """

    SUCCESS = "success"  # 成功响应
    ERROR = "error"      # 错误响应
    STATE = "state"      # 状态数据响应
    VIDEO = "video"      # 视频帧数据响应
    PONG = "pong"        # PING命令的响应


class RobotProtocol:
    """
    机器人通信的统一协议类

    提供编码和解码机器人通信消息的静态方法。该类作为工具类使用，
    所有方法都是静态的，不需要实例化。

    主要功能：
    1. 命令消息编码：将命令对象转换为字节流
    2. 响应消息解码：将字节流转换为响应对象
    3. 特定命令的便捷编码方法
    4. 视频帧编码/解码
    5. 消息格式验证
    """

    @staticmethod
    def encode_command(command_type: CommandType, data: Optional[Dict[str, Any]] = None) -> bytes:
        """
        编码命令消息以便发送给机器人主机

        将命令类型和可选数据编码为标准JSON格式的字节流，
        包含消息类型、命令类型、数据和时间戳。

        Args:
            command_type: 要发送的命令类型
            data: 可选的命令数据/参数字典

        Returns:
            编码后的字节流消息
        """
        # 构造标准命令消息格式
        message = {
            "type": "command",                    # 消息类型标识
            "command": command_type.value,       # 具体命令类型
            "data": data or {},                  # 命令数据，如果为空则使用空字典
            "timestamp": time.time()             # 当前时间戳，用于性能监控
        }
        # 将JSON对象编码为UTF-8字节流
        return json.dumps(message).encode('utf-8')

    @staticmethod
    def decode_response(raw_data: bytes) -> Dict[str, Any]:
        """
        解码从机器人主机接收的响应消息

        将字节流数据解码为JSON格式的响应对象。
        如果解码失败，返回错误响应消息。

        Args:
            raw_data: 从机器人接收的原始字节数据

        Returns:
            解码后的消息字典，如果解码失败则返回错误消息
        """
        try:
            # 尝试将UTF-8字节流解码为JSON对象
            message = json.loads(raw_data.decode('utf-8'))
            return message
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            # 解码失败时返回标准错误响应
            return {
                "type": "error",                              # 消息类型
                "response": ResponseType.ERROR.value,         # 响应类型
                "message": f"Failed to decode response: {e}",  # 错误描述
                "timestamp": time.time()                      # 错误发生时间
            }

    @staticmethod
    def encode_response(response_type: ResponseType, data: Optional[Dict[str, Any]] = None) -> bytes:
        """
        编码响应消息以便从机器人主机发送

        将响应类型和数据编码为标准JSON格式的字节流，
        包含消息类型、响应类型、数据和时间戳。

        Args:
            response_type: 响应类型
            data: 响应数据字典

        Returns:
            编码后的字节流消息
        """
        # 构造标准响应消息格式
        message = {
            "type": "response",                   # 消息类型标识
            "response": response_type.value,     # 具体响应类型
            "data": data or {},                  # 响应数据，如果为空则使用空字典
            "timestamp": time.time()             # 当前时间戳
        }
        # 将JSON对象编码为UTF-8字节流
        return json.dumps(message).encode('utf-8')

    @staticmethod
    def decode_command(raw_data: bytes) -> Dict[str, Any]:
        """
        解码从远程控制核心接收的命令消息

        将字节流数据解码为JSON格式的命令对象。
        如果解码失败，返回错误命令消息。

        Args:
            raw_data: 从远程控制核心接收的原始字节数据

        Returns:
            解码后的命令字典，如果解码失败则返回错误消息
        """
        try:
            # 尝试将UTF-8字节流解码为JSON对象
            message = json.loads(raw_data.decode('utf-8'))
            return message
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            # 解码失败时返回标准错误响应
            return {
                "type": "error",                              # 消息类型
                "command": "invalid",                         # 无效命令标识
                "message": f"Failed to decode command: {e}",  # 错误描述
                "timestamp": time.time()                      # 错误发生时间
            }

    @staticmethod
    def encode_move_command(direction: str, speed: float = 1.0) -> bytes:
        """
        编码移动命令

        创建包含移动方向和速度的命令消息。

        Args:
            direction: 移动方向 (forward, backward, left, right等)
            speed: 移动速度 (0.0到1.0之间)

        Returns:
            编码后的移动命令字节流
        """
        return RobotProtocol.encode_command(
            CommandType.MOVE,
            {"direction": direction, "speed": speed}  # 移动参数
        )

    @staticmethod
    def encode_arm_joint_command(arm: str, joint_index: int, angle: float) -> bytes:
        """
        编码机械臂关节控制命令

        创建设置特定机械臂关节角度的命令消息。

        Args:
            arm: 机械臂标识符 (例如 "left", "right")
            joint_index: 关节索引 (从0开始)
            angle: 目标关节角度 (弧度)

        Returns:
            编码后的关节控制命令字节流
        """
        return RobotProtocol.encode_command(
            CommandType.SET_ARM_JOINT,
            {"arm": arm, "joint_index": joint_index, "angle": angle}  # 关节参数
        )

    @staticmethod
    def encode_camera_command(position: List[float], target: Optional[List[float]] = None) -> bytes:
        """
        编码摄像头位置命令

        创建设置摄像头位置和目标点的命令消息。

        Args:
            position: 摄像头位置坐标 [x, y, z]
            target: 可选的摄像头目标点坐标 [x, y, z]

        Returns:
            编码后的摄像头控制命令字节流
        """
        data = {"position": position}  # 必需的位置参数
        if target:
            data["target"] = target   # 可选的目标点参数
        return RobotProtocol.encode_command(CommandType.SET_CAMERA_POSITION, data)

    @staticmethod
    def encode_video_frame(frame_data: bytes, width: int, height: int,
                          quality: int = 80, camera_id: str = "main") -> bytes:
        """
        编码视频帧以便传输

        将原始图像数据编码为Base64格式，包含视频帧的元数据信息。

        Args:
            frame_data: 原始图像数据 (JPEG编码)
            width: 帧宽度
            height: 帧高度
            quality: JPEG质量
            camera_id: 摄像头标识符

        Returns:
            编码后的视频消息字节流
        """
        # 将JPEG字节数据编码为Base64字符串，便于JSON传输
        frame_b64 = base64.b64encode(frame_data).decode('utf-8')
        data = {
            "frame": frame_b64,       # Base64编码的图像数据
            "width": width,           # 图像宽度
            "height": height,         # 图像高度
            "quality": quality,       # JPEG质量
            "camera_id": camera_id,   # 摄像头ID
            "format": "jpeg"          # 图像格式
        }
        return RobotProtocol.encode_response(ResponseType.VIDEO, data)

    @staticmethod
    def decode_video_frame(message: Dict[str, Any]) -> Optional[bytes]:
        """
        从消息中解码视频帧

        将Base64编码的视频帧数据解码回原始字节数据。

        Args:
            message: 已解码的视频消息字典

        Returns:
            原始JPEG帧数据，如果无效则返回None
        """
        try:
            # 验证消息类型是否为视频响应
            if message.get("response") != ResponseType.VIDEO.value:
                return None

            data = message.get("data", {})
            frame_b64 = data.get("frame")
            if not frame_b64:
                return None

            # 将Base64字符串解码回原始字节数据
            return base64.b64decode(frame_b64)
        except Exception:
            return None

    @staticmethod
    def encode_robot_state(position: Dict[str, float], rotation: Dict[str, float],
                          arm_joints: Dict[str, List[float]], status: str = "connected") -> bytes:
        """
        编码机器人状态信息

        将机器人的位置、旋转、关节角度等状态信息编码为响应消息。

        Args:
            position: 机器人位置坐标 {x, y, z}
            rotation: 机器人旋转角度 {roll, pitch, yaw}
            arm_joints: 机械臂关节位置 {"left": [...], "right": [...]}
            status: 机器人状态字符串

        Returns:
            编码后的状态消息字节流
        """
        data = {
            "position": position,      # 位置信息
            "rotation": rotation,      # 旋转信息
            "arm_joints": arm_joints,  # 关节角度信息
            "status": status,          # 连接状态
            "timestamp": time.time()   # 状态更新时间
        }
        return RobotProtocol.encode_response(ResponseType.STATE, data)

    @staticmethod
    def is_valid_message(message: Dict[str, Any]) -> bool:
        """
        检查消息是否具有有效的结构

        验证消息是否包含必需的字段，以及消息类型是否匹配相应的字段。

        Args:
            message: 已解码的消息字典

        Returns:
            如果消息结构有效则返回True
        """
        # 检查必需的基本字段
        required_fields = ["type", "timestamp"]
        if not all(field in message for field in required_fields):
            return False

        # 根据消息类型验证相应的字段
        if message["type"] == "command":
            return "command" in message
        elif message["type"] == "response":
            return "response" in message

        return False

    @staticmethod
    def create_error_response(error_message: str) -> bytes:
        """
        创建标准错误响应

        生成包含错误描述的标准化错误响应消息。

        Args:
            error_message: 错误描述文本

        Returns:
            编码后的错误响应字节流
        """
        return RobotProtocol.encode_response(
            ResponseType.ERROR,
            {"message": error_message}  # 错误信息
        )

    @staticmethod
    def create_success_response(data: Optional[Dict[str, Any]] = None) -> bytes:
        """
        创建标准成功响应

        生成包含可选成功数据的标准化成功响应消息。

        Args:
            data: 可选的成功数据字典

        Returns:
            编码后的成功响应字节流
        """
        return RobotProtocol.encode_response(ResponseType.SUCCESS, data or {})