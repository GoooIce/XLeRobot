"""
统一远程控制核心

该模块用一个单一、简化的远程控制接口替换了所有独立的控制器
(ManiSkillController, MuJoCoController, RealRobotController)，通过ZeroMQ
与机器人主机进行通信。

设计特点：
1. 统一的接口：支持所有机器人类型
2. 异步通信：使用ZeroMQ进行高性能消息传递
3. 自动重连：内置连接管理和健康检查
4. 状态缓存：维护机器人状态的本地副本
5. 错误处理：完善的异常处理和日志记录
"""

# 导入异步编程支持
import asyncio
# 导入日志模块
import logging
# 导入时间模块
import time
# 导入类型提示
from typing import Any, Dict, List, Optional

# 导入ZeroMQ消息队列库
import zmq
import zmq.asyncio

# 导入本地模块
from .config import ServerConfig
from .protocol import RobotProtocol, CommandType, ResponseType


class RemoteCore:
    """
    所有机器人类型的统一远程控制核心

    该类提供了与机器人主机通信的统一接口，支持：
    - 命令发送和响应接收
    - 连接状态管理
    - 自动心跳检测
    - 机器人状态缓存
    - 视频数据获取
    """

    def __init__(self, config: ServerConfig):
        """
        初始化远程控制核心

        设置ZeroMQ上下文、套接字、状态缓存和日志记录器。

        Args:
            config: 包含机器人和连接设置的服务器配置
        """
        self.config = config
        self.config.validate()  # 验证配置参数

        # ==================== ZeroMQ设置 ====================
        # 创建异步ZeroMQ上下文
        self.context = zmq.asyncio.Context()
        # 命令发送套接字 (PUSH模式)
        self.cmd_socket: Optional[zmq.asyncio.Socket] = None
        # 数据接收套接字 (PULL模式)
        self.data_socket: Optional[zmq.asyncio.Socket] = None

        # ==================== 连接状态管理 ====================
        self.connected = False           # 连接状态标志
        self.last_ping_time = 0          # 上次ping时间
        self.ping_interval = 5.0         # ping间隔时间（秒）

        # ==================== 机器人状态缓存 ====================
        # 存储机器人的当前状态信息
        self.robot_state = {
            'status': 'disconnected',    # 连接状态
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},        # 位置坐标
            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}, # 旋转角度
            'arm_joints': {'left': [0.0] * 6, 'right': [0.0] * 6}, # 机械臂关节角度
            'base_joints': [0.0, 0.0, 0.0],  # 底座关节 [x, y, 旋转]
            'velocity': {               # 速度信息
                'linear': {'x': 0.0, 'y': 0.0, 'z': 0.0},    # 线速度
                'angular': {'x': 0.0, 'y': 0.0, 'z': 0.0}    # 角速度
            }
        }

        # ==================== 日志设置 ====================
        # 创建专用的日志记录器
        self.logger = logging.getLogger(f"RemoteCore-{config.robot_type}")
        self.logger.setLevel(logging.INFO)

        # 输出初始化信息
        print(f"RemoteCore initialized for {config.robot_type}")
        print(f"Will connect to: {config.get_robot_cmd_address()}")

    async def connect(self) -> bool:
        """
        连接到机器人主机

        创建ZeroMQ套接字，连接到机器人主机，并测试连接的可用性。
        如果连接成功，启动后台任务用于数据接收和心跳检测。

        Returns:
            如果连接成功返回True，否则返回False
        """
        try:
            self.logger.info(f"Connecting to {self.config.robot_type} host...")

            # ==================== 创建ZeroMQ套接字 ====================
            # 创建命令发送套接字 (PUSH模式，用于单向发送命令)
            self.cmd_socket = self.context.socket(zmq.PUSH)
            # 创建数据接收套接字 (PULL模式，用于单向接收数据)
            self.data_socket = self.context.socket(zmq.PULL)

            # ==================== 设置套接字选项 ====================
            # 设置LINGER选项，控制关闭时的行为
            self.cmd_socket.setsockopt(zmq.LINGER, 1000)
            self.data_socket.setsockopt(zmq.LINGER, 1000)
            # 设置接收超时时间，防止无限期等待
            self.data_socket.setsockopt(zmq.RCVTIMEO, self.config.polling_timeout_ms)

            # ==================== 连接到机器人主机 ====================
            # 获取命令和数据通道的完整地址
            cmd_addr = self.config.get_robot_cmd_address()
            data_addr = self.config.get_robot_data_address()

            # 建立连接
            self.cmd_socket.connect(cmd_addr)
            self.data_socket.connect(data_addr)

            self.logger.info(f"ZeroMQ sockets connected")

            # ==================== 测试连接可用性 ====================
            # 使用ping命令测试连接是否正常工作
            ping_success = await self._ping_robot()
            if ping_success:
                self.connected = True
                self.robot_state['status'] = 'connected'
                self.logger.info(f"Successfully connected to {self.config.robot_type} host")

                # 启动后台任务
                asyncio.create_task(self._data_receiver_loop())  # 数据接收循环
                asyncio.create_task(self._ping_loop())            # 心跳检测循环

                return True
            else:
                self.logger.error("Failed to ping robot host")
                await self.disconnect()
                return False

        except Exception as e:
            self.logger.error(f"Connection failed: {e}")
            await self.disconnect()
            return False

    async def disconnect(self) -> bool:
        """
        断开与机器人主机的连接

        关闭ZeroMQ套接字，更新连接状态，并清理资源。

        Returns:
            如果断开连接成功返回True
        """
        try:
            # 更新连接状态
            self.connected = False
            self.robot_state['status'] = 'disconnected'

            # 关闭命令套接字
            if self.cmd_socket:
                self.cmd_socket.close()
                self.cmd_socket = None

            # 关闭数据套接字
            if self.data_socket:
                self.data_socket.close()
                self.data_socket = None

            self.logger.info("Disconnected from robot host")
            return True

        except Exception as e:
            self.logger.error(f"Disconnection error: {e}")
            return False

    async def move(self, direction: str, speed: float = 1.0) -> Dict[str, Any]:
        """
        发送移动命令给机器人

        编码并发送移动命令，包含移动方向和速度参数。

        Args:
            direction: 移动方向 (forward, backward, left, right等)
            speed: 移动速度 (0.0到1.0之间)

        Returns:
            包含状态和详细信息的响应字典
        """
        # 检查连接状态
        if not self.connected:
            return {'status': 'error', 'message': 'Not connected to robot host'}

        try:
            # ==================== 输入验证 ====================
            # 将速度限制在有效范围内 [0, 1]
            speed = max(0.0, min(1.0, speed))

            # ==================== 发送移动命令 ====================
            # 使用协议编码移动命令
            cmd_data = RobotProtocol.encode_move_command(direction, speed)
            await self.cmd_socket.send(cmd_data)

            self.logger.debug(f"Sent move command: {direction} @ {speed}")

            # 返回成功响应
            return {
                'status': 'success',
                'direction': direction,
                'speed': speed,
                'robot_type': self.config.robot_type
            }

        except Exception as e:
            self.logger.error(f"Move command failed: {e}")
            return {'status': 'error', 'message': str(e)}

    async def stop(self) -> Dict[str, Any]:
        """
        停止机器人移动

        发送停止命令，实际上是将移动速度设为0。

        Returns:
            包含状态的响应字典
        """
        return await self.move('stop', 0.0)

    async def get_state(self) -> Dict[str, Any]:
        """
        获取当前机器人状态

        返回缓存的机器人状态信息，包括位置、旋转、关节角度等。

        Returns:
            当前机器人状态字典
        """
        state = self.robot_state.copy()
        state['robot_type'] = self.config.robot_type
        state['connected'] = self.connected
        state['timestamp'] = time.time()
        return state

    async def set_arm_joint(self, arm: str, joint_index: int, angle: float) -> Dict[str, Any]:
        """Set arm joint angle.

        Args:
            arm: Arm identifier ("left" or "right")
            joint_index: Joint index (0-based)
            angle: Target angle in radians

        Returns:
            Response dictionary with status
        """
        if not self.connected:
            return {'status': 'error', 'message': 'Not connected to robot host'}

        try:
            cmd_data = RobotProtocol.encode_arm_joint_command(arm, joint_index, angle)
            await self.cmd_socket.send(cmd_data)

            self.logger.debug(f"Sent arm joint command: {arm}[{joint_index}] = {angle}")

            return {
                'status': 'success',
                'arm': arm,
                'joint_index': joint_index,
                'angle': angle
            }

        except Exception as e:
            self.logger.error(f"Arm joint command failed: {e}")
            return {'status': 'error', 'message': str(e)}

    async def reset(self) -> Dict[str, Any]:
        """Reset robot to initial state.

        Returns:
            Response dictionary with status
        """
        if not self.connected:
            return {'status': 'error', 'message': 'Not connected to robot host'}

        try:
            cmd_data = RobotProtocol.encode_command(CommandType.RESET)
            await self.cmd_socket.send(cmd_data)

            self.logger.info("Sent reset command")

            return {'status': 'success', 'message': 'Reset command sent'}

        except Exception as e:
            self.logger.error(f"Reset command failed: {e}")
            return {'status': 'error', 'message': str(e)}

    async def get_camera_frame(self) -> Optional[bytes]:
        """Get latest camera frame bytes.

        Returns:
            Raw JPEG bytes or None if unavailable
        """
        return getattr(self, '_last_frame_bytes', None)

    async def get_camera_frame_base64(self) -> Optional[str]:
        """Get latest camera frame as base64 string.

        Returns:
            Base64 encoded JPEG frame or None if unavailable
        """
        return getattr(self, '_last_frame_b64', None)

    async def set_camera_position(self, position: List[float], target: Optional[List[float]] = None) -> Dict[str, Any]:
        """Set camera position and target.

        Args:
            position: Camera position [x, y, z]
            target: Optional camera target [x, y, z]

        Returns:
            Response dictionary with status
        """
        if not self.connected:
            return {'status': 'error', 'message': 'Not connected to robot host'}

        try:
            cmd_data = RobotProtocol.encode_camera_command(position, target)
            await self.cmd_socket.send(cmd_data)

            self.logger.debug(f"Sent camera command: pos={position}, target={target}")

            return {
                'status': 'success',
                'position': position,
                'target': target
            }

        except Exception as e:
            self.logger.error(f"Camera command failed: {e}")
            return {'status': 'error', 'message': str(e)}

    async def reset_camera(self) -> Dict[str, Any]:
        """Reset camera to default position.

        Returns:
            Response dictionary with status
        """
        if not self.connected:
            return {'status': 'error', 'message': 'Not connected to robot host'}

        try:
            cmd_data = RobotProtocol.encode_command(CommandType.RESET_CAMERA)
            await self.cmd_socket.send(cmd_data)

            self.logger.debug("Sent camera reset command")

            return {'status': 'success', 'message': 'Camera reset command sent'}

        except Exception as e:
            self.logger.error(f"Camera reset failed: {e}")
            return {'status': 'error', 'message': str(e)}

    def get_capabilities(self) -> Dict[str, bool]:
        """Get robot capabilities.

        Returns:
            Dictionary of capability flags
        """
        return {
            'movement': True,
            'arm_control': True,
            'camera_control': True,
            'video_streaming': True,
            'state_feedback': True,
            'remote_control': True,
            'robot_type': self.config.robot_type,
            'connected': self.connected
        }

    async def _ping_robot(self) -> bool:
        """Ping the robot host to test connectivity.

        Returns:
            True if ping successful
        """
        try:
            cmd_data = RobotProtocol.encode_command(CommandType.PING)
            await self.cmd_socket.send(cmd_data)

            # Wait for pong response (with timeout)
            start_time = time.time()
            timeout = self.config.connect_timeout_s

            while time.time() - start_time < timeout:
                try:
                    # Check for response
                    data = await asyncio.wait_for(
                        self.data_socket.recv(),
                        timeout=0.1
                    )
                    response = RobotProtocol.decode_response(data)

                    if response.get('response') == ResponseType.PONG.value:
                        return True

                except asyncio.TimeoutError:
                    continue
                except Exception:
                    break

            return False

        except Exception as e:
            self.logger.error(f"Ping failed: {e}")
            return False

    async def _ping_loop(self):
        """Background task to periodically ping the robot host."""
        while self.connected:
            try:
                current_time = time.time()
                if current_time - self.last_ping_time >= self.ping_interval:
                    ping_success = await self._ping_robot()
                    if not ping_success:
                        self.logger.warning("Ping failed - connection may be lost")
                        # Could implement reconnection logic here
                    self.last_ping_time = current_time

                await asyncio.sleep(1.0)  # Check every second

            except Exception as e:
                self.logger.error(f"Ping loop error: {e}")
                break

    async def _data_receiver_loop(self):
        """Background task to receive data from robot host."""
        while self.connected:
            try:
                # Receive data with timeout
                data = await asyncio.wait_for(
                    self.data_socket.recv(),
                    timeout=self.config.polling_timeout_ms / 1000.0
                )

                # Decode response
                response = RobotProtocol.decode_response(data)

                # Process different response types
                response_type = response.get('response')

                if response_type == ResponseType.STATE.value:
                    self._update_robot_state(response.get('data', {}))

                elif response_type == ResponseType.VIDEO.value:
                    self._update_video_frame(response.get('data', {}))

                elif response_type == ResponseType.ERROR.value:
                    self.logger.warning(f"Robot host error: {response.get('data', {}).get('message', 'Unknown error')}")

            except asyncio.TimeoutError:
                # Normal timeout - continue loop
                continue

            except Exception as e:
                self.logger.error(f"Data receiver error: {e}")
                # Brief pause before retrying
                await asyncio.sleep(0.1)

    def _update_robot_state(self, state_data: Dict[str, Any]):
        """Update cached robot state from received data."""
        try:
            # Update state fields that are present in the received data
            for key, value in state_data.items():
                if key in self.robot_state:
                    self.robot_state[key] = value

            self.robot_state['status'] = 'connected'

        except Exception as e:
            self.logger.error(f"Failed to update robot state: {e}")

    def _update_video_frame(self, video_data: Dict[str, Any]):
        """Update cached video frame from received data."""
        try:
            # Cache base64 frame
            frame_b64 = video_data.get('frame')
            if frame_b64:
                self._last_frame_b64 = frame_b64

                frame_bytes = RobotProtocol.decode_video_frame({'response': ResponseType.VIDEO.value, 'data': video_data})
                if frame_bytes:
                    self._last_frame_bytes = frame_bytes

        except Exception as e:
            self.logger.error(f"Failed to update video frame: {e}")

    async def __aenter__(self):
        """Async context manager entry."""
        await self.connect()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.disconnect()
