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
XLeRobot客户端模块

该模块实现了XLeRobot的客户端控制接口，运行在控制端，通过网络与机器人主机通信。
客户端负责：
1. 建立与主机的网络连接
2. 发送控制指令到机器人
3. 接收机器人的传感器观测数据
4. 提供键盘遥操作功能

技术说明：
- 使用ZMQ进行网络通信（TODO: 考虑迁移到gRPC）
- 支持多速度级别的键盘控制
- 实现看门狗机制确保安全性
- 支持相机图像数据的接收和解码
"""

# TODO(aliberts, Steven, Pepijn): 考虑使用gRPC替代ZMQ以提高性能和可靠性？

# 导入标准库
import base64          # 用于图像数据的base64编码/解码
import json            # 用于序列化和反序列化数据
import logging         # 用于日志记录
from functools import cached_property  # 用于缓存属性计算结果
from typing import Any, Dict, Optional, Tuple  # 类型注解

# 导入第三方库
import cv2             # OpenCV，用于图像处理
import numpy as np     # NumPy，用于数值计算
import zmq             # ZMQ，用于网络通信

# 导入自定义模块
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError  # 设备错误类
from ..robot import Robot  # 基础机器人类
from .config_xlerobot import XLerobotConfig, XLerobotClientConfig  # 配置类


class XLerobotClient(Robot):
    """
    XLeRobot客户端类

    该类实现了XLeRobot的客户端控制接口，通过网络与机器人主机进行通信。
    客户端运行在控制端，负责发送控制指令和接收传感器数据。

    主要功能：
    - 建立和维护与主机的ZMQ网络连接
    - 发送控制指令到机器人主机
    - 接收和解码机器人的观测数据（包括相机图像）
    - 提供多速度级别的键盘遥操作控制
    - 实现连接状态管理和错误处理

    Attributes:
        config_class (type): 配置类，为XLerobotClientConfig
        name (str): 客户端名称标识符
    """

    config_class = XLerobotClientConfig  # 关联的配置类
    name = "xlerobot_client"             # 客户端名称

    def __init__(self, config: XLerobotClientConfig):
        """
        初始化XLeRobot客户端

        Args:
            config (XLerobotClientConfig): 客户端配置对象，包含网络参数和其他设置
        """
        super().__init__(config)  # 调用父类初始化

        # 基本配置信息
        self.config = config
        self.id = config.id              # 客户端ID
        self.robot_type = config.type    # 机器人类型

        # 网络配置
        self.remote_ip = config.remote_ip  # 主机IP地址
        self.port_zmq_cmd = config.port_zmq_cmd  # 命令发送端口
        self.port_zmq_observations = config.port_zmq_observations  # 观测数据接收端口

        # 控制配置
        self.teleop_keys = config.teleop_keys  # 遥操作键位映射

        # 网络超时配置
        self.polling_timeout_ms = config.polling_timeout_ms  # 轮询超时时间
        self.connect_timeout_s = config.connect_timeout_s    # 连接超时时间

        # ZMQ网络通信对象
        self.zmq_context = None          # ZMQ上下文
        self.zmq_cmd_socket = None       # 命令发送套接字
        self.zmq_observation_socket = None  # 观测数据接收套接字

        # 数据缓存
        self.last_frames = {}            # 最后一帧图像数据缓存
        self.last_remote_state = {}      # 最后一次远程状态数据缓存

        # 速度控制配置
        # 定义三个速度级别：慢速、中速、快速
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # 慢速：XY速度0.1m/s，转向速度30°/s
            {"xy": 0.2, "theta": 60},  # 中速：XY速度0.2m/s，转向速度60°/s
            {"xy": 0.3, "theta": 90},  # 快速：XY速度0.3m/s，转向速度90°/s
        ]
        self.speed_index = 0  # 当前速度级别索引（0=慢速）

        # 连接状态管理
        self._is_connected = False  # 连接状态标志
        self.logs = {}              # 日志记录字典

    @cached_property
    def _state_ft(self) -> dict[str, type]:
        """
        定义机器人状态特征的数据类型映射

        该属性定义了机器人所有可控制状态特征的数据类型，包括：
        - 左臂关节位置（6个关节）
        - 右臂关节位置（6个关节）
        - 头部电机位置（2个电机）
        - 底盘速度（3个方向）

        Returns:
            dict[str, type]: 状态特征名称到数据类型的映射字典
        """
        return dict.fromkeys(
            (
                # 左臂关节位置状态
                "left_arm_shoulder_pan.pos",    # 左肩关节摆动位置
                "left_arm_shoulder_lift.pos",   # 左肩关节抬起位置
                "left_arm_elbow_flex.pos",      # 左肘关节弯曲位置
                "left_arm_wrist_flex.pos",      # 左腕关节弯曲位置
                "left_arm_wrist_roll.pos",      # 左腕关节滚动位置
                "left_arm_gripper.pos",         # 左手夹爪位置

                # 右臂关节位置状态
                "right_arm_shoulder_pan.pos",   # 右肩关节摆动位置
                "right_arm_shoulder_lift.pos",  # 右肩关节抬起位置
                "right_arm_elbow_flex.pos",     # 右肘关节弯曲位置
                "right_arm_wrist_flex.pos",     # 右腕关节弯曲位置
                "right_arm_wrist_roll.pos",     # 右腕关节滚动位置
                "right_arm_gripper.pos",        # 右手夹爪位置

                # 头部电机位置状态
                "head_motor_1.pos",             # 头部电机1位置
                "head_motor_2.pos",             # 头部电机2位置

                # 底盘速度状态
                "x.vel",                        # X方向速度（前进/后退）
                "y.vel",                        # Y方向速度（左移/右移）
                "theta.vel",                    # 旋转速度（左转/右转）
            ),
            float,  # 所有状态值都是浮点数类型
        )

    @cached_property
    def _state_order(self) -> tuple[str, ...]:
        """
        定义状态特征的顺序

        Returns:
            tuple[str, ...]: 状态特征名称的元组，按定义顺序排列
        """
        return tuple(self._state_ft.keys())

    @cached_property
    def _cameras_ft(self) -> dict[str, tuple[int, int, int]]:
        """
        定义相机图像特征的数据类型

        Returns:
            dict[str, tuple[int, int, int]]: 相机名称到图像形状的映射（高度、宽度、通道数）
        """
        return {name: (cfg.height, cfg.width, 3) for name, cfg in self.config.cameras.items()}

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """
        定义观测特征的完整集合

        观测特征包括所有状态特征和相机图像特征

        Returns:
            dict[str, type | tuple]: 完整的观测特征字典
        """
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        """
        定义动作特征的数据类型

        动作特征与状态特征一一对应

        Returns:
            dict[str, type]: 动作特征数据类型字典
        """
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        """
        获取连接状态

        Returns:
            bool: 是否已连接到主机
        """
        return self._is_connected

    @property
    def is_calibrated(self) -> bool:
        """
        获取校准状态

        Note:
            对于XLeRobot客户端，校准由主机端处理，这里返回占位符

        Returns:
            bool: 校准状态（未实现）
        """
        pass

    def connect(self) -> None:
        """
        建立与远程机器人的ZMQ网络连接

        该方法创建并配置ZMQ套接字，建立与机器人主机的双向通信通道：
        - PUSH套接字：用于发送控制指令到主机
        - PULL套接字：用于接收主机的观测数据

        Raises:
            DeviceAlreadyConnectedError: 如果已经建立连接
            DeviceNotConnectedError: 如果连接超时

        Network Architecture:
            Client (控制端)          Host (机器人端)
            PUSH  →  cmd_port     ←  PULL
            PULL  ←  obs_port     →  PUSH
        """
        # 检查是否已经连接
        if self._is_connected:
            raise DeviceAlreadyConnectedError(
                "LeKiwi Daemon is already connected. Do not run `robot.connect()` twice."
            )

        # 创建ZMQ上下文
        self.zmq_context = zmq.Context()

        # 创建并配置命令发送套接字（PUSH模式）
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PUSH)
        zmq_cmd_locator = f"tcp://{self.remote_ip}:{self.port_zmq_cmd}"
        self.zmq_cmd_socket.connect(zmq_cmd_locator)
        # CONFLATE选项确保只保留最新的消息，防止消息积压
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)

        # 创建并配置观测数据接收套接字（PULL模式）
        self.zmq_observation_socket = self.zmq_context.socket(zmq.PULL)
        zmq_observations_locator = f"tcp://{self.remote_ip}:{self.port_zmq_observations}"
        self.zmq_observation_socket.connect(zmq_observations_locator)
        # CONFLATE选项确保只接收最新的观测数据
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)

        # 等待主机发送第一条消息以确认连接成功
        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)
        socks = dict(poller.poll(self.connect_timeout_s * 1000))  # 转换为毫秒

        # 检查是否在超时时间内收到主机的消息
        if self.zmq_observation_socket not in socks or socks[self.zmq_observation_socket] != zmq.POLLIN:
            raise DeviceNotConnectedError("Timeout waiting for LeKiwi Host to connect expired.")

        # 标记连接成功
        self._is_connected = True

    def calibrate(self) -> None:
        pass

    def _poll_and_get_latest_message(self) -> Optional[str]:
        """Polls the ZMQ socket for a limited time and returns the latest message string."""
        poller = zmq.Poller()
        poller.register(self.zmq_observation_socket, zmq.POLLIN)

        try:
            socks = dict(poller.poll(self.polling_timeout_ms))
        except zmq.ZMQError as e:
            logging.error(f"ZMQ polling error: {e}")
            return None

        if self.zmq_observation_socket not in socks:
            logging.info("No new data available within timeout.")
            return None

        last_msg = None
        while True:
            try:
                msg = self.zmq_observation_socket.recv_string(zmq.NOBLOCK)
                last_msg = msg
            except zmq.Again:
                break

        if last_msg is None:
            logging.warning("Poller indicated data, but failed to retrieve message.")

        return last_msg

    def _parse_observation_json(self, obs_string: str) -> Optional[Dict[str, Any]]:
        """Parses the JSON observation string."""
        try:
            return json.loads(obs_string)
        except json.JSONDecodeError as e:
            logging.error(f"Error decoding JSON observation: {e}")
            return None

    def _decode_image_from_b64(self, image_b64: str) -> Optional[np.ndarray]:
        """Decodes a base64 encoded image string to an OpenCV image."""
        if not image_b64:
            return None
        try:
            jpg_data = base64.b64decode(image_b64)
            np_arr = np.frombuffer(jpg_data, dtype=np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                logging.warning("cv2.imdecode returned None for an image.")
            return frame
        except (TypeError, ValueError) as e:
            logging.error(f"Error decoding base64 image data: {e}")
            return None

    def _remote_state_from_obs(
        self, observation: Dict[str, Any]
    ) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        """Extracts frames, and state from the parsed observation."""

        flat_state = {key: observation.get(key, 0.0) for key in self._state_order}

        state_vec = np.array([flat_state[key] for key in self._state_order], dtype=np.float32)

        obs_dict: Dict[str, Any] = {**flat_state, "observation.state": state_vec}

        # Decode images
        current_frames: Dict[str, np.ndarray] = {}
        for cam_name, image_b64 in observation.items():
            if cam_name not in self._cameras_ft:
                continue
            frame = self._decode_image_from_b64(image_b64)
            if frame is not None:
                current_frames[cam_name] = frame

        return current_frames, obs_dict

    def _get_data(self) -> Tuple[Dict[str, np.ndarray], Dict[str, Any], Dict[str, Any]]:
        """
        Polls the video socket for the latest observation data.

        Attempts to retrieve and decode the latest message within a short timeout.
        If successful, updates and returns the new frames, speed, and arm state.
        If no new data arrives or decoding fails, returns the last known values.
        """

        # 1. Get the latest message string from the socket
        latest_message_str = self._poll_and_get_latest_message()

        # 2. If no message, return cached data
        if latest_message_str is None:
            return self.last_frames, self.last_remote_state

        # 3. Parse the JSON message
        observation = self._parse_observation_json(latest_message_str)

        # 4. If JSON parsing failed, return cached data
        if observation is None:
            return self.last_frames, self.last_remote_state

        # 5. Process the valid observation data
        try:
            new_frames, new_state = self._remote_state_from_obs(observation)
        except Exception as e:
            logging.error(f"Error processing observation data, serving last observation: {e}")
            return self.last_frames, self.last_remote_state

        self.last_frames = new_frames
        self.last_remote_state = new_state

        return new_frames, new_state

    def get_observation(self) -> dict[str, Any]:
        """
        从远程机器人获取观测数据

        该方法获取机器人的完整观测数据，包括：
        - 机械臂当前位置状态
        - 底盘速度（已转换为机体坐标系速度：x, y, theta）
        - 相机图像帧

        通过ZMQ接收数据，并转换为适合处理的格式

        Returns:
            dict[str, Any]: 包含所有观测数据的字典

        Raises:
            DeviceNotConnectedError: 如果客户端未连接到主机

        Data Flow:
            Host (机器人端)  →  ZMQ  →  Client (控制端)
            传感器数据编码  →  网络传输  →  数据解码处理
        """
        # 检查连接状态
        if not self._is_connected:
            raise DeviceNotConnectedError("LeKiwiClient is not connected. You need to run `robot.connect()`.")

        # 获取最新的观测数据
        frames, obs_dict = self._get_data()

        # 处理每个配置的相机数据
        for cam_name, frame in frames.items():
            if frame is None:
                # 如果图像数据无效，使用黑色图像占位
                logging.warning("Frame is None")
                frame = np.zeros((640, 480, 3), dtype=np.uint8)  # 创建黑色640x480图像
            obs_dict[cam_name] = frame  # 将图像添加到观测字典中

        return obs_dict

    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        """
        将键盘输入转换为底盘运动控制指令

        该方法处理用户的键盘输入，转换为机器人的底盘运动指令。
        支持三级速度调节和多方向移动控制。

        Args:
            pressed_keys (np.ndarray): 当前按下的按键列表

        Returns:
            dict: 包含底盘速度控制指令的字典

        Control Mapping:
            i/k: 前进/后退 (+x/-x方向)
            j/l: 左移/右移 (+y/-y方向)
            u/o: 左转/右转 (+z/-z方向)
            n/m: 加速/减速
        """
        # 速度控制逻辑
        if self.teleop_keys["speed_up"] in pressed_keys:
            # 提高速度级别（最大到2级）
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            # 降低速度级别（最小到0级）
            self.speed_index = max(self.speed_index - 1, 0)

        # 获取当前速度设置
        speed_setting = self.speed_levels[self.speed_index]
        xy_speed = speed_setting["xy"]      # XY平面移动速度（米/秒）
        theta_speed = speed_setting["theta"]  # 旋转速度（度/秒）

        # 初始化控制指令
        x_cmd = 0.0    # X方向速度（前进/后退，米/秒）
        y_cmd = 0.0    # Y方向速度（左移/右移，米/秒）
        theta_cmd = 0.0  # 旋转速度（左转/右转，度/秒）

        # 处理移动控制键
        if self.teleop_keys["forward"] in pressed_keys:
            x_cmd += xy_speed      # 前进
        if self.teleop_keys["backward"] in pressed_keys:
            x_cmd -= xy_speed      # 后退
        if self.teleop_keys["left"] in pressed_keys:
            y_cmd += xy_speed      # 左移
        if self.teleop_keys["right"] in pressed_keys:
            y_cmd -= xy_speed      # 右移
        if self.teleop_keys["rotate_left"] in pressed_keys:
            theta_cmd += theta_speed  # 左转
        if self.teleop_keys["rotate_right"] in pressed_keys:
            theta_cmd -= theta_speed  # 右转

        return {
            # 头部电机暂不由键盘控制
            # "head_motor_1.pos": 0.0,  # TODO: 实现头部控制
            # "head_motor_2.pos": 0.0,

            # 底盘速度控制指令
            "x.vel": x_cmd,      # X方向速度指令
            "y.vel": y_cmd,      # Y方向速度指令
            "theta.vel": theta_cmd,  # 旋转速度指令
        }

    def configure(self):
        pass

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """
        发送控制指令到机器人

        该方法将控制指令发送到远程机器人主机。指令被转换为电机空间坐标，
        并通过ZMQ网络发送给主机执行。

        Args:
            action (dict[str, Any]): 包含目标电机位置的字典。

        Raises:
            DeviceNotConnectedError: 如果机器人未连接。

        Returns:
            dict[str, Any]: 实际发送到电机的动作指令，可能经过裁剪处理。

        Command Flow:
            Client (控制端)  →  ZMQ (JSON)  →  Host (机器人端)
            动作指令字典    →  序列化发送  →  电机控制执行
        """
        # 检查连接状态
        if not self._is_connected:
            raise DeviceNotConnectedError(
                "ManipulatorRobot is not connected. You need to run `robot.connect()`."
            )

        # 将动作指令序列化为JSON格式并发送到主机
        # 注意：动作指令应该是电机空间坐标
        self.zmq_cmd_socket.send_string(json.dumps(action))

        # TODO(Steven): 当可以记录非numpy数组值时，移除这个numpy转换
        # 将动作指令转换为numpy数组格式以便记录和处理
        actions = np.array(
            [action.get(k, 0.0) for k in self._state_order],  # 按预定义顺序获取动作值
            dtype=np.float32
        )

        # 构建返回的动作指令字典
        action_sent = {key: actions[i] for i, key in enumerate(self._state_order)}
        action_sent["action"] = actions  # 添加完整的动作数组
        return action_sent

    def disconnect(self):
        """
        断开与机器人的网络连接

        该方法安全地断开与机器人主机的网络连接，释放所有网络资源。

        执行步骤：
        1. 检查连接状态
        2. 关闭观测数据接收套接字
        3. 关闭命令发送套接字
        4. 终止ZMQ上下文
        5. 更新连接状态

        Raises:
            DeviceNotConnectedError: 如果机器人未连接

        Resource Cleanup:
            - ZMQ sockets
            - ZMQ context
            - Connection state
        """
        # 检查连接状态
        if not self._is_connected:
            raise DeviceNotConnectedError(
                "LeKiwi is not connected. You need to run `robot.connect()` before disconnecting."
            )

        # 关闭ZMQ套接字和上下文
        self.zmq_observation_socket.close()  # 关闭观测数据套接字
        self.zmq_cmd_socket.close()         # 关闭命令套接字
        self.zmq_context.term()             # 终止ZMQ上下文

        # 更新连接状态
        self._is_connected = False
