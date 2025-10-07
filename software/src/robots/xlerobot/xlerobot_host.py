#!/usr/bin/env python

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
XLeRobot主机模块

该模块实现了XLeRobot的主机控制接口，运行在机器人端。
主机负责：
1. 接收客户端的控制指令
2. 直接控制机器人硬件执行动作
3. 采集传感器数据并编码发送给客户端
4. 实现看门狗安全机制

运行环境：
- 运行在机器人本体或连接到机器人的计算机上
- 需要直接访问机器人硬件接口
- 提供网络服务供远程客户端连接

架构说明：
- Client (控制端)  →  ZMQ  →  Host (机器人端)  →  Hardware (机器人硬件)
- 控制指令发送    网络传输   指令接收执行     电机驱动
- 传感器数据采集  ←  网络传输  ←  数据编码发送  ←  传感器读取
"""

# 导入标准库
import base64    # 用于图像数据的base64编码
import json      # 用于数据序列化
import logging   # 用于日志记录
import time      # 用于时间相关操作

# 导入第三方库
import cv2       # OpenCV，用于图像处理
import zmq       # ZMQ，用于网络通信

# 导入自定义模块
from .xlerobot import XLerobot                              # 核心机器人控制类
from .config_xlerobot import XLerobotConfig, XLerobotHostConfig  # 配置类


class XLerobotHost:
    """
    XLeRobot主机类

    该类实现了XLeRobot的主机服务，运行在机器人端。
    主机负责与客户端进行网络通信，接收控制指令并发送传感器数据。

    主要功能：
    - 建立ZMQ网络服务器，监听客户端连接
    - 接收客户端发送的控制指令
    - 控制机器人执行相应动作
    - 采集传感器数据并编码后发送给客户端
    - 实现看门狗安全机制

    网络架构：
    Client (控制端)      Host (机器人端)
    PUSH  →  cmd_port     ←  PULL
    PULL  ←  obs_port     →  PUSH
    """

    def __init__(self, config: XLerobotHostConfig):
        """
        初始化XLeRobot主机

        Args:
            config (XLerobotHostConfig): 主机配置对象
        """
        # 创建ZMQ上下文
        self.zmq_context = zmq.Context()

        # 创建并配置命令接收套接字（PULL模式）
        # 客户端使用PUSH发送命令到这里
        self.zmq_cmd_socket = self.zmq_context.socket(zmq.PULL)
        self.zmq_cmd_socket.setsockopt(zmq.CONFLATE, 1)  # 只保留最新命令
        # 绑定到所有网络接口，监听指定端口
        self.zmq_cmd_socket.bind(f"tcp://*:{config.port_zmq_cmd}")

        # 创建并配置观测数据发送套接字（PUSH模式）
        # 向连接的客户端PUSH观测数据
        self.zmq_observation_socket = self.zmq_context.socket(zmq.PUSH)
        self.zmq_observation_socket.setsockopt(zmq.CONFLATE, 1)  # 只发送最新数据
        # 绑定到所有网络接口，监听指定端口
        self.zmq_observation_socket.bind(f"tcp://*:{config.port_zmq_observations}")

        # 保存运行配置参数
        self.connection_time_s = config.connection_time_s          # 连接持续时间
        self.watchdog_timeout_ms = config.watchdog_timeout_ms      # 看门狗超时时间
        self.max_loop_freq_hz = config.max_loop_freq_hz            # 最大循环频率

    def disconnect(self):
        """
        断开网络连接，清理资源

        该方法安全地关闭所有ZMQ套接字和上下文，释放网络资源。
        """
        self.zmq_observation_socket.close()  # 关闭观测数据发送套接字
        self.zmq_cmd_socket.close()         # 关闭命令接收套接字
        self.zmq_context.term()             # 终止ZMQ上下文


def main():
    """
    XLeRobot主函数

    该函数启动XLeRobot主机服务，执行以下操作：
    1. 初始化机器人硬件连接
    2. 启动网络服务监听客户端连接
    3. 进入主控制循环，处理命令和发送观测数据
    4. 实现看门狗安全机制
    5. 优雅地处理程序退出

    主循环流程：
    - 接收客户端控制指令
    - 执行机器人控制动作
    - 采集传感器数据
    - 编码并发送观测数据给客户端
    - 执行看门狗检查
    - 控制循环频率
    """
    # 第一步：配置并连接机器人硬件
    logging.info("Configuring Xlerobot")
    robot_config = XLerobotConfig(id="my_xlerobot_pc")  # 创建机器人配置
    robot = XLerobot(robot_config)                      # 初始化机器人控制对象

    logging.info("Connecting Xlerobot")
    robot.connect()  # 连接机器人硬件

    # 第二步：启动网络服务
    logging.info("Starting HostAgent")
    host_config = XLerobotHostConfig()  # 创建主机配置
    host = XLerobotHost(host_config)    # 初始化主机服务

    # 初始化看门狗相关变量
    last_cmd_time = time.time()    # 记录最后一次收到命令的时间
    watchdog_active = False        # 看门狗是否已激活

    logging.info("Waiting for commands...")
    try:
        # 主控制循环
        start = time.perf_counter()  # 记录程序开始时间
        duration = 0

        # 循环运行直到达到配置的时间限制
        while duration < host.connection_time_s:
            loop_start_time = time.time()  # 记录循环开始时间

            # 第一阶段：接收并处理客户端控制指令
            try:
                # 尝试接收客户端发送的控制命令（非阻塞模式）
                msg = host.zmq_cmd_socket.recv_string(zmq.NOBLOCK)
                data = dict(json.loads(msg))  # 解析JSON格式的控制指令

                # 执行机器人控制动作
                _action_sent = robot.send_action(data)

                # 更新看门狗状态（收到新命令）
                last_cmd_time = time.time()
                watchdog_active = False

            except zmq.Again:
                # 没有收到命令，但第一次需要记录日志
                if not watchdog_active:
                    logging.warning("No command available")
            except Exception as e:
                # 其他异常处理
                logging.error("Message fetching failed: %s", e)

            # 第二阶段：执行看门狗安全检查
            now = time.time()
            # 检查是否超过看门狗超时时间
            if (now - last_cmd_time > host.watchdog_timeout_ms / 1000) and not watchdog_active:
                logging.warning(
                    f"Command not received for more than {host.watchdog_timeout_ms} milliseconds. Stopping the base."
                )
                watchdog_active = True
                robot.stop_base()  # 停止机器人底盘运动，确保安全

            # 第三阶段：采集传感器数据
            last_observation = robot.get_observation()

            # 第四阶段：编码图像数据为base64格式以便网络传输
            for cam_key, _ in robot.cameras.items():
                # 将OpenCV图像编码为JPEG格式，然后转换为base64字符串
                ret, buffer = cv2.imencode(
                    ".jpg", last_observation[cam_key], [int(cv2.IMWRITE_JPEG_QUALITY), 90]
                )
                if ret:
                    # 编码成功，转换为base64字符串
                    last_observation[cam_key] = base64.b64encode(buffer).decode("utf-8")
                else:
                    # 编码失败，使用空字符串
                    last_observation[cam_key] = ""

            # 第五阶段：发送观测数据给客户端
            try:
                # 将观测数据序列化为JSON格式并发送（非阻塞模式）
                host.zmq_observation_socket.send_string(json.dumps(last_observation), flags=zmq.NOBLOCK)
            except zmq.Again:
                # 没有客户端连接，丢弃观测数据
                logging.info("Dropping observation, no client connected")

            # 第六阶段：控制循环频率
            elapsed = time.time() - loop_start_time
            # 计算需要休眠的时间以维持设定的循环频率
            time.sleep(max(1 / host.max_loop_freq_hz - elapsed, 0))

            # 更新运行时长
            duration = time.perf_counter() - start

        print("Cycle time reached.")  # 达到配置的运行时间限制

    except KeyboardInterrupt:
        # 用户通过Ctrl+C中断程序
        print("Keyboard interrupt received. Exiting...")
    finally:
        # 清理资源，优雅退出
        print("Shutting down Lekiwi Host.")
        robot.disconnect()  # 断开机器人硬件连接
        host.disconnect()   # 关闭网络服务

    logging.info("Finished LeKiwi cleanly")


if __name__ == "__main__":
    main()
