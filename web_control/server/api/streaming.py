# 导入异步编程支持库
import asyncio
# 导入Base64编码库，用于图像数据编码
import base64
# 导入类型检查相关模块
from typing import TYPE_CHECKING, Optional

# 导入数值计算库，用于生成测试图像
import numpy as np

# 类型检查导入：避免循环导入问题
if TYPE_CHECKING:
    from core.remote_core import RemoteCore


class VideoStreamManager:
    """
    视频流管理器

    负责管理和控制视频流的传输，包括：
    - 启动和停止视频流
    - 管理多个客户端的视频流任务
    - 生成测试图像帧
    - 与远程控制核心集成获取实时视频数据
    """

    def __init__(self) -> None:
        """初始化视频流管理器"""
        # 视频流状态标志
        self.streaming = False
        # 视频帧率（每秒帧数）
        self.frame_rate = 30
        # 帧间隔时间（秒）
        self.frame_interval = 1.0 / self.frame_rate
        # 存储所有客户端的视频流任务，键为客户端ID，值为异步任务
        self.stream_tasks: dict[str, asyncio.Task] = {}
        # 远程控制核心引用，用于获取实时视频数据
        self.remote_core: Optional['RemoteCore'] = None

    def attach_remote_core(self, remote_core: 'RemoteCore') -> None:
        """
        附加远程控制核心实例

        Args:
            remote_core: 远程控制核心实例，用于获取机器人实时视频数据
        """
        self.remote_core = remote_core

    async def start_stream(self) -> dict:
        """
        启动视频流

        Returns:
            包含启动状态的字典
        """
        # 如果视频流尚未启动，则启动它
        if not self.streaming:
            self.streaming = True
            print("Video stream started")  # 输出启动信息
        return {'status': 'streaming_started'}

    async def stop_stream(self, sid: Optional[str] = None) -> dict:
        """
        停止视频流

        Args:
            sid: 可选的客户端会话ID。如果提供，只停止指定客户端的视频流；如果为None，停止所有视频流

        Returns:
            包含停止状态的字典
        """
        # 设置视频流状态为停止
        self.streaming = False

        # 如果指定了客户端ID，只停止该客户端的视频流
        if sid and sid in self.stream_tasks:
            task = self.stream_tasks.pop(sid)  # 移除并获取任务
            if not task.cancelled():
                task.cancel()  # 取消异步任务
                print(f"Cancelled video stream task for client {sid}")  # 输出取消信息
        # 如果没有指定客户端ID，停止所有客户端的视频流
        elif sid is None:
            for client_sid, task in list(self.stream_tasks.items()):
                if not task.cancelled():
                    task.cancel()  # 取消每个异步任务
                    print(f"Cancelled video stream task for client {client_sid}")  # 输出取消信息
            self.stream_tasks.clear()  # 清空所有任务

        print("Video stream stopped")  # 输出停止信息
        return {'status': 'streaming_stopped'}

    def _generate_fallback_frame(self) -> Optional[str]:
        """
        生成备用测试图像帧

        当无法从机器人获取实时视频时，生成随机彩色测试图像

        Returns:
            Base64编码的JPEG图像字符串，如果生成失败则返回None
        """
        try:
            import cv2  # 尝试导入OpenCV库
        except ImportError:
            return None  # 如果OpenCV不可用，返回None

        # 生成随机彩色测试图像 (480x640像素，3通道RGB)
        test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # 将图像编码为JPEG格式，设置质量为80%
        success, buffer = cv2.imencode('.jpg', test_frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not success:
            return None  # 编码失败则返回None

        # 将JPEG数据编码为Base64字符串以便传输
        return base64.b64encode(buffer).decode('utf-8')

    async def get_frame(self) -> Optional[dict]:
        """
        获取视频帧数据

        优先从远程机器人获取实时视频帧，如果不可用则使用测试图像

        Returns:
            包含视频帧数据的字典，包括帧数据、尺寸、时间戳等信息
        """
        frame_base64 = None  # 存储Base64编码的帧数据
        source = 'test'      # 数据源标识

        # 尝试从远程机器人核心获取实时视频帧
        if self.remote_core and self.remote_core.connected:
            try:
                frame_base64 = await self.remote_core.get_camera_frame_base64()
                if frame_base64:
                    # 如果成功获取帧，设置数据源为机器人类型
                    source = self.remote_core.config.robot_type.lower()
            except Exception as exc:
                print(f"Error getting frame from remote core: {exc}")  # 输出错误信息

        # 如果无法获取实时帧，使用备用测试帧
        if not frame_base64:
            frame_base64 = self._generate_fallback_frame()
            if frame_base64:
                source = 'test_jpeg'  # 标记为测试JPEG图像

        # 如果仍然无法获取帧数据，返回None
        if not frame_base64:
            return None

        # 获取当前事件循环的时间戳
        loop = asyncio.get_running_loop()
        return {
            'frame': frame_base64,      # Base64编码的图像数据
            'width': 640,              # 图像宽度
            'height': 480,             # 图像高度
            'channels': 3,             # 颜色通道数（RGB）
            'timestamp': loop.time(),  # 时间戳
            'source': source           # 数据源标识
        }

    async def stream_frames(self, socket_io, sid: str) -> None:
        """
        为指定客户端流式传输视频帧

        Args:
            socket_io: Socket.IO服务器实例
            sid: 客户端会话ID
        """
        print(f"Starting video stream for client {sid}")  # 输出开始信息
        try:
            # 持续发送视频帧，直到流被停止
            while self.streaming:
                # 检查当前任务是否已被取消
                if asyncio.current_task().cancelled():
                    break

                # 获取当前视频帧
                frame_data = await self.get_frame()
                if frame_data:
                    # 通过WebSocket向指定客户端发送视频帧
                    await socket_io.emit('video_frame', frame_data, to=sid)

                # 等待下一帧的时间间隔，控制帧率
                await asyncio.sleep(self.frame_interval)
        except asyncio.CancelledError:
            # 任务被正常取消，不做任何处理
            pass
        except Exception as exc:
            # 发生异常时输出错误信息
            print(f"Stream error for client {sid}: {exc}")
        finally:
            # 清理任务记录
            self.stream_tasks.pop(sid, None)
            print(f"Video stream ended for client {sid}")  # 输出结束信息


# 创建全局视频管理器实例
video_manager = VideoStreamManager()
