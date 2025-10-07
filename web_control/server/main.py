# 导入时间模块，用于性能监控和节流控制
import time
# 导入异步编程支持库
import asyncio

# 导入Socket.IO服务器，用于WebSocket通信
import socketio
# 导入ASGI服务器，用于运行FastAPI应用
import uvicorn
# 导入FastAPI框架，用于构建REST API
from fastapi import FastAPI
# 导入CORS中间件，允许跨域请求
from fastapi.middleware.cors import CORSMiddleware

# 导入本地模块
from api.streaming import video_manager  # 视频流管理器
from core.config import ServerConfig     # 服务器配置
from core.remote_core import RemoteCore  # 远程控制核心

# ==================== 配置加载和验证 ====================
# 从环境变量文件加载简化配置
config = ServerConfig.from_env('.env')
config.validate()  # 验证配置参数

# 输出服务器配置信息
print(f"Server Configuration:")
print(config)

# ==================== 核心组件初始化 ====================
# 创建统一的远程控制核心
remote_core = RemoteCore(config)
# 将远程核心附加到视频管理器，用于获取实时视频数据
video_manager.attach_remote_core(remote_core)


def _init_client_state() -> dict:
    """
    初始化客户端状态

    为新连接的客户端创建初始状态记录，用于速率限制和节流控制。

    Returns:
        包含客户端初始状态的字典
    """
    now = time.time()
    return {
        'last_command_time': 0.0,     # 上次命令时间
        'command_count': 0,           # 当前窗口内的命令计数
        'window_start': now,          # 速率限制窗口开始时间
        'throttle_violations': 0,     # 节流违规次数
        'last_throttle_time': 0.0     # 上次节流时间
    }


# 全局客户端状态字典，存储所有连接客户端的状态信息
client_states: dict[str, dict] = {}

# ==================== 速率限制配置 ====================
# 防止客户端发送过多请求的保护机制
RATE_LIMIT_CONFIG = {
    'max_commands_per_second': 20,    # 每秒最大命令数
    'min_command_interval': 0.05,     # 最小命令间隔（秒）
    'rate_limit_window': 1.0,         # 速率限制窗口时间（秒）
    'throttle_penalty_duration': 2.0, # 节流惩罚持续时间（秒）
    'max_throttle_violations': 3,     # 最大节流违规次数
}

async def startup_event():
    """
    应用启动事件处理函数

    在FastAPI应用启动时自动调用，初始化远程控制核心的连接。
    """
    print("Initializing remote control core...")
    success = await remote_core.connect()
    if success:
        print(f"Remote core connected to {config.robot_type} host successfully")
    else:
        print("Remote core connection failed")

async def shutdown_event():
    """
    应用关闭事件处理函数

    在FastAPI应用关闭时自动调用，清理远程控制核心的资源。
    """
    # 在关闭时清理资源
    print("Shutting down remote control core...")
    await remote_core.disconnect()
    print("Remote core shut down")

# ==================== Socket.IO服务器设置 ====================
# 创建异步Socket.IO服务器，用于实时WebSocket通信
sio = socketio.AsyncServer(
    async_mode='asgi',           # 使用ASGI模式
    cors_allowed_origins='*',    # 允许所有来源的跨域请求
    logger=True,                 # 启用Socket.IO日志记录
    engineio_logger=True         # 启用Engine.IO日志记录
)

# ==================== FastAPI应用设置 ====================
# 创建FastAPI应用实例
app = FastAPI(title="XLeRobot Web Control API", version="0.1.0")

# 注册启动和关闭事件处理器
app.add_event_handler("startup", startup_event)
app.add_event_handler("shutdown", shutdown_event)

# 添加CORS中间件，允许跨域请求
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],         # 允许所有来源
    allow_credentials=False,     # 不允许凭据
    allow_methods=["*"],         # 允许所有HTTP方法
    allow_headers=["*"],         # 允许所有请求头
)

# 创建ASGI应用，结合FastAPI和Socket.IO
socket_app = socketio.ASGIApp(sio, app)

@app.get("/")
async def root():
    """
    根端点

    Returns:
        包含服务器运行状态的消息
    """
    return {"message": "XLeRobot Web Control Server is running"}

@app.get("/health")
async def health_check():
    """
    健康检查端点

    返回服务器和远程控制核心的健康状态信息，包括连接状态和能力。

    Returns:
        包含健康状态和控制器信息的字典
    """
    controller_info = {
        'type': config.robot_type,
        'connected': remote_core.connected,
        'capabilities': remote_core.get_capabilities()
    }

    return {
        "status": "healthy",
        "service": "XLeRobot Web Control",
        "controller": controller_info
    }

@app.get("/robot/info")
async def robot_info():
    """Robot controller info endpoint"""
    return {
        'controller_type': config.robot_type,
        'connected': remote_core.connected,
        'capabilities': remote_core.get_capabilities(),
        'state': await remote_core.get_state() if remote_core.connected else None
    }

@app.get("/robot/controllers")
async def available_controllers():
    """Get available controllers"""
    return {
        'available': ['maniskill', 'mujoco', 'xlerobot'],
        'current': config.robot_type,
        'description': 'Unified remote control supports all robot types via host programs'
    }

@app.post("/robot/camera/reset")
async def reset_camera():
    """Reset camera to default position"""
    try:
        result = await remote_core.reset_camera()
        return result
    except Exception as e:
        return {"error": f"Camera reset failed: {str(e)}"}

@app.post("/robot/camera/position")
async def set_camera_position(request: dict):
    """Set camera position"""
    position = request.get('position')
    target = request.get('target')

    if not position:
        return {"error": "Position is required"}

    try:
        result = await remote_core.set_camera_position(position, target)
        return result
    except Exception as e:
        return {"error": f"Set camera position failed: {str(e)}"}

@app.get("/robot/camera/info")
async def get_camera_info():
    """Get camera info"""
    try:
        # For simplified architecture, return basic camera info
        return {
            'camera_id': 0,
            'position': [2.0, 2.0, 2.0],
            'target': [0.0, 0.0, 0.0],
            'frame_size': {'width': config.video_width, 'height': config.video_height},
            'fps': config.video_fps,
            'quality': config.video_quality
        }
    except Exception as e:
        return {"error": f"Get camera info failed: {str(e)}"}

# ==================== 速率限制和节流函数 ====================
def check_rate_limit(sid: str) -> bool:
    """
    检查客户端是否在速率限制范围内

    Args:
        sid: 客户端会话ID

    Returns:
        如果客户端在速率限制范围内返回True
    """
    current_time = time.time()
    client_state = client_states.get(sid)

    if not client_state:
        return True

    # 如果时间窗口已过，重置命令计数
    if current_time - client_state['window_start'] >= RATE_LIMIT_CONFIG['rate_limit_window']:
        client_state['command_count'] = 0
        client_state['window_start'] = current_time

    return client_state['command_count'] < RATE_LIMIT_CONFIG['max_commands_per_second']

def check_throttle(sid: str) -> bool:
    """
    检查客户端是否发送命令过快

    Args:
        sid: 客户端会话ID

    Returns:
        如果客户端没有违反节流规则返回True
    """
    current_time = time.time()
    client_state = client_states.get(sid)

    if not client_state:
        return True

    # 检查客户端是否在惩罚期内
    if (client_state['throttle_violations'] >= RATE_LIMIT_CONFIG['max_throttle_violations'] and
        current_time - client_state['last_throttle_time'] < RATE_LIMIT_CONFIG['throttle_penalty_duration']):
        return False

    # 如果惩罚期已过，重置违规计数
    if current_time - client_state['last_throttle_time'] >= RATE_LIMIT_CONFIG['throttle_penalty_duration']:
        client_state['throttle_violations'] = 0

    # 检查命令之间的最小间隔
    if current_time - client_state['last_command_time'] < RATE_LIMIT_CONFIG['min_command_interval']:
        client_state['throttle_violations'] += 1
        client_state['last_throttle_time'] = current_time
        return False

    return True

def update_client_state(sid: str, *, timestamp: float) -> None:
    """Update client state after successful command"""
    client_state = client_states.setdefault(sid, _init_client_state())
    client_state['last_command_time'] = timestamp
    client_state['command_count'] += 1

@sio.event
async def connect(sid, environ, auth):
    """
    客户端连接事件处理器

    当新的WebSocket客户端连接时调用，初始化客户端状态并发送欢迎消息。

    Args:
        sid: 客户端会话ID
        environ: 连接环境信息
        auth: 认证信息
    """
    print(f"Client connected: {sid}")
    # 初始化客户端状态
    client_states[sid] = _init_client_state()
    await sio.emit('connection_established', {'message': 'Connected to XLeRobot control server'}, to=sid)

@sio.event
async def disconnect(sid):
    """
    客户端断开连接事件处理器

    当客户端断开连接时调用，清理相关的视频流任务和状态信息。

    Args:
        sid: 客户端会话ID
    """
    print(f"Client disconnected: {sid}")

    # 取消该客户端的视频流任务
    if sid in video_manager.stream_tasks:
        task = video_manager.stream_tasks[sid]
        if not task.cancelled():
            task.cancel()
            print(f"Cancelled video stream task for disconnected client {sid}")
        del video_manager.stream_tasks[sid]

    # 清理客户端状态
    if client_states.pop(sid, None) is not None:
        print(f"Cleaned up state for client {sid}")

@sio.event
async def ping(sid, data):
    """Ping-pong test event"""
    print(f"Received ping from {sid}: {data}")
    await sio.emit('pong', {'message': 'pong', 'timestamp': data.get('timestamp')}, to=sid)

@sio.event
async def move_command(sid, data):
    """
    移动命令事件处理器（带速率限制和节流控制）

    处理客户端发送的移动命令，包括速率限制检查、节流控制和命令执行。

    Args:
        sid: 客户端会话ID
        data: 包含移动命令数据的字典
    """
    direction = data.get('direction')      # 移动方向
    speed = data.get('speed', 1.0)        # 移动速度，默认1.0
    timestamp = data.get('timestamp', time.time() * 1000)  # 客户端时间戳

    current_time = time.time()

    # ==================== 速率限制检查 ====================
    if not check_rate_limit(sid):
        await sio.emit('command_received', {
            'type': 'move',
            'status': 'rate_limited',
            'message': f'Rate limit exceeded: maximum {RATE_LIMIT_CONFIG["max_commands_per_second"]} commands per second',
            'max_rate': RATE_LIMIT_CONFIG['max_commands_per_second'],
            'client_timestamp': timestamp,
            'server_timestamp': current_time * 1000
        }, to=sid)
        return

    # ==================== 节流控制检查 ====================
    if not check_throttle(sid):
        client_state = client_states.setdefault(sid, _init_client_state())
        await sio.emit('command_received', {
            'type': 'move',
            'status': 'throttled',
            'message': f'Command throttled: minimum {RATE_LIMIT_CONFIG["min_command_interval"]*1000:.0f}ms interval required',
            'min_interval': RATE_LIMIT_CONFIG['min_command_interval'] * 1000,
            'violations': client_state['throttle_violations'],
            'penalty_remaining': max(0, RATE_LIMIT_CONFIG['throttle_penalty_duration'] - (current_time - client_state['last_throttle_time'])) if client_state['throttle_violations'] >= RATE_LIMIT_CONFIG['max_throttle_violations'] else 0,
            'client_timestamp': timestamp,
            'server_timestamp': current_time * 1000
        }, to=sid)
        return

    if not remote_core.connected:
        await sio.emit('command_received', {
            'type': 'move',
            'status': 'error',
            'message': 'Remote core not connected to robot host',
            'client_timestamp': timestamp,
            'server_timestamp': current_time * 1000
        }, to=sid)
        return

    # Log command with rate limiting info
    client_state = client_states.setdefault(sid, _init_client_state())
    print(f"[{sid[:8]}] Move command: {direction} (speed={speed:.1f}) - count={client_state['command_count'] + 1}/{RATE_LIMIT_CONFIG['max_commands_per_second']}")

    # Execute robot command
    try:
        result = await remote_core.move(direction, speed)

        # Update client state after successful command
        update_client_state(sid, timestamp=current_time)

        # Get current robot state
        robot_state = await remote_core.get_state()

        # Calculate latency
        latency = (current_time * 1000) - timestamp if timestamp else 0

        # Send success response with detailed metrics
        await sio.emit('command_received', {
            'type': 'move',
            'direction': direction,
            'speed': speed,
            'status': result.get('status', 'executed'),
            'robot_state': robot_state,
            'metrics': {
                'latency': latency,
                'commands_in_window': client_state['command_count'],
                'max_commands': RATE_LIMIT_CONFIG['max_commands_per_second'],
                'throttle_violations': client_state['throttle_violations']
            },
            'client_timestamp': timestamp,
            'server_timestamp': current_time * 1000
        }, to=sid)

    except Exception as e:
        await sio.emit('command_received', {
            'type': 'move',
            'status': 'error',
            'message': f'Command execution failed: {str(e)}',
            'client_timestamp': timestamp,
            'server_timestamp': current_time * 1000
        }, to=sid)

@sio.event
async def start_video_stream(sid):
    """Start video streaming"""
    print(f"Client {sid} requested start video stream")
    
    if sid in video_manager.stream_tasks:
        task = video_manager.stream_tasks[sid]
        if not task.cancelled():
            task.cancel()
        del video_manager.stream_tasks[sid]
    
    result = await video_manager.start_stream()
    await sio.emit('stream_status', result, to=sid)
    
    task = asyncio.create_task(video_manager.stream_frames(sio, sid))
    video_manager.stream_tasks[sid] = task

@sio.event
async def stop_video_stream(sid):
    """Stop video streaming"""
    print(f"Client {sid} requested stop video stream")
    result = await video_manager.stop_stream(sid)
    await sio.emit('stream_status', result, to=sid)


@sio.event
async def reset_camera(sid):
    """Reset camera event handler"""
    print(f"Client {sid} requested camera reset")

    try:
        result = await remote_core.reset_camera()
        await sio.emit('camera_action_result', {
            'action': 'reset',
            **result
        }, to=sid)
        print(f"Camera reset result: {result['status']}")
    except Exception as e:
        await sio.emit('camera_action_result', {
            'action': 'reset',
            'status': 'error',
            'message': f'Camera reset failed: {str(e)}'
        }, to=sid)

@sio.event
async def set_camera_position(sid, data):
    """Set camera position event handler"""
    print(f"Client {sid} requested set camera position")

    position = data.get('position')
    target = data.get('target')

    if not position:
        await sio.emit('camera_action_result', {
            'action': 'set_position',
            'status': 'error',
            'message': 'Position is required'
        }, to=sid)
        return

    try:
        result = await remote_core.set_camera_position(position, target)
        await sio.emit('camera_action_result', {
            'action': 'set_position',
            **result
        }, to=sid)
        print(f"Camera position set result: {result['status']}")
    except Exception as e:
        await sio.emit('camera_action_result', {
            'action': 'set_position',
            'status': 'error',
            'message': f'Set camera position failed: {str(e)}'
        }, to=sid)

@sio.event
async def get_camera_info(sid):
    """Get camera info event handler"""
    print(f"Client {sid} requested camera info")

    try:
        # Return basic camera info for simplified architecture
        camera_info = {
            'camera_id': 0,
            'position': [2.0, 2.0, 2.0],
            'target': [0.0, 0.0, 0.0],
            'frame_size': {'width': config.video_width, 'height': config.video_height},
            'fps': config.video_fps,
            'quality': config.video_quality
        }
        await sio.emit('camera_info_result', {
            'status': 'success',
            'camera_info': camera_info
        }, to=sid)
    except Exception as e:
        await sio.emit('camera_info_result', {
            'status': 'error',
            'message': f'Get camera info failed: {str(e)}'
        }, to=sid)

# ==================== 程序入口点 ====================
if __name__ == "__main__":
    """
    主程序入口

    启动XLeRobot Web控制服务器，使用uvicorn ASGI服务器运行应用。
    """
    uvicorn.run(
        "main:socket_app",           # 指定要运行的应用
        host=config.ui_host,          # 监听地址
        port=config.ui_port,          # 监听端口
        reload=False,                # 禁用自动重载（简化架构）
        log_level="info"             # 日志级别
    )
