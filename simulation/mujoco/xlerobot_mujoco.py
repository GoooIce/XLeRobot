"""
XLeRobot MuJoCo仿真控制模块

该模块实现了XLeRobot在MuJoCo物理仿真环境中的键盘控制接口。
提供实时的3D可视化和交互式控制功能，用于机器人运动学的仿真测试。

主要功能：
- 加载和渲染MuJoCo仿真模型
- 实现键盘交互式控制
- 提供底盘运动和机械臂控制
- 实时显示控制状态和反馈信息
- 支持多速度级别调节

使用说明：
- 确保有scene.xml文件描述机器人模型
- 使用键盘按键控制机器人运动
- 观察3D可视化界面中的机器人状态

依赖库：
- mujoco: 物理仿真引擎
- mujoco_viewer: 3D可视化界面
- glfw: 键盘输入处理
- numpy: 数值计算
"""

# 导入标准库
import time  # 时间控制功能

# 导入第三方库
import mujoco_viewer  # MuJoCo 3D可视化器
import numpy as np     # NumPy数值计算库
import glfw            # GLFW图形库，用于键盘输入
import mujoco          # MuJoCo物理仿真引擎


class XLeRobotController:
    """
    XLeRobot MuJoCo仿真控制器类

    该类实现了XLeRobot在MuJoCo仿真环境中的完整控制功能。
    包括模型加载、可视化渲染、键盘控制、运动学计算等。

    主要功能：
    - 加载MuJoCo仿真模型
    - 配置3D可视化相机
    - 实现键盘交互控制
    - 计算机器人运动学
    - 提供实时状态反馈

    控制映射：
    - Home/End: 前进/后退
    - Delete/PageDown: 左移/右移
    - Insert/PageUp: 左转/右转
    - Q/A,S/D,W/E: 左臂关节控制
    - U/J,K/L,I,O: 右臂关节控制
    """

    def __init__(self, mjcf_path):
        """
        初始化XLeRobot仿真控制器

        Args:
            mjcf_path (str): MuJoCo模型文件路径（XML格式）
        """
        # 第一阶段：加载MuJoCo仿真模型
        self.model = mujoco.MjModel.from_xml_path(mjcf_path)  # 从XML文件创建模型
        self.data = mujoco.MjData(self.model)                 # 创建仿真数据对象
        mujoco.mj_forward(self.model, self.data)              # 执行前向动力学计算

        # 第二阶段：配置可视化渲染器
        self.render_freq = 60              # 渲染频率：60Hz
        self.render_interval = 1.0 / self.render_freq  # 渲染间隔时间（秒）
        self.viewer = mujoco_viewer.MujocoViewer(self.model, self.data)  # 创建3D查看器

        # 第三阶段：配置跟踪相机
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)  # 设置默认相机参数
        self.camera.type = mujoco.mjtCamera.mjCAMERA_TRACKING  # 使用跟踪相机模式
        self.camera.trackbodyid = self.model.body("chassis").id  # 跟踪底盘
        self.camera.distance = 3.0          # 相机距离：3米
        self.camera.azimuth = 90.0          # 方位角：90度
        self.camera.elevation = -30.0       # 仰角：-30度
        self.camera.lookat = np.array([0.0, 0.0, 0.0])  # 相机注视点

        # 第四阶段：初始化控制参数
        self.abs_vel = np.array([1, 1, 1])           # 绝对速度设置 [x, y, theta]
        self.chassis_ref_vel = np.zeros(3)           # 底盘参考速度 [vx, vy, vtheta]
        self.qCmd = np.zeros(self.model.nu)          # 位置控制指令向量
        self.qdCmd = np.zeros(self.model.nu)         # 速度控制指令向量
        self.qFb = np.zeros(self.model.nu)           # 位置反馈向量
        self.qdFb = np.zeros(self.model.nu)          # 速度反馈向量
        self.last_render_time = time.time()          # 上次渲染时间
        self.kp = 1                                   # 位置控制比例增益

        # 初始化键盘状态映射字典
        # 记录每个控制按键的当前按下状态
        self.key_states = {
            # 底盘运动控制键
            "home": False,      # 前进键 (+x方向)
            "end": False,       # 后退键 (-x方向)
            "delete": False,    # 左移键 (+y方向)
            "page_down": False, # 右移键 (-y方向)
            "insert": False,    # 左转键 (+z方向，逆时针)
            "page_up": False,   # 右转键 (-z方向，顺时针)

            # 左臂关节控制键（前3个关节）
            "q": False,         # 左臂关节1正向旋转
            "a": False,         # 左臂关节1负向旋转
            "w": False,         # 左臂关节2正向旋转
            "s": False,         # 左臂关节2负向旋转
            "e": False,         # 左臂关节3正向旋转
            "d": False,         # 左臂关节3负向旋转

            # 右臂关节控制键（前3个关节）
            "u": False,         # 右臂关节1正向旋转
            "j": False,         # 右臂关节1负向旋转
            "i": False,         # 右臂关节2正向旋转
            "k": False,         # 右臂关节2负向旋转
            "o": False,         # 右臂关节3正向旋转
            "l": False,         # 右臂关节3负向旋转
        }

    def update_feedback(self):
        """
        更新机器人状态反馈

        从MuJoCo仿真环境中获取当前的机器人状态，包括：
        - 关节位置 (qpos)
        - 关节速度 (qvel)

        这些状态信息用于控制反馈和状态显示。
        """
        self.qFb = self.data.qpos   # 获取所有关节的位置反馈
        self.qdFb = self.data.qvel  # 获取所有关节的速度反馈

    def update_keyboards(self):
        """
        更新键盘状态检测

        通过GLFW库检测查看器窗口中的键盘输入状态。
        将物理按键映射到逻辑控制命令。

        Key Mapping:
        - 底盘控制：Home/End/Delete/PageDown/Insert/PageUp
        - 左臂控制：Q/A/W/S/E/D (关节1-3)
        - 右臂控制：U/J/I/K/O/L (关节1-3)

        Note:
            如果查看器窗口不可用，则跳过键盘状态更新。
        """
        try:
            # 获取查看器窗口句柄
            window = self.viewer.window
            if window is None:
                return  # 窗口不存在，直接返回

            # 定义按键名称到GLFW键码的映射
            key_map = {
                # 底盘运动控制键
                "home": glfw.KEY_HOME,
                "end": glfw.KEY_END,
                "delete": glfw.KEY_DELETE,
                "page_down": glfw.KEY_PAGE_DOWN,
                "insert": glfw.KEY_INSERT,
                "page_up": glfw.KEY_PAGE_UP,

                # 左臂关节控制键
                "q": glfw.KEY_Q,
                "a": glfw.KEY_A,
                "w": glfw.KEY_W,
                "s": glfw.KEY_S,
                "e": glfw.KEY_E,
                "d": glfw.KEY_D,

                # 右臂关节控制键
                "u": glfw.KEY_U,
                "j": glfw.KEY_J,
                "i": glfw.KEY_I,
                "k": glfw.KEY_K,
                "o": glfw.KEY_O,
                "l": glfw.KEY_L,
            }

            # 遍历所有控制键，更新其按下状态
            for key_name, glfw_key in key_map.items():
                # 检查按键是否被按下，并更新状态字典
                self.key_states[key_name] = glfw.get_key(window, glfw_key) == glfw.PRESS

        except Exception:
            # 键盘状态检测出现异常，静默忽略
            pass

    def update_reference(self):
        # X-direction (forward/backward)
        yaw = self.qFb[2]
        rotmz = np.array(
            [
                [np.cos(yaw), np.sin(yaw), 0],
                [-np.sin(yaw), np.cos(yaw), 0],
                [0, 0, 1],
            ]
        )
        chassis_vel = rotmz @ self.qdFb[0:3]

        self.chassis_ref_vel = np.zeros(3)
        if self.key_states["home"]:
            self.chassis_ref_vel[0] = self.abs_vel[0]
        elif self.key_states["end"]:
            self.chassis_ref_vel[0] = -self.abs_vel[0]
        if self.key_states["delete"]:
            self.chassis_ref_vel[1] = self.abs_vel[1]
        elif self.key_states["page_down"]:
            self.chassis_ref_vel[1] = -self.abs_vel[1]

        if self.key_states["insert"]:
            self.chassis_ref_vel[2] = self.abs_vel[2]
        elif self.key_states["page_up"]:
            self.chassis_ref_vel[2] = -self.abs_vel[2]

        k_p = 10
        k_p_rot = 100
        self.qdCmd[0] = self.chassis_ref_vel[0] * np.cos(yaw) + \
                        self.chassis_ref_vel[1] * np.cos(yaw + 1.5708) + \
                        k_p * (self.chassis_ref_vel[0] - chassis_vel[0]) * np.cos(yaw) + \
                        k_p * (self.chassis_ref_vel[1] - chassis_vel[1]) * np.cos(yaw + 1.5708)
        self.qdCmd[1] = self.chassis_ref_vel[0] * np.sin(yaw) + \
                        self.chassis_ref_vel[1] * np.sin(yaw + 1.5708) + \
                        k_p * (self.chassis_ref_vel[0] - chassis_vel[0]) * np.sin(yaw) + \
                        k_p * (self.chassis_ref_vel[1] - chassis_vel[1]) * np.sin(yaw + 1.5708)
        self.qdCmd[2] = self.chassis_ref_vel[2] + k_p_rot * (self.chassis_ref_vel[2] - chassis_vel[2])

        radius = 0.1
        vel2wheel_matrix = np.array(
            [[0, 1, -radius], [-np.sqrt(3) * 0.5, -0.5, -radius], [np.sqrt(3) * 0.5, -0.5, -radius]]
        )
        coe_vel_to_wheel = 20
        self.qCmd[15:18] = coe_vel_to_wheel * np.dot(vel2wheel_matrix, chassis_vel)
        self.qdCmd[2] = np.clip(self.qdCmd[2], -1.0, 1.0)

        # Left arm joint control (qCmd[3:9])
        arm_step = 0.005

        # Left arm joint 1
        if self.key_states["q"]:
            self.qCmd[3] += arm_step
        elif self.key_states["a"]:
            self.qCmd[3] -= arm_step

        # Left arm joint 2
        if self.key_states["w"]:
            self.qCmd[4] += arm_step
        elif self.key_states["s"]:
            self.qCmd[4] -= arm_step

        # Left arm joint 3
        if self.key_states["e"]:
            self.qCmd[5] += arm_step
        elif self.key_states["d"]:
            self.qCmd[5] -= arm_step

        # Right arm joint control (qCmd[9:15])
        # Right arm joint 1
        if self.key_states["u"]:
            self.qCmd[9] += arm_step
        elif self.key_states["j"]:
            self.qCmd[9] -= arm_step

        # Right arm joint 2
        if self.key_states["i"]:
            self.qCmd[10] += arm_step
        elif self.key_states["k"]:
            self.qCmd[10] -= arm_step

        # Right arm joint 3
        if self.key_states["o"]:
            self.qCmd[11] += arm_step
        elif self.key_states["l"]:
            self.qCmd[11] -= arm_step

        # Keep other joints at zero
        self.qCmd[6:9] = 0.0  # Left arm joints 4-6
        self.qCmd[12:15] = 0.0  # Right arm joints 4-6

    def update_control(self):
        self.qdCmd[0:3] = self.kp * self.qdCmd[0:3]
        self.data.ctrl[:3] = self.qdCmd[:3]
        self.data.ctrl[3:] = self.qCmd[3:]

    def render_ui(self):
        current_time = time.time()

        if current_time - self.last_render_time >= self.render_interval:
            self.viewer.cam = self.camera
            self.viewer._overlay[mujoco.mjtGridPos.mjGRID_TOPLEFT] = [
                f"Time: {self.data.time:.3f} sec",
                "",
            ]
            self.viewer._overlay[mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT] = [
                "=== CHASSIS MOVEMENT (INCREMENTAL) ===\n"
                "Forward/Backward   (+x/-x): press Home/End\n"
                "Leftward/Rightward (+y/-y): press Delete/Page Down\n"
                "Rotate CCW/CW      (+z/-z): press Insert/Page Up\n"
                "\n=== LEFT ARM CONTROLS ===\n"
                "Joint1: q(+)/a(-)    Joint2: w(+)/s(-)    Joint3: e(+)/d(-)\n"
                "\n=== RIGHT ARM CONTROLS ===\n"
                "Joint1: u(+)/j(-)    Joint2: i(+)/k(-)    Joint3: o(+)/l(-)\n"
                f"\ncommand: Chassis Vel: [{self.qdCmd[0]:.2f}, {self.qdCmd[1]:.2f}, {self.qdCmd[2]:.2f}]\n"
                f"feedback: Chassis Vel: [{self.qdFb[0]:.2f}, {self.qdFb[1]:.2f}, {self.qdFb[2]:.2f}]\n"
                f"Left Arm: [{self.qCmd[3]:.2f}, {self.qCmd[4]:.2f}, {self.qCmd[5]:.2f}]\n"
                f"Right Arm: [{self.qCmd[9]:.2f}, {self.qCmd[10]:.2f}, {self.qCmd[11]:.2f}]",
                "",
            ]

            self.viewer.render()
            self.last_render_time = current_time

    def run(self):
        """
        XLeRobot主控制循环

        该方法是仿真的核心循环，按照固定频率执行以下步骤：
        1. 更新机器人状态反馈
        2. 检测键盘输入状态
        3. 计算参考控制指令
        4. 更新控制器输出
        5. 推进物理仿真
        6. 渲染用户界面

        循环频率：约500Hz (2ms间隔)
        """
        print("Starting XLeRobot keyboard Controller...")

        # 主控制循环，持续运行直到查看器窗口关闭
        while self.viewer.is_alive:
            # 第一步：获取机器人当前状态
            self.update_feedback()

            # 第二步：检测键盘输入
            self.update_keyboards()

            # 第三步：根据键盘输入计算控制指令
            self.update_reference()

            # 第四步：更新控制器输出
            self.update_control()

            # 第五步：推进物理仿真一步
            mujoco.mj_step(self.model, self.data)

            # 第六步：渲染用户界面和状态信息
            self.render_ui()

            # 控制循环频率，约500Hz
            time.sleep(0.002)

        # 循环结束，清理资源
        self.cleanup()

    def cleanup(self):
        """
        清理仿真资源

        在仿真结束后关闭查看器窗口并释放相关资源。
        """
        self.viewer.close()  # 关闭3D查看器窗口
        print("XLeRobot controller stopped.")  # 输出停止信息


def main():
    """
    XLeRobot仿真程序入口点

    该函数是程序的入口点，负责：
    1. 初始化仿真控制器
    2. 启动主控制循环
    3. 处理程序退出信号

    使用说明：
    - 确保当前目录存在scene.xml文件（MuJoCo模型文件）
    - 运行程序后将显示3D可视化窗口
    - 使用键盘控制机器人运动
    - 按Ctrl+C可安全退出程序
    """
    try:
        # 初始化仿真控制器
        mjcf_path = "scene.xml"  # MuJoCo模型文件路径
        controller = XLeRobotController(mjcf_path)  # 创建控制器实例

        # 启动主控制循环
        controller.run()  # 运行仿真控制

    except KeyboardInterrupt:
        # 处理用户中断信号（Ctrl+C）
        print("\nReceived keyboard interrupt, shutting down...")


if __name__ == "__main__":
    # 当作为主程序运行时执行main函数
    main()
