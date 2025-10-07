"""
JoyCon陀螺仪跟踪模块

此模块提供了JoyCon陀螺仪数据的实时跟踪和姿态计算功能。
能够计算JoyCon的三维旋转角度和方向向量，用于指向控制或物体旋转。

主要功能：
- 实时读取陀螺仪数据
- 计算JoyCon的当前旋转角度
- 提供方向向量和指针位置
- 支持陀螺仪校准
"""

from .wrappers import PythonicJoyCon
from glm import vec2, vec3, quat, angleAxis, eulerAngles
from typing import Optional
import time


class GyroTrackingJoyCon(PythonicJoyCon):
    """
    JoyCon陀螺仪跟踪器

    基于PythonicJoyCon的专门类，用于跟踪陀螺仪数据并推导JoyCon的当前旋转状态。
    可用于创建指针、旋转物体或指向特定方向的应用。
    使用前需要进行校准。
    """

    def __init__(self, *args, **kwargs):
        """
        初始化陀螺仪跟踪器

        参数:
            *args: 传递给父类的位置参数
            **kwargs: 传递给父类的关键字参数
        """
        super().__init__(*args, simple_mode=False, **kwargs)

        # 设置内部状态：重置方向向量
        self.reset_orientation()

        # 注册更新回调函数
        self.register_update_hook(self._gyro_update_hook)

    @property
    def pointer(self) -> Optional[vec2]:
        """
        获取指针位置（2D向量）

        基于JoyCon方向计算屏幕上的指针位置。当JoyCon指向前方时返回有效坐标。

        返回:
            Optional[vec2]: 2D指针坐标，如果JoyCon没有指向前方则返回None
        """
        d = self.direction
        if d.x <= 0:  # JoyCon没有指向前方
            return None
        return vec2(d.y, -d.z) / d.x

    @property
    def direction(self) -> vec3:
        """
        获取当前方向向量

        返回JoyCon当前指向的三维方向向量。

        返回:
            vec3: 三维方向向量
        """
        return self.direction_X

    @property
    def rotation(self) -> vec3:
        """
        获取当前旋转角度（欧拉角）

        返回JoyCon当前的三维旋转角度。

        返回:
            vec3: 包含roll、pitch、yaw的三维旋转角度向量
        """
        return -eulerAngles(self.direction_Q)

    # 校准状态标志
    is_calibrating = False

    def calibrate(self, seconds=2):
        """
        开始陀螺仪校准

        在指定时间内收集陀螺仪数据以计算偏移量。校准期间JoyCon应保持静止。

        参数:
            seconds (int): 校准时间（秒），默认为2秒
        """
        # 初始化校准累加器
        self.calibration_acumulator = vec3(0)
        self.calibration_acumulations = 0
        # 设置校准结束时间
        self.is_calibrating = time.time() + seconds

    def _set_calibration(self, gyro_offset=None):
        """
        设置陀螺仪校准参数

        根据收集的数据计算并设置陀螺仪偏移量。

        参数:
            gyro_offset (vec3, optional): 手动指定的陀螺仪偏移量
        """
        if not gyro_offset:
            # 计算平均偏移量
            c = vec3(1, self._ime_yz_coeff, self._ime_yz_coeff)
            gyro_offset = self.calibration_acumulator * c
            gyro_offset /= self.calibration_acumulations
            # 添加基础偏移量
            gyro_offset += vec3(
                    self._GYRO_OFFSET_X,
                    self._GYRO_OFFSET_Y,
                    self._GYRO_OFFSET_Z,
                )
        # 结束校准并应用偏移量
        self.is_calibrating = False
        self.set_gyro_calibration(gyro_offset)

    def reset_orientation(self):
        """
        重置方向向量

        将JoyCon的方向重置为初始状态，所有方向向量指向坐标轴正方向。
        """
        self.direction_X = vec3(1, 0, 0)  # X轴方向
        self.direction_Y = vec3(0, 1, 0)  # Y轴方向
        self.direction_Z = vec3(0, 0, 1)  # Z轴方向
        self.direction_Q = quat()          # 四元数表示的旋转

    @staticmethod
    def _gyro_update_hook(self):
        """
        陀螺仪更新钩子函数

        定期调用以更新陀螺仪数据并计算新的方向向量。
        处理校准过程和实时方向更新。

        参数:
            self: JoyCon实例
        """
        # 处理校准过程
        if self.is_calibrating:
            if self.is_calibrating < time.time():
                # 校准时间结束，设置校准参数
                self._set_calibration()
            else:
                # 收集校准数据
                for xyz in self.gyro:
                    self.calibration_acumulator += xyz
                self.calibration_acumulations += 3

        # 更新方向向量
        for gx, gy, gz in self.gyro_in_rad:
            # TODO: 需要找出为什么1/86有效，而不是1/60或1/(60*30)
            # 计算旋转变换
            rotation \
                = angleAxis(gx * (-1/86), self.direction_X) \
                * angleAxis(gy * (-1/86), self.direction_Y) \
                * angleAxis(gz * (-1/86), self.direction_Z)

            # 应用旋转变换到方向向量
            self.direction_X *= rotation
            self.direction_Y *= rotation
            self.direction_Z *= rotation
            self.direction_Q *= rotation
