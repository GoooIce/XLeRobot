"""
JoyCon Pythonic封装模块

此模块提供了对底层JoyCon类的Python风格封装，使接口更加易用和符合Python编程习惯。
通过属性、元组和列表等方式简化数据访问。

主要功能：
- 将Java风格的getter方法转换为Python属性
- 将相关的XY/XYZ数据打包为元组
- 将多个传感器测量值打包为列表
- 支持左JoyCon的Y/Z轴反转以匹配右JoyCon
"""

from .joycon import JoyCon


# 最好能将此类合并到父类中（如果原作者同意的话）
class PythonicJoyCon(JoyCon):
    """
    JoyCon的Python风格封装类

    这是JoyCon父类的封装类，通过以下方式创建更符合Python习惯的接口：
    * 使用属性替代Java风格的getter和setter方法
    * 将相关的XY/XYZ数据打包为元组
    * 将陀螺仪和加速度计的多个测量值打包为列表
    * 添加左JoyCon的Y/Z轴反转选项以匹配右JoyCon（默认启用）
    """

    def __init__(self, *a, invert_left_ime_yz=True, **kw):
        """
        初始化PythonicJoyCon

        参数:
            *a: 传递给父类的位置参数
            invert_left_ime_yz (bool): 是否反转左JoyCon的Y/Z轴，默认True
            **kw: 传递给父类的关键字参数
        """
        super().__init__(*a, **kw)
        # 设置Y/Z轴系数：左JoyCon为-1（反转），右JoyCon为1（正常）
        self._ime_yz_coeff = -1 if invert_left_ime_yz and self.is_left() else 1

    # 电池状态属性
    is_charging   = property(JoyCon.get_battery_charging)   # 充电状态
    battery_level = property(JoyCon.get_battery_level)     # 电池电量

    # 右JoyCon按键属性
    r             = property(JoyCon.get_button_r)           # R按键
    zr            = property(JoyCon.get_button_zr)          # ZR按键
    plus          = property(JoyCon.get_button_plus)        # Plus按键
    a             = property(JoyCon.get_button_a)           # A按键
    b             = property(JoyCon.get_button_b)           # B按键
    x             = property(JoyCon.get_button_x)           # X按键
    y             = property(JoyCon.get_button_y)           # Y按键
    stick_r_btn   = property(JoyCon.get_button_r_stick)     # 右摇杆按键
    home          = property(JoyCon.get_button_home)        # Home按键
    right_sr      = property(JoyCon.get_button_right_sr)    # 右SR按键
    right_sl      = property(JoyCon.get_button_right_sl)    # 右SL按键

    # 左JoyCon按键属性
    l             = property(JoyCon.get_button_l)           # L按键  # noqa: E741
    zl            = property(JoyCon.get_button_zl)          # ZL按键
    minus         = property(JoyCon.get_button_minus)       # Minus按键
    stick_l_btn   = property(JoyCon.get_button_l_stick)     # 左摇杆按键
    up            = property(JoyCon.get_button_up)          # 上方向键
    down          = property(JoyCon.get_button_down)        # 下方向键
    left          = property(JoyCon.get_button_left)        # 左方向键
    right         = property(JoyCon.get_button_right)       # 右方向键
    capture       = property(JoyCon.get_button_capture)     # Capture按键
    left_sr       = property(JoyCon.get_button_left_sr)     # 左SR按键
    left_sl       = property(JoyCon.get_button_left_sl)     # 左SL按键

    # LED控制方法别名
    set_led_on       = JoyCon.set_player_lamp_on        # 设置LED常亮
    set_led_flashing = JoyCon.set_player_lamp_flashing  # 设置LED闪烁
    set_led          = JoyCon.set_player_lamp           # 设置LED模式
    disconnect       = JoyCon.disconnect_device        # 断开设备连接

    @property
    def stick_l(self):
        """
        获取左摇杆位置

        返回:
            tuple: (horizontal, vertical) 摇杆水平和垂直位置
        """
        return (
            self.get_stick_left_horizontal(),
            self.get_stick_left_vertical(),
        )

    @property
    def stick_r(self):
        """
        获取右摇杆位置

        返回:
            tuple: (horizontal, vertical) 摇杆水平和垂直位置
        """
        return (
            self.get_stick_right_horizontal(),
            self.get_stick_right_vertical(),
        )

    @property
    def accel(self):
        """
        获取加速度计原始数据

        返回3个加速度计测量值的列表，每个值为原始ADC读数。

        返回:
            list: 包含3个(x, y, z)元组的列表，每个元组代表一次测量
        """
        c = self._ime_yz_coeff  # Y/Z轴系数（用于左JoyCon轴反转）
        return [
            (
                self.get_accel_x(i),        # X轴加速度
                self.get_accel_y(i) * c,    # Y轴加速度（可能反转）
                self.get_accel_z(i) * c,    # Z轴加速度（可能反转）
            )
            for i in range(3)  # 获取3个测量样本
        ]

    @property
    def accel_in_g(self):
        """
        获取以g为单位的加速度计数据

        将原始ADC读数转换为以重力加速度g为单位的数据。

        返回:
            list: 包含3个(x, y, z)元组的列表，每个元组代表一次以g为单位的测量
        """
        c = 4.0 / 0x4000      # ADC到g的转换系数
        c2 = c * self._ime_yz_coeff  # 考虑轴反转的Y/Z转换系数
        return [
            (
                self.get_accel_x(i) * c,    # X轴加速度（g）
                self.get_accel_y(i) * c2,   # Y轴加速度（g，可能反转）
                self.get_accel_z(i) * c2,   # Z轴加速度（g，可能反转）
            )
            for i in range(3)  # 获取3个测量样本
        ]

    @property
    def gyro(self):
        c = self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i),
                self.get_gyro_y(i) * c,
                self.get_gyro_z(i) * c,
            )
            for i in range(3)
        ]

    @property
    def gyro_in_deg(self):
        c = 0.06103
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i) * c,
                self.get_gyro_y(i) * c2,
                self.get_gyro_z(i) * c2,
            )
            for i in range(3)
        ]

    @property
    def gyro_in_rad(self):
        c = 0.0001694 * 3.1415926536
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i) * c,
                self.get_gyro_y(i) * c2,
                self.get_gyro_z(i) * c2,
            )
            for i in range(3)
        ]

    @property
    def gyro_in_rot(self):
        c = 0.0001694
        c2 = c * self._ime_yz_coeff
        return [
            (
                self.get_gyro_x(i) * c,
                self.get_gyro_y(i) * c2,
                self.get_gyro_z(i) * c2,
            )
            for i in range(3)
        ]
