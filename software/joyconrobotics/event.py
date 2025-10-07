"""
JoyCon按钮事件处理模块

此模块提供了JoyCon按钮事件的检测、缓冲和处理功能。
能够实时监测按钮状态变化，并生成相应的事件通知。

主要功能：
- 实时检测按钮按下和释放事件
- 支持左右JoyCon的所有按钮
- 可选择是否跟踪摇杆按键事件
- 提供事件缓冲和迭代器接口
"""

from .wrappers import PythonicJoyCon


class ButtonEventJoyCon(PythonicJoyCon):
    """
    JoyCon按钮事件处理器

    继承自PythonicJoyCon类，增加了按钮状态变化的事件检测功能。
    能够监测所有按钮的状态变化，并通过事件缓冲机制提供事件通知。
    """

    def __init__(self, *args, track_sticks=False, **kwargs):
        """
        初始化按钮事件处理器

        参数:
            *args: 传递给父类的位置参数
            track_sticks (bool): 是否跟踪摇杆按键事件，默认为False
            **kwargs: 传递给父类的关键字参数
        """
        super().__init__(*args, **kwargs)

        # 事件缓冲区，存储检测到的按钮事件 (TODO: 可能应该使用deque替代)
        self._events_buffer = []

        # 事件处理器字典（未在当前实现中使用）
        self._event_handlers = {}

        # 是否跟踪摇杆按键事件
        self._event_track_sticks = track_sticks

        # 初始化所有按钮的前一状态，用于检测状态变化
        self._previous_stick_l_btn = 0   # 左摇杆按键前一状态
        self._previous_stick_r_btn = 0   # 右摇杆按键前一状态
        self._previous_stick_r  = self._previous_stick_l  = (0, 0)  # 摇杆位置前一状态
        self._previous_r        = self._previous_l        = 0      # R/L按键前一状态
        self._previous_zr       = self._previous_zl       = 0      # ZR/ZL按键前一状态
        self._previous_plus     = self._previous_minus    = 0      # Plus/Minus按键前一状态
        self._previous_a        = self._previous_right    = 0      # A/右按键前一状态
        self._previous_b        = self._previous_down     = 0      # B/下按键前一状态
        self._previous_x        = self._previous_up       = 0      # X/上按键前一状态
        self._previous_y        = self._previous_left     = 0      # Y/左按键前一状态
        self._previous_home     = self._previous_capture  = 0      # Home/Capture按键前一状态
        self._previous_right_sr = self._previous_left_sr  = 0      # 右SR/左SR按键前一状态
        self._previous_right_sl = self._previous_left_sl  = 0      # 右SL/左SL按键前一状态

        # 根据JoyCon类型注册相应的更新钩子函数
        if self.is_left():
            self.register_update_hook(self._event_tracking_update_hook_left)
        else:
            self.register_update_hook(self._event_tracking_update_hook_right)

    def joycon_button_event(self, button, state):  # 可重写的方法
        """
        处理按钮事件

        当检测到按钮状态变化时调用此方法，将事件添加到缓冲区。

        参数:
            button (str): 按钮名称
            state (int): 按钮状态 (1表示按下，0表示释放)
        """
        self._events_buffer.append((button, state))

    def events(self):
        """
        事件迭代器

        返回缓冲区中的所有事件，并在返回时清空缓冲区。

        生成:
            tuple: (button_name, button_state) 按钮事件元组
        """
        while self._events_buffer:
            yield self._events_buffer.pop(0)

    @staticmethod
    def _event_tracking_update_hook_right(self):
        """
        右JoyCon事件跟踪更新钩子函数

        检测右JoyCon所有按钮的状态变化，包括摇杆按键（如果启用）。
        当检测到状态变化时，生成相应的事件。

        参数:
            self: JoyCon实例
        """
        # 检查右摇杆按键状态变化（如果启用跟踪）
        if self._event_track_sticks:
            pressed = self.stick_r_btn
            if self._previous_stick_r_btn != pressed:
                self._previous_stick_r_btn = pressed
                self.joycon_button_event("stick_r_btn", pressed)

        # 检查R按键状态变化
        pressed = self.r
        if self._previous_r != pressed:
            self._previous_r = pressed
            self.joycon_button_event("r", pressed)

        # 检查ZR按键状态变化
        pressed = self.zr
        if self._previous_zr != pressed:
            self._previous_zr = pressed
            self.joycon_button_event("zr", pressed)

        # 检查Plus按键状态变化
        pressed = self.plus
        if self._previous_plus != pressed:
            self._previous_plus = pressed
            self.joycon_button_event("plus", pressed)

        # 检查A按键状态变化
        pressed = self.a
        if self._previous_a != pressed:
            self._previous_a = pressed
            self.joycon_button_event("a", pressed)

        # 检查B按键状态变化
        pressed = self.b
        if self._previous_b != pressed:
            self._previous_b = pressed
            self.joycon_button_event("b", pressed)

        # 检查X按键状态变化
        pressed = self.x
        if self._previous_x != pressed:
            self._previous_x = pressed
            self.joycon_button_event("x", pressed)

        # 检查Y按键状态变化
        pressed = self.y
        if self._previous_y != pressed:
            self._previous_y = pressed
            self.joycon_button_event("y", pressed)

        # 检查Home按键状态变化
        pressed = self.home
        if self._previous_home != pressed:
            self._previous_home = pressed
            self.joycon_button_event("home", pressed)

        # 检查右SR按键状态变化
        pressed = self.right_sr
        if self._previous_right_sr != pressed:
            self._previous_right_sr = pressed
            self.joycon_button_event("right_sr", pressed)

        # 检查右SL按键状态变化
        pressed = self.right_sl
        if self._previous_right_sl != pressed:
            self._previous_right_sl = pressed
            self.joycon_button_event("right_sl", pressed)

    @staticmethod
    def _event_tracking_update_hook_left(self):
        """
        左JoyCon事件跟踪更新钩子函数

        检测左JoyCon所有按钮的状态变化，包括摇杆按键（如果启用）。
        当检测到状态变化时，生成相应的事件。

        参数:
            self: JoyCon实例
        """
        # 检查左摇杆按键状态变化（如果启用跟踪）
        if self._event_track_sticks:
            pressed = self.stick_l_btn
            if self._previous_stick_l_btn != pressed:
                self._previous_stick_l_btn = pressed
                self.joycon_button_event("stick_l_btn", pressed)

        # 检查L按键状态变化
        pressed = self.l
        if self._previous_l != pressed:
            self._previous_l = pressed
            self.joycon_button_event("l", pressed)

        # 检查ZL按键状态变化
        pressed = self.zl
        if self._previous_zl != pressed:
            self._previous_zl = pressed
            self.joycon_button_event("zl", pressed)

        # 检查Minus按键状态变化
        pressed = self.minus
        if self._previous_minus != pressed:
            self._previous_minus = pressed
            self.joycon_button_event("minus", pressed)

        # 检查上方向键状态变化
        pressed = self.up
        if self._previous_up != pressed:
            self._previous_up = pressed
            self.joycon_button_event("up", pressed)

        # 检查下方向键状态变化
        pressed = self.down
        if self._previous_down != pressed:
            self._previous_down = pressed
            self.joycon_button_event("down", pressed)

        # 检查左方向键状态变化
        pressed = self.left
        if self._previous_left != pressed:
            self._previous_left = pressed
            self.joycon_button_event("left", pressed)

        # 检查右方向键状态变化
        pressed = self.right
        if self._previous_right != pressed:
            self._previous_right = pressed
            self.joycon_button_event("right", pressed)

        # 检查Capture按键状态变化
        pressed = self.capture
        if self._previous_capture != pressed:
            self._previous_capture = pressed
            self.joycon_button_event("capture", pressed)

        # 检查左SR按键状态变化
        pressed = self.left_sr
        if self._previous_left_sr != pressed:
            self._previous_left_sr = pressed
            self.joycon_button_event("left_sr", pressed)

        # 检查左SL按键状态变化
        pressed = self.left_sl
        if self._previous_left_sl != pressed:
            self._previous_left_sl = pressed
            self.joycon_button_event("left_sl", pressed)
