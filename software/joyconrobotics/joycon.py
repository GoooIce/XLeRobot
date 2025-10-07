"""
JoyCon底层通信模块

此模块提供了与Nintendo Switch JoyCon设备直接通信的核心功能。
负责HID设备的连接、数据传输、传感器设置和状态读取。

主要功能：
- JoyCon设备的连接和断开
- HID报告的读写操作
- 陀螺仪和加速度计校准
- 按钮状态读取
- LED指示灯控制
"""

from .constants import JOYCON_VENDOR_ID, JOYCON_PRODUCT_IDS, JOYCON_SERIAL_HEAD
from .constants import JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID, JOYCON_REPORT_DEFAULT, JOYCON_SERIAL_SUPPORT
import hid
import time
import threading
from typing import Optional

# TODO: disconnect, power off sequence - 需要实现断开连接和关机序列


class JoyCon:
    """
    JoyCon设备通信核心类

    提供与JoyCon设备的底层HID通信功能，包括数据读写、校准和状态监控。
    """

    # 输入报告大小（字节）
    _INPUT_REPORT_SIZE = 49
    # 输入报告周期（秒）
    _INPUT_REPORT_PERIOD = 0.015
    # 震动数据格式
    _RUMBLE_DATA = b'\x00\x01\x40\x40\x00\x01\x40\x40'

    # 设备属性定义
    vendor_id  : int           # 供应商ID
    product_id : int           # 产品ID
    serial     : Optional[str] # 序列号
    simple_mode: bool          # 简单模式标志
    color_body : (int, int, int) # 机身颜色RGB值
    color_btn  : (int, int, int) # 按键颜色RGB值

    def __init__(self, vendor_id: int, product_id: int, serial: str = None, simple_mode=False):
        """
        初始化JoyCon设备连接

        参数:
            vendor_id (int): JoyCon供应商ID
            product_id (int): JoyCon产品ID
            serial (str, optional): JoyCon序列号
            simple_mode (bool): 是否使用简单模式，默认False

        异常:
            ValueError: 当提供的ID或序列号无效时抛出
        """
        # 验证供应商ID
        if vendor_id != JOYCON_VENDOR_ID:
            raise ValueError(f'vendor_id is invalid: {vendor_id!r}')

        # 验证产品ID
        if product_id not in JOYCON_PRODUCT_IDS:
            raise ValueError(f'product_id is invalid: {product_id!r}')

        # 允许标准序列号格式或非标准MAC地址格式
        is_standard_serial = serial[:12] in JOYCON_SERIAL_HEAD
        is_mac_address = len(serial) == 17 and serial.count(':') == 5
        if not (is_standard_serial or is_mac_address):
            raise ValueError(f'serial is invalid: {serial!r}')

        # 设置设备基本信息
        self.vendor_id   = vendor_id
        self.product_id  = product_id
        self.serial      = serial
        self.simple_mode = simple_mode  # TODO: 用于报告模式0x3f

        # 根据序列号格式判断是否需要校准
        if serial[:12] not in JOYCON_SERIAL_HEAD:
            self.calibrate_value = True   # 非标准序列号需要校准
        else:
            self.calibrate_value = False  # 标准序列号不需要校准

        # 设置内部状态
        self._input_hooks = []                                   # 输入钩子函数列表
        self._input_report = bytes(self._INPUT_REPORT_SIZE)     # 输入报告缓冲区
        self._packet_number = 0                                 # 数据包编号
        # 设置默认加速度计校准参数
        self.set_accel_calibration((0, 0, 0), (1, 1, 1))
        # 设置默认陀螺仪校准参数
        self.set_gyro_calibration((0, 0, 0), (1, 1, 1))

        # 连接到JoyCon设备
        self.enable = True  # 启用标志
        self._joycon_device = self._open(vendor_id, product_id, serial=None)
        self._read_joycon_data()    # 读取JoyCon数据（颜色、校准信息等）
        self._setup_sensors()       # 设置传感器

        # 在守护线程中启动与JoyCon的通信
        self._update_input_report_thread \
            = threading.Thread(target=self._update_input_report)
        self._update_input_report_thread.setDaemon(True)  # 设置为守护线程
        self._update_input_report_thread.start()          # 启动线程

    def _open(self, vendor_id, product_id, serial):
        """
        打开JoyCon设备连接

        尝试使用不同的hid库实现连接到JoyCon设备。

        参数:
            vendor_id (int): 供应商ID
            product_id (int): 产品ID
            serial (str): 序列号

        返回:
            hid设备对象

        异常:
            IOError: 当连接失败时抛出
            Exception: 当不支持的hid实现时抛出
        """
        try:
            if hasattr(hid, "device"):  # hidapi库实现
                _joycon_device = hid.device()
                _joycon_device.open(vendor_id, product_id, serial)
            elif hasattr(hid, "Device"):  # hid库实现
                _joycon_device = hid.Device(vendor_id, product_id, serial)
            else:
                raise Exception("Implementation of hid is not recognized!")
        except IOError as e:
            raise IOError('joycon connect failed') from e
        return _joycon_device

    def _close(self):
        """
        关闭JoyCon设备连接

        安全地关闭设备连接并清理资源。
        """
        if hasattr(self, "_joycon_device"):
            self._joycon_device.close()
            self.enable = False
            del self._joycon_device

    def _read_input_report(self) -> bytes:
        """
        读取输入报告

        从JoyCon设备读取固定大小的输入数据报告。

        返回:
            bytes: 包含输入数据的字节数组
        """
        return bytes(self._joycon_device.read(self._INPUT_REPORT_SIZE))

    def _write_output_report(self, command, subcommand, argument):
        """
        写入输出报告

        向JoyCon设备发送命令和数据。

        参数:
            command (bytes): 命令字节
            subcommand (bytes): 子命令字节
            argument (bytes): 参数数据
        """
        # TODO: 添加详细文档说明各字节的含义
        # 构造输出报告数据包
        self._joycon_device.write(b''.join([
            command,                                                    # 命令字节
            self._packet_number.to_bytes(1, byteorder='little'),       # 数据包编号
            self._RUMBLE_DATA,                                         # 震动数据
            subcommand,                                                # 子命令
            argument,                                                  # 参数
        ]))
        # 更新数据包编号（0-15循环）
        self._packet_number = (self._packet_number + 1) & 0xF

    def _send_subcmd_get_response(self, subcommand, argument) -> (bool, bytes):
        # TODO: handle subcmd when daemon is running
        self._write_output_report(b'\x01', subcommand, argument)

        report = self._read_input_report()
        while report[0] != 0x21:  # TODO, avoid this, await daemon instead
            report = self._read_input_report()

        # TODO, remove, see the todo above
        # assert report[1:2] != subcommand, "THREAD carefully"

        # TODO: determine if the cut bytes are worth anything

        return report[13] & 0x80, report[13:]  # (ack, data)

    def _spi_flash_read(self, address, size) -> bytes:
        assert size <= 0x1d
        argument = address.to_bytes(4, "little") + size.to_bytes(1, "little")
        
        ack, report = self._send_subcmd_get_response(b'\x10', argument)
        if not ack:
            raise IOError("After SPI read @ {address:#06x}: got NACK")
        
        out = report[7:size+7]
        
        # if out == JOYCON_REPORT_DEFAULT and report[:3] != JOYCON_COLOR_SUPPORT:
        #     print(f'{report[:3]=}')
        #     raise IOError("Something else than the expected ACK was recieved!")
        
        return out

    def _update_input_report(self):  # daemon thread
        while True and self.enable == True :
            report = self._read_input_report()
            # TODO, handle input reports of type 0x21 and 0x3f
            while report[0] != 0x30:
                report = self._read_input_report()

            self._input_report = report

            for callback in self._input_hooks:
                callback(self)

    def _read_joycon_data(self):
        color_data = self._spi_flash_read(0x6050, 6)
        
        # TODO: use this
        # stick_cal_addr = 0x8012 if self.is_left else 0x801D
        # stick_cal  = self._spi_flash_read(stick_cal_addr, 8)

        # user IME data
        if self._spi_flash_read(0x8026, 2) == b"\xB2\xA1":
            # print(f"Calibrate {self.serial} IME with user data")
            imu_cal = self._spi_flash_read(0x8028, 24)

        # factory IME data
        else:
            # print(f"Calibrate {self.serial} IME with factory data")
            imu_cal = self._spi_flash_read(0x6020, 24)

        self.color_body = tuple(color_data[:3])
        self.color_btn  = tuple(color_data[3:])

        self.set_accel_calibration((
                self._to_int16le_from_2bytes(imu_cal[ 0], imu_cal[ 1]),
                self._to_int16le_from_2bytes(imu_cal[ 2], imu_cal[ 3]),
                self._to_int16le_from_2bytes(imu_cal[ 4], imu_cal[ 5]),
            ), (
                self._to_int16le_from_2bytes(imu_cal[ 6], imu_cal[ 7]),
                self._to_int16le_from_2bytes(imu_cal[ 8], imu_cal[ 9]),
                self._to_int16le_from_2bytes(imu_cal[10], imu_cal[11]),
            )
        )
        self.set_gyro_calibration((
                self._to_int16le_from_2bytes(imu_cal[12], imu_cal[13]),
                self._to_int16le_from_2bytes(imu_cal[14], imu_cal[15]),
                self._to_int16le_from_2bytes(imu_cal[16], imu_cal[17]),
            ), (
                self._to_int16le_from_2bytes(imu_cal[18], imu_cal[19]),
                self._to_int16le_from_2bytes(imu_cal[20], imu_cal[21]),
                self._to_int16le_from_2bytes(imu_cal[22], imu_cal[23]),
            )
        )

    def _setup_sensors(self):
        # Enable 6 axis sensors
        self._write_output_report(b'\x01', b'\x40', b'\x01')
        # It needs delta time to update the setting
        time.sleep(0.02)
        # Change format of input report
        self._write_output_report(b'\x01', b'\x03', b'\x30')

    @staticmethod
    def _to_int16le_from_2bytes(hbytebe, lbytebe):
        uint16le = (lbytebe << 8) | hbytebe
        int16le = uint16le if uint16le < 32768 else uint16le - 65536
        return int16le

    def _get_nbit_from_input_report(self, offset_byte, offset_bit, nbit):
        byte = self._input_report[offset_byte]
        return (byte >> offset_bit) & ((1 << nbit) - 1)

    def __del__(self):
        self._close()

    def set_gyro_calibration(self, offset_xyz=None, coeff_xyz=None):
        if offset_xyz:
            self._GYRO_OFFSET_X, \
            self._GYRO_OFFSET_Y, \
            self._GYRO_OFFSET_Z = offset_xyz
        if coeff_xyz:
            cx, cy, cz = coeff_xyz
            self._GYRO_COEFF_X = 0x343b / cx if (cx != 0x343b and cx != 0 and self.calibrate_value) else 1
            self._GYRO_COEFF_Y = 0x343b / cy if (cy != 0x343b and cy != 0 and self.calibrate_value) else 1
            self._GYRO_COEFF_Z = 0x343b / cz if (cz != 0x343b and cz != 0 and self.calibrate_value) else 1

    def set_accel_calibration(self, offset_xyz=None, coeff_xyz=None):
        if offset_xyz:
            self._ACCEL_OFFSET_X, \
            self._ACCEL_OFFSET_Y, \
            self._ACCEL_OFFSET_Z = offset_xyz
        if coeff_xyz:
            cx, cy, cz = coeff_xyz
            self._ACCEL_COEFF_X = 0x4000 / cx if (cx != 0x4000 and cx != 0) else 1
            self._ACCEL_COEFF_Y = 0x4000 / cy if (cy != 0x4000 and cy != 0) else 1
            self._ACCEL_COEFF_Z = 0x4000 / cz if (cz != 0x4000 and cz != 0) else 1

    def register_update_hook(self, callback):
        self._input_hooks.append(callback)
        return callback  # this makes it so you could use it as a decorator

    def is_left(self):
        """
        判断是否为左JoyCon控制器

        返回:
            bool: 如果是左JoyCon返回True，否则返回False
        """
        return self.product_id == JOYCON_L_PRODUCT_ID

    def is_right(self):
        """
        判断是否为右JoyCon控制器

        返回:
            bool: 如果是右JoyCon返回True，否则返回False
        """
        return self.product_id == JOYCON_R_PRODUCT_ID

    def get_battery_charging(self):
        """
        获取电池充电状态

        返回:
            int: 1表示正在充电，0表示未充电
        """
        return self._get_nbit_from_input_report(2, 4, 1)

    def get_battery_level(self):
        """
        获取电池电量等级

        返回:
            int: 电池电量等级（0-7，数值越大电量越足）
        """
        return self._get_nbit_from_input_report(2, 5, 3)

    def get_button_y(self):
        """
        获取Y按键状态（右JoyCon）或上方向键状态（左JoyCon）

        返回:
            int: 1表示按下，0表示释放
        """
        return self._get_nbit_from_input_report(3, 0, 1)

    def get_button_x(self):
        """
        获取X按键状态（右JoyCon）或上方向键状态（左JoyCon）

        返回:
            int: 1表示按下，0表示释放
        """
        return self._get_nbit_from_input_report(3, 1, 1)

    def get_button_b(self):
        return self._get_nbit_from_input_report(3, 2, 1)

    def get_button_a(self):
        return self._get_nbit_from_input_report(3, 3, 1)

    def get_button_right_sr(self):
        return self._get_nbit_from_input_report(3, 4, 1)

    def get_button_right_sl(self):
        return self._get_nbit_from_input_report(3, 5, 1)

    def get_button_r(self):
        return self._get_nbit_from_input_report(3, 6, 1)

    def get_button_zr(self):
        return self._get_nbit_from_input_report(3, 7, 1)

    def get_button_minus(self):
        return self._get_nbit_from_input_report(4, 0, 1)

    def get_button_plus(self):
        return self._get_nbit_from_input_report(4, 1, 1)

    def get_button_r_stick(self):
        return self._get_nbit_from_input_report(4, 2, 1)

    def get_button_l_stick(self):
        return self._get_nbit_from_input_report(4, 3, 1)

    def get_button_home(self):
        return self._get_nbit_from_input_report(4, 4, 1)

    def get_button_capture(self):
        return self._get_nbit_from_input_report(4, 5, 1)

    def get_button_charging_grip(self):
        return self._get_nbit_from_input_report(4, 7, 1)

    def get_button_down(self):
        return self._get_nbit_from_input_report(5, 0, 1)

    def get_button_up(self):
        return self._get_nbit_from_input_report(5, 1, 1)

    def get_button_right(self):
        return self._get_nbit_from_input_report(5, 2, 1)

    def get_button_left(self):
        return self._get_nbit_from_input_report(5, 3, 1)

    def get_button_left_sr(self):
        return self._get_nbit_from_input_report(5, 4, 1)

    def get_button_left_sl(self):
        return self._get_nbit_from_input_report(5, 5, 1)

    def get_button_l(self):
        return self._get_nbit_from_input_report(5, 6, 1)

    def get_button_zl(self):
        return self._get_nbit_from_input_report(5, 7, 1)

    def get_stick_left_horizontal(self):
        return self._get_nbit_from_input_report(6, 0, 8) \
            | (self._get_nbit_from_input_report(7, 0, 4) << 8)

    def get_stick_left_vertical(self):
        return self._get_nbit_from_input_report(7, 4, 4) \
            | (self._get_nbit_from_input_report(8, 0, 8) << 4)

    def get_stick_right_horizontal(self):
        return self._get_nbit_from_input_report(9, 0, 8) \
            | (self._get_nbit_from_input_report(10, 0, 4) << 8)

    def get_stick_right_vertical(self):
        return self._get_nbit_from_input_report(10, 4, 4) \
            | (self._get_nbit_from_input_report(11, 0, 8) << 4)

    def get_accel_x(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[13 + sample_idx * 12],
            self._input_report[14 + sample_idx * 12])
        return (data - self._ACCEL_OFFSET_X) * self._ACCEL_COEFF_X

    def get_accel_y(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[15 + sample_idx * 12],
            self._input_report[16 + sample_idx * 12])
        return (data - self._ACCEL_OFFSET_Y) * self._ACCEL_COEFF_Y

    def get_accel_z(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[17 + sample_idx * 12],
            self._input_report[18 + sample_idx * 12])
        return (data - self._ACCEL_OFFSET_Z) * self._ACCEL_COEFF_Z

    def get_gyro_x(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[19 + sample_idx * 12],
            self._input_report[20 + sample_idx * 12])
        return (data - self._GYRO_OFFSET_X) * self._GYRO_COEFF_X

    def get_gyro_y(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[21 + sample_idx * 12],
            self._input_report[22 + sample_idx * 12])
        return (data - self._GYRO_OFFSET_Y) * self._GYRO_COEFF_Y

    def get_gyro_z(self, sample_idx=0):
        if sample_idx not in (0, 1, 2):
            raise IndexError('sample_idx should be between 0 and 2')
        data = self._to_int16le_from_2bytes(
            self._input_report[23 + sample_idx * 12],
            self._input_report[24 + sample_idx * 12])
        return (data - self._GYRO_OFFSET_Z) * self._GYRO_COEFF_Z

    def get_status(self) -> dict:
        return {
            "battery": {
                "charging": self.get_battery_charging(),
                "level": self.get_battery_level(),
            },
            "buttons": {
                "right": {
                    "y": self.get_button_y(),
                    "x": self.get_button_x(),
                    "b": self.get_button_b(),
                    "a": self.get_button_a(),
                    "sr": self.get_button_right_sr(),
                    "sl": self.get_button_right_sl(),
                    "r": self.get_button_r(),
                    "zr": self.get_button_zr(),
                },
                "shared": {
                    "minus": self.get_button_minus(),
                    "plus": self.get_button_plus(),
                    "r-stick": self.get_button_r_stick(),
                    "l-stick": self.get_button_l_stick(),
                    "home": self.get_button_home(),
                    "capture": self.get_button_capture(),
                    "charging-grip": self.get_button_charging_grip(),
                },
                "left": {
                    "down": self.get_button_down(),
                    "up": self.get_button_up(),
                    "right": self.get_button_right(),
                    "left": self.get_button_left(),
                    "sr": self.get_button_left_sr(),
                    "sl": self.get_button_left_sl(),
                    "l": self.get_button_l(),
                    "zl": self.get_button_zl(),
                }
            },
            "analog-sticks": {
                "left": {
                    "horizontal": self.get_stick_left_horizontal(),
                    "vertical": self.get_stick_left_vertical(),
                },
                "right": {
                    "horizontal": self.get_stick_right_horizontal(),
                    "vertical": self.get_stick_right_vertical(),
                },
            },
            "accel": {
                "x": self.get_accel_x(),
                "y": self.get_accel_y(),
                "z": self.get_accel_z(),
            },
            "gyro": {
                "x": self.get_gyro_x(),
                "y": self.get_gyro_y(),
                "z": self.get_gyro_z(),
            },
        }

    def set_player_lamp_on(self, on_pattern: int):
        self._write_output_report(
            b'\x01', b'\x30',
            (on_pattern & 0xF).to_bytes(1, byteorder='little'))

    def set_player_lamp_flashing(self, flashing_pattern: int):
        self._write_output_report(
            b'\x01', b'\x30',
            ((flashing_pattern & 0xF) << 4).to_bytes(1, byteorder='little'))

    def set_player_lamp(self, pattern: int):
        self._write_output_report(
            b'\x01', b'\x30',
            pattern.to_bytes(1, byteorder='little'))

    def disconnect_device(self):
        self._write_output_report(b'\x01', b'\x06', b'\x00')


if __name__ == '__main__':
    import pyjoycon.device as d
    ids = d.get_L_id() if None not in d.get_L_id() else d.get_R_id()

    if None not in ids:
        joycon = JoyCon(*ids)
        lamp_pattern = 0
        while True:
            print(joycon.get_status())
            joycon.set_player_lamp_on(lamp_pattern)
            lamp_pattern = (lamp_pattern + 1) & 0xf
            time.sleep(0.2)
