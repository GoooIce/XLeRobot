#!/usr/bin/env python3
"""
SO100/SO101机器人简化键盘控制程序
修复了动作格式转换问题
使用P控制，键盘仅改变目标关节角度
"""

import time
import logging
import traceback

# 设置日志系统
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# 关节校准系数 - 需要手动编辑
# 格式：[关节名称, 零位偏移量(度), 缩放因子]
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # 关节1：零位偏移量，缩放因子
    ['shoulder_lift', 2.0, 0.97],     # 关节2：零位偏移量，缩放因子
    ['elbow_flex', 0.0, 1.05],        # 关节3：零位偏移量，缩放因子
    ['wrist_flex', 0.0, 0.94],        # 关节4：零位偏移量，缩放因子
    ['wrist_roll', 0.0, 0.5],        # 关节5：零位偏移量，缩放因子
    ['gripper', 0.0, 1.0],           # 关节6：零位偏移量，缩放因子
]

def apply_joint_calibration(joint_name, raw_position):
    """
    应用关节校准系数

    Args:
        joint_name: 关节名称
        raw_position: 原始位置值

    Returns:
        calibrated_position: 校准后的位置值
    """
    # 遍历校准系数列表
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]  # 零位偏移量
            scale = joint_cal[2]   # 缩放因子
            calibrated_position = (raw_position - offset) * scale
            return calibrated_position
    return raw_position  # 如果找不到校准系数，返回原始值


def move_to_zero_position(robot, duration=3.0, kp=0.5):
    """
    使用P控制缓慢移动机器人到零位

    Args:
        robot: 机器人实例
        duration: 移动到零位所需时间（秒）
        kp: 比例增益
    """
    print("使用P控制缓慢移动机器人到零位...")

    # 获取当前机器人状态
    current_obs = robot.get_observation()

    # 提取当前关节位置
    current_positions = {}
    for key, value in current_obs.items():
        if key.endswith('.pos'):
            motor_name = key.removesuffix('.pos')
            current_positions[motor_name] = value

    # 零位目标
    zero_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
    }

    # 计算控制步数
    control_freq = 50  # 50Hz控制频率
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq

    print(f"将在{duration}秒内使用P控制移动到零位，控制频率：{control_freq}Hz，比例增益：{kp}")

    # 执行P控制循环
    for step in range(total_steps):
        # 获取当前机器人状态
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                # 应用校准系数
                calibrated_value = apply_joint_calibration(motor_name, value)
                current_positions[motor_name] = calibrated_value

        # P控制计算
        robot_action = {}
        for joint_name, target_pos in zero_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos

                # P控制：输出 = Kp * 误差
                control_output = kp * error

                # 将控制输出转换为位置命令
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position

        # 向机器人发送动作
        if robot_action:
            robot.send_action(robot_action)

        # 显示进度
        if step % (control_freq // 2) == 0:  # 每0.5秒显示一次进度
            progress = (step / total_steps) * 100
            print(f"移动到零位进度：{progress:.1f}%")

        time.sleep(step_time)

    print("机器人已移动到零位")

def return_to_start_position(robot, start_positions, kp=0.5, control_freq=50):
    """
    使用P控制返回起始位置

    Args:
        robot: 机器人实例
        start_positions: 起始关节位置字典
        kp: 比例增益
        control_freq: 控制频率（Hz）
    """
    print("返回起始位置...")

    control_period = 1.0 / control_freq
    max_steps = int(5.0 * control_freq)  # 最长5秒

    # 执行P控制循环返回起始位置
    for step in range(max_steps):
        # 获取当前机器人状态
        current_obs = robot.get_observation()
        current_positions = {}
        for key, value in current_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                current_positions[motor_name] = value  # 不应用校准系数

        # P控制计算
        robot_action = {}
        total_error = 0
        for joint_name, target_pos in start_positions.items():
            if joint_name in current_positions:
                current_pos = current_positions[joint_name]
                error = target_pos - current_pos
                total_error += abs(error)

                # P控制：输出 = Kp * 误差
                control_output = kp * error

                # 将控制输出转换为位置命令
                new_position = current_pos + control_output
                robot_action[f"{joint_name}.pos"] = new_position

        # 向机器人发送动作
        if robot_action:
            robot.send_action(robot_action)

        # 检查是否到达起始位置
        if total_error < 2.0:  # 如果总误差小于2度，认为到达
            print("已返回起始位置")
            break

        time.sleep(control_period)

    print("返回起始位置完成")

def p_control_loop(robot, keyboard, target_positions, start_positions, kp=0.5, control_freq=50):
    """
    P控制循环

    Args:
        robot: 机器人实例
        keyboard: 键盘实例
        target_positions: 目标关节位置字典
        start_positions: 起始关节位置字典
        kp: 比例增益
        control_freq: 控制频率（Hz）
    """
    control_period = 1.0 / control_freq

    print(f"启动P控制循环，控制频率：{control_freq}Hz，比例增益：{kp}")

    # 主控制循环
    while True:
        try:
            # 获取键盘输入
            keyboard_action = keyboard.get_action()

            if keyboard_action:
                # 处理键盘输入，更新目标位置
                for key, value in keyboard_action.items():
                    if key == 'x':
                        # 退出程序，先返回起始位置
                        print("检测到退出命令，返回起始位置...")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
                        return

                    # 关节控制映射
                    joint_controls = {
                        'q': ('shoulder_pan', -1),    # 关节1减少
                        'a': ('shoulder_pan', 1),     # 关节1增加
                        'w': ('shoulder_lift', -1),   # 关节2减少
                        's': ('shoulder_lift', 1),    # 关节2增加
                        'e': ('elbow_flex', -1),      # 关节3减少
                        'd': ('elbow_flex', 1),       # 关节3增加
                        'r': ('wrist_flex', -1),      # 关节4减少
                        'f': ('wrist_flex', 1),       # 关节4增加
                        't': ('wrist_roll', -1),      # 关节5减少
                        'g': ('wrist_roll', 1),       # 关节5增加
                        'y': ('gripper', -1),         # 关节6减少
                        'h': ('gripper', 1),          # 关节6增加
                    }

                    # 更新目标位置
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name in target_positions:
                            current_target = target_positions[joint_name]
                            new_target = int(current_target + delta)
                            target_positions[joint_name] = new_target
                            print(f"更新目标位置 {joint_name}: {current_target} -> {new_target}")

            # 获取当前机器人状态
            current_obs = robot.get_observation()

            # 提取当前关节位置
            current_positions = {}
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    # 应用校准系数
                    calibrated_value = apply_joint_calibration(motor_name, value)
                    current_positions[motor_name] = calibrated_value

            # P控制计算
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos - current_pos

                    # P控制：输出 = Kp * 误差
                    control_output = kp * error

                    # 将控制输出转换为位置命令
                    new_position = current_pos + control_output
                    robot_action[f"{joint_name}.pos"] = new_position

            # 向机器人发送动作
            if robot_action:
                robot.send_action(robot_action)

            time.sleep(control_period)

        except KeyboardInterrupt:
            print("用户中断程序")
            break
        except Exception as e:
            print(f"P控制循环错误：{e}")
            traceback.print_exc()
            break

def main():
    """主函数"""
    print("LeRobot 简化键盘控制示例（P控制）")
    print("="*50)

    try:
        # 导入必要的模块
        from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
        from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig

        # 获取端口
        port = input("请输入SO100机器人USB端口（例如：/dev/ttyACM0）：").strip()

        # 如果直接按回车，使用默认端口
        if not port:
            port = "/dev/ttyACM0"
            print(f"使用默认端口：{port}")
        else:
            print(f"连接到端口：{port}")

        # 配置机器人
        robot_config = SO100FollowerConfig(port=port)
        robot = SO100Follower(robot_config)

        # 配置键盘
        keyboard_config = KeyboardTeleopConfig()
        keyboard = KeyboardTeleop(keyboard_config)

        # 连接设备
        robot.connect()
        keyboard.connect()

        print("设备连接成功！")

        # 询问是否重新校准
        while True:
            calibrate_choice = input("是否要重新校准机器人？(y/n)：").strip().lower()
            if calibrate_choice in ['y', 'yes']:
                print("开始重新校准...")
                robot.calibrate()
                print("校准完成！")
                break
            elif calibrate_choice in ['n', 'no']:
                print("使用之前的校准文件")
                break
            else:
                print("请输入y或n")

        # 读取起始关节角度
        print("读取起始关节角度...")
        start_obs = robot.get_observation()
        start_positions = {}
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value)  # 不应用校准系数

        print("起始关节角度：")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")

        # 移动到零位
        move_to_zero_position(robot, duration=3.0)

        # 初始化目标位置为零位（整数）
        target_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
          }


        print("键盘控制说明：")
        print("- Q/A: 关节1 (shoulder_pan) 减少/增加")
        print("- W/S: 关节2 (shoulder_lift) 减少/增加")
        print("- E/D: 关节3 (elbow_flex) 减少/增加")
        print("- R/F: 关节4 (wrist_flex) 减少/增加")
        print("- T/G: 关节5 (wrist_roll) 减少/增加")
        print("- Y/H: 关节6 (gripper) 减少/增加")
        print("- X: 退出程序（先返回起始位置）")
        print("- ESC: 退出程序")
        print("="*50)
        print("注意：机器人会持续移动到目标位置")

        # 启动P控制循环
        p_control_loop(robot, keyboard, target_positions, start_positions, kp=0.5, control_freq=50)

        # 断开连接
        robot.disconnect()
        keyboard.disconnect()
        print("程序结束")

    except Exception as e:
        print(f"程序执行失败：{e}")
        traceback.print_exc()
        print("请检查：")
        print("1. 机器人是否正确连接")
        print("2. USB端口是否正确")
        print("3. 是否有足够的权限访问USB设备")
        print("4. 机器人是否正确配置")

if __name__ == "__main__":
    main() 