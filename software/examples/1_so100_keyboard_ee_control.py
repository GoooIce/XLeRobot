#!/usr/bin/env python3
"""
SO100/SO101机器人简化键盘末端控制程序
修复了动作格式转换问题
使用P控制，键盘仅改变目标末端位置和关节角度
"""

import time
import logging
import traceback
import math

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

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    计算两连杆机械臂的逆运动学，考虑关节偏移

    Parameters:
        x: 末端执行器x坐标
        y: 末端执行器y坐标
        l1: 上臂长度（默认0.1159米）
        l2: 下臂长度（默认0.1350米）

    Returns:
        joint2, joint3: URDF文件中定义的关节角度（度）
    """
    # 计算关节2和关节3在theta1和theta2中的偏移
    theta1_offset = math.atan2(0.028, 0.11257)  # 关节2为0时的theta1偏移
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset  # 关节3为0时的theta2偏移

    # 计算原点到目标点的距离
    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2  # 最大可达距离

    # 如果目标点超出最大工作空间，将其缩放到边界
    if r > r_max:
        scale_factor = r_max / r
        x *= scale_factor
        y *= scale_factor
        r = r_max

    # 如果目标点小于最小工作空间（|l1-l2|），将其缩放
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale_factor = r_min / r
        x *= scale_factor
        y *= scale_factor
        r = r_min

    # 使用余弦定理计算theta2
    cos_theta2 = -(r**2 - l1**2 - l2**2) / (2 * l1 * l2)

    # 计算theta2（肘部角度）
    theta2 = math.pi - math.acos(cos_theta2)

    # 计算theta1（肩部角度）
    beta = math.atan2(y, x)
    gamma = math.atan2(l2 * math.sin(theta2), l1 + l2 * math.cos(theta2))
    theta1 = beta + gamma

    # 将theta1和theta2转换为关节2和关节3角度
    joint2 = theta1 + theta1_offset
    joint3 = theta2 + theta2_offset

    # 确保角度在URDF限制范围内
    joint2 = max(-0.1, min(3.45, joint2))
    joint3 = max(-0.2, min(math.pi, joint3))

    # 从弧度转换为度
    joint2_deg = math.degrees(joint2)
    joint3_deg = math.degrees(joint3)

    # 调整角度以匹配机器人坐标系
    joint2_deg = 90-joint2_deg
    joint3_deg = joint3_deg-90

    return joint2_deg, joint3_deg

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

def p_control_loop(robot, keyboard, target_positions, start_positions, current_x, current_y, kp=0.5, control_freq=50):
    """
    P控制循环 - 实现末端位置控制

    Args:
        robot: 机器人实例
        keyboard: 键盘实例
        target_positions: 目标关节位置字典
        start_positions: 起始关节位置字典
        current_x: 当前x坐标
        current_y: 当前y坐标
        kp: 比例增益
        control_freq: 控制频率（Hz）
    """
    control_period = 1.0 / control_freq

    # 初始化俯仰角控制变量
    pitch = 0.0  # 初始俯仰角调整
    pitch_step = 1  # 俯仰角调整步长

    print(f"启动P控制循环，控制频率：{control_freq}Hz，比例增益：{kp}")
    
    while True:
        try:
            # Get keyboard input
            keyboard_action = keyboard.get_action()
            
            if keyboard_action:
                # Process keyboard input, update target positions
                for key, value in keyboard_action.items():
                    if key == 'x':
                        # Exit program, first return to start position
                        print("Exit command detected, returning to start position...")
                        return_to_start_position(robot, start_positions, 0.2, control_freq)
                        return
                    
                    # Joint control mapping
                    joint_controls = {
                        'q': ('shoulder_pan', -1),    # Joint 1 decrease
                        'a': ('shoulder_pan', 1),     # Joint 1 increase
                        't': ('wrist_roll', -1),      # Joint 5 decrease
                        'g': ('wrist_roll', 1),       # Joint 5 increase
                        'y': ('gripper', -1),         # Joint 6 decrease
                        'h': ('gripper', 1),          # Joint 6 increase
                    }
                    
                    # x,y coordinate control
                    xy_controls = {
                        'w': ('x', -0.004),  # x decrease
                        's': ('x', 0.004),   # x increase
                        'e': ('y', -0.004),  # y decrease
                        'd': ('y', 0.004),   # y increase
                    }
                    
                    # Pitch control
                    if key == 'r':
                        pitch += pitch_step
                        print(f"Increase pitch adjustment: {pitch:.3f}")
                    elif key == 'f':
                        pitch -= pitch_step
                        print(f"Decrease pitch adjustment: {pitch:.3f}")
                    
                    if key in joint_controls:
                        joint_name, delta = joint_controls[key]
                        if joint_name in target_positions:
                            current_target = target_positions[joint_name]
                            new_target = int(current_target + delta)
                            target_positions[joint_name] = new_target
                            print(f"Update target position {joint_name}: {current_target} -> {new_target}")
                    
                    elif key in xy_controls:
                        coord, delta = xy_controls[key]
                        if coord == 'x':
                            current_x += delta
                            # Calculate target angles for joint2 and joint3
                            joint2_target, joint3_target = inverse_kinematics(current_x, current_y)
                            target_positions['shoulder_lift'] = joint2_target
                            target_positions['elbow_flex'] = joint3_target
                            print(f"Update x coordinate: {current_x:.4f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
                        elif coord == 'y':
                            current_y += delta
                            # Calculate target angles for joint2 and joint3
                            joint2_target, joint3_target = inverse_kinematics(current_x, current_y)
                            target_positions['shoulder_lift'] = joint2_target
                            target_positions['elbow_flex'] = joint3_target
                            print(f"Update y coordinate: {current_y:.4f}, joint2={joint2_target:.3f}, joint3={joint3_target:.3f}")
            
            # Apply pitch adjustment to wrist_flex
            # Calculate wrist_flex target position based on shoulder_lift and elbow_flex
            if 'shoulder_lift' in target_positions and 'elbow_flex' in target_positions:
                target_positions['wrist_flex'] = - target_positions['shoulder_lift'] - target_positions['elbow_flex'] + pitch
                # Show current pitch value (display every 100 steps to avoid screen flooding)
                if hasattr(p_control_loop, 'step_counter'):
                    p_control_loop.step_counter += 1
                else:
                    p_control_loop.step_counter = 0
                
                if p_control_loop.step_counter % 100 == 0:
                    print(f"Current pitch adjustment: {pitch:.3f}, wrist_flex target: {target_positions['wrist_flex']:.3f}")
            
            # Get current robot state
            current_obs = robot.get_observation()
            
            # Extract current joint positions
            current_positions = {}
            for key, value in current_obs.items():
                if key.endswith('.pos'):
                    motor_name = key.removesuffix('.pos')
                    # Apply calibration coefficients
                    calibrated_value = apply_joint_calibration(motor_name, value)
                    current_positions[motor_name] = calibrated_value
            
            # P control calculation
            robot_action = {}
            for joint_name, target_pos in target_positions.items():
                if joint_name in current_positions:
                    current_pos = current_positions[joint_name]
                    error = target_pos - current_pos
                    
                    # P control: output = Kp * error
                    control_output = kp * error
                    
                    # Convert control output to position command
                    new_position = current_pos + control_output
                    robot_action[f"{joint_name}.pos"] = new_position
            
            # Send action to robot
            if robot_action:
                robot.send_action(robot_action)
            
            time.sleep(control_period)
            
        except KeyboardInterrupt:
            print("User interrupted program")
            break
        except Exception as e:
            print(f"P control loop error: {e}")
            traceback.print_exc()
            break

def main():
    """主函数 - 末端位置控制程序入口"""
    print("LeRobot 简化键盘末端控制示例（P控制）")
    print("="*50)

    try:
        # 导入必要的模块
        from lerobot.robots.so100_follower import SO100Follower, SO100FollowerConfig
        from lerobot.teleoperators.keyboard import KeyboardTeleop, KeyboardTeleopConfig

        # 获取端口
        port = input("请输入SO100机器人的USB端口（例如：/dev/ttyACM0）：").strip()

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
        
        # Read initial joint angles
        print("Reading initial joint angles...")
        start_obs = robot.get_observation()
        start_positions = {}
        for key, value in start_obs.items():
            if key.endswith('.pos'):
                motor_name = key.removesuffix('.pos')
                start_positions[motor_name] = int(value)  # Don't apply calibration coefficients
        
        print("Initial joint angles:")
        for joint_name, position in start_positions.items():
            print(f"  {joint_name}: {position}°")
        
        # Move to zero position
        move_to_zero_position(robot, duration=3.0)
        
        # Initialize target positions as current positions (integers)
        target_positions = {
        'shoulder_pan': 0.0,
        'shoulder_lift': 0.0,
        'elbow_flex': 0.0,
        'wrist_flex': 0.0,
        'wrist_roll': 0.0,
        'gripper': 0.0
          }
        
        # Initialize x,y coordinate control
        x0, y0 = 0.1629, 0.1131
        current_x, current_y = x0, y0
        print(f"Initialize end effector position: x={current_x:.4f}, y={current_y:.4f}")
        
        
        print("Keyboard control instructions:")
        print("- Q/A: Joint 1 (shoulder_pan) decrease/increase")
        print("- W/S: Control end effector x coordinate (joint2+3)")
        print("- E/D: Control end effector y coordinate (joint2+3)")
        print("- R/F: Pitch adjustment increase/decrease (affects wrist_flex)")
        print("- T/G: Joint 5 (wrist_roll) decrease/increase")
        print("- Y/H: Joint 6 (gripper) decrease/increase")
        print("- X: Exit program (return to start position first)")
        print("- ESC: Exit program")
        print("="*50)
        print("Note: Robot will continuously move to target positions")
        
        # Start P control loop
        p_control_loop(robot, keyboard, target_positions, start_positions, current_x, current_y, kp=0.5, control_freq=50)
        
        # Disconnect
        robot.disconnect()
        keyboard.disconnect()
        print("Program ended")
        
    except Exception as e:
        print(f"Program execution failed: {e}")
        traceback.print_exc()
        print("Please check:")
        print("1. Whether the robot is properly connected")
        print("2. Whether the USB port is correct")
        print("3. Whether you have sufficient permissions to access USB devices")
        print("4. Whether the robot is properly configured")

if __name__ == "__main__":
    main() 