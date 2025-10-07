import gymnasium as gym
import numpy as np
import sapien
import pygame
import time

# 注释掉的导入（可能用于未来的功能扩展）
#from mani_skill.envs.sapien_env import BaseEnv
#from mani_skill.utils import gym_utils
#from mani_skill.utils.wrappers import RecordEpisode

import tyro
from dataclasses import dataclass
from typing import List, Optional, Annotated, Union

@dataclass
class Args:
    """命令行参数配置类"""

    env_id: Annotated[str, tyro.conf.arg(aliases=["-e"])] = "PushCube-v1"
    """要模拟的任务环境ID"""

    obs_mode: Annotated[str, tyro.conf.arg(aliases=["-o"])] = "none"
    """观察模式"""

    robot_uids: Annotated[Optional[str], tyro.conf.arg(aliases=["-r"])] = None
    """要使用的机器人UID。可以是逗号分隔的UID列表或空字符串表示无代理。如果未提供，则默认为环境的默认机器人"""

    sim_backend: Annotated[str, tyro.conf.arg(aliases=["-b"])] = "auto"
    """使用的仿真后端。可以是'auto'、'cpu'、'gpu'"""

    reward_mode: Optional[str] = None
    """奖励模式"""

    num_envs: Annotated[int, tyro.conf.arg(aliases=["-n"])] = 1
    """运行的环境数量"""

    control_mode: Annotated[Optional[str], tyro.conf.arg(aliases=["-c"])] = None
    """控制模式"""

    render_mode: str = "rgb_array"
    """渲染模式"""

    shader: str = "default"
    """更改环境中所有相机用于渲染的着色器。默认是'default'，速度很快。也可以是'rt'用于光线追踪和生成照片级真实感渲染。也可以是'rt-fast'用于更快但质量较低的光线追踪渲染器"""

    record_dir: Optional[str] = None
    """保存录制的目录"""

    pause: Annotated[bool, tyro.conf.arg(aliases=["-p"])] = False
    """如果使用人类渲染模式，加载时自动暂停仿真"""

    quiet: bool = False
    """禁用详细输出"""

    seed: Annotated[Optional[Union[int, List[int]]], tyro.conf.arg(aliases=["-s"])] = None
    """随机动作和仿真器的种子。可以是单个整数或整数列表。默认为None（无种子）"""

def get_mapped_joints(robot):
    """
    从机器人获取当前关节位置并正确映射到目标关节

    映射关系：
    - full_joints[0,2] → current_joints[0,1] (底盘x位置和旋转)
    - full_joints[3,6,9,11,13] → current_joints[2,3,4,5,6] (第一臂关节)
    - full_joints[4,7,10,12,14] → current_joints[7,8,9,10,11] (第二臂关节)
    - full_joints[15,16] → current_joints[12,13] (夹爪关节)

    Args:
        robot: 机器人实例

    Returns:
        np.ndarray: 映射后的关节位置数组，形状与目标关节匹配
    """
    if robot is None:
        return np.zeros(16)  # 动作的默认大小

    # 获取完整关节位置
    full_joints = robot.get_qpos()

    # 如果需要，将张量转换为numpy数组
    if hasattr(full_joints, 'numpy'):
        full_joints = full_joints.numpy()

    # 处理2D张量/数组的情况
    if full_joints.ndim > 1:
        full_joints = full_joints.squeeze()

    # 创建具有正确大小的映射关节数组
    mapped_joints = np.zeros(16)

    # 根据指定映射映射关节
    if len(full_joints) >= 15:
        # 底盘关节: [0,2] → [0,1]
        mapped_joints[0] = full_joints[0]  # 底盘X位置
        mapped_joints[1] = full_joints[2]  # 底盘旋转

        # 第一臂: [3,6,9,11,13] → [2,3,4,5,6]
        mapped_joints[2] = full_joints[3]   # 旋转关节
        mapped_joints[3] = full_joints[6]   # 俯仰关节
        mapped_joints[4] = full_joints[9]   # 肘部关节
        mapped_joints[5] = full_joints[11]  # 腕部俯仰关节
        mapped_joints[6] = full_joints[13]  # 腕部滚转关节

        # 第二臂: [4,7,10,12,14] → [7,8,9,10,11]
        mapped_joints[7] = full_joints[4]   # 第二臂旋转关节
        mapped_joints[8] = full_joints[7]   # 第二臂俯仰关节
        mapped_joints[9] = full_joints[10]  # 第二臂肘部关节
        mapped_joints[10] = full_joints[12] # 第二臂腕部俯仰关节
        mapped_joints[11] = full_joints[14] # 第二臂腕部滚转关节

        # 夹爪关节: [15,16] → [12,13]
        if len(full_joints) >= 16:
            mapped_joints[12] = full_joints[15]  # 第一臂夹爪
        if len(full_joints) >= 17:
            mapped_joints[13] = full_joints[16]  # 第二臂夹爪

    return mapped_joints

def main(args: Args):
    """
    主函数：启动机器人控制演示程序

    Args:
        args: 命令行参数
    """
    pygame.init()

    # 初始化控制窗口
    screen_width, screen_height = 600, 750
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Control Window - Use keys to move")
    font = pygame.font.SysFont(None, 24)

    # 设置numpy打印选项
    np.set_printoptions(suppress=True, precision=3)
    verbose = not args.quiet

    # 处理随机种子
    if isinstance(args.seed, int):
        args.seed = [args.seed]
    if args.seed is not None:
        np.random.seed(args.seed[0])

    # 配置并行渲染设置
    parallel_in_single_scene = args.render_mode == "human"
    if args.render_mode == "human" and args.obs_mode in ["sensor_data", "rgb", "rgbd", "depth", "point_cloud"]:
        print("禁用并行单场景/GUI渲染，因为观察模式是视觉模式。将观察模式更改为state或state_dict以查看并行环境渲染")
        parallel_in_single_scene = False
    if args.render_mode == "human" and args.num_envs == 1:
        parallel_in_single_scene = False

    # 环境配置参数
    env_kwargs = dict(
        obs_mode=args.obs_mode,
        reward_mode=args.reward_mode,
        control_mode=args.control_mode,
        render_mode=args.render_mode,
        sensor_configs=dict(shader_pack=args.shader),
        human_render_camera_configs=dict(shader_pack=args.shader),
        viewer_camera_configs=dict(shader_pack=args.shader),
        num_envs=args.num_envs,
        sim_backend=args.sim_backend,
        enable_shadow=True,
        parallel_in_single_scene=parallel_in_single_scene,
    )

    # 处理机器人UID配置
    if args.robot_uids is not None:
        env_kwargs["robot_uids"] = tuple(args.robot_uids.split(","))
        if len(env_kwargs["robot_uids"]) == 1:
            env_kwargs["robot_uids"] = env_kwargs["robot_uids"][0]

    # 创建环境
    env: BaseEnv = gym.make(
        args.env_id,
        **env_kwargs
    )

    # 配置录制设置
    record_dir = args.record_dir
    if record_dir:
        record_dir = record_dir.format(env_id=args.env_id)
        env = RecordEpisode(env, record_dir, info_on_video=False, save_trajectory=False, max_steps_per_video=gym_utils.find_max_episode_steps_value(env))

    # 打印环境信息
    if verbose:
        print("观察空间", env.observation_space)
        print("动作空间", env.action_space)
        if env.unwrapped.agent is not None:
            print("控制模式", env.unwrapped.control_mode)
        print("奖励模式", env.unwrapped.reward_mode)

    # 重置环境
    obs, _ = env.reset(seed=args.seed, options=dict(reconfigure=True))
    if args.seed is not None and env.action_space is not None:
        env.action_space.seed(args.seed[0])

    # 配置渲染器
    if args.render_mode is not None:
        viewer = env.render()
        if isinstance(viewer, sapien.utils.Viewer):
            viewer.paused = args.pause
        env.render()

    # 初始化动作向量
    action = env.action_space.sample() if env.action_space is not None else None
    action = np.zeros_like(action)

    # 初始化目标关节位置（全零）
    target_joints = np.zeros_like(action)
    # 定义目标关节变化的步长
    joint_step = 0.01

    # 定义每个关节的比例控制器增益
    p_gain = np.ones_like(action)  # 默认所有增益为1.0
    # 可以在这里调整特定增益
    p_gain[0] = 1.0     # 底盘前进/后退
    p_gain[1] = 0.5     # 底盘旋转 - 较低增益以实现更平滑的转向
    p_gain[2:7] = 1.0   # 第一臂关节
    p_gain[7:12] = 1.0  # 第二臂关节
    p_gain[12:14] = 0.04  # 夹爪关节 - 较低增益以实现平滑控制

    # 获取初始关节位置（如果可用）
    current_joints = np.zeros_like(action)
    robot = None

    # 尝试获取机器人实例以直接访问
    if hasattr(env.unwrapped, "agent"):
        robot = env.unwrapped.agent.robot
    elif hasattr(env.unwrapped, "agents") and len(env.unwrapped.agents) > 0:
        robot = env.unwrapped.agents[0]  # 如果存在多个机器人，获取第一个

    print("机器人实例", robot)

    # 获取正确映射的关节
    current_joints = get_mapped_joints(robot)

    # 确保target_joints是与current_joints相同形状的numpy数组
    target_joints = np.zeros_like(current_joints)

    # 添加预热阶段的步骤计数器
    step_counter = 0
    warmup_steps = 50  # 预热步数
    
    # 主控制循环
    while True:
        # 处理pygame事件
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                env.close()
                return

        # 获取按键状态
        keys = pygame.key.get_pressed()

        # 基于按键更新目标关节位置 - 仅在预热阶段后
        if step_counter >= warmup_steps:
            # 底盘前进/后退 - 直接控制
            if keys[pygame.K_w]:
                action[0] = 0.1  # 前进
            elif keys[pygame.K_s]:
                action[0] = -0.1  # 后退
            else:
                action[0] = 0.0  # 停止前进/后退运动

            # 底盘转向 - 使用目标关节和P控制
            if keys[pygame.K_a]:
                target_joints[1] += joint_step*2  # 左转
            elif keys[pygame.K_d]:
                target_joints[1] -= joint_step*2  # 右转

            # 第一臂控制 - 使用目标关节和P控制
            if keys[pygame.K_7]:
                target_joints[2] += joint_step  # 第一臂旋转关节正转
            if keys[pygame.K_y]:
                target_joints[2] -= joint_step  # 第一臂旋转关节反转
            if keys[pygame.K_8]:
                target_joints[3] += joint_step  # 第一臂俯仰关节正转
            if keys[pygame.K_u]:
                target_joints[3] -= joint_step  # 第一臂俯仰关节反转
            if keys[pygame.K_9]:
                target_joints[4] += joint_step  # 第一臂肘部关节正转
            if keys[pygame.K_i]:
                target_joints[4] -= joint_step  # 第一臂肘部关节反转
            if keys[pygame.K_0]:
                target_joints[5] += joint_step  # 第一臂腕部俯仰关节正转
            if keys[pygame.K_o]:
                target_joints[5] -= joint_step  # 第一臂腕部俯仰关节反转
            if keys[pygame.K_MINUS]:
                target_joints[6] += joint_step  # 第一臂腕部滚转关节正转
            if keys[pygame.K_p]:
                target_joints[6] -= joint_step  # 第一臂腕部滚转关节反转

            # 第二臂控制 - 使用目标关节和P控制
            if keys[pygame.K_h]:
                target_joints[7] += joint_step  # 第二臂旋转关节正转
            if keys[pygame.K_n]:
                target_joints[7] -= joint_step  # 第二臂旋转关节反转
            if keys[pygame.K_j]:
                target_joints[8] += joint_step  # 第二臂俯仰关节正转
            if keys[pygame.K_m]:
                target_joints[8] -= joint_step  # 第二臂俯仰关节反转
            if keys[pygame.K_k]:
                target_joints[9] += joint_step  # 第二臂肘部关节正转
            if keys[pygame.K_COMMA]:
                target_joints[9] -= joint_step  # 第二臂肘部关节反转
            if keys[pygame.K_l]:
                target_joints[10] += joint_step  # 第二臂腕部俯仰关节正转
            if keys[pygame.K_PERIOD]:
                target_joints[10] -= joint_step  # 第二臂腕部俯仰关节反转
            if keys[pygame.K_SEMICOLON]:
                target_joints[11] += joint_step  # 第二臂腕部滚转关节正转
            if keys[pygame.K_SLASH]:
                target_joints[11] -= joint_step  # 第二臂腕部滚转关节反转

            # 夹爪控制 - 在张开和闭合之间切换
            if keys[pygame.K_f]:
                # 切换第一个夹爪（索引12）
                if target_joints[12] < 0.4:  # 如果关闭或部分关闭
                    target_joints[12] = 2.5  # 张开
                else:
                    target_joints[12] = 0.1  # 关闭
                # 添加小延迟以防止多次切换
                pygame.time.delay(200)

            if keys[pygame.K_g]:
                # 切换第二个夹爪（索引13）
                if target_joints[13] < 0.4:  # 如果关闭或部分关闭
                    target_joints[13] = 2.5  # 张开
                else:
                    target_joints[13] = 0.1  # 关闭
                # 添加小延迟以防止多次切换
                pygame.time.delay(200)
        
        # Get current joint positions using our mapping function
        current_joints = get_mapped_joints(robot)
        
        # Simple P controller for arm joints only (not base)
        if step_counter < warmup_steps:
            action = np.zeros_like(action)
        else:
            # Apply P control to turning (index 1) and arm joints (indices 2-11) and grippers (indices 12-13)
            # Base forward/backward (index 0) is already set directly above
            for i in range(1, len(action)):
                action[i] = p_gain[i] * (target_joints[i] - current_joints[i])
        
        # Clip actions to be within reasonable bound
        
        screen.fill((0, 0, 0))
        
        text = font.render("Controls:", True, (255, 255, 255))
        screen.blit(text, (10, 10))
        
        # Add warmup status to display
        if step_counter < warmup_steps:
            warmup_text = font.render(f"WARMUP: {step_counter}/{warmup_steps} steps", True, (255, 0, 0))
            screen.blit(warmup_text, (300, 10))
        
        control_texts = [
            "W/S: joint[0] (+/-)",
            "A/D: joint[1] (+/-)",
            "Y/U: joint[2] (+/-)",
            "8/I: joint[3] (+/-)",
            "9/O: joint[4] (+/-)",
            "0/P: joint[5] (+/-)",
            "-/[: joint[6] (+/-)",
            "H/N: joint[7] (+/-)",
            "J/M: joint[8] (+/-)",
            "K/,: joint[9] (+/-)",
            "L/.: joint[10] (+/-)",
            ";/?: joint[11] (+/-)",
            "R: Reset targets to current"
        ]
        
        col_height = len(control_texts) // 2 + len(control_texts) % 2
        for i, txt in enumerate(control_texts):
            col = 0 if i < col_height else 1
            row = i if i < col_height else i - col_height
            ctrl_text = font.render(txt, True, (255, 255, 255))
            screen.blit(ctrl_text, (10 + col * 200, 40 + row * 25))
        
        # Display full joints (before mapping)
        y_pos = 40 + col_height * 30 + 10
        
        # Get full joint positions
        full_joints = robot.get_qpos() if robot is not None else np.zeros(17)
        
        # Convert tensor to numpy array if needed
        if hasattr(full_joints, 'numpy'):
            full_joints = full_joints.numpy()
        
        # Handle case where it's a 2D tensor/array
        if full_joints.ndim > 1:
            full_joints = full_joints.squeeze()
            
        # Display full joints in two rows
        full_joints_text1 = font.render(
            f"Full Joints (1-8): {np.round(full_joints[:8], 2)}", 
            True, (255, 150, 0)
        )
        screen.blit(full_joints_text1, (10, y_pos))
        y_pos += 25
        
        full_joints_text2 = font.render(
            f"Full Joints (9-17): {np.round(full_joints[8:], 2)}", 
            True, (255, 150, 0)
        )
        screen.blit(full_joints_text2, (10, y_pos))
        y_pos += 30
        
        # Display current joint positions in three logical groups
        # Group 1: Base control [0,1]
        base_joints = current_joints[0:2]
        base_text = font.render(
            f"Base [0,1]: {np.round(base_joints, 2)}", 
            True, (255, 255, 0)
        )
        screen.blit(base_text, (10, y_pos))
        
        # Group 2: First arm [2,3,4,5,6]
        y_pos += 25
        arm1_joints = current_joints[2:7]
        arm1_text = font.render(
            f"Arm 1 [2,3,4,5,6]: {np.round(arm1_joints, 2)}", 
            True, (255, 255, 0)
        )
        screen.blit(arm1_text, (10, y_pos))
        
        # Group 3: Second arm [7,8,9,10,11]
        y_pos += 25
        arm2_joints = current_joints[7:12]
        arm2_text = font.render(
            f"Arm 2 [7,8,9,10,11]: {np.round(arm2_joints, 2)}", 
            True, (255, 255, 0)
        )
        screen.blit(arm2_text, (10, y_pos))
        
        # Display target joint positions in three logical groups
        y_pos += 35
        
        # Group 1: Base control [0,1]
        base_targets = target_joints[0:2]
        base_target_text = font.render(
            f"Base Target [0,1]: {np.round(base_targets, 2)}", 
            True, (0, 255, 0)
        )
        screen.blit(base_target_text, (10, y_pos))
        
        # Group 2: First arm [2,3,4,5,6]
        y_pos += 25
        arm1_targets = target_joints[2:7]
        arm1_target_text = font.render(
            f"Arm 1 Target [2,3,4,5,6]: {np.round(arm1_targets, 2)}", 
            True, (0, 255, 0)
        )
        screen.blit(arm1_target_text, (10, y_pos))
        
        # Group 3: Second arm [7,8,9,10,11]
        y_pos += 25
        arm2_targets = target_joints[7:12]
        arm2_target_text = font.render(
            f"Arm 2 Target [7,8,9,10,11]: {np.round(arm2_targets, 2)}", 
            True, (0, 255, 0)
        )
        screen.blit(arm2_target_text, (10, y_pos))
        
        # Group 4: Grippers [12,13]
        y_pos += 25
        gripper_targets = target_joints[12:14]
        gripper_target_text = font.render(
            f"Grippers Target [12,13]: {np.round(gripper_targets, 2)}", 
            True, (0, 255, 0)
        )
        screen.blit(gripper_target_text, (10, y_pos))
        
        # Display current action values (velocities) in three logical groups
        y_pos += 35
        
        # Group 1: Base control [0,1]
        base_actions = action[0:2]
        base_action_text = font.render(
            f"Base Velocity [0,1]: {np.round(base_actions, 2)}", 
            True, (255, 255, 255)
        )
        screen.blit(base_action_text, (10, y_pos))
        
        # Group 2: First arm [2,3,4,5,6]
        y_pos += 25
        arm1_actions = action[2:7]
        arm1_action_text = font.render(
            f"Arm 1 Velocity [2,3,4,5,6]: {np.round(arm1_actions, 2)}", 
            True, (255, 255, 255)
        )
        screen.blit(arm1_action_text, (10, y_pos))
        
        # Group 3: Second arm [7,8,9,10,11]
        y_pos += 25
        arm2_actions = action[7:12]
        arm2_action_text = font.render(
            f"Arm 2 Velocity [7,8,9,10,11]: {np.round(arm2_actions, 2)}", 
            True, (255, 255, 255)
        )
        screen.blit(arm2_action_text, (10, y_pos))
        
        # Group 4: Grippers [12,13]
        y_pos += 25
        gripper_actions = action[12:14]
        gripper_action_text = font.render(
            f"Grippers Velocity [12,13]: {np.round(gripper_actions, 2)}", 
            True, (255, 255, 255)
        )
        screen.blit(gripper_action_text, (10, y_pos))
        
        pygame.display.flip()
        
        obs, reward, terminated, truncated, info = env.step(action)
        step_counter += 1
        
        if args.render_mode is not None:
            env.render()
        
        time.sleep(0.01)
        
        if args.render_mode is None or args.render_mode != "human":
            if (terminated | truncated).any():
                break
    
    pygame.quit()
    env.close()

    if record_dir:
        print(f"Saving video to {record_dir}")


if __name__ == "__main__":
    parsed_args = tyro.cli(Args)
    main(parsed_args)
