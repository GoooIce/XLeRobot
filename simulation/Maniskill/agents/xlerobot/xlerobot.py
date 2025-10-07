from copy import deepcopy
from typing import Dict, Tuple

import numpy as np
import sapien
import sapien.physx as physx
import torch

from mani_skill import PACKAGE_ASSET_DIR
from mani_skill.agents.base_agent import BaseAgent, Keyframe
from mani_skill.agents.controllers import *
from mani_skill.agents.registration import register_agent
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import common, sapien_utils
from mani_skill.utils.structs import Pose
from mani_skill.utils.structs.actor import Actor
from mani_skill.utils.structs.link import Link
from mani_skill.utils.structs.types import Array

FETCH_WHEELS_COLLISION_BIT = 30
"""Fetch机器人轮子连杆的碰撞位掩码"""
FETCH_BASE_COLLISION_BIT = 31
"""Fetch机器人基座的碰撞位掩码"""


@register_agent()
class Xlerobot(BaseAgent):
    """
    Xlerobot双臂机器人类

    这是一个基于Maniskill框架的双臂机器人实现，支持：
    - 双机械臂控制
    - 双夹爪操作
    - 底盘移动
    - 头部云台控制
    - 多相机视觉系统
    """
    uid = "xlerobot"
    urdf_path = f"{PACKAGE_ASSET_DIR}/robots/xlerobot/xlerobot.urdf"
    urdf_config = dict(
        _materials=dict(
            gripper=dict(static_friction=2.0, dynamic_friction=2.0, restitution=0.0)
        ),
        link=dict(
            Fixed_Jaw=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            Moving_Jaw=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            Fixed_Jaw_2=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
            Moving_Jaw_2=dict(
                material="gripper", patch_radius=0.1, min_patch_radius=0.1
            ),
        ),
    )

    keyframes = dict(
        rest=Keyframe(
            pose=sapien.Pose(),
            # qpos 与 gen_spawn_positions_xlerobot.py 中使用的实际映射关系：
            # 索引:   0  1  2  3   4   5    6     7     8    9     10    11   12   13    14   15  16
            # 关节: [x, y, r, arm1, arm2, head, arm1, arm2, head, arm1, arm2, arm1, arm2, arm1, arm2, g1, g2]
            # 其中机械臂索引为：
            # - 第一臂: [3,6,9,11,13] (5个关节: Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll)
            # - 第二臂: [4,7,10,12,14] (5个关节: Rotation_2, Pitch_2, Elbow_2, Wrist_Pitch_2, Wrist_Roll_2)
            # - 底盘: [0,1,2] (x, y, rotation)
            # - 头部: [5,8] (pan, tilt)
            # - 夹爪: [15,16] (Jaw, Jaw_2)
            qpos=np.array([0, 0, 0,           # [0,1,2] 底盘: x, y, rotation
                          0, 0,              # [3,4] arm1, arm2 的第一个值
                          0,                 # [5] 头部 pan
                          3.14, 3.14,      # [6,7] arm1, arm2 的第二个值
                          0,                 # [8] 头部 tilt
                          3.14, 3.14,      # [9,10] arm1, arm2 的第三个值
                          0, 0,              # [11,12] arm1, arm2 的第四个值
                          1.57, 1.57,          # [13,14] arm1, arm2 的第五个值
                          0, 0]),            # [15,16] 夹爪: Jaw, Jaw_2
        )
    )

    @property
    def _sensor_configs(self):
        """
        配置双臂Fetch机器人的相机系统

        相机配置包括：
        - head_camera: 头部主相机，用于工作区概览
        - right_arm_camera: 第一臂的手部相机，用于精确操作
        - left_arm_camera: 第二臂的手部相机，用于精确操作
        """
        return [
            # 头部相机 - 主工作区概览相机
            CameraConfig(
                uid="fetch_head",
                pose=Pose.create_from_pq([0, 0, 0], [1, 0, 0, 0]),  # 单位变换矩阵
                width=256,
                height=256,
                fov=1.6,  # 宽视场角，用于工作区监控
                near=0.01,
                far=100,
                entity_uid="head_camera_link",  # 安装到专用头部相机连杆
            ),

            # 第一臂相机 - 手部安装相机，用于精确操作
            CameraConfig(
                uid="fetch_right_arm_camera",
                pose=Pose.create_from_pq([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),  # 单位变换矩阵
                width=128,
                height=128,
                fov=1.3,  # 视场角，用于操作监控
                near=0.01,
                far=100,
                entity_uid="Right_Arm_Camera",  # 安装到专用相机连杆
            ),

            # 第二臂相机 - 手部安装相机，用于精确操作
            CameraConfig(
                uid="fetch_left_arm_camera",
                pose=Pose.create_from_pq([0.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0]),  # 单位变换矩阵
                width=128,
                height=128,
                fov=1.3,  # 视场角，用于操作监控
                near=0.01,
                far=100,
                entity_uid="Left_Arm_Camera",  # 安装到专用相机连杆
            ),
        ]

    def __init__(self, *args, **kwargs):
        # 第一臂配置
        self.arm_joint_names = [
            "Rotation",    # 旋转关节
            "Pitch",       # 俯仰关节
            "Elbow",       # 肘部关节
            "Wrist_Pitch", # 腕部俯仰关节
            "Wrist_Roll",  # 腕部滚转关节
        ]
        self.arm_stiffness = 2e4      # 机械臂刚度
        self.arm_damping = 1e2       # 机械臂阻尼
        self.arm_force_limit = 250   # 机械臂力限值

        # 第一臂夹爪配置
        self.gripper_joint_names = [
            "Jaw",  # 夹爪关节
        ]
        self.gripper_stiffness = 50      # 夹爪刚度
        self.gripper_damping = 1e2       # 夹爪阻尼
        self.gripper_force_limit = 2.8  # 夹爪力限值

        self.ee_link_name = "Fixed_Jaw"  # 末端执行器连杆名称

        # 第二臂配置
        self.arm2_joint_names = [
            "Rotation_2",    # 第二臂旋转关节
            "Pitch_2",       # 第二臂俯仰关节
            "Elbow_2",       # 第二臂肘部关节
            "Wrist_Pitch_2", # 第二臂腕部俯仰关节
            "Wrist_Roll_2",  # 第二臂腕部滚转关节
        ]
        self.arm2_stiffness = 2e4      # 第二臂刚度
        self.arm2_damping = 1e2       # 第二臂阻尼
        self.arm2_force_limit = 250   # 第二臂力限值

        # 第二臂夹爪配置
        self.gripper2_joint_names = [
            "Jaw_2",  # 第二臂夹爪关节
        ]
        self.gripper2_stiffness = 50      # 第二臂夹爪刚度
        self.gripper2_damping = 1e2       # 第二臂夹爪阻尼
        self.gripper2_force_limit = 2.8  # 第二臂夹爪力限值

        self.ee2_link_name = "Fixed_Jaw_2"  # 第二臂末端执行器连杆名称

        # 身体（头部）关节配置
        self.body_joint_names = [
            "head_pan_joint",   # 头部水平转动关节
            "head_tilt_joint",  # 头部俯仰关节
        ]
        self.body_stiffness = 1e4      # 身体刚度
        self.body_damping = 1e2       # 身体阻尼
        self.body_force_limit = 200   # 身体力限值

        # 底盘关节配置
        self.base_joint_names = [
            "root_x_axis_joint",      # X轴移动关节
            "root_y_axis_joint",      # Y轴移动关节
            "root_z_rotation_joint",  # Z轴旋转关节
        ]

        super().__init__(*args, **kwargs)

    @property
    def _controller_configs(self):
        # -------------------------------------------------------------------------- #
        # 机械臂控制器配置
        # -------------------------------------------------------------------------- #
        # 机械臂关节位置控制器 - 绝对位置控制
        arm_pd_joint_pos = PDJointPosControllerConfig(
            self.arm_joint_names,    # 关节名称列表
            None,                    # 位置下限
            None,                    # 位置上限
            self.arm_stiffness,      # 刚度系数
            self.arm_damping,        # 阻尼系数
            self.arm_force_limit,    # 力限值
            normalize_action=False,  # 不归一化动作
        )

        # 机械臂关节位置控制器 - 增量位置控制
        arm_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm_joint_names,    # 关节名称列表
            -0.1,                    # 位置增量下限
            0.1,                     # 位置增量上限
            self.arm_stiffness,      # 刚度系数
            self.arm_damping,        # 阻尼系数
            self.arm_force_limit,    # 力限值
            use_delta=True,          # 使用增量模式
        )

        # 机械臂关节位置控制器 - 带目标的增量位置控制
        arm_pd_joint_target_delta_pos = deepcopy(arm_pd_joint_delta_pos)
        arm_pd_joint_target_delta_pos.use_target = True  # 启用目标位置跟踪

        # 末端执行器位置控制器 - 增量位置控制
        arm_pd_ee_delta_pos = PDEEPosControllerConfig(
            joint_names=self.arm_joint_names,  # 关节名称列表
            pos_lower=-0.1,                    # 位置增量下限
            pos_upper=0.1,                     # 位置增量上限
            stiffness=self.arm_stiffness,      # 刚度系数
            damping=self.arm_damping,          # 阻尼系数
            force_limit=self.arm_force_limit,  # 力限值
            ee_link=self.ee_link_name,         # 末端执行器连杆
            urdf_path=self.urdf_path,          # URDF文件路径
        )

        # 末端执行器位姿控制器 - 增量位姿控制
        arm_pd_ee_delta_pose = PDEEPoseControllerConfig(
            joint_names=self.arm_joint_names,  # 关节名称列表
            pos_lower=-0.1,                    # 位置增量下限
            pos_upper=0.1,                     # 位置增量上限
            rot_lower=-0.1,                    # 旋转增量下限
            rot_upper=0.1,                     # 旋转增量上限
            stiffness=self.arm_stiffness,      # 刚度系数
            damping=self.arm_damping,          # 阻尼系数
            force_limit=self.arm_force_limit,  # 力限值
            ee_link=self.ee_link_name,         # 末端执行器连杆
            urdf_path=self.urdf_path,          # URDF文件路径
        )

        # 末端执行器位置控制器 - 带目标的增量位置控制
        arm_pd_ee_target_delta_pos = deepcopy(arm_pd_ee_delta_pos)
        arm_pd_ee_target_delta_pos.use_target = True  # 启用目标位置跟踪
        arm_pd_ee_target_delta_pose = deepcopy(arm_pd_ee_delta_pose)
        arm_pd_ee_target_delta_pose.use_target = True  # 启用目标位姿跟踪

        # 末端执行器位姿控制器 - 对齐模式的增量位姿控制（用于人机交互/遥操作）
        arm_pd_ee_delta_pose_align = deepcopy(arm_pd_ee_delta_pose)
        arm_pd_ee_delta_pose_align.frame = "ee_align"  # 设置坐标系对齐模式

        # 机械臂关节速度控制器
        arm_pd_joint_vel = PDJointVelControllerConfig(
            self.arm_joint_names,    # 关节名称列表
            -1.0,                    # 速度下限
            1.0,                     # 速度上限
            self.arm_damping,        # 阻尼系数（可能需要单独调整）
            self.arm_force_limit,    # 力限值
        )

        # 机械臂关节位置和速度控制器
        arm_pd_joint_pos_vel = PDJointPosVelControllerConfig(
            self.arm_joint_names,    # 关节名称列表
            None,                    # 位置下限
            None,                    # 位置上限
            self.arm_stiffness,      # 刚度系数
            self.arm_damping,        # 阻尼系数
            self.arm_force_limit,    # 力限值
            normalize_action=True,   # 归一化动作
        )

        # 机械臂关节位置和速度控制器 - 增量模式
        arm_pd_joint_delta_pos_vel = PDJointPosVelControllerConfig(
            self.arm_joint_names,    # 关节名称列表
            -0.1,                    # 增量下限
            0.1,                     # 增量上限
            self.arm_stiffness,      # 刚度系数
            self.arm_damping,        # 阻尼系数
            self.arm_force_limit,    # 力限值
            use_delta=True,          # 使用增量模式
        )

        # -------------------------------------------------------------------------- #
        # 夹爪控制器配置
        # -------------------------------------------------------------------------- #
        # 对于SO100夹爪，我们使用常规的PD关节位置控制器而不是模拟控制器
        gripper_pd_joint_pos = PDJointPosControllerConfig(
            self.gripper_joint_names,  # 夹爪关节名称列表
            -20,                       # 闭合位置（根据需要更新）
            20,                        # 张开位置（根据需要更新）
            self.gripper_stiffness,    # 夹爪刚度
            self.gripper_damping,      # 夹爪阻尼
            self.gripper_force_limit,  # 夹爪力限值
        )

        # -------------------------------------------------------------------------- #
        # 身体（头部）控制器配置
        # -------------------------------------------------------------------------- #
        # 身体关节增量位置控制器
        body_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.body_joint_names,    # 身体关节名称列表
            -0.1,                     # 增量下限
            0.1,                      # 增量上限
            self.body_stiffness,      # 身体刚度
            self.body_damping,        # 身体阻尼
            self.body_force_limit,    # 身体力限值
            use_delta=True,           # 使用增量模式
        )

        # 高刚度身体关节位置控制器 - 用于保持身体不动
        stiff_body_pd_joint_pos = PDJointPosControllerConfig(
            self.body_joint_names,    # 身体关节名称列表
            None,                     # 位置下限
            None,                     # 位置上限
            1e5,                      # 高刚度
            1e5,                      # 高阻尼
            1e5,                      # 高力限值
            normalize_action=False,   # 不归一化动作
        )

        # -------------------------------------------------------------------------- #
        # 底盘控制器配置
        # -------------------------------------------------------------------------- #
        # 底盘前向速度控制器
        base_pd_joint_vel = PDBaseForwardVelControllerConfig(
            self.base_joint_names,    # 底盘关节名称列表
            lower=[-1, -3.14],       # 速度下限 [x轴速度, 旋转速度]
            upper=[1, 3.14],         # 速度上限 [x轴速度, 旋转速度]
            damping=1000,             # 阻尼系数
            force_limit=500,          # 力限值
        )

        # 添加第二臂的控制器配置
        # 第二臂关节位置控制器 - 绝对位置控制
        arm2_pd_joint_pos = PDJointPosControllerConfig(
            self.arm2_joint_names,   # 第二臂关节名称列表
            None,                    # 位置下限
            None,                    # 位置上限
            self.arm2_stiffness,     # 第二臂刚度
            self.arm2_damping,       # 第二臂阻尼
            self.arm2_force_limit,   # 第二臂力限值
            normalize_action=False,  # 不归一化动作
        )

        # 第二臂关节位置控制器 - 增量位置控制
        arm2_pd_joint_delta_pos = PDJointPosControllerConfig(
            self.arm2_joint_names,   # 第二臂关节名称列表
            -0.1,                    # 增量下限
            0.1,                     # 增量上限
            self.arm2_stiffness,     # 第二臂刚度
            self.arm2_damping,       # 第二臂阻尼
            self.arm2_force_limit,   # 第二臂力限值
            use_delta=True,          # 使用增量模式
        )

        # 第二臂夹爪控制器 - 使用常规PD关节位置控制器而不是模拟控制器
        gripper2_pd_joint_pos = PDJointPosControllerConfig(
            self.gripper2_joint_names,  # 第二臂夹爪关节名称列表
            -20,                        # 闭合位置（根据需要更新）
            20,                         # 张开位置（根据需要更新）
            self.gripper2_stiffness,    # 第二臂夹爪刚度
            self.gripper2_damping,      # 第二臂夹爪阻尼
            self.gripper2_force_limit,  # 第二臂夹爪力限值
        )

        # 创建双臂控制器配置字典
        controller_configs = dict(
            # 第一臂增量位置控制器组合
            pd_joint_delta_pos=dict(
                arm=arm_pd_joint_delta_pos,      # 第一臂控制器
                gripper=gripper_pd_joint_pos,    # 第一臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第一臂绝对位置控制器组合
            pd_joint_pos=dict(
                arm=arm_pd_joint_pos,            # 第一臂控制器
                gripper=gripper_pd_joint_pos,    # 第一臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第一臂带目标的增量位置控制器组合
            # TODO(jigu): 如何为以下控制器添加边界
            pd_joint_target_delta_pos=dict(
                arm=arm_pd_joint_target_delta_pos,  # 第一臂控制器
                gripper=gripper_pd_joint_pos,       # 第一臂夹爪控制器
                body=body_pd_joint_delta_pos,       # 身体控制器
                base=base_pd_joint_vel,             # 底盘控制器
            ),

            # 第一臂速度控制器组合 - 使用时需要小心
            pd_joint_vel=dict(
                arm=arm_pd_joint_vel,            # 第一臂速度控制器
                gripper=gripper_pd_joint_pos,    # 第一臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第一臂位置速度控制器组合
            pd_joint_pos_vel=dict(
                arm=arm_pd_joint_pos_vel,        # 第一臂位置速度控制器
                gripper=gripper_pd_joint_pos,    # 第一臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第一臂增量位置速度控制器组合
            pd_joint_delta_pos_vel=dict(
                arm=arm_pd_joint_delta_pos_vel,  # 第一臂增量位置速度控制器
                gripper=gripper_pd_joint_pos,    # 第一臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第一臂增量位置高刚度身体控制器组合
            pd_joint_delta_pos_stiff_body=dict(
                arm=arm_pd_joint_delta_pos,      # 第一臂控制器
                gripper=gripper_pd_joint_pos,    # 第一臂夹爪控制器
                body=stiff_body_pd_joint_pos,    # 高刚度身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第二臂默认控制器配置
            pd_joint_delta_pos_arm2=dict(
                arm=arm2_pd_joint_delta_pos,     # 第二臂控制器
                gripper=gripper2_pd_joint_pos,   # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第二臂绝对位置控制器组合
            pd_joint_pos_arm2=dict(
                arm=arm2_pd_joint_pos,           # 第二臂控制器
                gripper=gripper2_pd_joint_pos,   # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第二臂带目标的增量位置控制器组合
            pd_joint_target_delta_pos_arm2=dict(
                arm=arm_pd_joint_target_delta_pos,  # 第一臂控制器
                gripper=gripper2_pd_joint_pos,      # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,       # 身体控制器
                base=base_pd_joint_vel,             # 底盘控制器
            ),

            # 第二臂速度控制器组合
            pd_joint_vel_arm2=dict(
                arm=arm_pd_joint_vel,            # 第一臂速度控制器
                gripper=gripper2_pd_joint_pos,    # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第二臂位置速度控制器组合
            pd_joint_pos_vel_arm2=dict(
                arm=arm_pd_joint_pos_vel,        # 第一臂位置速度控制器
                gripper=gripper2_pd_joint_pos,   # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第二臂增量位置速度控制器组合
            pd_joint_delta_pos_vel_arm2=dict(
                arm=arm_pd_joint_delta_pos_vel,  # 第一臂增量位置速度控制器
                gripper=gripper2_pd_joint_pos,   # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 第二臂增量位置高刚度身体控制器组合
            pd_joint_delta_pos_stiff_body_arm2=dict(
                arm=arm2_pd_joint_delta_pos,     # 第二臂控制器
                gripper=gripper2_pd_joint_pos,   # 第二臂夹爪控制器
                body=stiff_body_pd_joint_pos,   # 高刚度身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),

            # 双臂默认控制器配置
            pd_joint_delta_pos_dual_arm=dict(
                base=base_pd_joint_vel,          # 底盘控制器
                arm1=arm_pd_joint_delta_pos,     # 第一臂控制器
                arm2=arm2_pd_joint_delta_pos,    # 第二臂控制器
                gripper1=gripper_pd_joint_pos,   # 第一臂夹爪控制器
                gripper2=gripper2_pd_joint_pos,  # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
            ),

            # 双臂绝对位置控制器组合
            pd_joint_pos_dual_arm=dict(
                arm1=arm_pd_joint_pos,           # 第一臂控制器
                arm2=arm2_pd_joint_pos,          # 第二臂控制器
                gripper1=gripper_pd_joint_pos,   # 第一臂夹爪控制器
                gripper2=gripper2_pd_joint_pos,  # 第二臂夹爪控制器
                body=body_pd_joint_delta_pos,    # 身体控制器
                base=base_pd_joint_vel,          # 底盘控制器
            ),
        )

        # 深拷贝控制器配置，防止用户修改原始配置
        return deepcopy(controller_configs)

    def _after_init(self):
        """
        初始化后处理：获取和设置机器人各连杆的引用
        """
        # 第一臂的连杆和夹爪设置
        self.finger1_link: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Fixed_Jaw"  # 固定夹爪
        )
        self.finger2_link: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Moving_Jaw"  # 活动夹爪
        )
        self.finger1_tip: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Fixed_Jaw_tip"  # 固定夹爪尖端
        )
        self.finger2_tip: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Moving_Jaw_tip"  # 活动夹爪尖端
        )
        self.tcp: Link = self.finger1_link  # 工具中心点（第一臂）

        # 第二臂的连杆和夹爪设置
        self.finger1_link_2: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Fixed_Jaw_2"  # 第二臂固定夹爪
        )
        self.finger2_link_2: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Moving_Jaw_2"  # 第二臂活动夹爪
        )
        self.finger1_tip_2: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Fixed_Jaw_2_tip"  # 第二臂固定夹爪尖端
        )
        self.finger2_tip_2: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "Moving_Jaw_2_tip"  # 第二臂活动夹爪尖端
        )
        self.tcp_2: Link = self.finger1_link_2  # 工具中心点（第二臂）

        # 底盘连杆设置
        self.base_link: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "base_link"  # 基座连杆
        )
        self.top_base_link: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "top_base_link"  # 顶部基座连杆
        )

        # 设置轮子和底盘的碰撞组
        for link in [self.top_base_link]:
            link.set_collision_group_bit(
                group=2, bit_idx=FETCH_WHEELS_COLLISION_BIT, bit=1  # 轮子碰撞组
            )
        self.base_link.set_collision_group_bit(
            group=2, bit_idx=FETCH_BASE_COLLISION_BIT, bit=1  # 底盘碰撞组
        )

        # 头部相机连杆
        self.head_camera_link: Link = sapien_utils.get_obj_by_name(
            self.robot.get_links(), "head_camera_link"  # 头部相机连杆
        )

        # 接触力查询缓存
        self.queries: Dict[
            str, Tuple[physx.PhysxGpuContactPairImpulseQuery, Tuple[int]]
        ] = dict()

    def get_arm_joint_indices(self):
        """
        根据当前qpos映射获取每个机械臂的关节索引

        Returns:
            dict: 包含'arm1', 'arm2', 'base', 'head', 'grippers'键的字典
        """
        return {
            'base': [0, 1, 2],              # x, y, rotation（底盘）
            'arm1': [3, 6, 9, 11, 13],      # 第一臂：Rotation, Pitch, Elbow, Wrist_Pitch, Wrist_Roll
            'arm2': [4, 7, 10, 12, 14],     # 第二臂：Rotation_2, Pitch_2, Elbow_2, Wrist_Pitch_2, Wrist_Roll_2
            'head': [5, 8],                 # 头部：pan, tilt（水平转动，俯仰）
            'grippers': [15, 16]            # 夹爪：Jaw, Jaw_2
        }

    def map_full_joints_to_current(self, full_joints):
        """
        将完整的关节数组（17个元素）映射到当前的关节顺序

        根据规范进行映射：
        - full_joints[0,1,2] → current_joints[0,1,2]（底盘x位置、y位置、旋转）
        - full_joints[3,6,9,11,13] → current_joints[5,6,7,8,9]（第一臂关节）
        - full_joints[4,7,10,12,14] → current_joints[10,11,12,13,14]（第二臂关节）
        - full_joints[15] → current_joints[15]（第一臂夹爪）
        - full_joints[16] → current_joints[16]（第二臂夹爪）
        - full_joints[5,8] → current_joints[3,4]（头部关节）

        Args:
            full_joints: 形状为(..., 17)的数组，包含原始关节顺序

        Returns:
            current_joints: 形状为(..., 17)的数组，包含当前关节顺序
        """
        if torch.is_tensor(full_joints):
            current_joints = torch.zeros_like(full_joints)
        else:
            current_joints = np.zeros_like(full_joints)

        # 底盘关节：x, y, rotation
        current_joints[..., 0] = full_joints[..., 0]  # 底盘x位置
        current_joints[..., 1] = full_joints[..., 1]  # 底盘y位置
        current_joints[..., 2] = full_joints[..., 2]  # 底盘旋转

        # 头部关节
        current_joints[..., 3] = full_joints[..., 5]  # 头部水平转动
        current_joints[..., 4] = full_joints[..., 8]  # 头部俯仰

        # 第一臂关节：[3,6,9,11,13] → [5,6,7,8,9]
        current_joints[..., 5] = full_joints[..., 3]   # 旋转关节
        current_joints[..., 6] = full_joints[..., 6]   # 俯仰关节
        current_joints[..., 7] = full_joints[..., 9]   # 肘部关节
        current_joints[..., 8] = full_joints[..., 11]  # 腕部俯仰关节
        current_joints[..., 9] = full_joints[..., 13]  # 腕部滚转关节

        # 第二臂关节：[4,7,10,12,14] → [10,11,12,13,14]
        current_joints[..., 10] = full_joints[..., 4]   # 第二臂旋转关节
        current_joints[..., 11] = full_joints[..., 7]   # 第二臂俯仰关节
        current_joints[..., 12] = full_joints[..., 10]  # 第二臂肘部关节
        current_joints[..., 13] = full_joints[..., 12]  # 第二臂腕部俯仰关节
        current_joints[..., 14] = full_joints[..., 14]  # 第二臂腕部滚转关节

        # 夹爪关节
        current_joints[..., 15] = full_joints[..., 15]  # 第一臂夹爪
        current_joints[..., 16] = full_joints[..., 16]  # 第二臂夹爪

        return current_joints

    def map_current_joints_to_full(self, current_joints):
        """
        将当前关节顺序映射回完整的关节数组顺序
        这是map_full_joints_to_current的逆函数

        Args:
            current_joints: 形状为(..., 17)的数组，包含当前关节顺序

        Returns:
            full_joints: 形状为(..., 17)的数组，包含原始关节顺序
        """
        if torch.is_tensor(current_joints):
            full_joints = torch.zeros_like(current_joints)
        else:
            full_joints = np.zeros_like(current_joints)

        # 底盘关节
        full_joints[..., 0] = current_joints[..., 0]   # 底盘x位置
        full_joints[..., 1] = current_joints[..., 1]   # 底盘y位置
        full_joints[..., 2] = current_joints[..., 2]   # 底盘旋转

        # 第一臂关节：[5,6,7,8,9] → [3,6,9,11,13]
        full_joints[..., 3] = current_joints[..., 5]   # 旋转关节
        full_joints[..., 6] = current_joints[..., 6]   # 俯仰关节
        full_joints[..., 9] = current_joints[..., 7]   # 肘部关节
        full_joints[..., 11] = current_joints[..., 8]  # 腕部俯仰关节
        full_joints[..., 13] = current_joints[..., 9]  # 腕部滚转关节

        # 第二臂关节：[10,11,12,13,14] → [4,7,10,12,14]
        full_joints[..., 4] = current_joints[..., 10]  # 第二臂旋转关节
        full_joints[..., 7] = current_joints[..., 11]  # 第二臂俯仰关节
        full_joints[..., 10] = current_joints[..., 12] # 第二臂肘部关节
        full_joints[..., 12] = current_joints[..., 13] # 第二臂腕部俯仰关节
        full_joints[..., 14] = current_joints[..., 14] # 第二臂腕部滚转关节

        # 头部关节
        full_joints[..., 5] = current_joints[..., 3]   # 头部水平转动
        full_joints[..., 8] = current_joints[..., 4]   # 头部俯仰

        # 夹爪关节
        full_joints[..., 15] = current_joints[..., 15] # 第一臂夹爪
        full_joints[..., 16] = current_joints[..., 16] # 第二臂夹爪

        return full_joints

    def is_grasping(self, object: Actor, min_force=0.5, max_angle=110, arm_id=None):
        """
        检查机器人是否正在抓握物体

        Args:
            object (Actor): 要检查的物体
            min_force (float, optional): 被认为抓握物体的最小力（牛顿）。默认为0.5。
            max_angle (int, optional): 被认为抓握的最大接触角度。默认为110度。
            arm_id (int, optional): 要检查的机械臂（1为第一臂，2为第二臂）。
                                   如果为None（默认），检查双臂，任一臂抓握即返回True。
        """
        if arm_id is None:
            # 检查双臂，任一臂抓握即返回True
            arm1_grasping = self._check_single_arm_grasping(object, min_force, max_angle, arm_id=1)
            arm2_grasping = self._check_single_arm_grasping(object, min_force, max_angle, arm_id=2)
            return torch.logical_or(arm1_grasping, arm2_grasping)
        else:
            # 检查指定的机械臂
            return self._check_single_arm_grasping(object, min_force, max_angle, arm_id)

    def _check_single_arm_grasping(self, object: Actor, min_force=0.5, max_angle=110, arm_id=1):
        """
        内部方法：检查指定机械臂是否正在抓握物体

        Args:
            object (Actor): 要检查的物体
            min_force (float): 最小抓握力
            max_angle (int): 最大接触角度
            arm_id (int): 机械臂ID（1或2）
        """
        if arm_id == 1:
            finger1_link = self.finger1_link  # 第一臂固定夹爪
            finger2_link = self.finger2_link  # 第一臂活动夹爪
        elif arm_id == 2:
            finger1_link = self.finger1_link_2  # 第二臂固定夹爪
            finger2_link = self.finger2_link_2  # 第二臂活动夹爪
        else:
            raise ValueError(f"Invalid arm_id: {arm_id}. Must be 1 or 2.")

        # 获取夹爪与物体的接触力
        l_contact_forces = self.scene.get_pairwise_contact_forces(
            finger1_link, object
        )
        r_contact_forces = self.scene.get_pairwise_contact_forces(
            finger2_link, object
        )
        lforce = torch.linalg.norm(l_contact_forces, axis=1)  # 左侧夹爪接触力大小
        rforce = torch.linalg.norm(r_contact_forces, axis=1)  # 右侧夹爪接触力大小

        # 夹爪张开的方向
        ldirection = finger1_link.pose.to_transformation_matrix()[..., :3, 1]
        rdirection = -finger2_link.pose.to_transformation_matrix()[..., :3, 1]
        langle = common.compute_angle_between(ldirection, l_contact_forces)  # 左侧接触角
        rangle = common.compute_angle_between(rdirection, r_contact_forces)  # 右侧接触角

        # 检查力大小和角度是否满足抓握条件
        lflag = torch.logical_and(
            lforce >= min_force, torch.rad2deg(langle) <= max_angle
        )
        rflag = torch.logical_and(
            rforce >= min_force, torch.rad2deg(rangle) <= max_angle
        )
        return torch.logical_and(lflag, rflag)  # 双侧都满足才认为是抓握

    def is_static(self, threshold=0.2, base_threshold: float = 0.05):
        """
        检查机器人是否处于静止状态

        Args:
            threshold (float): 关节速度阈值。默认为0.2。
            base_threshold (float): 底盘速度阈值（未使用）。默认为0.05。

        Returns:
            torch.Tensor: 布尔值张量，表示机器人是否静止
        """
        qvel = self.robot.get_qvel()[
            :, 3:-2
        ]  # 排除底盘关节
        return torch.max(torch.abs(qvel), 1)[0] <= threshold

    @staticmethod
    def build_grasp_pose(approaching, closing, center):
        """
        构建抓握位姿

        Args:
            approaching (np.ndarray): 接近方向向量
            closing (np.ndarray): 闭合方向向量
            center (np.ndarray): 抓握中心点

        Returns:
            sapien.Pose: 抓握位姿
        """
        assert np.abs(1 - np.linalg.norm(approaching)) < 1e-3  # 确保接近方向是单位向量
        assert np.abs(1 - np.linalg.norm(closing)) < 1e-3      # 确保闭合方向是单位向量
        assert np.abs(approaching @ closing) <= 1e-3           # 确保两个方向正交
        ortho = np.cross(closing, approaching)                # 计算正交向量
        T = np.eye(4)                                        # 构建齐次变换矩阵
        T[:3, :3] = np.stack([ortho, closing, approaching], axis=1)  # 设置旋转部分
        T[:3, 3] = center                                    # 设置平移部分
        return sapien.Pose(T)

    @property
    def tcp_pos(self):
        """
        计算第一臂的工具中心点位置

        计算固定夹爪尖端和活动夹爪尖端的中点作为工具中心点
        """
        return (self.finger1_tip.pose.p + self.finger2_tip.pose.p) / 2

    @property
    def tcp_pose(self):
        """
        获取第一臂的工具中心点位姿
        """
        return Pose.create_from_pq(self.tcp_pos, self.finger1_link.pose.q)

    @property
    def tcp_pos_2(self):
        """
        计算第二臂的工具中心点位置

        计算第二臂固定夹爪尖端和活动夹爪尖端的中点作为工具中心点
        """
        return (self.finger1_tip_2.pose.p + self.finger2_tip_2.pose.p) / 2

    @property
    def tcp_pose_2(self):
        """
        获取第二臂的工具中心点位姿
        """
        return Pose.create_from_pq(self.tcp_pos_2, self.finger1_link_2.pose.q)
