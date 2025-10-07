from typing import Any, Dict, Union

import numpy as np
import sapien as sapien
import sapien.physx as physx
import torch
from sapien import Pose

from mani_skill.agents.robots import Fetch, Panda, Xlerobot
from mani_skill.envs.sapien_env import BaseEnv
from mani_skill.sensors.camera import CameraConfig
from mani_skill.utils import sapien_utils
from mani_skill.utils.registration import register_env
from mani_skill.utils.scene_builder import SceneBuilder
from mani_skill.utils.scene_builder.registration import REGISTERED_SCENE_BUILDERS
from mani_skill.utils.structs.types import GPUMemoryConfig, SimConfig


@register_env("SceneManipulation-v1", max_episode_steps=200)
class SceneManipulationEnv(BaseEnv):
    """
    在复杂场景中模拟操作任务的基础环境

    创建这个基础环境仅用于探索/可视化，没有成功/失败指标或奖励。

    Args:
        robot_uids: 要放置到场景中的机器人类型。默认为"fetch"

        scene_builder_cls:
            用于构建场景的场景构建器类。默认为ReplicaCAD。此外，还支持任何AI2THOR场景构建器。

        build_config_idxs (optional): 要采样的构建配置（静态构建）。你的scene_builder_cls可能需要也可能不需要这些。
        init_config_idxs (optional): 要采样的初始化配置（额外的初始化选项）。你的scene_builder_cls可能需要也可能不需要这些。

        reconfiguration_freq: 重新配置频率。None表示根据环境数量自动设置。
    """

    SUPPORTED_ROBOTS = ["panda", "fetch", "xlerobot"]  # 支持的机器人类型
    agent: Union[Panda, Fetch, Xlerobot]  # 机器人代理实例

    def __init__(
        self,
        *args,
        robot_uids="fetch",  # 默认使用fetch机器人
        scene_builder_cls: Union[str, SceneBuilder] = "ReplicaCAD",  # 默认场景构建器
        build_config_idxs=None,  # 构建配置索引
        init_config_idxs=None,   # 初始化配置索引
        num_envs=1,              # 环境数量
        reconfiguration_freq=None,  # 重新配置频率
        **kwargs
    ):
        """
        初始化场景操作环境

        Args:
            robot_uids: 机器人类型标识符
            scene_builder_cls: 场景构建器类或类名
            build_config_idxs: 构建配置索引列表
            init_config_idxs: 初始化配置索引列表
            num_envs: 并行环境数量
            reconfiguration_freq: 重新配置频率
        """
        # 如果场景构建器是字符串，则从注册表中获取对应的类
        if isinstance(scene_builder_cls, str):
            scene_builder_cls = REGISTERED_SCENE_BUILDERS[
                scene_builder_cls
            ].scene_builder_cls

        # 创建场景构建器实例
        self.scene_builder: SceneBuilder = scene_builder_cls(self)
        self.build_config_idxs = build_config_idxs  # 存储构建配置索引
        self.init_config_idxs = init_config_idxs   # 存储初始化配置索引

        # 自动设置重新配置频率
        if reconfiguration_freq is None:
            if num_envs == 1:
                reconfiguration_freq = 1  # 单环境每次重置都重新配置
            else:
                reconfiguration_freq = 0  # 多环境从不重新配置

        # 调用父类初始化
        super().__init__(
            *args,
            robot_uids=robot_uids,
            reconfiguration_freq=reconfiguration_freq,
            num_envs=num_envs,
            **kwargs
        )

    @property
    def _default_sim_config(self):
        """
        获取默认的仿真配置

        Returns:
            SimConfig: 仿真配置对象
        """
        return SimConfig(
            spacing=50,  # 环境间距
            gpu_memory_config=GPUMemoryConfig(
                found_lost_pairs_capacity=2**25,    # 丢失配对容量
                max_rigid_patch_count=2**21,        # 最大刚性补丁数量
                max_rigid_contact_count=2**23,      # 最大刚性接触数量
            ),
        )

    def reset(self, seed=None, options=None):
        """
        重置环境

        Args:
            seed: 随机种子
            options: 重置选项字典，可包含reconfigure、build_config_idxs、init_config_idxs等

        Returns:
            重置后的观察结果
        """
        # 默认不重新配置
        if options is None:
            options = dict(reconfigure=False)

        # 设置回合随机数生成器
        self._set_episode_rng(seed, options.get("env_idx", torch.arange(self.num_envs)))

        # 处理重新配置逻辑
        if "reconfigure" in options and options["reconfigure"]:
            # 如果要求重新配置，更新配置索引
            self.build_config_idxs = options.get(
                "build_config_idxs", self.build_config_idxs
            )
            self.init_config_idxs = options.get("init_config_idxs", None)
        else:
            # 如果不重新配置，确保没有构建配置索引
            assert (
                "build_config_idxs" not in options
            ), "options dict cannot contain build_config_idxs without reconfigure=True"
            self.init_config_idxs = options.get(
                "init_config_idxs", self.init_config_idxs
            )

        # 确保配置索引是列表格式
        if isinstance(self.build_config_idxs, int):
            self.build_config_idxs = [self.build_config_idxs]
        if isinstance(self.init_config_idxs, int):
            self.init_config_idxs = [self.init_config_idxs]

        return super().reset(seed, options)

    def _load_lighting(self, options: dict):
        """
        加载光照设置

        Args:
            options: 光照配置选项
        """
        # 如果场景构建器处理光照，则直接返回
        if self.scene_builder.builds_lighting:
            return
        # 否则调用父类的光照加载方法
        return super()._load_lighting(options)

    def _load_agent(self, options: dict):
        """
        加载机器人代理

        Args:
            options: 机器人加载选项
        """
        # 使用场景构建器指定的机器人初始位姿加载机器人
        super()._load_agent(
            options,
            self.scene_builder.robot_initial_pose,
        )

    def _load_scene(self, options: dict):
        """
        加载场景

        Args:
            options: 场景加载选项
        """
        if self.scene_builder.build_configs is not None:
            # 如果有构建配置，使用指定或采样的配置构建场景
            self.scene_builder.build(
                self.build_config_idxs
                if self.build_config_idxs is not None
                else self.scene_builder.sample_build_config_idxs()
            )
        else:
            # 否则直接构建场景
            self.scene_builder.build()

    def _initialize_episode(self, env_idx: torch.Tensor, options: dict):
        """
        初始化回合

        Args:
            env_idx: 环境索引张量
            options: 初始化选项
        """
        with torch.device(self.device):
            if self.scene_builder.init_configs is not None:
                # 如果有初始化配置，使用指定或采样的配置初始化
                self.scene_builder.initialize(
                    env_idx,
                    (
                        self.init_config_idxs
                        if self.init_config_idxs is not None
                        else self.scene_builder.sample_init_config_idxs()
                    ),
                )
            else:
                # 否则直接初始化
                self.scene_builder.initialize(env_idx)

    def evaluate(self) -> dict:
        """
        评估当前回合状态

        Returns:
            dict: 评估结果字典（基础环境返回空字典）
        """
        return dict()

    def compute_dense_reward(self, obs: Any, action: torch.Tensor, info: Dict):
        """
        计算密集奖励

        Args:
            obs: 观察结果
            action: 执行的动作
            info: 信息字典

        Returns:
            float: 密集奖励值（基础环境返回0）
        """
        return 0

    def compute_normalized_dense_reward(
        self, obs: Any, action: torch.Tensor, info: Dict
    ):
        """
        计算归一化密集奖励

        Args:
            obs: 观察结果
            action: 执行的动作
            info: 信息字典

        Returns:
            float: 归一化密集奖励值
        """
        return self.compute_dense_reward(obs=obs, action=action, info=info) / 1

    @property
    def _default_sensor_configs(self):
        """
        获取默认传感器配置

        Returns:
            list: 相机配置列表
        """
        if self.robot_uids == "fetch":
            return []  # Fetch机器人默认不添加额外传感器

        # 创建基础相机配置，用于观察场景
        pose = sapien_utils.look_at([0.3, 0, 0.6], [-0.1, 0, 0.1])  # 相机位姿
        return [CameraConfig("base_camera", pose, 128, 128, np.pi / 2, 0.01, 100)]

    @property
    def _default_human_render_camera_configs(self):
        """
        获取默认人体渲染相机配置

        Returns:
            list: 渲染相机配置列表或单个相机配置
        """
        if self.robot_uids == "fetch" or self.robot_uids == "xlerobot":
            # Fetch或Xlerobot使用安装在机器人顶部的相机
            robot_camera_pose = sapien_utils.look_at([1, 0, 0.6], [0, 0, 0.3])
            robot_camera_config = CameraConfig(
                "render_camera",
                robot_camera_pose,
                640,    # 宽度
                480,    # 高度
                1,      # 视场角
                0.01,   # 近平面
                100,    # 远平面
                mount=self.agent.top_base_link,  # 安装在机器人顶部基座
            )
            return [robot_camera_config]

        if self.robot_uids == "panda":
            # Panda机器人使用特定的相机位姿
            pose = sapien_utils.look_at([0.4, 0.4, 0.8], [0.0, 0.0, 0.4])
        else:
            # 其他机器人使用默认的远距离观察位姿
            pose = sapien_utils.look_at([0, 10, -3], [0, 0, 0])
        return CameraConfig("render_camera", pose, 512, 512, 1, 0.01, 100)
