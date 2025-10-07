# # 任务文档自动生成代码
#
# 该脚本用于自动生成 XLeRobot 项目中任务相关的文档。
# 它会扫描任务定义文件，提取任务信息，并生成格式化的文档。

# 要包含的任务类别列表
TASK_CATEGORIES_TO_INCLUDE = [
    "tabletop",              # 桌面任务
    "humanoid",              # 人形机器人任务
    "mobile_manipulation",  # 移动操控任务
    "quadruped",            # 四足机器人任务
    "control",              # 控制任务
    "drawing",              # 绘画任务
]

# 任务类别名称映射（用于文档文件夹命名）
TASK_CATEGORIES_NAME_MAP = {"tabletop": "table_top_gripper"}
# 生成任务文档的文件夹名称
GENERATED_TASKS_DOCS_FOLDER = "tasks"

# 全局任务文档头部信息（包含徽章定义）
GLOBAL_TASK_HEADER = """<!-- THIS IS ALL GENERATED DOCUMENTATION. DO NOT MODIFY THIS FILE -->
# [asset-badge]: https://img.shields.io/badge/download%20asset-yes-blue.svg
# [dense-reward-badge]: https://img.shields.io/badge/dense%20reward-yes-green.svg
# [sparse-reward-badge]: https://img.shields.io/badge/sparse%20reward-yes-green.svg
# [no-dense-reward-badge]: https://img.shields.io/badge/dense%20reward-no-red.svg
# [no-sparse-reward-badge]: https://img.shields.io/badge/sparse%20reward-no-red.svg
# [demos-badge]: https://img.shields.io/badge/demos-yes-green.svg
# """
# 全局任务文档后部说明信息
GLOBAL_TASK_POST_HEADER = """
The document here has both a high-level overview/list of all tasks in a table as well as detailed task cards with video demonstrations after.
"""

# 任务类别头部信息定义
# 每个类别都有其特定的描述和介绍信息
TASK_CATEGORIES_HEADERS = {
    # 桌面双指夹爪任务
    "tabletop": """# Table-Top 2 Finger Gripper Tasks

These are tasks situated on table and involve a two-finger gripper arm robot manipulating objects on the surface.""",

    # 人形机器人任务
    "humanoid": """# Humanoid Tasks
Both real-world humanoids and the Mujoco humanoid are supported in ManiSkill, and we are still in the process of adding more tasks. Humanoid category of tasks generally considers control of robots with two legs and two arms.""",

    # 移动操控任务
    "mobile_manipulation": """# Mobile Manipulation Tasks

These are tasks where a mobile manipulator is used to manipulate objects. This cateogry primarily uses robots with mobile bases like Fetch or Stretch robots.

For additional tasks, including scene-level mobile manipulation, please check out the [external benchmarks/tasks page](../external/index.md).
# """,

    # 四足机器人任务
    "quadruped": """# Quadruped Tasks

These are tasks where a quadruped robot is used for locomotion and/or manipulation. This cateogry primarily uses robots with four legs like the ANYmal or Unitree go robots.""",

    # 控制任务
    "control": """# Control Tasks

These are classic control tasks where the objective is to control a robot to reach a particular state, similar to the [DM Control suite](https://github.com/deepmind/dm_control) but with GPU parallelized simulation and rendering.""",

    # 绘画任务
    "drawing": """# Drawing Tasks

These are tasks where the robot is controlled to draw a specific shape or pattern.""",
}
# 导入必要的模块和库
# import urllib.request           # 用于网络请求，获取视频和缩略图
# import mani_skill.envs         # ManiSkill 环境模块
# from mani_skill.utils.download_demo import DATASET_SOURCES  # 演示数据源
# from mani_skill.utils.registration import REGISTERED_ENVS   # 注册的环境列表
# import os                       # 操作系统接口
# import importlib               # 动态导入模块
# import inspect                 # 检查模块和函数
# from pathlib import Path       # 路径操作
# import cv2                     # OpenCV，用于图像处理
# import tempfile               # 临时文件处理


def main():
    """
    主函数：执行任务文档生成的核心流程

    该函数负责：
    1. 扫描任务定义文件
    2. 解析任务类别和信息
    3. 生成文档表格和任务卡片
    4. 处理视频缩略图
    5. 输出格式化的文档文件
    """
#     # 获取文档基础目录
#     base_dir = Path(__file__).parent / "source"

#     # 获取 mani_skill/envs/tasks 的路径
#     tasks_dir = Path(mani_skill.envs.__file__).parent / "tasks"

#     # 创建字典存储任务信息
#     task_info = {}

#     # 遍历任务目录中的所有子文件夹
#     for root, dirs, files in os.walk(tasks_dir):
#         for file in files:
#             # 只处理 Python 文件，排除以 __ 开头的文件
#             if file.endswith(".py") and not file.startswith("__"):
#                 # 获取相对导入路径
#                 rel_path = os.path.relpath(os.path.join(root, file), tasks_dir.parent)
#                 module_path = rel_path.replace(os.sep, ".")[:-3]  # 移除 .py 扩展名

#                 # 动态导入模块
#                 try:
#                     module = importlib.import_module(f"mani_skill.envs.{module_path}")

#                     # 查找此模块中定义的所有类
#                     classes = inspect.getmembers(module, inspect.isclass)

#                     # 存储在此模块中定义的类（而不是导入的类）
#                     local_classes = [
#                         cls
#                         for name, cls in classes
#                         if cls.__module__ == f"mani_skill.envs.{module_path}"
#                     ]

#                     if local_classes:
#                         task_info[module_path] = local_classes

#                 except Exception as e:
#                     print(f"Error importing {module_path}: {e}")
#     # 过滤只包含已注册的环境类和有文档字符串的类
#     filtered_task_info = {}
#     for module_path, classes in task_info.items():
#         registered_classes = []
#         for cls in classes:
#             # 检查此类是否已注册为环境
#             for env_id, env_spec in REGISTERED_ENVS.items():
#                 if env_spec.cls == cls:
#                     registered_classes.append(dict(env_id=env_id, cls=cls))
#                     break
#         if registered_classes:
#             filtered_task_info[module_path] = registered_classes

#     task_info = filtered_task_info
#     # 按任务类型对任务进行分类
#     categorized_tasks = {k: [] for k in TASK_CATEGORIES_TO_INCLUDE}

#     for module_path in task_info.keys():
#         parts = module_path.split(".")
#         if parts[0] == "tasks":
#             category = parts[1]
#             if category in categorized_tasks:
#                 categorized_tasks[category].append(module_path)

#     # 为每个类别和模块生成文档
#     print("\nTask Documentation:")
#     for category, modules in categorized_tasks.items():
#         print(f"\n{category}:")
#         # 如果目录不存在则创建
#         category_name = TASK_CATEGORIES_NAME_MAP.get(category, category)
#         os.makedirs(
#             f"{base_dir}/{GENERATED_TASKS_DOCS_FOLDER}/{category_name}", exist_ok=True
#         )

#         # 删除此类别现有的 index.md 文件
#         if os.path.exists(
#             f"{base_dir}/{GENERATED_TASKS_DOCS_FOLDER}/{category_name}/index.md"
#         ):
#             os.remove(f"{base_dir}/{GENERATED_TASKS_DOCS_FOLDER}/{category_name}/index.md")

#         # 写入文档头部信息
#         if category in TASK_CATEGORIES_HEADERS:
#             with open(
#                 f"{base_dir}/{GENERATED_TASKS_DOCS_FOLDER}/{category_name}/index.md", "w"
#             ) as f:
#                 f.write(GLOBAL_TASK_HEADER)                    # 写入全局头部信息
#                 f.write(TASK_CATEGORIES_HEADERS[category])    # 写入类别特定头部
#                 f.write(GLOBAL_TASK_POST_HEADER)              # 写入全局后部信息

#         # 生成任务简表（TLDR 表格）
#         env_id_to_thumbnail_path = {}  # 存储环境ID到缩略图路径的映射
#         with open(
#             f"{base_dir}/{GENERATED_TASKS_DOCS_FOLDER}/{category_name}/index.md", "a"
#         ) as f:
#             f.write("\n## Task Table\n")
#             f.write(
#                 "Table of all simulationenvironments in this category. Task column is the environment ID, Preview is a thumbnail pair of the first and last frames of an example success demonstration. Max steps is the task's default max episode steps, generally tuned for RL workflows."
#             )
#             f.write('\n<table class="table">')
#             f.write("\n<thead>")
#             f.write('\n<tr class="row-odd">')
#             f.write('\n<th class="head"><p>Task</p></th>')                    # 任务名称列
#             f.write('\n<th class="head"><p>Preview</p></th>')                # 预览图列
#             f.write('\n<th class="head"><p>Dense Reward</p></th>')           # 密集奖励列
#             f.write('\n<th class="head"><p>Success/Fail Conditions</p></th>') # 成功/失败条件列
#             f.write('\n<th class="head"><p>Demos</p></th>')                  # 演示列
#             f.write('\n<th class="head"><p>Max Episode Steps</p></th>')       # 最大步数列
#             f.write("\n</tr>")
#             f.write("\n</thead>")
#             f.write("\n<tbody>")
#             for module in sorted(modules):
#                 environment_data = task_info[module]
#                 classes = [env_data["cls"] for env_data in environment_data]
#                 env_ids = [env_data["env_id"] for env_data in environment_data]

#                 # Add row for each environment
#                 for row_idx, (cls, env_id) in enumerate(zip(classes, env_ids)):
#                     # Get reward mode info
#                     dense = (
#                         "✅"
#                         if hasattr(cls, "SUPPORTED_REWARD_MODES")
#                         and "dense" in cls.SUPPORTED_REWARD_MODES
#                         else "❌"
#                     )
#                     sparse = (
#                         "✅"
#                         if hasattr(cls, "SUPPORTED_REWARD_MODES")
#                         and "sparse" in cls.SUPPORTED_REWARD_MODES
#                         else "❌"
#                     )
#                     max_eps_steps = (
#                         REGISTERED_ENVS[env_id].max_episode_steps
#                         if REGISTERED_ENVS[env_id].max_episode_steps is not None
#                         else "N/A"
#                     )
#                     demos = "✅" if env_id in DATASET_SOURCES else "❌"
#                     # Get video thumbnail if available
#                     thumbnail = ""
#                     thumbnail_last = ""
#                     if hasattr(cls, "_sample_video_link") and cls._sample_video_link:
#                         video_url = cls._sample_video_link
#                         thumbnail_paths = [
#                             video_url.replace(".mp4", "_thumb_first.png"),
#                             video_url.replace(".mp4", "_thumb_last.png"),
#                         ]
#                         # Check if thumbnail already exists online
#                         thumbnails_exist = False
#                         for thumbnail_path in thumbnail_paths:
#                             try:
#                                 urllib.request.urlopen(thumbnail_path)
#                                 # If no error, thumbnail exists
#                                 thumbnails_exist = True
#                             except urllib.error.URLError:
#                                 thumbnails_exist = False
#                                 break
#                         # Also check locally in figures/env_demos
#                         local_thumbnail_paths = [
#                             os.path.join(
#                                 os.path.dirname(__file__),
#                                 f"{base_dir}/_static/env_thumbnails",
#                                 os.path.basename(thumbnail_path),
#                             )
#                             for thumbnail_path in thumbnail_paths
#                         ]
#                         if os.path.exists(local_thumbnail_paths[0]) and os.path.exists(
#                             local_thumbnail_paths[1]
#                         ):
#                             thumbnails_exist = True

#                         if not thumbnails_exist:
#                             # Create temp file to store video
#                             # with tempfile.NamedTemporaryFile(suffix='.mp4', delete=False) as tmp_video:
#                             # urllib.request.urlretrieve(video_url, tmp_video.name)

#                             # Extract first frame and resize maintaining aspect ratio
#                             cap = cv2.VideoCapture(
#                                 os.path.join(
#                                     os.path.dirname(__file__),
#                                     "../figures/environment_demos",
#                                     os.path.basename(video_url),
#                                 )
#                             )
#                             # Get first frame
#                             ret, first_frame = cap.read()

#                             # Get last frame by seeking to end
#                             frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
#                             cap.set(cv2.CAP_PROP_POS_FRAMES, frame_count - 1)
#                             ret, last_frame = cap.read()
#                             cap.release()

#                             # Process both frames
#                             for frame, output_path in [
#                                 (first_frame, thumbnail_paths[0]),
#                                 (last_frame, thumbnail_paths[1]),
#                             ]:
#                                 height, width = frame.shape[:2]
#                                 if height > width:
#                                     # Scale height to 256, maintain aspect ratio
#                                     scale = 256.0 / height
#                                     new_width = int(width * scale)
#                                     frame = cv2.resize(
#                                         frame,
#                                         (new_width, 256),
#                                         interpolation=cv2.INTER_AREA,
#                                     )
#                                 else:
#                                     # Scale width to 256, maintain aspect ratio
#                                     scale = 256.0 / width
#                                     new_height = int(height * scale)
#                                     frame = cv2.resize(
#                                         frame,
#                                         (256, new_height),
#                                         interpolation=cv2.INTER_AREA,
#                                     )

#                                 # Save as compressed PNG
#                                 output_path = os.path.join(
#                                     os.path.dirname(__file__),
#                                     f"{base_dir}/_static/env_thumbnails",
#                                     os.path.basename(output_path),
#                                 )
#                                 cv2.imwrite(
#                                     output_path, frame, [cv2.IMWRITE_PNG_COMPRESSION, 9]
#                                 )

#                             # # Clean up temp file
#                             # os.unlink(tmp_video.name)
#                         thumbnail_first_path = os.path.join(
#                             "../../_static/env_thumbnails",
#                             os.path.basename(
#                                 video_url.replace(".mp4", "_thumb_first.png")
#                             ),
#                         )
#                         thumbnail_last_path = os.path.join(
#                             "../../_static/env_thumbnails",
#                             os.path.basename(
#                                 video_url.replace(".mp4", "_thumb_last.png")
#                             ),
#                         )
#                         thumbnail = f"<img style='min-width:min(50%, 100px);max-width:100px;height:auto' src='{thumbnail_first_path}' alt='{env_id}'>"
#                         thumbnail_last = f"<img style='min-width:min(50%, 100px);max-width:100px;height:auto' src='{thumbnail_last_path}' alt='{env_id}'>"
#                         env_id_to_thumbnail_path[env_id] = [
#                             thumbnail_first_path,
#                             thumbnail_last_path,
#                         ]

#                     # f.write(f"| {env_id} | <div style='display:flex;gap:4px;align-items:center'>{thumbnail} {thumbnail_last}</div> | {dense} | {sparse} |")
#                     f.write(
#                         f"\n<tr class=\"row-{'even' if row_idx % 2 == 1 else 'odd'}\">"
#                     )
#                     f.write(
#                         f'\n<td><p><a href="#{env_id.lower()}">{env_id}</a></p></td>'
#                     )
#                     f.write(
#                         f"\n<td><div style='display:flex;gap:4px;align-items:center'>{thumbnail if thumbnail != '' else ''} {thumbnail_last if thumbnail_last != '' else ''}</div></td>"
#                     )
#                     f.write(f"\n<td><p>{dense}</p></td>")
#                     f.write(f"\n<td><p>{sparse}</p></td>")
#                     f.write(f"\n<td><p>{demos}</p></td>")
#                     f.write(f"\n<td><p>{max_eps_steps}</p></td>")
#                     f.write(f"\n</tr>")

#             f.write("\n</tbody>")
#             f.write("\n</table>")
#             f.write("\n")

#         # Generate all the detailed task cards
#         for module in sorted(modules):
#             environment_data = task_info[module]
#             classes = [env_data["cls"] for env_data in environment_data]
#             env_ids = [env_data["env_id"] for env_data in environment_data]
#             # print(f"\n  {module}:")
#             for cls, env_id in zip(classes, env_ids):
#                 # Check if dense reward function is overridden
#                 from mani_skill.envs.sapien_env import BaseEnv

#                 has_custom_dense = hasattr(cls, "compute_dense_reward") and (
#                     cls.compute_dense_reward != BaseEnv.compute_dense_reward
#                     or cls.compute_normalized_dense_reward
#                     != BaseEnv.compute_normalized_dense_reward
#                 )
#                 if has_custom_dense and "dense" not in cls.SUPPORTED_REWARD_MODES:
#                     print(
#                         f"Warning: {cls.__name__}, {env_id} has custom dense reward but dense not in SUPPORTED_REWARD_MODES"
#                     )
#                 does_not_have_custom_dense = (
#                     not has_custom_dense and "dense" in cls.SUPPORTED_REWARD_MODES
#                 )
#                 if does_not_have_custom_dense:
#                     print(
#                         f"Warning: {cls.__name__}, {env_id} does not have custom dense reward but dense is in SUPPORTED_REWARD_MODES"
#                     )

#                 # Extract docstring
#                 if cls.__doc__:
#                     with open(
#                         f"{base_dir}/{GENERATED_TASKS_DOCS_FOLDER}/{category_name}/index.md",
#                         "a",
#                     ) as f:
#                         f.write(f"\n## {env_id}\n\n")
#                         # Write reward modes if available
#                         if hasattr(cls, "SUPPORTED_REWARD_MODES"):
#                             if "dense" in cls.SUPPORTED_REWARD_MODES:
#                                 f.write("![dense-reward][dense-reward-badge]\n")
#                             else:
#                                 f.write("![no-dense-reward][no-dense-reward-badge]\n")
#                             if "sparse" in cls.SUPPORTED_REWARD_MODES:
#                                 f.write("![sparse-reward][sparse-reward-badge]\n")
#                             else:
#                                 f.write("![no-sparse-reward][no-sparse-reward-badge]\n")
#                             if env_id in DATASET_SOURCES:
#                                 f.write("![demos][demos-badge]\n")
#                         if (
#                             REGISTERED_ENVS[env_id].asset_download_ids is not None
#                             and len(REGISTERED_ENVS[env_id].asset_download_ids) > 0
#                         ):
#                             f.write("![asset-badge][asset-badge]\n")
#                         """:::{dropdown} Task Card\n:icon: note\n:color: primary"""
#                         # Clean up docstring and write to file
#                         f.write(
#                             ":::{dropdown} Task Card\n:icon: note\n:color: primary\n\n"
#                         )
#                         doc_lines = [line.strip() for line in cls.__doc__.split("\n")]
#                         while doc_lines and not doc_lines[0]:
#                             doc_lines.pop(0)
#                         while doc_lines and not doc_lines[-1]:
#                             doc_lines.pop()
#                         if doc_lines:
#                             f.write("\n".join(doc_lines))
#                             f.write("\n")
#                         f.write(":::\n")
#                         # Add video link if available
#                         if (
#                             hasattr(cls, "_sample_video_link")
#                             and cls._sample_video_link is not None
#                         ):
#                             f.write(
#                                 '\n<div style="display: flex; justify-content: center;">\n'
#                             )
#                             f.write(
#                                 f'<video preload="none" controls="True" width="100%" style="max-width: min(100%, 512px);" poster="{env_id_to_thumbnail_path[env_id][0]}">\n'
#                             )
#                             f.write(
#                                 f'<source src="{cls._sample_video_link}" type="video/mp4">\n'
#                             )
#                             f.write("</video>\n")
#                             f.write("</div>\n")
#                         else:
#                             print(
#                                 f"Warning: {cls.__name__}, {env_id} has no sample video link"
#                             )
#                 else:
#                     print(f"Warning: {cls.__name__}, {env_id} has no docstring")


# 脚本入口点
# 当直接运行此脚本时，将执行 main() 函数
if __name__ == "__main__":
#     main()
