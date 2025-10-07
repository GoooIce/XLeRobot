import os
import sys

# Configuration file for the Sphinx documentation builder.
# Sphinx文档构建器的配置文件
#
# For the full list of built-in configuration values, see the documentation:
# 有关内置配置值的完整列表，请参阅文档：
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# 项目信息设置
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

# 项目名称
project = 'XLeRobot'
# 版权信息
copyright = "2025, XLeRobot Contributors"
# 作者信息
author = 'Vector & Nicole'
# 版本号
release = '0.3.0'

# -- Internationalization ---------------------------------------------------
# 国际化设置：指定文档语言为简体中文
language = 'zh_CN'

# 导入用于检查模块是否安装的工具
import importlib.util
# 导入日志记录模块
import logging
# 创建当前模块的日志记录器
logger = logging.getLogger(__name__)


def skip_mani_skill(app):
    """
    检查 mani_skill 模块是否安装，如果未安装则跳过相关文档

    Args:
        app: Sphinx 应用程序对象
    """
    # 检查 mani_skill 模块是否存在
    if importlib.util.find_spec("mani_skill") is None:
        app.logger.warning("mani_skill not installed, skipping related docs")
        # 在这里可以使用逻辑动态跳过文件

def skip_unresolvable(app):
    """
    检查依赖模块是否安装，如果未安装则记录警告信息

    Args:
        app: Sphinx 应用程序对象
    """
    # 尝试导入 mani_skill 模块
    try:
        import mani_skill
    except ImportError:
        logging.warning("mani_skill not installed — skipping API imports.")
    # 尝试导入 sapien 模块
    try:
        import sapien
    except ImportError:
        logging.warning("sapien not installed — skipping API imports.")

# def setup(app):
#     app.connect("builder-inited", skip_mani_skill)
#     app.connect("builder-inited", skip_unresolvable)

# -- General configuration ---------------------------------------------------
# 通用配置
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

# Sphinx 扩展列表
extensions = [
    "sphinx.ext.autodoc",         # 自动从源代码中的文档字符串生成API文档
    "sphinx.ext.autosummary",     # 自动生成摘要表
    "sphinx.ext.mathjax",         # 支持数学公式渲染
    "sphinx.ext.viewcode",        # 在文档中添加源代码链接
    "sphinx.ext.napoleon",        # 支持 Google/NumPy 风格的文档字符串
    "sphinx.ext.intersphinx",     # 支持跨项目链接
    "sphinx_copybutton",          # 代码块添加复制按钮
    "myst_parser",                # 支持 Markdown 格式
    "sphinx_subfigure",           # 支持子图表
    "sphinxcontrib.video",        # 支持视频嵌入
    "sphinx_togglebutton",        # 支持内容折叠按钮
    "sphinx_design"               # 支持设计组件
]

# MyST 解析器启用扩展
# https://myst-parser.readthedocs.io/en/latest/syntax/optional.html
myst_enable_extensions = ["colon_fence", "dollarmath"]
# MyST 标题锚点级别
# https://github.com/executablebooks/MyST-Parser/issues/519#issuecomment-1037239655
myst_heading_anchors = 4

# 模板文件路径
templates_path = ["_templates"]
# 排除模式（已注释）
# exclude_patterns = ["Hardware/reference/_autosummary/*"]


# -- Options for HTML output -------------------------------------------------
# HTML 输出选项
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

# HTML 主题
html_theme = "pydata_sphinx_theme"
# HTML logo 图标路径
html_logo = "_static/logo_black.png"
# 网站图标路径
html_favicon = "_static/favicon.svg"


# 版本切换器配置
# json_url = "https://maniskill.readthedocs.io/en/latest/_static/version_switcher.json"
json_url = "_static/version_switcher.json"
# 从环境变量获取版本信息
version_match = os.environ.get("READTHEDOCS_VERSION")
# if version_match is None:
#     version_match = "v" + __version__
# 侧边栏配置（已注释）
# html_sidebars = {
#   "**": []
# }

# HTML 主题选项配置
html_theme_options = {
    # "use_edit_page_button": True,  # 编辑页面按钮（已注释）

    # 图标链接配置
    "icon_links": [
        {
            "name": "GitHub",  # GitHub 链接名称
            "url": "https://github.com/Vector-Wangel/XLeRobot",  # GitHub 仓库地址
            "icon": "fa-brands fa-github",  # GitHub 图标
        },
        {
            "name": "Website",  # 网站链接名称
            "url": "https://twitter.com/VectorWang2",  # 网站地址
            "icon": "fa-solid fa-globe",  # 地球图标
        },
        {
            "name": "Language",  # 语言切换
            "icon": "fa-solid fa-language",  # 语言图标
            "type": "dropdown",  # 下拉菜单类型，从 "custom" 改为 "dropdown"
            "items": [
                {
                    "name": "English",  # 英文选项
                    "url": "index.html",  # 英文页面链接，从 "#" 改为实际链接
                },
                {
                    "name": "中文",  # 中文选项
                    "url": "README_CN.html",  # 中文页面链接，从 "#" 改为实际链接
                }
            ]
        }
    ],

    # 外部链接（已注释）
    # "external_links": [
    #     {"name": "Latest Updates", "url": "https://twitter.com/VectorWang2"},
    # ],

    # 暗色模式 logo 配置
    "logo": {
        "image_dark": "_static/logo_white.png",  # 暗色模式下显示的白色 logo
    },

    # 导航栏中心配置（已注释）
    # "navbar_center": ["version-switcher", "navbar-nav"],

    # 版本切换器显示设置
    "show_version_switcher": False,  # 不显示版本切换器
    "show_version_warning_banner": False,  # 不显示版本警告横幅

    # 版本切换器具体配置（已注释）
    # "switcher": {
    #     "json_url": json_url,
    #     "version_match": version_match,
    # },
}
# HTML 上下文配置
html_context = {
    "display_github": True,  # 显示 GitHub 链接
    "github_user": "haosulab",  # GitHub 用户名
    "github_repo": "ManiSkill",  # GitHub 仓库名
    "github_version": "main",  # GitHub 版本分支
    "conf_py_path": "/source/",  # 配置文件路径
    "doc_path": "docs/source"  # 文档路径
}

# 自定义 CSS 文件
html_css_files = [
    'css/custom.css',  # 自定义样式表
]

# 静态文件路径
html_static_path = ['_static']  # 静态资源目录
# JavaScript 文件
html_js_files = ['js/lang-switch.js']  # 语言切换脚本

### Autodoc 自动文档配置 ###
# 类型提示显示方式：在函数签名中显示
autodoc_typehints = "signature"
# 类型提示描述目标：所有类型
autodoc_typehints_description_target = "all"
# 自动文档默认标志：显示成员、显示继承关系、显示无文档成员
autodoc_default_flags = ['members', 'show-inheritance', 'undoc-members']

# 自动生成摘要
autosummary_generate = True

# 从目录树中移除的路径（已注释）
# remove_from_toctrees = ["_autosummary/*"]

# 跨项目链接映射配置
intersphinx_mapping = {'gymnasium': ('https://gymnasium.farama.org/', None)}  # 链接到 Gymnasium 文档

# -- Options for LaTeX output -----------------------------------------------
# LaTeX 输出选项
# 支持中文 PDF 生成
latex_engine = "xelatex"  # 使用 XeLaTeX 引擎，支持中文
latex_use_xindy = False    # 不使用 xindy 索引工具
# LaTeX 元素配置
latex_elements = {
    "preamble": "\\usepackage[UTF8]{ctex}\n",  # 在文档前言中加载中文支持包
}
