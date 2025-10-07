"""
JoyCon设备常量定义模块

此模块定义了与Nintendo Switch JoyCon设备相关的所有常量，
包括供应商ID、产品ID、序列号格式等硬件标识信息。
"""

# JoyCon供应商ID (Nintendo)
JOYCON_VENDOR_ID = 0x057E

# JoyCon左控制器产品ID
JOYCON_L_PRODUCT_ID = 0x2006

# JoyCon右控制器产品ID
JOYCON_R_PRODUCT_ID = 0x2007

# 支持的JoyCon序列号前缀
JOYCON_SERIAL_SUPPORT = '9c:54:'

# JoyCon左控制器序列号前缀
JOYCON_SERIAL_HEAD_L = '9c:54:00:b0:'

# JoyCon右控制器序列号前缀
JOYCON_SERIAL_HEAD_R = '9c:54:00:e0:'

# 默认报告数据格式
JOYCON_REPORT_DEFAULT = b'\x00\x00\x00\x00\x00\x00'

# 颜色支持标识
JOYCON_COLOR_SUPPORT = b'\x80\x03\x00'

# 所有支持的JoyCon产品ID元组
JOYCON_PRODUCT_IDS = (JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID)

# 所有JoyCon序列号前缀元组
JOYCON_SERIAL_HEAD = (JOYCON_SERIAL_HEAD_L, JOYCON_SERIAL_HEAD_R)