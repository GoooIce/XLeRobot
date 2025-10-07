"""
JoyCon设备检测和管理模块

此模块提供了JoyCon设备的自动检测、识别和管理功能。
能够扫描系统中连接的JoyCon设备，并根据设备类型进行分类。

主要功能：
- 自动检测连接的JoyCon设备
- 区分左、右JoyCon控制器
- 获取设备的供应商ID、产品ID和序列号
- 支持调试模式输出设备详细信息
"""

import hid
from .constants import JOYCON_VENDOR_ID, JOYCON_PRODUCT_IDS
from .constants import JOYCON_L_PRODUCT_ID, JOYCON_R_PRODUCT_ID


def get_device_ids(debug=False):
    """
    获取所有连接的JoyCon设备ID列表

    扫描系统中所有HID设备，筛选出JoyCon设备并返回其标识信息。

    参数:
        debug (bool): 是否开启调试模式，开启后会打印设备详细信息

    返回:
        list: 包含元组(vendor_id, product_id, serial_number)的列表
              每个元组代表一个连接的JoyCon设备
    """
    # 枚举系统中所有HID设备
    devices = hid.enumerate(0, 0)

    out = []
    for device in devices:
        vendor_id      = device["vendor_id"]      # 供应商ID
        product_id     = device["product_id"]     # 产品ID
        product_string = device["product_string"] # 产品名称字符串
        serial = device.get('serial') or device.get("serial_number")  # 序列号

        # 过滤非JoyCon设备
        if vendor_id != JOYCON_VENDOR_ID:
            continue
        if product_id not in JOYCON_PRODUCT_IDS:
            continue
        if not product_string:
            continue

        # 修复：移除有问题的早期返回逻辑
        # 支持所有序列号格式，不只是 '9c:54:' 开头的
        if debug and serial and serial[0:6] != '9c:54:':
            print(f'非标准序列号格式: {serial=}')

        # 将设备信息添加到输出列表
        out.append((vendor_id, product_id, serial))

        # 调试模式下打印设备详细信息
        if debug:
            print(product_string)
            print(f"\tvendor_id  is {vendor_id!r}")
            print(f"\tproduct_id is {product_id!r}")
            print(f"\tserial     is {serial!r}")

    return out


def is_id_L(id):
    """
    判断设备ID是否为左JoyCon控制器

    参数:
        id (tuple): 设备ID元组 (vendor_id, product_id, serial_number)

    返回:
        bool: 如果是左JoyCon返回True，否则返回False
    """
    return id[1] == JOYCON_L_PRODUCT_ID


def get_ids_of_type(lr, **kw):
    """
    获取指定类型的JoyCon设备ID列表

    根据传入的类型参数筛选对应的JoyCon设备。

    参数:
        lr (str): 设备类型，"L"表示左JoyCon，"R"表示右JoyCon
        **kw: 传递给get_device_ids的额外参数

    返回:
        list: 包含指定类型JoyCon设备ID元组的列表
    """
    if lr.lower() == "l":
        product_id = JOYCON_L_PRODUCT_ID  # 左JoyCon产品ID
    else:
        product_id = JOYCON_R_PRODUCT_ID  # 右JoyCon产品ID
    return [i for i in get_device_ids(**kw) if i[1] == product_id]


def get_R_ids(**kw):
    """
    获取所有右JoyCon设备ID列表

    参数:
        **kw: 传递给get_device_ids的额外参数

    返回:
        list: 包含右JoyCon设备ID元组的列表
    """
    return get_ids_of_type("R", **kw)


def get_L_ids(**kw):
    """
    获取所有左JoyCon设备ID列表

    参数:
        **kw: 传递给get_device_ids的额外参数

    返回:
        list: 包含左JoyCon设备ID元组的列表
    """
    return get_ids_of_type("L", **kw)


def get_R_id(**kw):
    """
    获取第一个右JoyCon设备ID

    参数:
        **kw: 传递给get_device_ids的额外参数

    返回:
        tuple: 右JoyCon设备ID元组 (vendor_id, product_id, serial_number)
               如果没有找到设备则返回 (None, None, None)
    """
    ids = get_R_ids(**kw)
    if not ids:
        return (None, None, None)
    return ids[0]


def get_L_id(**kw):
    """
    获取第一个左JoyCon设备ID

    参数:
        **kw: 传递给get_device_ids的额外参数

    返回:
        tuple: 左JoyCon设备ID元组 (vendor_id, product_id, serial_number)
               如果没有找到设备则返回 (None, None, None)
    """
    ids = get_L_ids(**kw)
    if not ids:
        return (None, None, None)
    return ids[0]
