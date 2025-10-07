"""
遥操作系统的工具函数模块。

提供SSL证书生成、日志记录等通用功能，确保系统的安全性和可维护性。
"""

import os
import subprocess
import logging
from pathlib import Path
from typing import Tuple

# 获取当前模块的日志记录器
logger = logging.getLogger(__name__)

def generate_ssl_certificates(cert_path: str = "cert.pem", key_path: str = "key.pem") -> bool:
    """
    如果SSL证书不存在，自动生成自签名SSL证书。

    Args:
        cert_path: 保存证书文件的路径
        key_path: 保存私钥文件的路径

    Returns:
        bool: 证书已存在或生成成功返回True，否则返回False
    """
    # 检查证书是否已经存在
    if os.path.exists(cert_path) and os.path.exists(key_path):
        logger.info(f"SSL证书已存在: {cert_path}, {key_path}")
        return True

    logger.info("未找到SSL证书，正在生成自签名证书...")

    try:
        # 使用openssl生成自签名证书
        cmd = [
            "openssl", "req", "-x509", "-newkey", "rsa:2048",  # 使用RSA 2048位加密
            "-keyout", key_path,                               # 私钥输出路径
            "-out", cert_path,                                 # 证书输出路径
            "-sha256", "-days", "365", "-nodes",              # SHA256算法，有效期365天，无密码保护
            "-subj", "/C=US/ST=Test/L=Test/O=Test/OU=Test/CN=localhost"  # 证书主题信息
        ]

        # 执行openssl命令
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)

        # 设置适当的文件权限（出于安全考虑，私钥仅所有者可读）
        os.chmod(key_path, 0o600)   # 私钥文件：仅所有者可读写
        os.chmod(cert_path, 0o644)  # 证书文件：所有者可读写，其他用户可读

        logger.info(f"SSL证书生成成功: {cert_path}, {key_path}")
        return True

    except subprocess.CalledProcessError as e:
        logger.error(f"生成SSL证书失败: {e}")
        logger.error(f"命令输出: {e.stderr}")
        return False
    except FileNotFoundError:
        logger.error("未找到OpenSSL。请安装OpenSSL以生成证书。")
        logger.error("Ubuntu/Debian系统: sudo apt-get install openssl")
        logger.error("macOS系统: brew install openssl")
        return False
    except Exception as e:
        logger.error(f"生成SSL证书时出现意外错误: {e}")
        return False

def ensure_ssl_certificates(cert_path: str = "cert.pem", key_path: str = "key.pem") -> bool:
    """
    确保SSL证书存在，必要时自动生成。

    Args:
        cert_path: 证书文件路径
        key_path: 私钥文件路径

    Returns:
        bool: 证书可用返回True，生成失败返回False
    """
    if not generate_ssl_certificates(cert_path, key_path):
        logger.error("无法确保SSL证书可用")
        logger.error("可能需要手动生成证书:")
        logger.error("openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -sha256 -days 365 -nodes -subj \"/C=US/ST=Test/L=Test/O=Test/OU=Test/CN=localhost\"")
        return False

    return True 