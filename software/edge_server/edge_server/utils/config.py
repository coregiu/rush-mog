import os

import yaml
from ament_index_python.packages import get_package_share_directory


def resolve_config_path(node) -> str:
    """配置优先级：ROS 参数 config_file > 环境变量 > 安装目录默认配置。"""
    if not node.has_parameter("config_file"):
        node.declare_parameter("config_file", "")
    path = node.get_parameter("config_file").get_parameter_value().string_value
    if not path:
        path = os.environ.get("EDGE_SERVER_CONFIG", "") or os.environ.get(
            "CENTER_SERVER_CONFIG", ""
        )
    if not path:
        path = os.path.join(
            get_package_share_directory("edge_server"), "config", "config.yaml"
        )
    return path


def load_config(node) -> dict:
    """读取 config.yaml；优先 edge_server:，兼容旧 center_server: 顶层命名空间。"""
    path = resolve_config_path(node)
    with open(path, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f) or {}
    cfg = root.get("edge_server", root.get("center_server", root))
    node.get_logger().info(f"Loaded edge_server config: {path}")
    return cfg
