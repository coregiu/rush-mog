"""轻量日志工具。

ROS2 节点本身使用 rclpy 日志（ros2 launch 输出 / ~/.ros/log）；
core 模块或无 ROS 上下文的脚本使用这里的 stdlib logger。
设置 CENTER_SERVER_LOG_FILE 可同时写入文件。
"""

import logging
import os


def get_logger(name: str = "center_server") -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        logger.setLevel(logging.INFO)
        fmt = logging.Formatter(
            "[%(asctime)s] [%(levelname)s] %(name)s: %(message)s"
        )
        stream = logging.StreamHandler()
        stream.setFormatter(fmt)
        logger.addHandler(stream)

        log_file = os.environ.get("CENTER_SERVER_LOG_FILE")
        if log_file:
            file_handler = logging.FileHandler(log_file, encoding="utf-8")
            file_handler.setFormatter(fmt)
            logger.addHandler(file_handler)
    return logger
