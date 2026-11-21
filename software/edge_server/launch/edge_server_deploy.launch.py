"""edge_server 部署启动文件：只起 Edge Server 主节点 + ROS 健康网关。

与 `edge_server.launch.py`（联调/全栈骨架）和 `robot_bringup.launch.py`（实车全栈）
的区别：这里**只**启动下面两个节点，不启动 VLA、感知、定位、导航、速度仲裁，
用于 systemd 开机自启，保证 80/8080/8090 三个端口只被这一个实例占用。

    edge_server_node    -> Web :80、MJPEG :8080、STM32 串口、/status
    health_monitor_node -> ROS 健康检查 + HTTP 网关 127.0.0.1:8090

用法：
    ros2 launch edge_server edge_server_deploy.launch.py
    ros2 launch edge_server edge_server_deploy.launch.py \
        config_file:=/opt/edge_server/config/deploy.yaml

配置优先级（未显式传 `config_file` 时）：
    环境变量 EDGE_SERVER_CONFIG > <install>/share/edge_server/config/config.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEPLOY_NODES = ("edge_server_node", "health_monitor_node")


def generate_launch_description() -> LaunchDescription:
    config_file = LaunchConfiguration("config_file")
    params = [{"config_file": config_file}]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.environ.get("EDGE_SERVER_CONFIG", ""),
                description="运行配置路径；留空则用 EDGE_SERVER_CONFIG 或安装目录默认配置",
            ),
            *[
                Node(
                    package="edge_server",
                    executable=name,
                    name=name,
                    output="screen",
                    emulate_tty=True,
                    parameters=params,
                )
                for name in DEPLOY_NODES
            ],
        ]
    )
