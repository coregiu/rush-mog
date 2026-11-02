from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


NODE_NAMES = [
    "center_bridge_node",
    "vla_inference_node",
    "action_generator_node",
    "reverse_operation_node",
    "safety_manager_node",
    "task_manager_node",
]


def generate_launch_description():
    # 默认空字符串表示使用安装目录 share/center_server/config/config.yaml
    config_file = LaunchConfiguration("config_file")

    nodes = [
        Node(
            package="center_server",
            executable=name,
            name=name,
            output="screen",
            parameters=[{"config_file": config_file}],
        )
        for name in NODE_NAMES
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Path to center_server config.yaml (empty = installed default)",
            ),
            *nodes,
        ]
    )
