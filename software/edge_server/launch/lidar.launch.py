"""只启动 D6/LD19 激光雷达驱动，便于接线/波特率单测。"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("edge_server"), "config", "robot.yaml"]
                ),
                description="Robot config.yaml (lidar.* section)",
            ),
            Node(
                package="edge_server",
                executable="lidar_node",
                name="lidar_node",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
