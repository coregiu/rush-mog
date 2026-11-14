"""robot_localization EKF：轮式 /odom + ICM-20948 /imu/data -> odom->base_link。

注意：启动 EKF 前把 edge_server 的 base.publish_tf 设为 false，避免两个节点
同时发布 odom->base_link。用法：
    ros2 launch edge_server ekf.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("edge_server"), "config", "ekf.yaml"]
                ),
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
