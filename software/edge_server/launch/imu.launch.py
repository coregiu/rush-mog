"""只启动 ICM-20948 IMU 驱动，便于 I2C 接线/地址排查。"""

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
                description="Robot config.yaml (imu.* section)",
            ),
            Node(
                package="edge_server",
                executable="imu_node",
                name="imu_node",
                output="screen",
                parameters=[{"config_file": config_file}],
            ),
        ]
    )
