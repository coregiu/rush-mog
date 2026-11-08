"""实车 bringup：center_server + ESP32 底盘桥 + 可选 Pi 直连 ICM-20948。

默认：树莓派连 RUSH-MOG Wi-Fi，通过 ESP32 的 /api/body 获取编码器遥测，
通过 /cmd 的 raw: 命令下发速度。串口直连模式用 base_transport:=serial。

启动前请确认：
- 树莓派/Orin 已装 ROS2 Humble；
- ESP32 已烧录带 /api/body、raw: 的固件，地址默认 192.168.4.1；
- 激光雷达驱动、相机驱动、robot_localization EKF、slam_toolbox/Nav2 另行启动
  （见 README / DESIGN.md 的实车启动顺序）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    start_imu = LaunchConfiguration("start_imu")
    base_transport = LaunchConfiguration("base_transport")

    center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("center_server"), "launch", "center_server.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": config_file,
            "start_base_bridge": "true",
            "base_transport": base_transport,
            "start_imu": start_imu,
            "start_perception": "true",
            "start_navigation": "true",
            "start_localization": "true",
            "start_map_manager": "true",
            "start_object_search": "true",
            "start_velocity_mux": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("center_server"), "config", "robot.yaml"]
                ),
                description="Robot config.yaml",
            ),
            DeclareLaunchArgument(
                "start_imu",
                default_value="false",
                description="true if ICM-20948 is wired to the companion computer I2C",
            ),
            DeclareLaunchArgument(
                "base_transport",
                default_value="esp32",
                description="esp32 = STM32 via ESP32 HTTP; serial = direct STM32 UART",
            ),
            center,
        ]
    )
