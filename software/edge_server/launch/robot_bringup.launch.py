"""实车 bringup：树莓派 Edge Server（网页/底盘） + edge_server + 可选 ICM-20948。

默认由 `edge_server_node` 直连 STM32 USART3（115200 ASCII 协议），同时提供
80 端口网页/API 和 8080 MJPEG；`/cmd_vel` 与网页手动控制共用同一个串口出口。
旧版 ESP32 HTTP 桥仅在 `base_transport:=esp32_http` 时启用。

启动前请确认：
- 树莓派/Orin 已装 ROS2 Humble；
- STM32 已烧录 body_control 固件，串口 115200 8N1，建议 udev 固定为 /dev/rush_base；
- 激光雷达默认启动（`start_lidar:=true`，`lidar.enabled=true`），UART4 接 D6/LD19；
- 80 端口需要 root 或 CAP_NET_BIND_SERVICE（systemd 示例见 software/edge_server/README.md）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    start_imu = LaunchConfiguration("start_imu")
    start_lidar = LaunchConfiguration("start_lidar")
    start_camera = LaunchConfiguration("start_camera")
    base_transport = LaunchConfiguration("base_transport")

    lidar = Node(
        package="edge_server",
        executable="lidar_node",
        output="screen",
        parameters=[{"config_file": config_file}],
        condition=IfCondition(start_lidar),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("edge_server"), "launch", "astra.launch.py"]
            )
        ),
        condition=IfCondition(start_camera),
    )

    center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("edge_server"), "launch", "edge_server.launch.py"]
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
                    [FindPackageShare("edge_server"), "config", "robot.yaml"]
                ),
                description="Robot config.yaml",
            ),
            DeclareLaunchArgument(
                "start_imu",
                default_value="false",
                description="true if ICM-20948 is wired to the companion computer I2C",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="start the LD19/D6 /scan driver (lidar.enabled must also be true)",
            ),
            DeclareLaunchArgument(
                "start_camera",
                default_value="false",
                description="start the Astra Pro color/depth driver (ros-humble-orbbec-camera)",
            ),
            DeclareLaunchArgument(
                "base_transport",
                default_value="edge_serial",
                description=(
                    "edge_serial = Pi Edge Server direct STM32 UART; "
                    "serial_binary = legacy binary; esp32_http = legacy ESP32 bridge"
                ),
            ),
            lidar,
            camera,
            center,
        ]
    )
