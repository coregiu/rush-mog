"""实车 bringup：center_server + STM32 底盘桥 + 可选 Pi 直连 ICM-20948。

启动前请确认：
- 树莓派/Orin 已装 ROS2 Humble、python3-serial；
- 底盘 udev 规则（示例 /etc/udev/rules.d/99-rush-mog.rules）：
    SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
      SYMLINK+="rush_base"
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

    center = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("center_server"), "launch", "center_server.launch.py"]
            )
        ),
        launch_arguments={
            "config_file": config_file,
            "start_base_bridge": "true",
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
            center,
        ]
    )
