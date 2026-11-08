"""可选：启动 Nav2 并把它最终速度重映射到 /nav/cmd_vel（需要 ros-humble-navigation2）。

    ros2 launch center_server nav2_stack.launch.py map:=/path/to/map.yaml

为什么需要重映射：
    center_server 的 velocity_mux 统一发布 /cmd_vel；Nav2 默认也发布 /cmd_vel，
    两者会互相覆盖。这里用 SetRemap 把 Nav2 组内所有 /cmd_vel 改到 /nav/cmd_vel，
    由 mux 以 nav 优先级统一仲裁。
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetRemap,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
            )
        ),
        launch_arguments={
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("nav2_bringup"), "launch", "localization_launch.py"]
            )
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("center_server"), "config", "nav2_params.yaml"]
                ),
            ),
            DeclareLaunchArgument("map", default_value="", description="Map YAML path"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            GroupAction(
                [
                    SetRemap(src="/cmd_vel", dst="/nav/cmd_vel"),
                    SetRemap(src="cmd_vel", dst="/nav/cmd_vel"),
                    SetRemap(src="/cmd_vel_smoothed", dst="/nav/cmd_vel"),
                    SetRemap(src="cmd_vel_smoothed", dst="/nav/cmd_vel"),
                    navigation,
                    localization,
                ]
            ),
        ]
    )
