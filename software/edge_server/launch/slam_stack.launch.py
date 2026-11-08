"""可选：启动 slam_toolbox 在线建图（需要 ros-humble-slam-toolbox）。

    ros2 launch edge_server slam_stack.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("edge_server"), "config", "slam_toolbox.yaml"]
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("slam_toolbox"),
                            "launch",
                            "online_async_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "slam_params_file": params_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
