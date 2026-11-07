"""center_server 一键启动（仿真/桌面联调默认配置）。

实车：
    ros2 launch center_server robot_bringup.launch.py
或：
    ros2 launch center_server center_server.launch.py \
        config_file:=.../robot.yaml start_base_bridge:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


CORE_NODES = [
    "center_bridge_node",
    "vla_inference_node",
    "action_generator_node",
    "reverse_operation_node",
    "safety_manager_node",
    "task_manager_node",
]


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_mux = LaunchConfiguration("start_velocity_mux")
    start_localization = LaunchConfiguration("start_localization")
    start_map = LaunchConfiguration("start_map_manager")
    start_nav = LaunchConfiguration("start_navigation")
    start_perception = LaunchConfiguration("start_perception")
    start_search = LaunchConfiguration("start_object_search")
    start_base = LaunchConfiguration("start_base_bridge")
    start_imu = LaunchConfiguration("start_imu")

    params = [
        {"config_file": config_file},
        {"use_sim_time": ParameterValue(use_sim_time, value_type=bool)},
    ]

    nodes = [
        Node(package="center_server", executable=name, output="screen", parameters=params)
        for name in CORE_NODES
    ]
    nodes += [
        Node(
            package="center_server",
            executable="velocity_mux_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_mux),
        ),
        Node(
            package="center_server",
            executable="localization_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_localization),
        ),
        Node(
            package="center_server",
            executable="map_manager_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_map),
        ),
        Node(
            package="center_server",
            executable="navigation_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_nav),
        ),
        Node(
            package="center_server",
            executable="target_detector_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_perception),
        ),
        Node(
            package="center_server",
            executable="object_search_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_search),
        ),
        Node(
            package="center_server",
            executable="base_bridge_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_base),
        ),
        Node(
            package="center_server",
            executable="imu_node",
            output="screen",
            parameters=params,
            condition=IfCondition(start_imu),
        ),
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value="",
                description="Path to config.yaml (empty = installed default)",
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use /clock"
            ),
            DeclareLaunchArgument(
                "start_velocity_mux",
                default_value="true",
                description="Start velocity arbitration node",
            ),
            DeclareLaunchArgument(
                "start_localization",
                default_value="true",
                description="Start localization health node",
            ),
            DeclareLaunchArgument(
                "start_map_manager",
                default_value="true",
                description="Start map/frontier manager",
            ),
            DeclareLaunchArgument(
                "start_navigation",
                default_value="true",
                description="Start navigate_to action server",
            ),
            DeclareLaunchArgument(
                "start_perception",
                default_value="true",
                description="Start target detector",
            ),
            DeclareLaunchArgument(
                "start_object_search",
                default_value="true",
                description="Start explore/search action servers",
            ),
            DeclareLaunchArgument(
                "start_base_bridge",
                default_value="false",
                description="Start STM32 serial bridge (real robot)",
            ),
            DeclareLaunchArgument(
                "start_imu",
                default_value="false",
                description="Start ICM-20948 I2C node on the companion computer",
            ),
            *nodes,
        ]
    )
