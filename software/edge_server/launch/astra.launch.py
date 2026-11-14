"""Astra Pro 深度相机：Orbbec 驱动 + base_link -> camera_link 静态 TF。

用法：
    ros2 launch edge_server astra.launch.py
    ros2 launch edge_server astra.launch.py depth_registration:=true

依赖：ros-humble-orbbec-camera（Astra 系列驱动）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    depth_registration = LaunchConfiguration("depth_registration")
    enable_point_cloud = LaunchConfiguration("enable_point_cloud")
    color_width = LaunchConfiguration("color_width")
    color_height = LaunchConfiguration("color_height")
    color_fps = LaunchConfiguration("color_fps")
    depth_width = LaunchConfiguration("depth_width")
    depth_height = LaunchConfiguration("depth_height")
    depth_fps = LaunchConfiguration("depth_fps")
    tf_x = LaunchConfiguration("tf_x")
    tf_y = LaunchConfiguration("tf_y")
    tf_z = LaunchConfiguration("tf_z")
    tf_yaw = LaunchConfiguration("tf_yaw")
    tf_pitch = LaunchConfiguration("tf_pitch")
    tf_roll = LaunchConfiguration("tf_roll")

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orbbec_camera"), "launch", "astra.launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "depth_registration": depth_registration,
            "enable_point_cloud": enable_point_cloud,
            "enable_colored_point_cloud": "false",
            "color_width": color_width,
            "color_height": color_height,
            "color_fps": color_fps,
            "depth_width": depth_width,
            "depth_height": depth_height,
            "depth_fps": depth_fps,
            "publish_tf": "true",
        }.items(),
    )
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="astra_camera_tf",
        arguments=[
            "--x", tf_x,
            "--y", tf_y,
            "--z", tf_z,
            "--yaw", tf_yaw,
            "--pitch", tf_pitch,
            "--roll", tf_roll,
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
        output="log",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("depth_registration", default_value="true"),
            DeclareLaunchArgument("enable_point_cloud", default_value="false"),
            DeclareLaunchArgument("color_width", default_value="640"),
            DeclareLaunchArgument("color_height", default_value="480"),
            DeclareLaunchArgument("color_fps", default_value="10"),
            DeclareLaunchArgument("depth_width", default_value="640"),
            DeclareLaunchArgument("depth_height", default_value="480"),
            DeclareLaunchArgument("depth_fps", default_value="10"),
            DeclareLaunchArgument("tf_x", default_value="0.08"),
            DeclareLaunchArgument("tf_y", default_value="0.0"),
            DeclareLaunchArgument("tf_z", default_value="0.22"),
            DeclareLaunchArgument("tf_yaw", default_value="0.0"),
            DeclareLaunchArgument("tf_pitch", default_value="0.0"),
            DeclareLaunchArgument("tf_roll", default_value="0.0"),
            camera,
            static_tf,
        ]
    )
