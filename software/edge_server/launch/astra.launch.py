"""Astra Pro 深度相机：驱动自动选择 + base_link -> camera_link 静态 TF。

两代 Orbbec 驱动并存，按目标机已安装的包自动选：

* `astra_camera`（legacy，OpenNI2 + libuvc）：**初代 Astra Pro**（2bc5:0403 深度 +
  2bc5:0501 UVC 彩色）必须用它；`orbbec_camera`（新版 Orbbec SDK）对 0403 会报
  `Device response size(0) < response header size(10)` 并反复 reset，不能驱动。
* `orbbec_camera`（新 SDK）：Astra Pro Plus / Astra 2 / Gemini 系列用它。

两个都装时优先 legacy（本仓库的实车相机是初代 Astra Pro）。

用法：
    ros2 launch edge_server astra.launch.py
    ros2 launch edge_server astra.launch.py color_fps:=15 depth_registration:=true

注意：初代 Astra Pro 的彩色流 640x480 只支持 30/15 fps，**没有 10 fps**；
写 10 会让 libuvc 返回 Invalid mode 并让驱动的空句柄解引用崩溃。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def _pick_driver() -> tuple[str, str]:
    """返回 (driver, share_dir)：legacy astra_camera 优先，其次 orbbec_camera。"""
    for name in ("astra_camera", "orbbec_camera"):
        try:
            return name, get_package_share_directory(name)
        except Exception:  # noqa: BLE001 - 包不存在时 PackageNotFoundError
            continue
    raise RuntimeError(
        "未找到 Astra 驱动：请安装 ros-humble-orbbec-camera（新相机）"
        "或源码编译 astra_camera（初代 Astra Pro，见 docs/DEPLOY.md §6.6）"
    )


def generate_launch_description():
    driver, share = _pick_driver()

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

    if driver == "astra_camera":
        # legacy：launch 是 XML，逐项对齐 astra_pro.launch.xml 的参数名
        camera = IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(share, "launch", "astra_pro.launch.xml")),
            launch_arguments={
                "camera_name": "camera",
                "depth_registration": depth_registration,
                "enable_point_cloud": enable_point_cloud,
                "enable_colored_point_cloud": "false",
                "enable_ir": "false",
                "color_width": color_width,
                "color_height": color_height,
                "color_fps": color_fps,
                "depth_width": depth_width,
                "depth_height": depth_height,
                "depth_fps": depth_fps,
                "publish_tf": "true",
            }.items(),
        )
    else:
        camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "astra.launch.py")),
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
            DeclareLaunchArgument("color_fps", default_value="15"),
            DeclareLaunchArgument("depth_width", default_value="640"),
            DeclareLaunchArgument("depth_height", default_value="480"),
            DeclareLaunchArgument("depth_fps", default_value="15"),
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
