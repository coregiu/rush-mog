import os
from glob import glob

from setuptools import find_packages, setup

package_name = "center_server"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=("tests", "tests.*")),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("center_server/config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="coregiu",
    maintainer_email="xxx@xxx.com",
    description="Rush-MoG center_server: ROS2 VLA + SLAM/navigation/object-search center",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 兼容 v1 的节点名
            "center_bridge_node = ros_nodes.bridge_node:main",
            "bridge_node = ros_nodes.bridge_node:main",
            "vla_inference_node = ros_nodes.vla_inference_node:main",
            "action_generator_node = ros_nodes.action_generator_node:main",
            "reverse_operation_node = ros_nodes.reverse_operation_node:main",
            "safety_manager_node = ros_nodes.safety_manager_node:main",
            "task_manager_node = ros_nodes.task_manager_node:main",
            # v2：SLAM / 导航 / 定位 / 目标搜索
            "velocity_mux_node = ros_nodes.velocity_mux_node:main",
            "localization_node = ros_nodes.localization_node:main",
            "map_manager_node = ros_nodes.map_manager_node:main",
            "navigation_node = ros_nodes.navigation_node:main",
            "target_detector_node = ros_nodes.target_detector_node:main",
            "object_search_node = ros_nodes.object_search_node:main",
            # v2：底盘/IMU 适配（实车启用）
            "base_bridge_node = ros_nodes.base_bridge_node:main",
            "esp32_bridge_node = ros_nodes.esp32_bridge_node:main",
            "imu_node = ros_nodes.imu_node:main",
            # 健康检查：ROS 话题/服务 + 供 edge_center 调用的 HTTP 网关
            "health_monitor_node = ros_nodes.health_monitor_node:main",
        ],
    },
)
