import os
from glob import glob

from setuptools import find_packages, setup

package_name = "center_server"

setup(
    name=package_name,
    version="0.1.0",
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
    description="Rush-MoG center_server: ROS2 VLA inference center",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # DESIGN.md 6.1 的节点名；bridge_node 保留为兼容别名
            "center_bridge_node = ros_nodes.bridge_node:main",
            "bridge_node = ros_nodes.bridge_node:main",
            "vla_inference_node = ros_nodes.vla_inference_node:main",
            "action_generator_node = ros_nodes.action_generator_node:main",
            "reverse_operation_node = ros_nodes.reverse_operation_node:main",
            "safety_manager_node = ros_nodes.safety_manager_node:main",
            "task_manager_node = ros_nodes.task_manager_node:main",
        ],
    },
)
