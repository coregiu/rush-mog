from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="center_server",
            executable="bridge_node",
            name="bridge_node",
            output="screen",
        ),
        Node(
            package="center_server",
            executable="vla_inference_node",
            name="vla_inference_node",
            output="screen",
        ),
        Node(
            package="center_server",
            executable="action_generator_node",
            name="action_generator_node",
            output="screen",
        ),
        Node(
            package="center_server",
            executable="reverse_operation_node",
            name="reverse_operation_node",
            output="screen",
        ),
    ])
