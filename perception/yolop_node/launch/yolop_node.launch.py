"""Launch file for the YOLOP lane/drivable-area node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("yolop_node")
    default_param_file = os.path.join(package_share, "config", "yolop_node.yaml")

    param_file = LaunchConfiguration("param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=default_param_file,
                description="Path to the YOLOP node parameter YAML file.",
            ),
            Node(
                package="yolop_node",
                executable="yolop_node",
                name="yolop_node",
                output="screen",
                parameters=[param_file],
            ),
        ]
    )
