"""Launch file for the Ultralytics road semantic segmentation node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("road_seg_node")
    default_param_file = os.path.join(package_share, "config", "road_seg_node.yaml")

    param_file = LaunchConfiguration("param_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "param_file",
                default_value=default_param_file,
                description="Path to the road segmentation parameter YAML file.",
            ),
            Node(
                package="road_seg_node",
                executable="road_seg_node",
                name="road_seg_node",
                output="screen",
                parameters=[param_file],
            ),
        ]
    )
