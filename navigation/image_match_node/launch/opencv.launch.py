"""Launch classical OpenCV map-based localization."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the OpenCV localization launch description."""
    package_share = get_package_share_directory('image_match_node')
    default_config = os.path.join(
        package_share, 'config', 'opencv_gazebo.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='OpenCV localization parameter file',
    )
    node = Node(
        package='image_match_node',
        executable='opencv_localization_node',
        name='opencv_localization',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )
    return LaunchDescription([config_arg, node])
