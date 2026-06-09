"""Launch EfficientLoFTR map-based visual localization."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the EfficientLoFTR localization launch description."""
    package_share = get_package_share_directory('image_match_node')
    default_config = os.path.join(
        package_share, 'config', 'efficientloftr_gazebo.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='EfficientLoFTR localization parameter file',
    )

    node = Node(
        package='image_match_node',
        executable='efficientloftr_localization_node',
        name='efficientloftr_localization',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, node])
