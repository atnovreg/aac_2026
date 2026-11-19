"""Start MID360, the front-half filter, and RViz for manual recording."""
from pathlib import Path
import os
import platform

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    workspace = next(parent for parent in Path(__file__).resolve().parents
                     if (parent / '.f1tenth-workspace').is_file())
    driver = Path(get_package_share_directory('livox_ros_driver2'))
    build = Path(os.environ.get('F1TENTH_FRONT_BUILD', str(workspace/'artifacts/f1tenth'/('jazzy-'+platform.machine())/'front_filter')))
    return LaunchDescription([
        # This entry point starts a physical MID-360, including its display nodes.
        SetParameter(name='use_sim_time', value=False),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(driver / 'launch_ROS2' / 'msg_MID360_launch.py'))),
        Node(executable=str(build / 'livox_front_filter'),
             parameters=[{'front_yaw_deg': 0.0}], output='screen'),
        # A coincident sensor-local display frame makes the TF tree explicit.
        # This is not a world pose or a SLAM transform.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='livox_display_frame', output='screen',
             arguments=['--frame-id', 'livox_frame',
                        '--child-frame-id', 'livox_display']),
        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', str(workspace / 'config' / 'lidar_front180.rviz')]),
    ])
