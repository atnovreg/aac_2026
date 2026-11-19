"""AGX live MID-360: clockwise 90-degree mounting viewed from above."""
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = Path(__file__).resolve().parent
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('xfer_format', default_value='0',
                              description='0: PointCloud2; 1: Livox CustomMsg'),
        DeclareLaunchArgument('user_config_path',
                              default_value=str(config / 'MID360_config.json')),
        Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
             name='livox_lidar_publisher', output='screen', parameters=[{
                 'use_sim_time': False,
                 'xfer_format': ParameterValue(LaunchConfiguration('xfer_format'),
                                               value_type=int),
                 'publish_freq': 10.0,
                 'frame_id': 'livox_frame',
                 'user_config_path': LaunchConfiguration('user_config_path'),
             }]),
        # base_link is midway between the axles at hub height (0.055 m).
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='rear_axle_mount', output='screen',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'rear_axle',
                        '--x', '-0.1675'],  # Half the 0.335 m wheelbase.
             parameters=[{'use_sim_time': False}]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mid360_position', output='screen',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'lidar_vehicle',
                        '--x', '0.116', '--y', '0', '--z', '0.177'],
             parameters=[{'use_sim_time': False}]),
        # lidar_vehicle has vehicle-aligned axes at the point cloud origin.
        # Sensor +X points right (-Y); sensor +Y points forward (+X).
        # Keep driver/IMU raw axes unchanged; consumers use TF for rotation.
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='mid360_mount', output='screen',
             arguments=['--frame-id', 'lidar_vehicle',
                        '--child-frame-id', 'livox_frame',
                        '--yaw', str(-math.pi / 2)],
             parameters=[{'use_sim_time': False}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(config / 'mid360.rviz')],
             parameters=[{'use_sim_time': False}]),
    ])
