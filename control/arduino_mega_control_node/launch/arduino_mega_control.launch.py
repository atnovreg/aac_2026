import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("arduino_mega_control_node"),
        "config",
        "arduino_mega_control_params.yaml",
    )

    param_file_launch_arg = DeclareLaunchArgument(
        "arduino_mega_control_param_file",
        default_value=params_file,
        description="Arduino Mega control node parameter file",
    )

    arduino_mega_control_node = Node(
        package="arduino_mega_control_node",
        executable="arduino_mega_control_node_exe",
        output={
            "stderr": "screen",
            "stdout": "screen",
        },
        emulate_tty=True,
        parameters=[
            LaunchConfiguration("arduino_mega_control_param_file"),
        ],
    )

    return LaunchDescription([
        param_file_launch_arg,
        arduino_mega_control_node,
    ])
