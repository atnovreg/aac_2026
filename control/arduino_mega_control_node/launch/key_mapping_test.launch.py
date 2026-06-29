import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("arduino_mega_control_node"),
        "config",
        "key_mapping_test_params.yaml",
    )

    arduino_mega_control_node = Node(
        package="arduino_mega_control_node",
        executable="arduino_mega_control_node_exe",
        output={
            "stderr": "screen",
            "stdout": "screen",
        },
        emulate_tty=True,
        parameters=[params_file],
    )

    return LaunchDescription([
        arduino_mega_control_node,
    ])
