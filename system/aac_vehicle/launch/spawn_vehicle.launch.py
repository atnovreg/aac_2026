import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share       = get_package_share_directory('aac_vehicle')
    gz_sim_pkg      = get_package_share_directory('ros_gz_sim')
    track_pkg_share = get_package_share_directory('simple_gazebo_track')

    world_file = os.path.join(track_pkg_share, 'worlds', 'simple_track.world')
    track_model_path = os.path.join(track_pkg_share, 'models')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true')
    use_sim_time = LaunchConfiguration('use_sim_time')

    xacro_file = os.path.join(pkg_share, 'urdf', 'aac_vehicle.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        output='screen'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 5초 대기: Gazebo world 로딩 + /robot_description 발행 완료 후 spawn
    spawn_vehicle = TimerAction(
        period=5.0,
        actions=[Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', '/robot_description',
                '-name',  'aac_vehicle',
                '-x', '0', '-y', '-6.5', '-z', '0.15',
            ],
            output='screen'
        )]
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/ackermann_cmd@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/aac_vehicle/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/world/simple_track_world/model/aac_vehicle/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/model/aac_vehicle/odometry', '/odom_gt'),
            ('/world/simple_track_world/model/aac_vehicle/joint_state', '/joint_states'),
        ],
        output='screen'
    )

    vehicle_node = Node(
        package='aac_vehicle',
        executable='vehicle_state_publisher_exe',
        parameters=[
            os.path.join(pkg_share, 'config', 'vehicle_params.yaml'),
            {'use_sim_time': use_sim_time},
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                track_model_path, ':',
                track_pkg_share, ':',
                EnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                track_model_path, ':',
                track_pkg_share, ':',
                EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
            ],
        ),
        robot_state_publisher,
        gz_sim,
        spawn_vehicle,
        bridge,
        vehicle_node,
    ])
