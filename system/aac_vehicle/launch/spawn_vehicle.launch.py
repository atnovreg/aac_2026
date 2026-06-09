import os
import yaml
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

    # sensor_config.yaml → xacro mappings for sensor TF
    sensor_config_file = os.path.join(pkg_share, 'config', 'sensor_config.yaml')
    with open(sensor_config_file, 'r') as f:
        sensor_cfg = yaml.safe_load(f)

    sensor_mappings = {}
    for sensor, params in sensor_cfg.items():
        x, y, z = params['xyz']
        roll, pitch, yaw = params['rpy']
        sensor_mappings[f'{sensor}_x']     = str(x)
        sensor_mappings[f'{sensor}_y']     = str(y)
        sensor_mappings[f'{sensor}_z']     = str(z)
        sensor_mappings[f'{sensor}_roll']  = str(roll)
        sensor_mappings[f'{sensor}_pitch'] = str(pitch)
        sensor_mappings[f'{sensor}_yaw']   = str(yaw)

    xacro_file = os.path.join(pkg_share, 'urdf', 'aac_vehicle.urdf.xacro')
    robot_description = xacro.process_file(xacro_file, mappings=sensor_mappings).toxml()

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
            # Vehicle control & state
            '/ackermann_cmd@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/model/aac_vehicle/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/world/simple_track_world/dynamic_pose/info'
            '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/world/simple_track_world/model/aac_vehicle/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
            # RPLIDAR A1
            '/lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            # Cameras (C920e) — Ignition publishes image to /<topic>, camera_info to /camera_info
            '/camera_front@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_rear@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            # Ultrasonic sensors — modeled as narrow-beam gpu_lidar (ranges[0] = distance)
            '/ultrasonic_front/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultrasonic_rear/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultrasonic_left/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/ultrasonic_right/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/model/aac_vehicle/odometry', '/odom'),
            (
                '/world/simple_track_world/dynamic_pose/info',
                '/gazebo/dynamic_pose',
            ),
            ('/world/simple_track_world/model/aac_vehicle/joint_state', '/joint_states'),
            # Camera: Ignition publishes to /camera_front, remap to ROS convention
            ('/camera_front', '/camera_front/image_raw'),
            ('/camera_rear', '/camera_rear/image_raw'),
        ],
        output='screen'
    )

    ground_truth_node = Node(
        package='aac_vehicle',
        executable='gazebo_ground_truth_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'input_topic': '/gazebo/dynamic_pose',
            'output_topic': '/odom_gt',
            'model_name': 'aac_vehicle',
            'frame_id': 'map',
            'child_frame_id': 'base_link_gt',
        }],
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
        ground_truth_node,
        vehicle_node,
    ])
