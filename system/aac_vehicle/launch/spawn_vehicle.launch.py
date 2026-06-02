import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share  = get_package_share_directory('aac_vehicle')
    gz_sim_pkg = get_package_share_directory('ros_gz_sim')

    # -------------------------------------------------------
    # Launch arguments
    # -------------------------------------------------------
    world_arg = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='Ignition world SDF file name (must be on IGN_GAZEBO_RESOURCE_PATH)')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock')

    world          = LaunchConfiguration('world')
    use_sim_time   = LaunchConfiguration('use_sim_time')

    # -------------------------------------------------------
    # Process xacro → robot_description string
    # -------------------------------------------------------
    xacro_file = os.path.join(pkg_share, 'urdf', 'aac_vehicle.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # -------------------------------------------------------
    # robot_state_publisher
    # -------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # -------------------------------------------------------
    # Ignition Gazebo (Fortress)
    # -------------------------------------------------------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world]}.items()
    )

    # -------------------------------------------------------
    # Spawn vehicle entity from robot_description topic
    # -------------------------------------------------------
    spawn_vehicle = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_aac_vehicle',
        arguments=[
            '-name', 'aac_vehicle',
            '-topic', '/robot_description',
            '-x', '0', '-y', '0', '-z', '0.08',   # z = wheel_radius above ground
        ],
        output='screen'
    )

    # -------------------------------------------------------
    # ros_gz_bridge: Ignition ↔ ROS2 topic bridge
    #
    #   Format: <ign_topic>@<ros_type>[<ign_type>
    #     [  = bidirectional
    #     [  = Ignition → ROS
    #     ]  = ROS → Ignition
    # -------------------------------------------------------
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Ackermann command: ROS → Ignition AckermannSteering plugin
            # Twist.linear.x = speed [m/s],  Twist.angular.z = steer_angle [rad]
            '/ackermann_twist_cmd_for_gazebo@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            # Odometry:         Ignition → ROS
            '/model/aac_vehicle/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            # Joint states:     Ignition → ROS
            '/world/empty/model/aac_vehicle/joint_state'
            '@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        remappings=[
            ('/model/aac_vehicle/odometry', '/odom'),
        ],
        output='screen'
    )

    # -------------------------------------------------------
    # Vehicle state publisher node
    # -------------------------------------------------------
    vehicle_node = Node(
        package='aac_vehicle',
        executable='vehicle_state_publisher_exe',
        name='vehicle_state_publisher',
        output='screen',
        parameters=[
            os.path.join(pkg_share, 'config', 'vehicle_params.yaml'),
            {'use_sim_time': use_sim_time},
        ]
    )

    return LaunchDescription([
        world_arg,
        use_sim_time_arg,
        robot_state_publisher,
        gz_sim,
        spawn_vehicle,
        bridge,
        vehicle_node,
    ])
