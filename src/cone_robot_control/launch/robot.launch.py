import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cone_robot_control')
    config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    ydlidar_launch_file = os.path.join(pkg_share, 'launch', 'ydlidar.launch.py')

    # Declare Launch Arguments
    declare_launch_lidar_cmd = DeclareLaunchArgument(
        'launch_lidar',
        default_value='false',
        description='Whether to launch YDLIDAR T-mini Plus driver node'
    )

    declare_launch_imu_cmd = DeclareLaunchArgument(
        'launch_imu',
        default_value='true',
        description='Whether to launch BNO08x IMU driver node'
    )

    declare_launch_gps_cmd = DeclareLaunchArgument(
        'launch_gps',
        default_value='true',
        description='Whether to launch Waveshare LC29H GPS/RTK driver node'
    )

    declare_launch_foxglove_cmd = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Whether to launch Foxglove WebSocket Bridge (port 8765)'
    )

    # 1. Cytron MDD10 Motor Controller Node
    mdd10_node = Node(
        package='cone_robot_control',
        executable='mdd10_motor_controller',
        name='mdd10_motor_controller',
        output='screen',
        parameters=[config_file]
    )

    # 2. BNO08x IMU Driver Node
    bno08x_node = Node(
        package='cone_robot_control',
        executable='bno08x_node',
        name='bno08x_node',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('launch_imu'))
    )

    # 3. Static Transform Publisher (base_link -> imu_link)
    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=['--x', '0', '--y', '0', '--z', '0.05', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
        condition=IfCondition(LaunchConfiguration('launch_imu'))
    )

    # 4. Waveshare LC29H(DA) GPS/RTK Driver & NTRIP Rover Node
    lc29h_gps_node = Node(
        package='cone_robot_control',
        executable='lc29h_gps_node',
        name='lc29h_gps_node',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(LaunchConfiguration('launch_gps'))
    )

    # 5. Static Transform Publisher (base_link -> gps_link)
    base_to_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_gps_broadcaster',
        arguments=['--x', '0', '--y', '0', '--z', '0.10', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'gps_link'],
        condition=IfCondition(LaunchConfiguration('launch_gps'))
    )

    # 6. Foxglove WebSocket Bridge Node (port 8765)
    foxglove_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_foxglove'))
    )

    # 7. Conditionally Include YDLIDAR Launch File
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file),
        condition=IfCondition(LaunchConfiguration('launch_lidar'))
    )

    return LaunchDescription([
        declare_launch_lidar_cmd,
        declare_launch_imu_cmd,
        declare_launch_gps_cmd,
        declare_launch_foxglove_cmd,
        mdd10_node,
        bno08x_node,
        base_to_imu_tf,
        lc29h_gps_node,
        base_to_gps_tf,
        foxglove_node,
        ydlidar_launch
    ])