import os
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory('cone_robot_control')
    config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    ydlidar_launch_file = os.path.join(pkg_share, 'launch', 'ydlidar.launch.py')

    robot_type_val = LaunchConfiguration('robot_type').perform(context).lower()
    launch_gps_val = LaunchConfiguration('launch_gps').perform(context).lower() == 'true'
    launch_lidar_val = LaunchConfiguration('launch_lidar').perform(context).lower() == 'true'
    launch_imu_val = LaunchConfiguration('launch_imu').perform(context).lower() == 'true'
    launch_foxglove_val = LaunchConfiguration('launch_foxglove').perform(context).lower() == 'true'
    launch_motion_val = LaunchConfiguration('launch_motion_controller').perform(context).lower() == 'true'

    # Resolve active hardware based on robot_type preset
    if robot_type_val == 'gps':
        enable_gps = True
        enable_lidar = False
    elif robot_type_val == 'lidar':
        enable_gps = False
        enable_lidar = True
    elif robot_type_val == 'all':
        enable_gps = True
        enable_lidar = True
    else:  # Custom / manual flags
        enable_gps = launch_gps_val
        enable_lidar = launch_lidar_val

    nodes_to_launch = []

    # 1. Cytron MDD10 Motor Controller Node
    nodes_to_launch.append(
        Node(
            package='cone_robot_control',
            executable='mdd10_motor_controller',
            name='mdd10_motor_controller',
            output='screen',
            parameters=[config_file]
        )
    )

    # 2. BNO08x IMU Driver Node & Static TF
    if launch_imu_val:
        nodes_to_launch.append(
            Node(
                package='cone_robot_control',
                executable='bno08x_node',
                name='bno08x_node',
                output='screen',
                parameters=[config_file]
            )
        )
        nodes_to_launch.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_imu_broadcaster',
                arguments=['--x', '0', '--y', '0', '--z', '0.05', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'imu_link']
            )
        )

    # 3. Step Motion Controller Node (Turn X° and Drive Y cm)
    if launch_motion_val:
        nodes_to_launch.append(
            Node(
                package='cone_robot_control',
                executable='step_motion_controller',
                name='step_motion_controller',
                output='screen',
                parameters=[config_file]
            )
        )

    # 4. Waveshare LC29H(DA) GPS/RTK Driver & Static TF
    if enable_gps:
        nodes_to_launch.append(
            Node(
                package='cone_robot_control',
                executable='lc29h_gps_node',
                name='lc29h_gps_node',
                output='screen',
                parameters=[config_file]
            )
        )
        nodes_to_launch.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_to_gps_broadcaster',
                arguments=['--x', '0', '--y', '0', '--z', '0.10', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'gps_link']
            )
        )

    # 5. YDLIDAR T-mini Plus Driver & 2D Laser Odometry
    if enable_lidar:
        if os.path.exists(ydlidar_launch_file):
            nodes_to_launch.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(ydlidar_launch_file)
                )
            )
        # 2D Laser Odometry Node (converts /scan -> /odom) if rf2o_laser_odometry is installed
        try:
            get_package_share_directory('rf2o_laser_odometry')
            nodes_to_launch.append(
                Node(
                    package='rf2o_laser_odometry',
                    executable='rf2o_laser_odometry_node',
                    name='rf2o_laser_odometry',
                    output='log',  # Redirect verbose odom INFO spam to log file, not terminal
                    parameters=[{
                        'laser_scan_topic': '/scan',
                        'odom_topic': '/odom',
                        'publish_tf': False,
                        'base_frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'laser_frame_id': 'laser_frame',
                        'freq': 6.0,  # Match YDLidar T-mini Plus scan rate (6Hz)
                        'init_pose_from_topic': ''  # CRITICAL: empty = don't wait for /base_pose_ground_truth (Gazebo-only topic)
                    }]
                )
            )
        except PackageNotFoundError:
            print("\n" + "=" * 75)
            print("  ⚠️  WARNING: 'rf2o_laser_odometry' is NOT installed!")
            print("  Topic /odom will NOT publish until it is installed.")
            print("  To install: sudo apt install ros-jazzy-rf2o-laser-odometry")
            print("  Or from source: git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git src/rf2o_laser_odometry")
            print("=" * 75 + "\n")

    # 6. Foxglove WebSocket Bridge Node (port 8765)
    if launch_foxglove_val:
        try:
            get_package_share_directory('foxglove_bridge')
            nodes_to_launch.append(
                Node(
                    package='foxglove_bridge',
                    executable='foxglove_bridge',
                    name='foxglove_bridge',
                    output='screen'
                )
            )
        except PackageNotFoundError:
            print("\n" + "=" * 75)
            print("  ⚠️  WARNING: 'foxglove_bridge' is NOT installed!")
            print("  To install: sudo apt install ros-jazzy-foxglove-bridge")
            print("=" * 75 + "\n")

    return nodes_to_launch


def generate_launch_description():
    # Declare Launch Arguments
    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type',
        default_value='all',
        description='Robot hardware preset: "all" (IMU+GPS+LiDAR), "gps" (IMU+GPS), "lidar" (IMU+LiDAR), or "custom"'
    )

    declare_launch_lidar_cmd = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Whether to launch YDLIDAR T-mini Plus driver node'
    )

    declare_launch_gps_cmd = DeclareLaunchArgument(
        'launch_gps',
        default_value='true',
        description='Whether to launch Waveshare LC29H GPS/RTK driver node'
    )

    declare_launch_imu_cmd = DeclareLaunchArgument(
        'launch_imu',
        default_value='true',
        description='Whether to launch BNO08x IMU driver node'
    )

    declare_launch_motion_cmd = DeclareLaunchArgument(
        'launch_motion_controller',
        default_value='true',
        description='Whether to launch Step Motion Controller node'
    )

    declare_launch_foxglove_cmd = DeclareLaunchArgument(
        'launch_foxglove',
        default_value='true',
        description='Whether to launch Foxglove WebSocket Bridge (port 8765)'
    )

    return LaunchDescription([
        declare_robot_type_cmd,
        declare_launch_lidar_cmd,
        declare_launch_gps_cmd,
        declare_launch_imu_cmd,
        declare_launch_motion_cmd,
        declare_launch_foxglove_cmd,
        OpaqueFunction(function=launch_setup)
    ])