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

    # Cytron MDD10 Motor Controller Node
    mdd10_node = Node(
        package='cone_robot_control',
        executable='mdd10_motor_controller',
        name='mdd10_motor_controller',
        output='screen',
        parameters=[config_file]
    )

    # Conditionally Include YDLIDAR Launch File
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ydlidar_launch_file),
        condition=IfCondition(LaunchConfiguration('launch_lidar'))
    )

    return LaunchDescription([
        declare_launch_lidar_cmd,
        mdd10_node,
        ydlidar_launch
    ])
