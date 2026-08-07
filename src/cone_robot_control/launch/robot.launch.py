import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the master YAML configuration file
    pkg_share = get_package_share_directory('cone_robot_control')
    config_file = os.path.join(pkg_share, 'config', 'robot_config.yaml')

    # Cytron MDD10 Motor Controller Node
    mdd10_node = Node(
        package='cone_robot_control',
        executable='mdd10_motor_controller',
        name='mdd10_motor_controller',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        mdd10_node
    ])
