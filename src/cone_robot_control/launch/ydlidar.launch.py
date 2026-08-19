import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('cone_robot_control')
    parameter_file = os.path.join(pkg_share, 'config', 'ydlidar_tmini_params.yaml')

    # YDLIDAR Driver Node
    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file]
    )

    # Static Transform Publisher (base_link -> laser_frame)
    # Adjust x, y, z offset (0.0, 0.0, 0.15m height) to match physical mounting on robot
    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.15', '--yaw', '0.0', '--pitch', '0.0', '--roll', '0.0', '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']
    )

    return LaunchDescription([
        ydlidar_node,
        tf2_node
    ])
