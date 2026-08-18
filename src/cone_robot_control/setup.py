import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'cone_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bandi',
    maintainer_email='user@todo.todo',
    description='ROS 2 control package for Cytron MDD10 Rev 2.0 tank steering on Raspberry Pi 5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mdd10_motor_controller = cone_robot_control.mdd10_motor_controller:main',
            'bno08x_node = cone_robot_control.bno08x_node:main',
            'lc29h_gps_node = cone_robot_control.lc29h_gps_node:main',
            'base_station_caster = cone_robot_control.base_station_caster:main',
            'step_motion_controller = cone_robot_control.step_motion_controller:main',
            'teleop_keyboard = cone_robot_control.teleop_keyboard:main',
            'simple_publisher = cone_robot_control.simple_publisher:main',
        ],
    },
)
