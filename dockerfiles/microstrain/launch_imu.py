#!/usr/bin/env python3
"""
BESS MicroStrain IMU Launch File
Launches MicroStrain 3DM-GQ7 IMU driver
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.environ.get('IMU_CONFIG', '/ros2_ws/config/microstrain/params.yaml')

    # Declare launch arguments
    port = DeclareLaunchArgument(
        'port',
        default_value='/dev/microstrain',
        description='Serial port for IMU'
    )

    baudrate = DeclareLaunchArgument(
        'baudrate',
        default_value='921600',
        description='Serial baudrate'
    )

    # IMU Node
    imu_node = Node(
        package='microstrain_inertial_driver',
        executable='microstrain_inertial_driver_node',
        name='microstrain_imu',
        output='screen',
        parameters=[
            config_file,
            {
                'port': LaunchConfiguration('port'),
                'baudrate': LaunchConfiguration('baudrate'),
                'frame_id': 'imu_link',
                'use_enu_frame': True,
                'filter_data_rate': 500,
                'imu_data_rate': 500,
                'publish_imu': True,
                'publish_filter': True,
                'publish_gnss': False,
            }
        ],
        remappings=[
            ('imu/data', '/imu/data'),
            ('filter/status', '/imu/filter_status'),
        ]
    )

    return LaunchDescription([
        port,
        baudrate,
        imu_node,
    ])
