#!/usr/bin/env python3
"""
BESS Dual Ouster LiDAR Launch File
Launches two Ouster sensors with PTP time synchronization
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():
    # Configuration file path
    config_file = os.environ.get('OUSTER_CONFIG', '/ros2_ws/config/ouster/params.yaml')

    # Declare launch arguments
    sensor1_hostname = DeclareLaunchArgument(
        'sensor1_hostname',
        default_value='10.0.1.11',
        description='Hostname/IP of first Ouster sensor'
    )

    sensor2_hostname = DeclareLaunchArgument(
        'sensor2_hostname',
        default_value='10.0.1.12',
        description='Hostname/IP of second Ouster sensor'
    )

    # First Ouster sensor (front-facing)
    ouster1_group = GroupAction([
        PushRosNamespace('ouster1'),
        Node(
            package='ouster_ros',
            executable='os_driver',
            name='os_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'sensor_hostname': LaunchConfiguration('sensor1_hostname'),
                    'lidar_port': 7501,
                    'imu_port': 7502,
                    'sensor_frame': 'ouster1_sensor',
                    'lidar_frame': 'ouster1_lidar',
                    'imu_frame': 'ouster1_imu',
                    'timestamp_mode': 'TIME_FROM_PTP_1588',
                    'ptp_utc_tai_offset': -37,
                }
            ],
            remappings=[
                ('points', '/ouster1/points'),
                ('imu', '/ouster1/imu'),
            ]
        )
    ])

    # Second Ouster sensor (rear-facing)
    ouster2_group = GroupAction([
        PushRosNamespace('ouster2'),
        Node(
            package='ouster_ros',
            executable='os_driver',
            name='os_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'sensor_hostname': LaunchConfiguration('sensor2_hostname'),
                    'lidar_port': 7503,
                    'imu_port': 7504,
                    'sensor_frame': 'ouster2_sensor',
                    'lidar_frame': 'ouster2_lidar',
                    'imu_frame': 'ouster2_imu',
                    'timestamp_mode': 'TIME_FROM_PTP_1588',
                    'ptp_utc_tai_offset': -37,
                }
            ],
            remappings=[
                ('points', '/ouster2/points'),
                ('imu', '/ouster2/imu'),
            ]
        )
    ])

    return LaunchDescription([
        sensor1_hostname,
        sensor2_hostname,
        ouster1_group,
        ouster2_group,
    ])
