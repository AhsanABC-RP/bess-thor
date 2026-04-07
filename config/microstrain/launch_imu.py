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

    aux_port = DeclareLaunchArgument(
        'aux_port',
        default_value='',
        description='Auxiliary serial port for RTK'
    )

    # IMU Node - let params.yaml control all settings
    imu_node = Node(
        package='microstrain_inertial_driver',
        executable='microstrain_inertial_driver_node',
        name='microstrain_imu',
        output='screen',
        parameters=[
            config_file,
            {
                'port': LaunchConfiguration('port'),
                'aux_port': LaunchConfiguration('aux_port'),
            }
        ],
        remappings=[
            ('imu/data', '/imu/data'),
        ]
    )

    return LaunchDescription([
        port,
        baudrate,
        aux_port,
        imu_node,
    ])
