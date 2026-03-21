#!/usr/bin/env python3
"""
BESS Odometry Launch File
Tesla CAN decoder + robot_localization EKF sensor fusion
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.environ.get('EKF_CONFIG', '/ros2_ws/config/ekf/ekf.yaml')

    # Arguments
    can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )

    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # SocketCAN bridge
    socketcan_bridge = Node(
        package='ros2_socketcan',
        executable='socket_can_receiver_node',
        name='socketcan_receiver',
        output='screen',
        parameters=[{
            'interface': LaunchConfiguration('can_interface'),
        }],
        remappings=[
            ('from_can_bus', '/can/rx'),
        ]
    )

    # Tesla CAN decoder
    tesla_decoder = Node(
        package='bess_odometry',  # Custom package
        executable='tesla_can_decoder',
        name='tesla_can_decoder',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'wheelbase': 2.875,
            'publish_rate': 50.0,
        }]
    )

    # robot_localization EKF
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    # Static TF for IMU
    tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu',
        arguments=['0', '0', '0.5', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        can_interface,
        use_sim_time,
        socketcan_bridge,
        tesla_decoder,
        ekf_node,
        tf_imu,
    ])
