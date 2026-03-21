#!/usr/bin/env python3
"""
BESS FAST-LIO Multi-LiDAR Launch File
Dual Ouster SLAM with real-time odometry
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.environ.get('FAST_LIO_CONFIG', '/ros2_ws/config/fast_lio/fast_lio_multi.yaml')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    rviz_enable = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Enable RViz visualization'
    )

    # FAST-LIO node
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
            # Dual LiDAR merged pointcloud
            ('/cloud_registered', '/slam/cloud_registered'),
            ('/Odometry', '/slam/odometry'),
            ('/path', '/slam/path'),
        ]
    )

    # Pointcloud merger for dual LiDAR
    merger_node = Node(
        package='pcl_ros',
        executable='pointcloud_concatenate_data_synchronizer_node',
        name='lidar_merger',
        output='screen',
        parameters=[{
            'input_topics': ['/ouster1/points', '/ouster2/points'],
            'output_topic': '/merged_points',
            'output_frame': 'base_link',
            'approximate_sync': True,
            'queue_size': 10,
        }]
    )

    # Static TF for LiDAR positions (update with actual calibration)
    tf_ouster1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_ouster1',
        arguments=['0.5', '0', '0.3', '0', '0', '0', 'base_link', 'ouster1_sensor']
    )

    tf_ouster2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_ouster2',
        arguments=['-0.5', '0', '0.3', '0', '0', '3.14159', 'base_link', 'ouster2_sensor']
    )

    return LaunchDescription([
        use_sim_time,
        rviz_enable,
        tf_ouster1,
        tf_ouster2,
        merger_node,
        fast_lio_node,
    ])
