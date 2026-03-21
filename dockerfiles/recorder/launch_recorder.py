#!/usr/bin/env python3
"""
BESS Rosbag2 Recorder Launch File
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime

def generate_launch_description():
    bag_dir = os.environ.get('BAG_DIR', '/data/bags')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

    # Topics to record
    topics = [
        '/ouster1/points',
        '/ouster1/imu',
        '/ouster2/points',
        '/ouster2/imu',
        '/camera1/image_raw',
        '/camera1/camera_info',
        '/camera2/image_raw',
        '/camera2/camera_info',
        '/imu/data',
        '/slam/odometry',
        '/slam/cloud_registered',
        '/odometry/filtered',
        '/tf',
        '/tf_static',
    ]

    # Declare arguments
    output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value=bag_dir,
        description='Output directory for bags'
    )

    compression = DeclareLaunchArgument(
        'compression',
        default_value='zstd',
        description='Compression format'
    )

    # Build command
    cmd = [
        'ros2', 'bag', 'record',
        '--storage', 'mcap',
        '--compression-mode', 'file',
        '--compression-format', LaunchConfiguration('compression'),
        '--output', f'{bag_dir}/bess_{timestamp}',
    ] + topics

    record_process = ExecuteProcess(
        cmd=cmd,
        output='screen'
    )

    return LaunchDescription([
        output_dir,
        compression,
        record_process,
    ])
