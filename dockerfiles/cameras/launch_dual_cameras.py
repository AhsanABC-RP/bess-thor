#!/usr/bin/env python3
"""
BESS Dual FLIR Oryx Camera Launch File
Launches two Oryx 10GigE cameras with hardware trigger sync
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
import os

def generate_launch_description():
    config_file = os.environ.get('CAMERA_CONFIG', '/ros2_ws/config/cameras/params.yaml')

    # Declare launch arguments
    camera1_serial = DeclareLaunchArgument(
        'camera1_serial',
        default_value='',
        description='Serial number of first Oryx camera'
    )

    camera2_serial = DeclareLaunchArgument(
        'camera2_serial',
        default_value='',
        description='Serial number of second Oryx camera'
    )

    # First camera (left)
    camera1_group = GroupAction([
        PushRosNamespace('camera1'),
        Node(
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name='camera_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'serial_number': LaunchConfiguration('camera1_serial'),
                    'frame_id': 'camera1_optical',
                    'acquisition_frame_rate_enable': False,
                    'trigger_mode': 'On',
                    'trigger_source': 'Line0',
                    'trigger_selector': 'FrameStart',
                    'ptp_enable': True,
                }
            ],
            remappings=[
                ('image_raw', '/camera1/image_raw'),
                ('camera_info', '/camera1/camera_info'),
            ]
        )
    ])

    # Second camera (right)
    camera2_group = GroupAction([
        PushRosNamespace('camera2'),
        Node(
            package='spinnaker_camera_driver',
            executable='camera_driver_node',
            name='camera_driver',
            output='screen',
            parameters=[
                config_file,
                {
                    'serial_number': LaunchConfiguration('camera2_serial'),
                    'frame_id': 'camera2_optical',
                    'acquisition_frame_rate_enable': False,
                    'trigger_mode': 'On',
                    'trigger_source': 'Line0',
                    'trigger_selector': 'FrameStart',
                    'ptp_enable': True,
                }
            ],
            remappings=[
                ('image_raw', '/camera2/image_raw'),
                ('camera_info', '/camera2/camera_info'),
            ]
        )
    ])

    return LaunchDescription([
        camera1_serial,
        camera2_serial,
        camera1_group,
        camera2_group,
    ])
