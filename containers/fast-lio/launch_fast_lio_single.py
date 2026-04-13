#!/usr/bin/env python3
"""
BESS FAST-LIO2 Single LiDAR Launch File
Tightly-coupled LiDAR-inertial odometry for single Ouster + GQ7

Hardware:
- Ouster OS1-128 (serial: 122313000586)
  - IP: 10.0.1.10, Mode: 1024x20 (20Hz, 131K pts/scan)
  - Topic: /ouster/points, Frame: os_sensor, lidar_type: 3

- MicroStrain GQ7 IMU
  - Topic: /imu/data @ 500Hz (use /imu/data_guarded after guard node)
  - Used for: EKF fusion, motion compensation, state estimation

Architecture:
  /imu/data -> [IMU Guard] -> /imu/data_guarded -> [FAST-LIO2]
  /ouster/points -----------------------------------------> [FAST-LIO2] -> /fast_lio/odometry
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, EmitEvent, RegisterEventHandler
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file from environment or default
    config_file = os.environ.get(
        'FAST_LIO_CONFIG',
        '/ros2_ws/config/fast_lio_single.yaml'
    )

    # Launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    use_imu_guard = DeclareLaunchArgument(
        'use_imu_guard',
        default_value='true',
        description='Enable IMU timestamp guard'
    )

    return LaunchDescription([
        use_sim_time,
        use_imu_guard,

        # ========================================
        # Static TF Publishers
        # ========================================

        # TF: map -> odom (identity until loop closure from GLIM)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # TF: odom -> camera_init (FAST-LIO's odometry frame)
        # camera_init is where FAST-LIO started tracking
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_odom_to_camera_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init']
        ),

        # TF: body -> base_link (FAST-LIO's body frame to standard frame)
        # FAST-LIO publishes camera_init -> body, we connect body -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_body_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link']
        ),

        # TF: base_link -> os_sensor (Ouster LiDAR mount position)
        # TODO: Update with calibrated values from Phase 5B
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_ouster',
            arguments=[
                '0.0', '0.0', '0.5',     # x, y, z (meters) - ~0.5m above base
                '0.0', '0.0', '0.0',     # roll, pitch, yaw (radians)
                'base_link', 'os_sensor'
            ]
        ),

        # TF: base_link -> imu_link (GQ7 IMU mount position)
        # TODO: Update with calibrated values from Phase 5B
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_to_imu',
            arguments=[
                '0.0', '0.0', '0.3',     # x, y, z (meters)
                '0.0', '0.0', '0.0',     # roll, pitch, yaw (radians)
                'base_link', 'imu_link'
            ]
        ),

        # ----------------------------------------------------------------------
        # Cross-SLAM TF unification — DO NOT REMOVE
        # ----------------------------------------------------------------------
        # FAST-LIO2, DLIO, and GLIM each publish to disjoint TF roots
        # (camera_init / dlio_odom / glim_map). Without these statics, Foxglove
        # cannot transform GLIM or DLIO map points into the `map` display frame,
        # and they render as "no points" even though the topics carry data.
        # Identity is fine — the SLAMs disagree on global pose by definition,
        # so the displayed offset BETWEEN the maps is the comparison signal.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_glim_map',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'glim_map']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_map_to_dlio_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'dlio_odom']
        ),

        # ----------------------------------------------------------------------
        # Camera optical-frame statics — single broadcaster
        # ----------------------------------------------------------------------
        # Each camera driver stamps its images with `<cam>_optical_frame`. None
        # of those frames are children of base_link, so Foxglove can't transform
        # camera images into the SLAM map and the 3D panel logs "no transform
        # from <cam>_optical_frame to map" for every cam.
        #
        # 8 separate `static_transform_publisher` processes (one per cam) tipped
        # the CycloneDDS per-host participant pool over its limit and killed
        # fastlio_mapping with "Failed to find a free participant index for
        # domain 0". camera_tf_broadcaster.py latches all 8 statics from a
        # single rclpy participant. Identity placeholders for now; replace with
        # Phase 5B calibrated extrinsics when available.
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/camera_tf_broadcaster.py'],
            name='camera_tf_broadcaster',
            output='screen',
        ),

        # ========================================
        # IMU Guard Node
        # ========================================
        # Prevents SLAM crashes from GQ7 timestamp rewinds
        # Input: /imu/data (raw from GQ7)
        # Output: /imu/data_guarded (monotonic timestamps)
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/imu_guard.py'],
            name='imu_guard',
            output='screen',
        ),

        # ========================================
        # FAST-LIO2 Mapping Node
        # ========================================
        fastlio_node := Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fastlio_mapping',
            output='screen',
            parameters=[
                config_file,
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            remappings=[
                # Input topics come from common.lid_topic / common.imu_topic
                # in fast_lio_single.yaml — Ericsii ROS 2 port reads params
                # directly to create subscribers, so input remappings are a
                # no-op and must live in the yaml.

                # Output topics — remap into the /fast_lio/* namespace so
                # they don't collide with /dlio/* or /glim/* when multiple
                # SLAM backends run in parallel.
                ('/cloud_registered', '/fast_lio/cloud_registered'),
                ('/cloud_registered_body', '/fast_lio/cloud_registered_body'),
                ('/Odometry', '/fast_lio/odometry'),
                ('/path', '/fast_lio/path'),
            ]
        ),

        # When fastlio_mapping dies (e.g. segfault on malformed cloud),
        # shut the whole launch down so the container exits and Docker's
        # restart policy brings it back up. Without this, ros2 launch keeps
        # the healthcheck wrapper alive and masks the dead SLAM node.
        RegisterEventHandler(
            OnProcessExit(
                target_action=fastlio_node,
                on_exit=[EmitEvent(event=Shutdown(reason='fastlio_mapping exited'))],
            )
        ),

        # Ground Segmentation: now runs as standalone bess-ground-seg container
        # Publishes: /ouster/points_nonground, /ouster/points_ground

        # ========================================
        # Scan Context Loop Closure Node (optional)
        # ========================================
        # Provides place recognition for loop closure
        # Input: /fast_lio/cloud_registered, /fast_lio/odometry
        # Output: /fast_lio/loop_closure, /fast_lio/keyframes
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/scancontext_node.py'],
            name='scancontext_loop_closure',
            output='screen',
        ),

        # ----------------------------------------------------------------------
        # camera_init -> body TF re-broadcaster @ 50 Hz (stamp = now())
        # ----------------------------------------------------------------------
        # FAST-LIO2 natively publishes camera_init -> body with stamp =
        # lidar_end_time of the scan it just processed, which ends up ~500-700 ms
        # behind wall-clock once you add scan duration + iEKF compute. Sensor
        # messages are stamped at host wall-clock, so Foxglove's `map -> os_lidar`
        # lookup at message-time hits ExtrapolationException and refuses to
        # render Ouster points under `map`. Re-stamping the pose at now() closes
        # that gap; see tf_republisher.py for the full reasoning.
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/tf_republisher.py'],
            name='fast_lio_tf_republisher',
            output='screen',
        ),
    ])
