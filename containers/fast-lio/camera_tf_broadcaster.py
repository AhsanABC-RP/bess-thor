#!/usr/bin/env python3
# Single rclpy participant that latches placeholder identity transforms for
# every camera optical frame on the rig. Replaces 8 individual
# `static_transform_publisher` processes — each of those was its own DDS
# participant, and adding them tipped the CycloneDDS per-host participant
# pool over its limit, killing fastlio_mapping with
# "Failed to find a free participant index for domain 0".
#
# Replace the identity transforms here with calibrated values once the
# Phase 5B target calibration exists. Until then the cameras at least
# resolve to base_link so Foxglove can render image overlays.

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

# REP-103 optical frame: X right, Y down, Z forward.
# REP-105 base frame:    X forward, Y left, Z up.
# Convert base -> optical: rotate -pi/2 about X, then -pi/2 about Z.
# Equivalent quaternion: (-0.5, 0.5, -0.5, 0.5)
OPTICAL_QUAT = (-0.5, 0.5, -0.5, 0.5)

CAMERAS = (
    'thermal1_optical_frame',
    'thermal2_optical_frame',
    'thermal3_optical_frame',
    'thermal4_optical_frame',
    'lucid1_optical_frame',
    'lucid2_optical_frame',
    'blackfly1_optical_frame',
    'blackfly2_optical_frame',
)


def main():
    rclpy.init()
    node = Node('camera_tf_broadcaster')
    bcast = StaticTransformBroadcaster(node)

    transforms = []
    now = node.get_clock().now().to_msg()
    for child in CAMERAS:
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'base_link'
        t.child_frame_id = child
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = OPTICAL_QUAT[0]
        t.transform.rotation.y = OPTICAL_QUAT[1]
        t.transform.rotation.z = OPTICAL_QUAT[2]
        t.transform.rotation.w = OPTICAL_QUAT[3]
        transforms.append(t)

    bcast.sendTransform(transforms)
    node.get_logger().info(
        f'broadcast {len(transforms)} placeholder optical-frame statics on base_link'
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
