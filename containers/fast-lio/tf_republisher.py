#!/usr/bin/env python3
# FAST-LIO2 stamps its camera_init -> body transform at lidar_end_time of the
# scan it just finished processing. With ~100 ms scan duration plus 100-400 ms
# iEKF + kdtree compute, the TF ends up ~500-700 ms behind wall-clock. Sensor
# messages (Ouster, IMU, cameras) carry host-wall stamps, so Foxglove's
# `map -> os_lidar` lookup at message-time raises ExtrapolationException and
# refuses to render Ouster points under `map` — even though the rest of the
# chain (base_link, os_sensor, os_lidar) is static and resolves fine.
#
# This node re-broadcasts the latest FAST-LIO2 pose as camera_init -> body
# with stamp = now() at 50 Hz, so the dynamic link is always current relative
# to incoming sensor messages. tf2 merges multiple publishers for the same
# child by picking the latest-stamped sample, so FAST-LIO2's own TF remains
# harmless alongside this one (it just loses the race).
#
# Cost: 50 Hz timer + one subscriber, no blocking.

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# Publish at 100 Hz and stamp each transform 50 ms in the future. The Ouster
# IMU is 500 Hz on host wall clock, so its stamps can be 2 ms older than the
# last even-100 Hz tick; biasing the tf header stamp forward guarantees the
# latest-in-buffer sample is always >= any incoming sensor message stamp, so
# `lookup_transform(target, source, msg.stamp)` never throws
# ExtrapolationException for Ouster/camera topics.
REPUBLISH_HZ = 100.0
FUTURE_BIAS_NS = 50_000_000  # 50 ms


class FastLioTfRepublisher(Node):
    def __init__(self):
        super().__init__('fast_lio_tf_republisher')
        self._latest_pose = None
        self._bcast = TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            Odometry, '/fast_lio/odometry', self._on_odom, qos
        )
        self.create_timer(1.0 / REPUBLISH_HZ, self._on_tick)
        self.get_logger().info(
            f'camera_init -> body re-broadcaster live @ {REPUBLISH_HZ:.0f} Hz '
            f'(stamp = now + {FUTURE_BIAS_NS / 1e6:.0f} ms)'
        )

    def _on_odom(self, msg: Odometry):
        self._latest_pose = msg.pose.pose

    def _on_tick(self):
        if self._latest_pose is None:
            return
        future = self.get_clock().now() + Duration(nanoseconds=FUTURE_BIAS_NS)
        t = TransformStamped()
        t.header.stamp = future.to_msg()
        t.header.frame_id = 'camera_init'
        t.child_frame_id = 'body'
        t.transform.translation.x = self._latest_pose.position.x
        t.transform.translation.y = self._latest_pose.position.y
        t.transform.translation.z = self._latest_pose.position.z
        t.transform.rotation = self._latest_pose.orientation
        self._bcast.sendTransform(t)


def main():
    rclpy.init()
    node = FastLioTfRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
