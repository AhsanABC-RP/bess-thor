#!/usr/bin/env python3
"""
BESS LiDAR Timestamp Filter — prevents GLIM crash on mid-drive restarts.

Problem: When GLIM restarts while the Ouster is running, the first frame
received has a stale timestamp (seconds in the past). GLIM tries to
integrate IMU from that timestamp but has no data that old, causing an
IndexedSlidingWindow crash.

Solution: Relay node that drops any PointCloud2 message whose header
timestamp is more than MAX_AGE_SEC behind the current ROS time.

Topology:
  /ouster/points (Ouster driver, BEST_EFFORT)
      |
      v
  [points_filter]  -- drops stale frames
      |
      v
  /ouster/points_filtered (BEST_EFFORT) --> GLIM subscribes here

Usage:
  python3 points_filter.py

  Environment variables:
    FILTER_MAX_AGE  — max age in seconds (default: 2.0)
    FILTER_INPUT    — input topic  (default: /ouster/points)
    FILTER_OUTPUT   — output topic (default: /ouster/points_filtered)
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2


class PointsFilter(Node):
    def __init__(self):
        super().__init__('points_filter')

        # Configuration from environment or defaults
        self.max_age = float(os.environ.get('FILTER_MAX_AGE', '2.0'))
        input_topic = os.environ.get('FILTER_INPUT', '/ouster/points')
        output_topic = os.environ.get('FILTER_OUTPUT', '/ouster/points_filtered')

        # Frame skipping: publish every Nth frame to GLIM, drop the rest.
        # FILTER_SKIP_N=2 means keep every 2nd frame (10Hz -> 5Hz).
        # FILTER_SKIP_N=1 means keep all (no skipping, default).
        self.skip_n = int(os.environ.get('FILTER_SKIP_N', '2'))
        self.frame_count = 0

        # Match Ouster driver QoS: BEST_EFFORT, VOLATILE, depth 1
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.pub = self.create_publisher(PointCloud2, output_topic, qos)
        self.sub = self.create_subscription(PointCloud2, input_topic, self._cb, qos)

        # Stats
        self.passed = 0
        self.dropped = 0
        self.skipped = 0

        # Gap detection: if consecutive frames are >gap_threshold apart,
        # GLIM's GTSAM smoother (smoother_lag=15s) will corrupt.
        # Log a FATAL marker so the watchdog can trigger a proactive restart.
        self.last_stamp_ns = None
        self.gap_threshold_sec = float(os.environ.get('FILTER_GAP_THRESHOLD', '5.0'))

        self.get_logger().info(
            f'Points filter active: {input_topic} -> {output_topic} '
            f'(max_age={self.max_age:.1f}s, skip_n={self.skip_n})'
        )

    def _cb(self, msg: PointCloud2):
        now = self.get_clock().now()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now_ns = now.nanoseconds
        age_sec = (now_ns - stamp_ns) / 1e9

        if abs(age_sec) > self.max_age:
            self.dropped += 1
            self.get_logger().warn(
                f'DROPPED stale frame: age={age_sec:+.3f}s '
                f'(stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}, '
                f'now={now_ns // 1_000_000_000}.{now_ns % 1_000_000_000:09d}) '
                f'[dropped={self.dropped}, passed={self.passed}]'
            )
            return

        # Frame skipping: only forward every Nth frame to GLIM.
        # Ouster still publishes all 10Hz for recording — we just don't feed GLIM every frame.
        self.frame_count += 1
        if self.skip_n > 1 and (self.frame_count % self.skip_n) != 0:
            self.skipped += 1
            return

        # Gap detection: check time delta between consecutive frames
        if self.last_stamp_ns is not None:
            delta_sec = abs(stamp_ns - self.last_stamp_ns) / 1e9
            if delta_sec > self.gap_threshold_sec:
                self.get_logger().fatal(
                    f'[GLIM-GAP-FATAL] {delta_sec:.1f}s gap between frames '
                    f'(threshold={self.gap_threshold_sec:.1f}s) — '
                    f'smoother state irrecoverable, GLIM needs restart'
                )
        self.last_stamp_ns = stamp_ns

        # Republish unchanged — rclpy passes the same Python object,
        # no serialization/deserialization overhead for intra-process.
        self.pub.publish(msg)
        self.passed += 1

        # Periodic status every 100 passed frames
        if self.passed % 100 == 0:
            self.get_logger().info(
                f'Filter stats: passed={self.passed}, skipped={self.skipped}, dropped={self.dropped}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointsFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Final stats: passed={node.passed}, skipped={node.skipped}, dropped={node.dropped}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
