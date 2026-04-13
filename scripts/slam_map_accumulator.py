#!/usr/bin/env python3
# SLAM unified-map accumulator — subscribes to /slam/cloud_registered (FAST-LIO2,
# already in camera_init frame), voxel-hashes into a bounded global grid, and
# publishes /slam/map_voxel on a timer with TRANSIENT_LOCAL durability so late
# Foxglove subscribers latch onto the current map immediately.
#
# This replaces the FAST-LIO2 publish.map_en path, which accumulates pcl_wait_pub
# with no voxel filter and quickly produces 100 MB+ messages that choke CycloneDDS
# fragment reassembly (symptom: all SLAM topics go silent once /Laser_map is large).
#
# Design:
#   - Integer voxel key = floor(xyz / voxel_size) -> dedup via dict
#   - Hard cap on voxel count; oldest voxels evicted FIFO when exceeded
#   - Single publish timer, not every scan — Foxglove doesn't need 10 Hz of this

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
from collections import OrderedDict


class SlamMapAccumulator(Node):
    def __init__(self):
        super().__init__('slam_map_accumulator')

        self.declare_parameter('input_topic', '/slam/cloud_registered')
        self.declare_parameter('output_topic', '/slam/map_voxel')
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('voxel_size', 0.25)
        self.declare_parameter('max_voxels', 2_000_000)
        self.declare_parameter('publish_period_sec', 2.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)
        self.pub_period = float(self.get_parameter('publish_period_sec').value)

        self.voxels = OrderedDict()

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._on_cloud, sub_qos
        )
        self.pub = self.create_publisher(PointCloud2, self.output_topic, pub_qos)
        self.timer = self.create_timer(self.pub_period, self._on_timer)

        self._pending_scans = 0
        self._log_every = 20
        self.get_logger().info(
            f'accumulating {self.input_topic} -> {self.output_topic} '
            f'in frame {self.frame_id}, voxel={self.voxel_size}m, cap={self.max_voxels}'
        )

    def _on_cloud(self, msg: PointCloud2):
        structured = pc2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        if structured.size == 0:
            return
        xyz = np.stack(
            [structured['x'], structured['y'], structured['z']], axis=-1
        ).astype(np.float32, copy=False)
        xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
        if xyz.shape[0] == 0:
            return

        keys = np.floor(xyz / self.voxel_size).astype(np.int32)
        packed = keys[:, 0].astype(np.int64) * 73856093 \
            ^ keys[:, 1].astype(np.int64) * 19349663 \
            ^ keys[:, 2].astype(np.int64) * 83492791
        _, idx = np.unique(packed, return_index=True)
        unique_keys = keys[idx]
        unique_xyz = xyz[idx]

        for k, p in zip(unique_keys, unique_xyz):
            tk = (int(k[0]), int(k[1]), int(k[2]))
            if tk in self.voxels:
                self.voxels.move_to_end(tk)
            else:
                self.voxels[tk] = p
                if len(self.voxels) > self.max_voxels:
                    self.voxels.popitem(last=False)

        self._pending_scans += 1

    def _on_timer(self):
        if not self.voxels:
            return
        xyz = np.array(list(self.voxels.values()), dtype=np.float32)
        header_stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        header = type('H', (), {})()
        msg = pc2.create_cloud(
            header=self._make_header(header_stamp),
            fields=fields,
            points=xyz,
        )
        self.pub.publish(msg)

        self.get_logger().info(
            f'published /slam/map_voxel  voxels={len(self.voxels)}  '
            f'scans_since_last={self._pending_scans}'
        )
        self._pending_scans = 0

    def _make_header(self, stamp):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = self.frame_id
        return h


def main():
    rclpy.init()
    node = SlamMapAccumulator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
