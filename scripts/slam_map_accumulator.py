#!/usr/bin/env python3
# SLAM unified-map accumulator — subscribes to /fast_lio/cloud_registered
# (FAST-LIO2, already in camera_init frame), voxel-hashes into a bounded global
# grid, and publishes /fast_lio/map_voxel on a timer with TRANSIENT_LOCAL
# durability so late Foxglove subscribers latch onto the current map immediately.
#
# This replaces the FAST-LIO2 publish.map_en path, which accumulates pcl_wait_pub
# with no voxel filter and quickly produces 100 MB+ messages that choke CycloneDDS
# fragment reassembly (symptom: all SLAM topics go silent once /Laser_map is large).
#
# Design:
#   - Integer voxel key = floor(xyz / voxel_size) -> dedup via OrderedDict
#   - Value is (x, y, z, intensity) — intensity comes from FAST-LIO2's
#     /slam/cloud_registered which carries the Ouster reflectivity at offset 32.
#     Latest-seen wins on voxel re-hit (drift-displaced points overwrite stale
#     ones). This lets Foxglove color the map by intensity like a raw Ouster
#     scan instead of a blank z-colored blob.
#   - Hard cap on voxel count; oldest voxels evicted FIFO when exceeded
#   - Optional temporal decay (decay_sec>0): voxels older than decay_sec are
#     dropped on each publish. Useful when upstream SLAM is drifting and you
#     want a "rolling window" map instead of a permanent-accumulation one.
#   - Single publish timer, not every scan — Foxglove doesn't need 10 Hz of this

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import time
from collections import OrderedDict


class SlamMapAccumulator(Node):
    def __init__(self):
        super().__init__('slam_map_accumulator')

        self.declare_parameter('input_topic', '/fast_lio/cloud_registered')
        self.declare_parameter('output_topic', '/fast_lio/map_voxel')
        self.declare_parameter('frame_id', 'camera_init')
        self.declare_parameter('voxel_size', 0.25)
        self.declare_parameter('max_voxels', 2_000_000)
        self.declare_parameter('publish_period_sec', 2.0)
        # decay_sec: if >0, voxels not updated within this many seconds are
        # dropped on publish. Useful for stationary rigs where upstream SLAM
        # integrate-only drift would otherwise smear the map into noise.
        # 0 = permanent accumulation (default — unified world map).
        self.declare_parameter('decay_sec', 0.0)

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.max_voxels = int(self.get_parameter('max_voxels').value)
        self.pub_period = float(self.get_parameter('publish_period_sec').value)
        self.decay_sec = float(self.get_parameter('decay_sec').value)

        # Value per voxel: (x, y, z, intensity, last_update_monotonic)
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
        # Discover whether the input carries an intensity field. FAST-LIO2's
        # /slam/cloud_registered does; raw /ouster/points does; some upstream
        # stacks don't. When missing, fall back to a constant so Foxglove
        # can still color the map (it just won't be meaningful).
        has_intensity = any(f.name == 'intensity' for f in msg.fields)
        field_names = ('x', 'y', 'z', 'intensity') if has_intensity \
            else ('x', 'y', 'z')

        structured = pc2.read_points(
            msg, field_names=field_names, skip_nans=True
        )
        if structured.size == 0:
            return

        xyz = np.stack(
            [structured['x'], structured['y'], structured['z']], axis=-1
        ).astype(np.float32, copy=False)
        if has_intensity:
            intensity = structured['intensity'].astype(np.float32, copy=False)
        else:
            intensity = np.zeros(xyz.shape[0], dtype=np.float32)

        finite = np.all(np.isfinite(xyz), axis=1) & np.isfinite(intensity)
        xyz = xyz[finite]
        intensity = intensity[finite]
        if xyz.shape[0] == 0:
            return

        keys = np.floor(xyz / self.voxel_size).astype(np.int32)
        packed = keys[:, 0].astype(np.int64) * 73856093 \
            ^ keys[:, 1].astype(np.int64) * 19349663 \
            ^ keys[:, 2].astype(np.int64) * 83492791
        _, idx = np.unique(packed, return_index=True)
        unique_keys = keys[idx]
        unique_xyz = xyz[idx]
        unique_int = intensity[idx]

        now = time.monotonic()
        for k, p, ii in zip(unique_keys, unique_xyz, unique_int):
            tk = (int(k[0]), int(k[1]), int(k[2]))
            # Latest-seen wins — lets drift-displaced points overwrite stale
            # ones as the pose estimate corrects. Stamp each voxel for decay.
            self.voxels[tk] = (
                float(p[0]), float(p[1]), float(p[2]), float(ii), now
            )
            self.voxels.move_to_end(tk)
            if len(self.voxels) > self.max_voxels:
                self.voxels.popitem(last=False)

        self._pending_scans += 1

    def _on_timer(self):
        # Optional temporal decay: drop voxels older than decay_sec.
        # Iterating over an OrderedDict while mutating is cheap because we
        # break as soon as we hit a voxel newer than the cutoff (insert
        # order = last-update order since every update calls move_to_end).
        if self.decay_sec > 0 and self.voxels:
            cutoff = time.monotonic() - self.decay_sec
            evicted = 0
            while self.voxels:
                _, first_val = next(iter(self.voxels.items()))
                if first_val[4] >= cutoff:
                    break
                self.voxels.popitem(last=False)
                evicted += 1
            if evicted:
                self.get_logger().debug(f'decay evicted {evicted} stale voxels')

        if not self.voxels:
            return

        # Flatten to (N, 4): x, y, z, intensity. Drop the last-update stamp.
        pts = np.array(
            [(v[0], v[1], v[2], v[3]) for v in self.voxels.values()],
            dtype=np.float32,
        )
        header_stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = pc2.create_cloud(
            header=self._make_header(header_stamp),
            fields=fields,
            points=pts,
        )
        self.pub.publish(msg)

        self.get_logger().info(
            f'published {self.output_topic}  voxels={len(self.voxels)}  '
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
