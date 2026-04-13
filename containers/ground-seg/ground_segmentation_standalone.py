#!/usr/bin/env python3
"""
BESS Standalone Ground Segmentation Node — Patchwork++ Style

Runs independently of any SLAM engine. Publishes ground/nonground splits
that can be consumed by GLIM, FAST-LIO, KISS-ICP, or any downstream node.

CRITICAL: Preserves ALL Ouster point fields (48-byte point_step) in output.
This ensures GLIM, FAST-LIO, and recorders see the full native field set
(x, y, z, intensity, t, reflectivity, ring, ambient, range).

Subscribes:
    /ouster/points (sensor_msgs/PointCloud2)

Publishes:
    /ouster/points_nonground (sensor_msgs/PointCloud2)
    /ouster/points_ground (sensor_msgs/PointCloud2)

References:
    - Patchwork++: https://github.com/url-kaist/patchwork-plusplus
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
import time


class GroundSegmentationStandalone(Node):
    """Ground segmentation using concentric zone model with full field preservation."""

    def __init__(self):
        super().__init__('ground_segmentation')

        # Parameters
        self.declare_parameter('sensor_height', 0.5)
        self.declare_parameter('num_zones', 4)
        self.declare_parameter('num_sectors', 16)
        self.declare_parameter('num_rings_per_zone', [2, 4, 4, 4])
        self.declare_parameter('min_range', 2.0)
        self.declare_parameter('max_range', 80.0)
        self.declare_parameter('uprightness_threshold', 0.7)
        self.declare_parameter('num_seed_points', 20)
        self.declare_parameter('input_topic', '/ouster/points')
        self.declare_parameter('ground_topic', '/ouster/points_ground')
        self.declare_parameter('nonground_topic', '/ouster/points_nonground')

        self.sensor_height = self.get_parameter('sensor_height').value
        self.num_zones = self.get_parameter('num_zones').value
        self.num_sectors = self.get_parameter('num_sectors').value
        self.num_rings_per_zone = self.get_parameter('num_rings_per_zone').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.uprightness_threshold = self.get_parameter('uprightness_threshold').value
        self.num_seed_points = self.get_parameter('num_seed_points').value

        input_topic = self.get_parameter('input_topic').value
        ground_topic = self.get_parameter('ground_topic').value
        nonground_topic = self.get_parameter('nonground_topic').value

        # Zone boundaries (exponential spacing)
        self.zone_ranges = self._compute_zone_ranges()

        # QoS matching Ouster driver output
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cloud_sub = self.create_subscription(
            PointCloud2, input_topic, self.cloud_callback, sensor_qos
        )
        self.ground_pub = self.create_publisher(PointCloud2, ground_topic, 10)
        self.nonground_pub = self.create_publisher(PointCloud2, nonground_topic, 10)

        # Stats
        self._process_times = []
        self._x_offset = None
        self._y_offset = None
        self._z_offset = None

        self.get_logger().info(
            f'Ground Segmentation standalone started — '
            f'preserving all point fields\n'
            f'  Input: {input_topic}\n'
            f'  Ground: {ground_topic}\n'
            f'  Nonground: {nonground_topic}\n'
            f'  Range: {self.min_range}-{self.max_range}m, '
            f'Zones: {self.num_zones}, Sectors: {self.num_sectors}'
        )

    def _compute_zone_ranges(self) -> list:
        zone_ranges = [self.min_range]
        span = self.max_range - self.min_range
        for i in range(self.num_zones):
            ratio = (i + 1) / self.num_zones
            zone_ranges.append(self.min_range + span * (ratio ** 1.5))
        return zone_ranges

    def cloud_callback(self, msg: PointCloud2):
        t0 = time.time()

        point_step = msg.point_step
        num_points = msg.width * msg.height
        if num_points == 0:
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if len(raw) < num_points * point_step:
            return
        raw = raw[:num_points * point_step].reshape(num_points, point_step)

        # Cache field offsets on first message
        if self._x_offset is None:
            for field in msg.fields:
                if field.name == 'x':
                    self._x_offset = field.offset
                elif field.name == 'y':
                    self._y_offset = field.offset
                elif field.name == 'z':
                    self._z_offset = field.offset
            if None in (self._x_offset, self._y_offset, self._z_offset):
                self.get_logger().error('PointCloud2 missing x/y/z fields')
                return

        # Extract XYZ for classification only (raw bytes preserved for output)
        x = np.ascontiguousarray(raw[:, self._x_offset:self._x_offset+4]).view(np.float32).flatten()
        y = np.ascontiguousarray(raw[:, self._y_offset:self._y_offset+4]).view(np.float32).flatten()
        z = np.ascontiguousarray(raw[:, self._z_offset:self._z_offset+4]).view(np.float32).flatten()

        points = np.column_stack([x, y, z])

        # Filter invalid points
        valid = np.isfinite(points).all(axis=1)

        # Ground segmentation on valid points only
        ground_mask_valid = self._segment_ground(points[valid])

        # Map back to full index space
        ground_mask = np.zeros(num_points, dtype=bool)
        valid_indices = np.where(valid)[0]
        ground_mask[valid_indices[ground_mask_valid]] = True

        # Split raw byte rows by mask — preserves ALL fields
        ground_bytes = raw[ground_mask]
        nonground_bytes = raw[~ground_mask]

        # Publish with original field structure intact
        self._publish_raw(self.ground_pub, ground_bytes, msg)
        self._publish_raw(self.nonground_pub, nonground_bytes, msg)

        elapsed = time.time() - t0
        self._process_times.append(elapsed)
        if len(self._process_times) % 100 == 0:
            avg = np.mean(self._process_times[-100:])
            n_ground = int(ground_mask.sum())
            self.get_logger().info(
                f'Ground: {n_ground}/{num_points} pts '
                f'({n_ground*100//max(num_points,1)}%), '
                f'{avg*1000:.1f}ms avg, '
                f'point_step={msg.point_step}'
            )

    def _publish_raw(self, publisher, rows: np.ndarray, template: PointCloud2):
        """Publish point cloud preserving original field layout."""
        out = PointCloud2()
        out.header = template.header
        out.height = 1
        out.width = len(rows)
        out.fields = template.fields
        out.is_bigendian = template.is_bigendian
        out.point_step = template.point_step
        out.row_step = template.point_step * len(rows)
        out.data = rows.tobytes()
        out.is_dense = template.is_dense
        publisher.publish(out)

    def _segment_ground(self, points: np.ndarray) -> np.ndarray:
        """Segment ground using concentric zone model. Returns mask over input."""
        n = len(points)
        ground_mask = np.zeros(n, dtype=bool)

        xy = points[:, :2]
        ranges = np.linalg.norm(xy, axis=1)
        angles = np.arctan2(xy[:, 1], xy[:, 0]) + np.pi  # [0, 2pi)

        valid_range = (ranges >= self.min_range) & (ranges < self.max_range)
        sector_indices = np.clip(
            (angles / (2 * np.pi) * self.num_sectors).astype(np.int32),
            0, self.num_sectors - 1
        )

        for zone_idx in range(self.num_zones):
            z_min = self.zone_ranges[zone_idx]
            z_max = self.zone_ranges[zone_idx + 1]
            in_zone = valid_range & (ranges >= z_min) & (ranges < z_max)
            if not np.any(in_zone):
                continue

            for sec in range(self.num_sectors):
                in_sector = in_zone & (sector_indices == sec)
                idxs = np.where(in_sector)[0]
                if len(idxs) < 3:
                    continue
                ground_idxs = self._fit_ground_plane(points[idxs], idxs, zone_idx)
                ground_mask[ground_idxs] = True

        return ground_mask

    def _fit_ground_plane(self, sector_pts, global_idxs, zone_idx):
        """Fit ground plane via SVD on lowest seed points."""
        order = np.argsort(sector_pts[:, 2])
        sorted_pts = sector_pts[order]
        sorted_idxs = global_idxs[order]

        n_seeds = min(self.num_seed_points, len(sorted_pts) // 3)
        if n_seeds < 3:
            return np.array([], dtype=np.intp)

        seeds = sorted_pts[:n_seeds]
        centroid = seeds.mean(axis=0)
        _, _, vh = np.linalg.svd(seeds - centroid)
        normal = vh[2]
        if normal[2] < 0:
            normal = -normal

        if abs(normal[2]) < self.uprightness_threshold:
            return np.array([], dtype=np.intp)

        # Adaptive threshold: larger for farther zones
        threshold = (0.1 + 0.05 * zone_idx) * (1 + zone_idx * 0.2)
        dists = np.abs((sorted_pts - centroid) @ normal)
        return sorted_idxs[dists < threshold]


def main():
    rclpy.init()
    node = GroundSegmentationStandalone()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
