#!/usr/bin/env python3
"""Accumulate FAST-LIO /cloud_registered scans into a map PLY file.
Saves incrementally every 100 keyframes so nothing is lost on kill.
"""
import sys, time, atexit
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2

KEYFRAME_INTERVAL = 5   # Keep every 5th scan (2Hz from 10Hz)
VOXEL_SIZE = 0.1         # 10cm voxel for final map
SAVE_EVERY = 100         # Save to disk every 100 keyframes

def log(msg):
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()

def pc2_to_xyz(msg):
    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(-1, msg.point_step)
    offsets = {f.name: f.offset for f in msg.fields}
    if 'x' not in offsets:
        return np.empty((0, 3), dtype=np.float32)
    x = np.frombuffer(data[:, offsets['x']:offsets['x']+4].tobytes(), dtype=np.float32)
    y = np.frombuffer(data[:, offsets['y']:offsets['y']+4].tobytes(), dtype=np.float32)
    z = np.frombuffer(data[:, offsets['z']:offsets['z']+4].tobytes(), dtype=np.float32)
    pts = np.column_stack([x, y, z])
    valid = np.all(np.isfinite(pts), axis=1) & (np.abs(pts).max(axis=1) < 10000)
    return pts[valid]

def voxel_downsample(pts, voxel_size):
    if len(pts) == 0:
        return pts
    keys = (pts / voxel_size).astype(np.int32)
    _, idx, counts = np.unique(
        keys[:, 0].astype(np.int64) * 1000000 + keys[:, 1].astype(np.int64) * 1000 + keys[:, 2].astype(np.int64),
        return_inverse=True, return_counts=True
    )
    centroids = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(centroids, idx, pts)
    centroids /= counts[:, None]
    return centroids.astype(np.float32)

def save_ply(pts, path):
    n = len(pts)
    header = f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.astype(np.float32).tobytes())
    xr = pts[:, 0].max() - pts[:, 0].min()
    yr = pts[:, 1].max() - pts[:, 1].min()
    zr = pts[:, 2].max() - pts[:, 2].min()
    log(f"SAVED {n:,} pts to {path} | extent {xr:.0f}m x {yr:.0f}m x {zr:.0f}m")

class MapAccumulator(Node):
    def __init__(self, output_path):
        super().__init__('map_accumulator')
        self.output_path = output_path
        self.all_points = []
        self.total_points = 0
        self.scan_count = 0
        self.keyframe_count = 0
        self.start_time = time.time()
        self.saved = False

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(PointCloud2, '/cloud_registered', self.cb, qos)
        log(f'Listening on /cloud_registered, keyframe every {KEYFRAME_INTERVAL} scans, saving every {SAVE_EVERY} keyframes')

    def cb(self, msg):
        self.scan_count += 1
        if self.scan_count == 1:
            log(f"First scan received! width={msg.width} point_step={msg.point_step}")
        if self.scan_count % KEYFRAME_INTERVAL != 0:
            return
        try:
            pts = pc2_to_xyz(msg)
        except Exception as e:
            log(f"ERROR: {e}")
            return
        if len(pts) == 0:
            return
        self.keyframe_count += 1
        self.all_points.append(pts)
        self.total_points += len(pts)
        elapsed = time.time() - self.start_time
        if self.keyframe_count % 10 == 0:
            log(f"[{elapsed:.0f}s] {self.keyframe_count} keyframes, {self.total_points:,} pts, scans={self.scan_count}")
        # Incremental save every SAVE_EVERY keyframes
        if self.keyframe_count % SAVE_EVERY == 0:
            self.save()

    def save(self):
        if not self.all_points:
            log("NO POINTS — nothing to save")
            return
        log(f"Merging {self.keyframe_count} keyframes ({self.total_points:,} raw pts)...")
        merged = np.vstack(self.all_points)
        log(f"Voxel downsampling at {VOXEL_SIZE}m...")
        ds = voxel_downsample(merged, VOXEL_SIZE)
        save_ply(ds, self.output_path)
        self.saved = True

def main():
    output = sys.argv[1] if len(sys.argv) > 1 else '/data/maps/fastlio_map_output.ply'
    rclpy.init()
    node = MapAccumulator(output)
    # atexit as last resort
    atexit.register(lambda: node.save() if not node.saved else None)
    try:
        rclpy.spin(node)
    except BaseException:
        node.save()

if __name__ == '__main__':
    main()
