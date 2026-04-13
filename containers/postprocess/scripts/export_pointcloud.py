#!/usr/bin/env python3
"""
BESS Point Cloud Export Tool

Exports point clouds from MCAP bags to LAS/LAZ format for use in
CloudCompare, QGIS, and other GIS tools.

Usage:
    python3 export_pointcloud.py <bag_path> --output <output.las>

Features:
    - Transforms points using SLAM odometry or ground truth
    - Supports georeferencing with UTM coordinates
    - Exports intensity, ring, and timestamp fields
    - Optional voxel downsampling
"""

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple

import laspy
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from tqdm import tqdm
from scipy.spatial.transform import Rotation


def extract_pointclouds_with_poses(
    bag_path: str,
    cloud_topic: str,
    odom_topic: str,
    max_clouds: Optional[int] = None,
    skip_every: int = 1
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Extract point clouds and transform to global frame using odometry."""

    all_points = []
    all_intensities = []
    all_times = []

    # First pass: collect all odometry
    odom_times = []
    odom_poses = []  # (position, quaternion)

    print(f"Reading odometry from {odom_topic}...")
    with Reader(bag_path) as reader:
        odom_conns = [c for c in reader.connections if c.topic == odom_topic]
        if not odom_conns:
            raise ValueError(f"Odometry topic {odom_topic} not found")

        for connection, timestamp, rawdata in reader.messages(connections=odom_conns):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t = timestamp * 1e-9
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            odom_times.append(t)
            odom_poses.append((
                np.array([pos.x, pos.y, pos.z]),
                np.array([ori.x, ori.y, ori.z, ori.w])
            ))

    odom_times = np.array(odom_times)
    print(f"  Found {len(odom_times)} odometry messages")

    if len(odom_times) == 0:
        raise ValueError("No odometry messages found")

    # Second pass: extract and transform point clouds
    print(f"Reading point clouds from {cloud_topic}...")
    cloud_count = 0

    with Reader(bag_path) as reader:
        cloud_conns = [c for c in reader.connections if c.topic == cloud_topic]
        if not cloud_conns:
            raise ValueError(f"Point cloud topic {cloud_topic} not found")

        total_msgs = sum(1 for _ in reader.messages(connections=cloud_conns))

    with Reader(bag_path) as reader:
        cloud_conns = [c for c in reader.connections if c.topic == cloud_topic]

        for connection, timestamp, rawdata in tqdm(
            reader.messages(connections=cloud_conns),
            total=total_msgs,
            desc="Processing clouds"
        ):
            cloud_count += 1
            if cloud_count % skip_every != 0:
                continue

            if max_clouds and len(all_points) >= max_clouds:
                break

            msg = deserialize_cdr(rawdata, connection.msgtype)
            cloud_time = timestamp * 1e-9

            # Find nearest odometry
            idx = np.searchsorted(odom_times, cloud_time)
            if idx >= len(odom_times):
                idx = len(odom_times) - 1
            elif idx > 0 and abs(odom_times[idx-1] - cloud_time) < abs(odom_times[idx] - cloud_time):
                idx = idx - 1

            position, quaternion = odom_poses[idx]
            rotation = Rotation.from_quat(quaternion)

            # Parse point cloud
            points, intensities = parse_pointcloud2(msg)

            if points is None or len(points) == 0:
                continue

            # Transform to global frame
            points_global = rotation.apply(points) + position

            all_points.append(points_global)
            all_intensities.append(intensities)
            all_times.append(np.full(len(points_global), cloud_time))

    if not all_points:
        raise ValueError("No point clouds extracted")

    print(f"  Processed {len(all_points)} clouds")

    return (
        np.vstack(all_points),
        np.concatenate(all_intensities),
        np.concatenate(all_times)
    )


def parse_pointcloud2(msg) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Parse ROS2 PointCloud2 message."""

    # Find field offsets
    field_info = {}
    for field in msg.fields:
        field_info[field.name] = (field.offset, field.datatype)

    if 'x' not in field_info or 'y' not in field_info or 'z' not in field_info:
        return None, None

    # Datatype mapping
    dtype_map = {
        1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
        5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64
    }

    point_step = msg.point_step
    num_points = msg.width * msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if len(data) < num_points * point_step:
        return None, None

    # Extract XYZ
    x_off, x_type = field_info['x']
    y_off, y_type = field_info['y']
    z_off, z_type = field_info['z']

    x = np.frombuffer(data, dtype=dtype_map[x_type], offset=x_off, count=num_points)
    y = np.frombuffer(data, dtype=dtype_map[y_type], offset=y_off, count=num_points)
    z = np.frombuffer(data, dtype=dtype_map[z_type], offset=z_off, count=num_points)

    # Handle strided access for point_step > 12
    if point_step > 12:
        indices = np.arange(num_points)
        x = data.view(dtype_map[x_type])[indices * (point_step // 4) + x_off // 4]
        y = data.view(dtype_map[y_type])[indices * (point_step // 4) + y_off // 4]
        z = data.view(dtype_map[z_type])[indices * (point_step // 4) + z_off // 4]

    points = np.column_stack([x, y, z]).astype(np.float64)

    # Filter invalid points
    valid = np.isfinite(points).all(axis=1) & (np.abs(points).max(axis=1) < 1000)
    points = points[valid]

    # Extract intensity if available
    if 'intensity' in field_info:
        i_off, i_type = field_info['intensity']
        intensity = np.frombuffer(data, dtype=dtype_map[i_type], offset=i_off, count=num_points)
        if point_step > 12:
            intensity = data.view(dtype_map[i_type])[indices * (point_step // 4) + i_off // 4]
        intensity = intensity[valid].astype(np.float32)
    else:
        intensity = np.ones(len(points), dtype=np.float32)

    return points, intensity


def voxel_downsample(points: np.ndarray, intensities: np.ndarray,
                     times: np.ndarray, voxel_size: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Voxel grid downsampling."""
    print(f"Downsampling with voxel size {voxel_size}m...")

    # Quantize to voxel grid
    voxel_indices = np.floor(points / voxel_size).astype(np.int32)

    # Use unique voxels
    _, unique_indices = np.unique(
        voxel_indices.view(dtype=[('x', np.int32), ('y', np.int32), ('z', np.int32)]).flatten(),
        return_index=True
    )

    print(f"  Reduced from {len(points)} to {len(unique_indices)} points")

    return points[unique_indices], intensities[unique_indices], times[unique_indices]


def write_las(output_path: str, points: np.ndarray, intensities: np.ndarray,
              times: Optional[np.ndarray] = None, crs_wkt: Optional[str] = None):
    """Write points to LAS/LAZ file."""

    print(f"Writing {len(points)} points to {output_path}...")

    # Create LAS file
    header = laspy.LasHeader(point_format=1, version="1.4")

    # Set scale and offset for precision
    header.scales = [0.01, 0.01, 0.01]  # 1cm precision — 0.001 overflows int32 with BNG coords
    header.offsets = [
        np.floor(points[:, 0].min()),
        np.floor(points[:, 1].min()),
        np.floor(points[:, 2].min())
    ]

    # Add CRS if provided
    if crs_wkt:
        header.add_crs(crs_wkt)

    las = laspy.LasData(header)

    # Set coordinates
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]

    # Set intensity (scale to 16-bit)
    intensity_norm = intensities / (intensities.max() + 1e-6)
    las.intensity = (intensity_norm * 65535).astype(np.uint16)

    # Set GPS time if available
    if times is not None:
        las.gps_time = times - times.min()

    # Compress if .laz
    if output_path.endswith('.laz'):
        las.write(output_path, laz_backend=laspy.compression.LazBackend.Laszip)
    else:
        las.write(output_path)

    print(f"  Done. File size: {Path(output_path).stat().st_size / 1e6:.1f} MB")


def main():
    parser = argparse.ArgumentParser(description="Export point clouds to LAS format")
    parser.add_argument("bag_path", help="Path to MCAP bag")
    parser.add_argument("--output", "-o", required=True, help="Output LAS/LAZ file")
    parser.add_argument("--cloud-topic", default="/ouster/points", help="Point cloud topic")
    parser.add_argument("--odom-topic", default="/slam/odometry", help="Odometry topic")
    parser.add_argument("--max-clouds", type=int, help="Maximum number of clouds to process")
    parser.add_argument("--skip-every", type=int, default=1, help="Process every Nth cloud")
    parser.add_argument("--voxel-size", type=float, help="Voxel size for downsampling (meters)")
    args = parser.parse_args()

    # Extract and transform point clouds
    points, intensities, times = extract_pointclouds_with_poses(
        args.bag_path,
        args.cloud_topic,
        args.odom_topic,
        args.max_clouds,
        args.skip_every
    )

    print(f"\nTotal points: {len(points)}")

    # Downsample if requested
    if args.voxel_size:
        points, intensities, times = voxel_downsample(
            points, intensities, times, args.voxel_size
        )

    # Write LAS
    write_las(args.output, points, intensities, times)

    print("\n=== Export Complete ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
