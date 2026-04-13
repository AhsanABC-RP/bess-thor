#!/usr/bin/env python3
"""
BESS Point Cloud Colorization

Projects RGB colors from camera images onto LiDAR point clouds.
Supports multiple cameras with known extrinsic calibration.

Usage:
    python3 colorize_pointcloud.py <bag_path> --output <output.las>

Requires camera calibration (intrinsics and extrinsics).
"""

import argparse
import sys
from pathlib import Path
from typing import Optional, Tuple, Dict

import cv2
import laspy
import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from scipy.spatial.transform import Rotation
from tqdm import tqdm


class CameraProjector:
    """Projects 3D points to camera image coordinates."""

    def __init__(self, K: np.ndarray, D: np.ndarray,
                 T_lidar_camera: np.ndarray, width: int, height: int):
        """
        Args:
            K: 3x3 camera intrinsic matrix
            D: Distortion coefficients (k1, k2, p1, p2, k3)
            T_lidar_camera: 4x4 transform from LiDAR to camera frame
            width, height: Image dimensions
        """
        self.K = K
        self.D = D
        self.T_lidar_camera = T_lidar_camera
        self.width = width
        self.height = height

    def project(self, points_lidar: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Project 3D points to 2D pixel coordinates.

        Returns:
            pixels: Nx2 array of (u, v) coordinates
            valid: Boolean mask of points that project into the image
        """
        # Transform to camera frame
        points_h = np.hstack([points_lidar, np.ones((len(points_lidar), 1))])
        points_cam = (self.T_lidar_camera @ points_h.T).T[:, :3]

        # Filter points behind camera
        valid = points_cam[:, 2] > 0.1

        # Project to image
        points_cam_valid = points_cam[valid]
        if len(points_cam_valid) == 0:
            return np.array([]), np.zeros(len(points_lidar), dtype=bool)

        # Normalize
        points_norm = points_cam_valid[:, :2] / points_cam_valid[:, 2:3]

        # Apply distortion (simplified - no distortion for now)
        # TODO: Add proper distortion correction

        # Apply intrinsics
        pixels = (self.K[:2, :2] @ points_norm.T).T + self.K[:2, 2]

        # Update valid mask for points in image bounds
        in_image = (
            (pixels[:, 0] >= 0) & (pixels[:, 0] < self.width) &
            (pixels[:, 1] >= 0) & (pixels[:, 1] < self.height)
        )

        full_valid = np.zeros(len(points_lidar), dtype=bool)
        full_valid[valid] = in_image

        full_pixels = np.zeros((len(points_lidar), 2))
        valid_indices = np.where(valid)[0]
        full_pixels[valid_indices[in_image]] = pixels[in_image]

        return full_pixels, full_valid


def load_calibration(calib_file: str) -> Dict[str, CameraProjector]:
    """Load camera calibration from YAML file."""
    import yaml

    with open(calib_file) as f:
        calib = yaml.safe_load(f)

    projectors = {}

    for cam_name, cam_calib in calib.get('cameras', {}).items():
        K = np.array(cam_calib['intrinsics']).reshape(3, 3)
        D = np.array(cam_calib.get('distortion', [0, 0, 0, 0, 0]))

        # Extrinsics: transform from LiDAR to camera
        extrinsics = cam_calib['extrinsics']
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('xyz', extrinsics['rotation']).as_matrix()
        T[:3, 3] = extrinsics['translation']

        projectors[cam_name] = CameraProjector(
            K, D, T,
            cam_calib['width'],
            cam_calib['height']
        )

    return projectors


def colorize_pointcloud_from_bag(
    bag_path: str,
    cloud_topic: str,
    image_topics: Dict[str, str],
    projectors: Dict[str, CameraProjector],
    odom_topic: str,
    max_clouds: Optional[int] = None,
    skip_every: int = 1
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Extract and colorize point clouds from bag.

    Returns:
        points: Nx3 array of XYZ coordinates
        colors: Nx3 array of RGB colors (0-255)
    """
    all_points = []
    all_colors = []

    # First pass: index all image timestamps
    print("Indexing images...")
    image_cache = {topic: {} for topic in image_topics.values()}

    with Reader(bag_path) as reader:
        for topic in image_topics.values():
            conns = [c for c in reader.connections if c.topic == topic]
            for connection, timestamp, rawdata in reader.messages(connections=conns):
                image_cache[topic][timestamp * 1e-9] = rawdata

    print(f"  Found {sum(len(v) for v in image_cache.values())} images")

    # Second pass: process point clouds with odometry
    print("Reading odometry...")
    odom_data = []
    with Reader(bag_path) as reader:
        odom_conns = [c for c in reader.connections if c.topic == odom_topic]
        for connection, timestamp, rawdata in reader.messages(connections=odom_conns):
            msg = deserialize_cdr(rawdata, connection.msgtype)
            t = timestamp * 1e-9
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            odom_data.append((t, np.array([pos.x, pos.y, pos.z]),
                            np.array([ori.x, ori.y, ori.z, ori.w])))

    odom_times = np.array([o[0] for o in odom_data])
    print(f"  Found {len(odom_data)} odometry messages")

    # Third pass: colorize point clouds
    print("Processing point clouds...")
    cloud_count = 0

    with Reader(bag_path) as reader:
        cloud_conns = [c for c in reader.connections if c.topic == cloud_topic]
        total_msgs = sum(1 for _ in reader.messages(connections=cloud_conns))

    with Reader(bag_path) as reader:
        cloud_conns = [c for c in reader.connections if c.topic == cloud_topic]
        image_conns = {t: [c for c in reader.connections if c.topic == t]
                      for t in image_topics.values()}

        for connection, timestamp, rawdata in tqdm(
            reader.messages(connections=cloud_conns),
            total=total_msgs,
            desc="Colorizing"
        ):
            cloud_count += 1
            if cloud_count % skip_every != 0:
                continue
            if max_clouds and len(all_points) >= max_clouds:
                break

            cloud_time = timestamp * 1e-9
            msg = deserialize_cdr(rawdata, connection.msgtype)

            # Parse point cloud
            points = parse_pointcloud2_xyz(msg)
            if len(points) == 0:
                continue

            # Find nearest odometry
            idx = np.searchsorted(odom_times, cloud_time)
            idx = min(max(idx, 0), len(odom_data) - 1)
            _, position, quaternion = odom_data[idx]
            rotation = Rotation.from_quat(quaternion)

            # Transform points to global frame
            points_global = rotation.apply(points) + position

            # Initialize colors (default gray)
            colors = np.full((len(points), 3), 128, dtype=np.uint8)

            # Find nearest images and colorize
            for cam_name, img_topic in image_topics.items():
                if cam_name not in projectors:
                    continue

                # Find nearest image
                img_times = np.array(list(image_cache[img_topic].keys()))
                if len(img_times) == 0:
                    continue

                img_idx = np.searchsorted(img_times, cloud_time)
                img_idx = min(max(img_idx, 0), len(img_times) - 1)

                if abs(img_times[img_idx] - cloud_time) > 0.1:  # 100ms threshold
                    continue

                # Load and decode image
                img_time = img_times[img_idx]
                img_rawdata = image_cache[img_topic][img_time]

                try:
                    img_conns = [c for c in reader.connections if c.topic == img_topic]
                    img_msg = deserialize_cdr(img_rawdata, img_conns[0].msgtype)

                    if 'compressed' in img_topic.lower():
                        img = cv2.imdecode(np.frombuffer(img_msg.data, np.uint8),
                                          cv2.IMREAD_COLOR)
                    else:
                        img = np.frombuffer(img_msg.data, np.uint8).reshape(
                            img_msg.height, img_msg.width, -1
                        )
                        if img.shape[2] == 1:
                            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

                    if img is None:
                        continue

                    # Project points and get colors
                    projector = projectors[cam_name]
                    pixels, valid = projector.project(points)

                    for i in np.where(valid)[0]:
                        u, v = int(pixels[i, 0]), int(pixels[i, 1])
                        if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                            colors[i] = img[v, u, ::-1]  # BGR to RGB

                except Exception as e:
                    continue

            all_points.append(points_global)
            all_colors.append(colors)

    if not all_points:
        raise ValueError("No point clouds processed")

    return np.vstack(all_points), np.vstack(all_colors)


def parse_pointcloud2_xyz(msg) -> np.ndarray:
    """Parse PointCloud2 to XYZ numpy array."""
    # Find field offsets
    x_offset = y_offset = z_offset = None
    for field in msg.fields:
        if field.name == 'x':
            x_offset = field.offset
        elif field.name == 'y':
            y_offset = field.offset
        elif field.name == 'z':
            z_offset = field.offset

    if None in (x_offset, y_offset, z_offset):
        return np.array([])

    point_step = msg.point_step
    num_points = msg.width * msg.height
    data = np.frombuffer(msg.data, dtype=np.uint8)

    if len(data) < num_points * point_step:
        return np.array([])

    data = data[:num_points * point_step].reshape(num_points, point_step)

    x = np.ascontiguousarray(data[:, x_offset:x_offset+4]).view(np.float32).flatten()
    y = np.ascontiguousarray(data[:, y_offset:y_offset+4]).view(np.float32).flatten()
    z = np.ascontiguousarray(data[:, z_offset:z_offset+4]).view(np.float32).flatten()

    points = np.column_stack([x, y, z])
    valid = np.isfinite(points).all(axis=1)

    return points[valid]


def write_colored_las(output_path: str, points: np.ndarray, colors: np.ndarray):
    """Write colored point cloud to LAS file."""
    print(f"Writing {len(points)} colored points to {output_path}...")

    header = laspy.LasHeader(point_format=2, version="1.4")
    header.scales = [0.01, 0.01, 0.01]  # 1cm precision — 0.001 overflows int32 with BNG coords
    header.offsets = [
        np.floor(points[:, 0].min()),
        np.floor(points[:, 1].min()),
        np.floor(points[:, 2].min())
    ]

    las = laspy.LasData(header)
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]

    # Colors in LAS are 16-bit
    las.red = (colors[:, 0].astype(np.uint16) * 256)
    las.green = (colors[:, 1].astype(np.uint16) * 256)
    las.blue = (colors[:, 2].astype(np.uint16) * 256)

    las.write(output_path)
    print(f"  Done. File size: {Path(output_path).stat().st_size / 1e6:.1f} MB")


def main():
    parser = argparse.ArgumentParser(description="Colorize point clouds from camera images")
    parser.add_argument("bag_path", help="Path to MCAP bag")
    parser.add_argument("--output", "-o", required=True, help="Output LAS file")
    parser.add_argument("--calib", "-c", help="Camera calibration YAML file")
    parser.add_argument("--cloud-topic", default="/ouster/points")
    parser.add_argument("--odom-topic", default="/slam/odometry")
    parser.add_argument("--max-clouds", type=int)
    parser.add_argument("--skip-every", type=int, default=5)
    args = parser.parse_args()

    # Default calibration if not provided
    if args.calib:
        projectors = load_calibration(args.calib)
    else:
        print("Warning: No calibration file provided. Using default (identity) transforms.")
        print("  Results will only be meaningful if cameras are well-aligned with LiDAR.")

        # Create placeholder projector with typical camera intrinsics
        K = np.array([
            [1000, 0, 2048],
            [0, 1000, 1536],
            [0, 0, 1]
        ])
        T = np.eye(4)

        projectors = {
            'camera1': CameraProjector(K, np.zeros(5), T, 4096, 3072),
        }

    # Image topics
    image_topics = {
        'camera1': '/camera1/camera_driver/image_masked',
        'camera2': '/camera2/camera_driver/image_masked',
        'camera3': '/camera3/camera_driver/image_masked',
    }

    # Process
    points, colors = colorize_pointcloud_from_bag(
        args.bag_path,
        args.cloud_topic,
        image_topics,
        projectors,
        args.odom_topic,
        args.max_clouds,
        args.skip_every
    )

    # Write output
    write_colored_las(args.output, points, colors)

    print("\n=== Colorization Complete ===")


if __name__ == "__main__":
    main()
