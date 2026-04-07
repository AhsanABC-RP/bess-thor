#!/usr/bin/env python3
"""SLAM Dense Reinjection: Transform raw LiDAR scans by SLAM trajectory poses.

Takes a bag with raw /ouster/points and SLAM /Odometry, matches by timestamp,
transforms full-resolution scans into world frame, writes dense PLY map.

Usage:
    python3 slam_reinject.py <bag_dir> [--odom-topic /Odometry] [--lidar-topic /ouster/points]
        [--output map.ply] [--every-n 5] [--voxel 0.05] [--max-range 100]
"""
import argparse, sys, time, struct
import numpy as np

def log(msg):
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()

def quat_to_rot(qx, qy, qz, qw):
    """Quaternion to 3x3 rotation matrix."""
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),    1-2*(qx*qx+qy*qy)]
    ])

def pose_to_matrix(pos, quat):
    """Position [x,y,z] + quaternion [x,y,z,w] → 4x4 transform."""
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(*quat)
    T[:3, 3] = pos
    return T

def pc2_to_xyz(data, point_step, fields):
    """Parse PointCloud2 binary data to Nx3 float32."""
    offsets = {f['name']: f['offset'] for f in fields}
    if 'x' not in offsets:
        return np.empty((0, 3), dtype=np.float32)
    arr = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step)
    x = np.frombuffer(arr[:, offsets['x']:offsets['x']+4].tobytes(), dtype=np.float32)
    y = np.frombuffer(arr[:, offsets['y']:offsets['y']+4].tobytes(), dtype=np.float32)
    z = np.frombuffer(arr[:, offsets['z']:offsets['z']+4].tobytes(), dtype=np.float32)
    pts = np.column_stack([x, y, z])
    valid = np.all(np.isfinite(pts), axis=1)
    return pts[valid]

def save_ply(pts, path):
    n = len(pts)
    header = f"ply\nformat binary_little_endian 1.0\nelement vertex {n}\nproperty float x\nproperty float y\nproperty float z\nend_header\n"
    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(pts.astype(np.float32).tobytes())
    log(f"Saved {n:,} points to {path}")

def voxel_downsample(pts, voxel_size):
    if len(pts) == 0 or voxel_size <= 0:
        return pts
    keys = (pts / voxel_size).astype(np.int32)
    _, idx, counts = np.unique(
        keys[:, 0].astype(np.int64) * 2000000 + keys[:, 1].astype(np.int64) * 2000 + keys[:, 2].astype(np.int64),
        return_inverse=True, return_counts=True
    )
    centroids = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(centroids, idx, pts)
    centroids /= counts[:, None]
    return centroids.astype(np.float32)

def main():
    parser = argparse.ArgumentParser(description='SLAM Dense Reinjection')
    parser.add_argument('bag_dir', help='Directory containing MCAP bag files')
    parser.add_argument('--trajectory', required=True, help='Trajectory .npy file (from save_trajectory.py)')
    parser.add_argument('--lidar-topic', default='/ouster/points', help='Raw LiDAR topic')
    parser.add_argument('--output', default='/data/maps/dense_reinjected.ply', help='Output PLY path')
    parser.add_argument('--every-n', type=int, default=5, help='Use every Nth scan (1=all, 5=2Hz)')
    parser.add_argument('--voxel', type=float, default=0.05, help='Voxel size for final downsample (0=none)')
    parser.add_argument('--max-range', type=float, default=100.0, help='Max LiDAR range to include')
    args = parser.parse_args()

    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import get_typestore, Stores
    except ImportError:
        log("ERROR: rosbags not installed. Run: pip3 install rosbags")
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # Load trajectory from .npy file
    log(f"Loading trajectory from {args.trajectory}...")
    traj = np.load(args.trajectory)  # Nx8: [timestamp_ns, x, y, z, qx, qy, qz, qw]
    log(f"  {len(traj)} poses loaded")
    if len(traj) == 0:
        log("ERROR: Empty trajectory")
        sys.exit(1)

    # Build pose list
    odom_poses = []
    for row in traj:
        t_ns = row[0]
        T = pose_to_matrix(row[1:4], row[4:8])
        odom_poses.append((int(t_ns), T))

    odom_times = np.array([t for t, _ in odom_poses])

    import glob, os
    mcap_files = sorted(glob.glob(os.path.join(args.bag_dir, '*.mcap')))
    if not mcap_files:
        log(f"ERROR: No .mcap files in {args.bag_dir}")
        sys.exit(1)
    log(f"Found {len(mcap_files)} MCAP files")

    # Pass 2: Read LiDAR scans, match to nearest pose, transform
    log(f"Pass 2: Reading LiDAR scans from {args.lidar_topic}, injecting every {args.every_n}...")
    all_points = []
    total_raw = 0
    scan_count = 0
    used_count = 0
    t0 = time.time()

    for fi, mcap_path in enumerate(mcap_files):
        log(f"  [{fi+1}/{len(mcap_files)}] {os.path.basename(mcap_path)}")
        try:
            reader = Reader(mcap_path)
            reader.open()
        except Exception as e:
            log(f"    SKIP: {e}")
            continue
        try:
            connections = [c for c in reader.connections if c.topic == args.lidar_topic]
            if not connections:
                continue
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                scan_count += 1
                if scan_count % args.every_n != 0:
                    continue

                # Find nearest odometry pose
                idx = np.searchsorted(odom_times, timestamp)
                if idx == 0:
                    best_idx = 0
                elif idx >= len(odom_times):
                    best_idx = len(odom_times) - 1
                else:
                    # Pick closer of idx-1 and idx
                    if abs(odom_times[idx] - timestamp) < abs(odom_times[idx-1] - timestamp):
                        best_idx = idx
                    else:
                        best_idx = idx - 1

                dt_ms = abs(odom_times[best_idx] - timestamp) / 1e6
                if dt_ms > 200:  # Skip if no pose within 200ms
                    continue

                # Deserialize point cloud
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                fields = [{'name': f.name, 'offset': f.offset} for f in msg.fields]
                pts = pc2_to_xyz(bytes(msg.data), msg.point_step, fields)
                if len(pts) == 0:
                    continue

                # Range filter
                ranges = np.linalg.norm(pts, axis=1)
                pts = pts[ranges < args.max_range]
                if len(pts) == 0:
                    continue

                # Transform to world frame
                T = odom_poses[best_idx][1]
                pts_world = (T[:3, :3] @ pts.T).T + T[:3, 3]
                all_points.append(pts_world.astype(np.float32))
                total_raw += len(pts_world)
                used_count += 1

                if used_count % 50 == 0:
                    elapsed = time.time() - t0
                    log(f"    {used_count} scans injected, {total_raw:,} pts, {elapsed:.0f}s")
        except Exception as e:
            log(f"    ERROR reading: {e}")
        finally:
            reader.close()

    log(f"\nTotal: {used_count} scans, {total_raw:,} raw points from {scan_count} total scans")

    if not all_points:
        log("ERROR: No points collected")
        sys.exit(1)

    # Merge and optionally voxel downsample
    log("Merging...")
    merged = np.vstack(all_points)

    if args.voxel > 0:
        log(f"Voxel downsampling at {args.voxel}m...")
        merged = voxel_downsample(merged, args.voxel)

    save_ply(merged, args.output)
    xr = merged[:, 0].max() - merged[:, 0].min()
    yr = merged[:, 1].max() - merged[:, 1].min()
    zr = merged[:, 2].max() - merged[:, 2].min()
    log(f"Map extent: {xr:.0f}m x {yr:.0f}m x {zr:.0f}m")

if __name__ == '__main__':
    main()
