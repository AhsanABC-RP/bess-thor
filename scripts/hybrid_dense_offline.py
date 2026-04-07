#!/usr/bin/env python3
"""Offline Hybrid Dense Map: FAST-LIO deskewed body-frame clouds + GLIM poses.

Approach:
  - /slam/cloud_registered_body: FAST-LIO IMU-deskewed clouds in body (IMU) frame
  - GLIM odom_corrected: loop-closed pose (odom → imu_link)
  - Direct transform: p_world = T_glim(t) @ p_body  (no correction hack needed)

FAST-LIO's 200Hz IMU deskew gives sharp scans. GLIM's loop closure gives drift-free poses.
Height-aware voxel: ground@0.10m, elevated@0.04m. Preserves intensity.

Usage:
    python3 hybrid_dense_offline.py <bag_dir> [--output map.ply]
        [--every-n 1] [--voxel-ground 0.10] [--voxel-elevated 0.04] [--max-range 100]
"""
import argparse, sys, time, os, glob
import numpy as np


def log(msg):
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),    1-2*(qx*qx+qy*qy)]
    ])


def pose_to_matrix(pos, quat):
    T = np.eye(4)
    T[:3, :3] = quat_to_rot(*quat)
    T[:3, 3] = pos
    return T


def pc2_to_xyz_intensity(data, point_step, fields):
    """Parse PointCloud2 binary data to Nx3 float32 xyz + Nx1 intensity."""
    offsets = {f['name']: f['offset'] for f in fields}
    if 'x' not in offsets:
        return None, None

    arr = np.frombuffer(data, dtype=np.uint8).reshape(-1, point_step)
    x = np.frombuffer(arr[:, offsets['x']:offsets['x']+4].tobytes(), dtype=np.float32)
    y = np.frombuffer(arr[:, offsets['y']:offsets['y']+4].tobytes(), dtype=np.float32)
    z = np.frombuffer(arr[:, offsets['z']:offsets['z']+4].tobytes(), dtype=np.float32)
    pts = np.column_stack([x, y, z])

    intensity = None
    if 'intensity' in offsets:
        intensity = np.frombuffer(
            arr[:, offsets['intensity']:offsets['intensity']+4].tobytes(), dtype=np.float32)

    # Filter invalid points (NaN, zero)
    valid = np.all(np.isfinite(pts), axis=1) & (np.linalg.norm(pts, axis=1) > 0.5)
    pts = pts[valid]
    if intensity is not None:
        intensity = intensity[valid]

    return pts, intensity


def find_nearest(times, target, tol_ns):
    """Find nearest timestamp in sorted array. Returns index or -1."""
    idx = np.searchsorted(times, target)
    best = -1
    best_dt = tol_ns + 1
    for candidate in [idx - 1, idx]:
        if 0 <= candidate < len(times):
            dt = abs(int(times[candidate]) - int(target))
            if dt < best_dt:
                best_dt = dt
                best = candidate
    if best_dt <= tol_ns:
        return best
    return -1


def height_aware_voxel(xyz, intensity, voxel_ground, voxel_elevated, ground_margin=1.5):
    """Height-aware voxel downsample: coarse for ground, fine for elevated."""
    if len(xyz) == 0:
        return xyz, intensity

    z_vals = xyz[:, 2]
    ground_z = np.percentile(z_vals, 5)
    elevated_mask = z_vals > (ground_z + ground_margin)
    ground_mask = ~elevated_mask

    keep_indices = []

    for mask, voxel in [(ground_mask, voxel_ground), (elevated_mask, voxel_elevated)]:
        if not np.any(mask):
            continue
        subset = xyz[mask]
        keys = (subset / voxel).astype(np.int64)
        hashes = keys[:, 0] * 4000000 + keys[:, 1] * 4000 + keys[:, 2]
        _, first_idx = np.unique(hashes, return_index=True)
        keep_indices.append(np.where(mask)[0][first_idx])

    if not keep_indices:
        return xyz, intensity

    idx = np.sort(np.concatenate(keep_indices))
    xyz_out = xyz[idx]
    int_out = intensity[idx] if intensity is not None else None

    n_elev = np.sum(xyz_out[:, 2] > (ground_z + ground_margin))
    n_ground = len(xyz_out) - n_elev
    log(f"  Voxel: ground@{voxel_ground}m={n_ground:,} elevated@{voxel_elevated}m={n_elev:,} "
        f"(ground_z={ground_z:.1f}m)")
    return xyz_out, int_out


def save_ply(xyz, intensity, path):
    n = len(xyz)
    has_int = intensity is not None and len(intensity) == n

    header = "ply\nformat binary_little_endian 1.0\n"
    header += f"element vertex {n}\n"
    header += "property float x\nproperty float y\nproperty float z\n"
    if has_int:
        header += "property float scalar_intensity\n"
    header += "end_header\n"

    if has_int:
        row = np.zeros((n, 4), dtype=np.float32)
        row[:, :3] = xyz.astype(np.float32)
        row[:, 3] = intensity.astype(np.float32)
    else:
        row = xyz.astype(np.float32)

    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        f.write(row.tobytes())

    fields = "xyz+intensity" if has_int else "xyz-only"
    log(f"Saved {n:,} points to {path} ({fields})")


def main():
    parser = argparse.ArgumentParser(description='Offline Hybrid Dense Map')
    parser.add_argument('bag_dir', help='Directory containing MCAP bag files')
    parser.add_argument('--cloud-topic', default='/slam/cloud_registered_body',
                        help='FAST-LIO body-frame cloud topic')
    parser.add_argument('--glim-odom-topic', default='/glim/odom_corrected',
                        help='GLIM corrected odom topic (odom→imu_link)')
    parser.add_argument('--output', default='/tmp/hybrid_dense.ply', help='Output PLY path')
    parser.add_argument('--every-n', type=int, default=1, help='Use every Nth cloud (1=all 10Hz)')
    parser.add_argument('--voxel-ground', type=float, default=0.10, help='Voxel size for ground')
    parser.add_argument('--voxel-elevated', type=float, default=0.04, help='Voxel size for elevated')
    parser.add_argument('--max-range', type=float, default=100.0, help='Max range from sensor')
    args = parser.parse_args()

    try:
        from rosbags.rosbag2 import Reader
        from rosbags.typesys import get_typestore, Stores
    except ImportError:
        log("ERROR: rosbags not installed. Run: pip3 install rosbags")
        sys.exit(1)

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    mcap_files = sorted(glob.glob(os.path.join(args.bag_dir, '*.mcap')))
    if not mcap_files:
        log(f"ERROR: No .mcap files in {args.bag_dir}")
        sys.exit(1)
    log(f"Found {len(mcap_files)} MCAP files")

    # --- Pass 1: Load GLIM poses (odom → imu_link) ---
    log(f"Pass 1: Reading GLIM poses from {args.glim_odom_topic}...")
    glim_list = []
    for fi, mcap_path in enumerate(mcap_files):
        try:
            reader = Reader(mcap_path)
            reader.open()
        except Exception as e:
            log(f"  SKIP {os.path.basename(mcap_path)}: {e}")
            continue
        try:
            connections = [c for c in reader.connections if c.topic == args.glim_odom_topic]
            if not connections:
                continue
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                p = msg.pose.pose.position
                q = msg.pose.pose.orientation
                T = pose_to_matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
                glim_list.append((timestamp, T))
        except Exception as e:
            log(f"  ERROR: {e}")
        finally:
            reader.close()

    if not glim_list:
        log("ERROR: No GLIM poses found in bag")
        sys.exit(1)

    glim_times = np.array([t for t, _ in glim_list], dtype=np.int64)
    glim_poses = [T for _, T in glim_list]
    log(f"  {len(glim_poses)} GLIM poses, {(glim_times[-1]-glim_times[0])/1e9:.1f}s span")

    # --- Pass 2: Read body-frame clouds, transform by GLIM pose ---
    log(f"\nPass 2: Reading {args.cloud_topic}, transforming by GLIM poses...")
    log(f"  Every {args.every_n} scan, max range {args.max_range}m")

    GLIM_MATCH_TOL = 200_000_000  # 200ms — GLIM publishes at ~19Hz

    all_xyz = []
    all_intensity = []
    total_raw = 0
    scan_count = 0
    used_count = 0
    skipped_no_pose = 0
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
            connections = [c for c in reader.connections if c.topic == args.cloud_topic]
            if not connections:
                log(f"    No {args.cloud_topic} topic")
                continue
            for conn, timestamp, rawdata in reader.messages(connections=connections):
                scan_count += 1
                if scan_count % args.every_n != 0:
                    continue

                # Find matching GLIM pose
                glim_idx = find_nearest(glim_times, timestamp, GLIM_MATCH_TOL)
                if glim_idx < 0:
                    skipped_no_pose += 1
                    continue

                T_glim = glim_poses[glim_idx]

                # Deserialize body-frame cloud
                msg = typestore.deserialize_cdr(rawdata, conn.msgtype)
                fields = [{'name': f.name, 'offset': f.offset} for f in msg.fields]
                pts, intensity = pc2_to_xyz_intensity(bytes(msg.data), msg.point_step, fields)
                if pts is None or len(pts) == 0:
                    continue

                # Range filter (body frame — distance from sensor origin)
                ranges = np.linalg.norm(pts, axis=1)
                mask = ranges < args.max_range
                pts = pts[mask]
                if intensity is not None:
                    intensity = intensity[mask]
                if len(pts) == 0:
                    continue

                # Direct transform: p_world = T_glim @ p_body
                R = T_glim[:3, :3].astype(np.float32)
                t = T_glim[:3, 3].astype(np.float32)
                pts_world = (pts @ R.T) + t

                all_xyz.append(pts_world)
                if intensity is not None:
                    all_intensity.append(intensity)
                total_raw += len(pts_world)
                used_count += 1

                if used_count % 50 == 0:
                    elapsed = time.time() - t0
                    log(f"    {used_count} scans, {total_raw:,} pts, "
                        f"skip={skipped_no_pose}, {elapsed:.0f}s")
        except Exception as e:
            log(f"    ERROR: {e}")
        finally:
            reader.close()

    log(f"\nTotal: {used_count} scans, {total_raw:,} raw points "
        f"({scan_count} total, {skipped_no_pose} skipped no pose)")

    if not all_xyz:
        log("ERROR: No points collected")
        sys.exit(1)

    # Merge
    log("Merging...")
    merged_xyz = np.vstack(all_xyz)
    merged_int = np.concatenate(all_intensity) if all_intensity else None

    # Height-aware voxel downsample
    if args.voxel_ground > 0:
        merged_xyz, merged_int = height_aware_voxel(
            merged_xyz, merged_int, args.voxel_ground, args.voxel_elevated)

    save_ply(merged_xyz, merged_int, args.output)

    xr = merged_xyz[:, 0].max() - merged_xyz[:, 0].min()
    yr = merged_xyz[:, 1].max() - merged_xyz[:, 1].min()
    zr = merged_xyz[:, 2].max() - merged_xyz[:, 2].min()
    log(f"Map extent: {xr:.0f}m x {yr:.0f}m x {zr:.0f}m")


if __name__ == '__main__':
    main()
