#!/usr/bin/env python3
"""
Dense reconstruction with per-point deskewing using GLIM loop-closed poses.

Each Ouster scan sweeps 360° in ~100ms. At 30km/h that's 83cm of motion blur
if you apply one rigid pose. This script interpolates the pose for EACH POINT
based on its individual timestamp within the scan.

Pipeline:
  1. Load GLIM loop-closed poses from submap dump (10Hz, globally consistent)
  2. For each /ouster/points scan:
     a. Each point has a 't' field (nanosecond offset within scan)
     b. Compute each point's absolute timestamp
     c. Interpolate GLIM pose at that exact time (position lerp + rotation slerp)
     d. Transform point to world frame using its individual pose
     e. Filter by local range (distance from sensor at scan midpoint)
  3. Displacement gating: only use scans where vehicle moved ≥threshold
  4. Voxel filter per chunk, write LAZ tiles

Usage:
    glim-dense-deskewed.py <bag_dir> <dump_dir> <output_dir>
        [--voxel 0.10] [--max-range 30] [--min-disp 0.3]
"""

import sys, os, time, re
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation

try:
    import laspy
except ImportError:
    sys.exit("pip install laspy lazrs")
try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    sys.exit("pip install mcap mcap-ros2-support")

OUSTER_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('_pad1', '<f4'),
    ('intensity', '<u2'), ('t', '<u4'), ('reflectivity', '<u2'),
    ('ring', '<u1'), ('ambient', '<u2'), ('range', '<u4'), ('_pad2', '<u1'),
    ('_pad3', '<f4'), ('_pad4', '<f4'), ('_pad5', '<f4'), ('_pad6', '<u4'),
])


def parse_glim_poses(dump_dir):
    """Read all per-frame T_world_lidar from GLIM submap data.txt files."""
    dump = Path(dump_dir)
    timestamps = []
    positions = []
    quaternions = []  # [x, y, z, w]

    for sd in sorted([d for d in dump.iterdir() if d.is_dir() and d.name.isdigit()],
                     key=lambda d: int(d.name)):
        data_file = sd / 'data.txt'
        if not data_file.exists():
            continue
        text = data_file.read_text()
        blocks = text.split('T_world_lidar:')

        for i, block in enumerate(blocks[1:]):
            lines = block.strip().split('\n')
            try:
                mat = np.zeros((4, 4))
                for row in range(4):
                    mat[row] = [float(x) for x in lines[row].split()]
                # Find timestamp
                pre = blocks[i]
                for sl in reversed(pre.strip().split('\n')):
                    if 'stamp:' in sl:
                        stamp = float(sl.split('stamp:')[1].strip())
                        timestamps.append(stamp)
                        positions.append(mat[:3, 3].copy())
                        quaternions.append(Rotation.from_matrix(mat[:3, :3]).as_quat())
                        break
            except (ValueError, IndexError):
                continue

    order = np.argsort(timestamps)
    return (np.array(timestamps)[order],
            np.array([positions[i] for i in order]),
            np.array([quaternions[i] for i in order]))


def interpolate_pose(t, ref_ts, ref_pos, ref_quat):
    """Interpolate position (lerp) and rotation (slerp) at time t."""
    idx = np.searchsorted(ref_ts, t, side='right') - 1
    idx = max(0, min(idx, len(ref_ts) - 2))

    t0, t1 = ref_ts[idx], ref_ts[idx + 1]
    dt = t1 - t0
    if dt < 1e-9:
        alpha = 0.0
    elif dt > 5.0:
        # Large gap — snap to nearest
        idx = idx if abs(t - t0) < abs(t - t1) else idx + 1
        return ref_pos[idx].copy(), ref_quat[idx].copy()
    else:
        alpha = np.clip((t - t0) / dt, 0.0, 1.0)

    # Lerp position
    pos = (1.0 - alpha) * ref_pos[idx] + alpha * ref_pos[idx + 1]

    # Slerp rotation
    q0 = ref_quat[idx]
    q1 = ref_quat[idx + 1]
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        q = (1 - alpha) * q0 + alpha * q1
    else:
        theta = np.arccos(np.clip(dot, -1, 1))
        sin_t = np.sin(theta)
        q = (np.sin((1 - alpha) * theta) / sin_t) * q0 + \
            (np.sin(alpha * theta) / sin_t) * q1
    q /= np.linalg.norm(q)

    return pos, q


def quat_to_matrix(q):
    """Convert quaternion [x,y,z,w] to 3x3 rotation matrix."""
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)],
    ])


def deskew_scan(xyz, point_t_ns, scan_ts, ref_ts, ref_pos, ref_quat):
    """Per-point deskewing: interpolate pose for each point's individual timestamp.

    xyz: (N, 3) points in LiDAR local frame
    point_t_ns: (N,) uint32 nanosecond offset from scan start
    scan_ts: float, scan header timestamp (seconds)
    ref_*: GLIM pose arrays for interpolation

    Returns: (N, 3) points in world frame
    """
    # Absolute timestamp per point
    point_ts = scan_ts + point_t_ns.astype(np.float64) / 1e9

    # Group points by time bins (every 5ms = ~18° of rotation)
    # This avoids interpolating 131K individual poses
    BIN_WIDTH = 0.005  # 5ms bins
    t_min, t_max = point_ts.min(), point_ts.max()
    n_bins = max(1, int(np.ceil((t_max - t_min) / BIN_WIDTH)))

    bin_idx = np.clip(((point_ts - t_min) / BIN_WIDTH).astype(int), 0, n_bins - 1)
    bin_times = t_min + (np.arange(n_bins) + 0.5) * BIN_WIDTH

    # Pre-compute pose for each bin
    bin_R = np.zeros((n_bins, 3, 3))
    bin_t = np.zeros((n_bins, 3))
    for b in range(n_bins):
        pos, quat = interpolate_pose(bin_times[b], ref_ts, ref_pos, ref_quat)
        bin_R[b] = quat_to_matrix(quat)
        bin_t[b] = pos

    # Transform each point by its bin's pose
    xyz_world = np.empty_like(xyz, dtype=np.float64)
    for b in range(n_bins):
        mask = bin_idx == b
        if not mask.any():
            continue
        pts = xyz[mask].astype(np.float64)
        xyz_world[mask] = (bin_R[b] @ pts.T).T + bin_t[b]

    return xyz_world


def mcap_sort_key(path):
    m = re.search(r'_(\d+)\.mcap$', str(path))
    return int(m.group(1)) if m else 0


def write_chunk(path, xyz, intensity, reflectivity, ambient, lidar_range, voxel):
    """Voxel filter + write LAZ chunk."""
    if voxel > 0 and len(xyz) > 0:
        vk = np.floor(xyz / voxel).astype(np.int32)
        vk_packed = np.ascontiguousarray(vk).view(
            np.dtype((np.void, vk.dtype.itemsize * 3))).ravel()
        _, uidx = np.unique(vk_packed, return_index=True)
        xyz = xyz[uidx]
        intensity = intensity[uidx]
        reflectivity = reflectivity[uidx]
        ambient = ambient[uidx]
        lidar_range = lidar_range[uidx]

    header = laspy.LasHeader(point_format=0, version="1.4")
    header.offsets = np.min(xyz, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])
    las = laspy.LasData(header)
    las.x = xyz[:, 0]; las.y = xyz[:, 1]; las.z = xyz[:, 2]
    las.intensity = intensity.astype(np.uint16)
    las.add_extra_dim(laspy.ExtraBytesParams(name="reflectivity", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="ambient", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="lidar_range", type=np.uint32))
    las.reflectivity = reflectivity
    las.ambient = ambient
    las.lidar_range = lidar_range
    las.write(path)
    return len(xyz) if voxel > 0 else len(uidx)


def main():
    import argparse
    parser = argparse.ArgumentParser(description='GLIM dense reconstruction with per-point deskewing')
    parser.add_argument('bag_dir', help='Path to bag directory with .mcap files')
    parser.add_argument('dump_dir', help='Path to GLIM dump directory')
    parser.add_argument('output_dir', help='Output directory for LAZ tiles')
    parser.add_argument('--voxel', type=float, default=0.10, help='Voxel size in metres (default 0.10)')
    parser.add_argument('--max-range', type=float, default=30.0, help='Max LOCAL range in metres')
    parser.add_argument('--min-disp', type=float, default=0.3, help='Min displacement between used scans')
    parser.add_argument('--chunk-size', type=int, default=10_000_000, help='Points per chunk before voxel filter')
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir)
    dump_dir = Path(args.dump_dir)
    out_dir = Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load GLIM poses
    print("Loading GLIM loop-closed poses...")
    pose_ts, pose_pos, pose_quat = parse_glim_poses(dump_dir)
    print(f"  {len(pose_ts)} poses, {pose_ts[-1]-pose_ts[0]:.0f}s span")

    # 2. Process scans
    mcap_files = sorted(
        [f for f in bag_dir.iterdir() if f.suffix == '.mcap'],
        key=mcap_sort_key
    )
    print(f"  {len(mcap_files)} mcap files")
    print(f"  Voxel: {args.voxel}m, Range: {args.max_range}m, Min disp: {args.min_disp}m")
    print()

    chunk_xyz = []
    chunk_int = []
    chunk_refl = []
    chunk_amb = []
    chunk_rng = []
    chunk_pts = 0
    chunk_idx = 0

    n_scans = 0
    n_used = 0
    n_skipped_range = 0
    n_skipped_disp = 0
    n_points_total = 0
    last_pos = None
    t_start = time.time()

    for mcap_path in mcap_files:
        try:
            with open(mcap_path, 'rb') as f:
                reader = make_reader(f, decoder_factories=[DecoderFactory()])
                for schema, channel, message, msg in reader.iter_decoded_messages(
                    topics=['/ouster/points']
                ):
                    n_scans += 1
                    scan_ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

                    # Skip scans outside GLIM pose range
                    if scan_ts < pose_ts[0] - 0.5 or scan_ts > pose_ts[-1] + 0.5:
                        continue

                    n_pts = msg.width * msg.height
                    if n_pts == 0:
                        continue

                    # Parse point cloud
                    cloud = np.frombuffer(msg.data, dtype=OUSTER_DTYPE, count=n_pts)
                    xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(np.float32)

                    # Valid point mask
                    valid = np.isfinite(xyz).all(axis=1) & (
                        (xyz[:, 0] != 0) | (xyz[:, 1] != 0) | (xyz[:, 2] != 0)
                    )

                    # LOCAL range filter (distance from sensor origin in scan frame)
                    if args.max_range > 0:
                        local_range = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2)
                        valid &= local_range <= args.max_range

                    if valid.sum() < 100:
                        continue

                    # Get sensor position for displacement check (at scan midpoint)
                    mid_pos, _ = interpolate_pose(scan_ts + 0.05, pose_ts, pose_pos, pose_quat)

                    # Displacement gating
                    if args.min_disp > 0 and last_pos is not None:
                        disp = np.linalg.norm(mid_pos - last_pos)
                        if disp < args.min_disp:
                            n_skipped_disp += 1
                            continue
                    last_pos = mid_pos.copy()

                    n_used += 1

                    # Per-point deskewing: transform each point using its individual timestamp
                    xyz_valid = xyz[valid]
                    t_ns = cloud['t'][valid]
                    xyz_world = deskew_scan(xyz_valid, t_ns, scan_ts,
                                          pose_ts, pose_pos, pose_quat)

                    # Accumulate
                    chunk_xyz.append(xyz_world.astype(np.float32))
                    chunk_int.append(cloud['intensity'][valid])
                    chunk_refl.append(cloud['reflectivity'][valid])
                    chunk_amb.append(cloud['ambient'][valid])
                    chunk_rng.append(cloud['range'][valid])
                    chunk_pts += valid.sum()
                    n_points_total += valid.sum()

                    # Write chunk when full
                    if chunk_pts >= args.chunk_size:
                        chunk_path = out_dir / f'tile_{chunk_idx:04d}.laz'
                        n_out = write_chunk(
                            str(chunk_path),
                            np.concatenate(chunk_xyz).astype(np.float64),
                            np.concatenate(chunk_int),
                            np.concatenate(chunk_refl),
                            np.concatenate(chunk_amb),
                            np.concatenate(chunk_rng),
                            args.voxel
                        )
                        print(f"  Tile {chunk_idx}: {n_out/1e6:.1f}M pts "
                              f"({n_scans} scans, {n_used} used, {time.time()-t_start:.0f}s)",
                              flush=True)
                        chunk_files_count = chunk_idx + 1
                        chunk_idx += 1
                        chunk_xyz, chunk_int, chunk_refl, chunk_amb, chunk_rng = [], [], [], [], []
                        chunk_pts = 0

        except Exception as e:
            print(f"  ERROR {mcap_path.name}: {e}")
            continue

    # Write remaining
    if chunk_pts > 0:
        chunk_path = out_dir / f'tile_{chunk_idx:04d}.laz'
        n_out = write_chunk(
            str(chunk_path),
            np.concatenate(chunk_xyz).astype(np.float64),
            np.concatenate(chunk_int),
            np.concatenate(chunk_refl),
            np.concatenate(chunk_amb),
            np.concatenate(chunk_rng),
            args.voxel
        )
        print(f"  Tile {chunk_idx}: {n_out/1e6:.1f}M pts (final)", flush=True)
        chunk_idx += 1

    elapsed = time.time() - t_start
    print(f"\n=== Complete ===")
    print(f"Scans: {n_used} used / {n_scans} total ({n_skipped_disp} disp-skipped)")
    print(f"Points: {n_points_total/1e6:.0f}M raw")
    print(f"Tiles: {chunk_idx} files in {out_dir}")
    print(f"Time: {elapsed:.0f}s ({elapsed/60:.1f}m)")

    # Count total output points
    total_out = 0
    for f in sorted(out_dir.glob('tile_*.laz')):
        las = laspy.read(str(f))
        total_out += len(las.points)
        print(f"  {f.name}: {len(las.points)/1e6:.1f}M pts ({f.stat().st_size/1e6:.0f}MB)")
    print(f"Total output: {total_out:,} pts")


if __name__ == '__main__':
    main()
