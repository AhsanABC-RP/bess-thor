#!/usr/bin/env python3
"""
Offline dense reconstruction using GLIM's loop-closed submap poses.

Reads raw Ouster scans from a rosbag (MCAP) and GLIM's optimized per-frame
poses from submap data.txt files, then produces a dense LAZ point cloud with
ALL Ouster scalar fields (intensity, reflectivity, ambient, range) preserved.

KEY INSIGHT: GLIM's traj_lidar.txt is the ODOMETRY trajectory, NOT the
loop-closed one. The loop-closed poses live in each submap's data.txt as
T_world_lidar per frame. These can differ by up to 8m from traj_lidar.txt.
This script uses the submap poses for ghost-free reconstruction.

Usage:
    glim-dense-reconstruct.py <bag_dir> <glim_dump_dir> [output.laz]

Examples:
    glim-dense-reconstruct.py /var/mnt/nvme1/slam-cache/bess_20260313_141013 \\
        /var/mnt/nvme2/glim_offline/bess_20260313_141013_v2/dump /tmp/dense.laz

    # With filtering
    glim-dense-reconstruct.py bag_dir dump_dir output.laz \\
        --voxel 0.02 --max-range 30 --min-displacement 1.0 --subsample 0.1

Requirements:
    pip install mcap mcap-ros2-support laspy lazrs numpy scipy
"""

import sys
import os
import time
import re
import numpy as np
from pathlib import Path
from collections import deque

try:
    import laspy
except ImportError:
    print("ERROR: pip install laspy lazrs", file=sys.stderr)
    sys.exit(1)

try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    print("ERROR: pip install mcap mcap-ros2-support", file=sys.stderr)
    sys.exit(1)

from scipy.spatial.transform import Rotation


# --- Ouster PointCloud2 dtype ---
OUSTER_DTYPE = np.dtype([
    ('x', np.float32),
    ('y', np.float32),
    ('z', np.float32),
    ('_pad1', np.uint8, 4),
    ('intensity', np.float32),
    ('t', np.uint32),
    ('reflectivity', np.uint16),
    ('ring', np.uint16),
    ('ambient', np.uint16),
    ('_pad2', np.uint8, 2),
    ('range', np.uint32),
    ('_pad3', np.uint8, 12),
])
assert OUSTER_DTYPE.itemsize == 48


def parse_submap_poses(dump_dir):
    """Parse GLIM dump submap data.txt files for per-frame T_world_lidar.

    Returns sorted arrays: timestamps (N,), T_world_lidar 4x4 matrices (N,4,4).
    These are the LOOP-CLOSED poses, unlike traj_lidar.txt which is odometry.
    """
    submap_dirs = sorted(
        [d for d in os.listdir(dump_dir) if re.match(r'^\d{6}$', d)],
        key=int
    )

    frames = []  # (timestamp, T_world_lidar_4x4)

    for sd in submap_dirs:
        path = os.path.join(dump_dir, sd, 'data.txt')
        if not os.path.exists(path):
            continue
        content = open(path).read()

        # Split on T_world_lidar: to find each frame's corrected pose
        blocks = content.split('T_world_lidar:')
        for bi, block in enumerate(blocks[1:], 1):
            lines = block.strip().split('\n')
            try:
                mat = []
                for r in range(4):
                    mat.append([float(v) for v in lines[r].split()])
                T = np.array(mat, dtype=np.float64)

                # Get timestamp: last "stamp:" before this T_world_lidar
                pre_text = 'T_world_lidar:'.join(blocks[:bi])
                stamp_matches = re.findall(r'stamp:\s+([\d.]+)', pre_text)
                if stamp_matches:
                    ts = float(stamp_matches[-1])
                    frames.append((ts, T))
            except (ValueError, IndexError):
                continue

    frames.sort(key=lambda x: x[0])

    timestamps = np.array([f[0] for f in frames], dtype=np.float64)
    matrices = np.array([f[1] for f in frames], dtype=np.float64)

    return timestamps, matrices


def interpolate_pose_matrix(ts, pose_ts, pose_matrices):
    """Interpolate between T_world_lidar 4x4 matrices at timestamp ts.

    Returns (R_3x3, t_3) or None.
    """
    if ts < pose_ts[0] - 0.5 or ts > pose_ts[-1] + 0.5:
        return None

    idx = np.searchsorted(pose_ts, ts) - 1
    idx = max(0, min(idx, len(pose_ts) - 2))

    t0, t1 = pose_ts[idx], pose_ts[idx + 1]
    dt = t1 - t0
    if dt < 1e-9:
        alpha = 0.0
    elif dt > 5.0:
        # Gap too large (e.g. between submaps) — use nearest
        if abs(ts - t0) < abs(ts - t1):
            return pose_matrices[idx][:3, :3], pose_matrices[idx][:3, 3]
        else:
            return pose_matrices[idx + 1][:3, :3], pose_matrices[idx + 1][:3, 3]
    else:
        alpha = np.clip((ts - t0) / dt, 0.0, 1.0)

    # Lerp position
    pos0 = pose_matrices[idx][:3, 3]
    pos1 = pose_matrices[idx + 1][:3, 3]
    pos = (1.0 - alpha) * pos0 + alpha * pos1

    # SLERP rotation
    R0 = pose_matrices[idx][:3, :3]
    R1 = pose_matrices[idx + 1][:3, :3]
    r0 = Rotation.from_matrix(R0)
    r1 = Rotation.from_matrix(R1)

    # Slerp via scipy
    from scipy.spatial.transform import Slerp
    slerp = Slerp([0.0, 1.0], Rotation.concatenate([r0, r1]))
    R = slerp(alpha).as_matrix()

    return R, pos


def _mcap_sort_key(path):
    """Sort mcap files numerically by trailing number."""
    m = re.search(r'_(\d+)\.mcap$', path)
    return int(m.group(1)) if m else 0


def iter_mcap_messages(bag_dir, topics):
    """Iterate decoded messages from all MCAP files in bag_dir, time-ordered."""
    mcap_files = sorted(
        [os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.mcap')],
        key=_mcap_sort_key
    )
    if not mcap_files:
        raise FileNotFoundError(f"No .mcap files in {bag_dir}")

    for mcap_path in mcap_files:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages(topics=topics):
                yield channel.topic, message.log_time / 1e9, decoded


def voxel_filter(xyz, scalars, voxel_size):
    """Voxel grid filter: keep one point per voxel cube. Chunked to avoid OOM."""
    n = len(xyz)
    CHUNK = 50_000_000  # 50M points per chunk
    if n <= CHUNK:
        voxel_coords = np.floor(xyz / voxel_size).astype(np.int32)
        voxel_keys = np.ascontiguousarray(voxel_coords).view(
            np.dtype((np.void, voxel_coords.dtype.itemsize * 3))
        ).ravel()
        _, unique_idx = np.unique(voxel_keys, return_index=True)
        return xyz[unique_idx], {k: v[unique_idx] for k, v in scalars.items()}

    # Chunked: process in spatial tiles to avoid OOM
    # Use coarse spatial binning (10m tiles) to split work
    tile_size = 10.0
    tile_x = np.floor(xyz[:, 0] / tile_size).astype(np.int32)
    tile_y = np.floor(xyz[:, 1] / tile_size).astype(np.int32)
    tile_keys = tile_x.astype(np.int64) * 100000 + tile_y.astype(np.int64)
    unique_tiles = np.unique(tile_keys)

    keep_mask = np.zeros(n, dtype=bool)
    for tile in unique_tiles:
        mask = tile_keys == tile
        tile_xyz = xyz[mask]
        vc = np.floor(tile_xyz / voxel_size).astype(np.int32)
        vk = np.ascontiguousarray(vc).view(
            np.dtype((np.void, vc.dtype.itemsize * 3))
        ).ravel()
        _, uidx = np.unique(vk, return_index=True)
        tile_indices = np.where(mask)[0][uidx]
        keep_mask[tile_indices] = True

    return xyz[keep_mask], {k: v[keep_mask] for k, v in scalars.items()}


def main():
    import argparse
    parser = argparse.ArgumentParser(description='GLIM dense offline reconstruction (submap-corrected)')
    parser.add_argument('bag_dir', help='Path to bag directory with .mcap files')
    parser.add_argument('dump_dir', help='Path to GLIM dump dir OR directory containing a TUM trajectory file (lio_sam_*_traj.txt)')
    parser.add_argument('output', nargs='?', default=None, help='Output LAZ path')
    parser.add_argument('--voxel', type=float, default=0, help='Voxel filter size in metres (e.g. 0.02 for 2cm)')
    parser.add_argument('--subsample', type=float, default=1.0, help='Random subsample fraction (0.25 = keep 25%%)')
    parser.add_argument('--max-range', type=float, default=0, help='Max range in metres (e.g. 30)')
    parser.add_argument('--min-displacement', type=float, default=0, help='Min displacement between scans in metres (e.g. 0.5)')
    parser.add_argument('--dedup', type=float, default=0, help='First-pass-wins dedup voxel size in metres (e.g. 0.5). Eliminates ghost from revisit zones by keeping only the first temporal pass per spatial voxel.')
    args = parser.parse_args()

    bag_dir = args.bag_dir
    dump_dir = args.dump_dir

    # Detect trajectory source: GLIM submap dir or TUM trajectory file
    traj_file = None
    if dump_dir.endswith('.txt') and os.path.isfile(dump_dir):
        traj_file = dump_dir
    elif os.path.isdir(dump_dir):
        # Check for TUM trajectory file in the directory
        import glob
        tum_files = glob.glob(os.path.join(dump_dir, '*_traj.txt')) + \
                    glob.glob(os.path.join(dump_dir, 'traj_*.txt'))
        if tum_files:
            traj_file = sorted(tum_files)[-1]  # use latest
        # Check for GLIM submap dirs
        elif os.path.isdir(os.path.join(dump_dir, '000000')):
            pass  # GLIM mode
        elif os.path.isdir(os.path.join(dump_dir, 'dump', '000000')):
            dump_dir = os.path.join(dump_dir, 'dump')
        else:
            print(f"ERROR: No submap dirs or trajectory files in {dump_dir}", file=sys.stderr)
            sys.exit(1)

    if not os.path.isdir(bag_dir):
        print(f"ERROR: Bag directory not found: {bag_dir}", file=sys.stderr)
        sys.exit(1)

    use_tum_trajectory = traj_file is not None

    # Default output path
    if args.output is None:
        bag_name = os.path.basename(bag_dir.rstrip('/'))
        output_path = f'/var/mnt/nvme2/extraction/{bag_name}_dense_reconstructed.laz'
    else:
        output_path = args.output

    print(f"=== GLIM Dense Reconstruction (submap-corrected) ===")
    print(f"Bag:      {bag_dir}")
    print(f"Dump:     {dump_dir}")
    print(f"Output:   {output_path}")
    if args.voxel > 0:
        print(f"Voxel:    {args.voxel:.3f}m")
    if args.subsample < 1.0:
        print(f"Subsamp:  {args.subsample*100:.0f}%")
    if args.max_range > 0:
        print(f"Range:    {args.max_range:.0f}m")
    if args.min_displacement > 0:
        print(f"Min disp: {args.min_displacement:.2f}m")
    if args.dedup > 0:
        print(f"Dedup:    {args.dedup:.2f}m (first-pass-wins)")
    print()

    # Load poses — either from GLIM submap data or TUM trajectory file
    if use_tum_trajectory:
        print(f"Loading TUM trajectory: {traj_file}")
        tum_data = np.loadtxt(traj_file, dtype=np.float64)
        tum_data = tum_data[np.argsort(tum_data[:, 0])]
        pose_ts = tum_data[:, 0]
        # Build 4x4 matrices from TUM format (ts x y z qx qy qz qw)
        pose_mats = np.zeros((len(tum_data), 4, 4), dtype=np.float64)
        for i in range(len(tum_data)):
            q = tum_data[i, 4:8]  # qx qy qz qw
            R = Rotation.from_quat(q).as_matrix()
            pose_mats[i, :3, :3] = R
            pose_mats[i, :3, 3] = tum_data[i, 1:4]
            pose_mats[i, 3, 3] = 1.0
        print(f"  {len(pose_ts)} poses, {pose_ts[0]:.3f} to {pose_ts[-1]:.3f} ({pose_ts[-1]-pose_ts[0]:.1f}s)")
        print(f"  Source: TUM trajectory (LIO-SAM / external SLAM)")
    else:
        print("Loading GLIM submap poses...")
        pose_ts, pose_mats = parse_submap_poses(dump_dir)
        print(f"  {len(pose_ts)} corrected frame poses from submaps")
        print(f"  Time range: {pose_ts[0]:.3f} to {pose_ts[-1]:.3f} ({pose_ts[-1]-pose_ts[0]:.1f}s)")
    print()

    # Accumulate all points
    all_xyz = []
    all_intensity = []
    all_reflectivity = []
    all_ambient = []
    all_range = []

    n_scans = 0
    n_matched = 0
    n_skipped = 0
    n_disp_skipped = 0
    n_dedup_dropped = 0
    n_points_total = 0
    t_start = time.time()
    last_acc_pos = None

    # First-pass-wins deduplication: track when each coarse voxel was first seen
    # Only drop points that arrive >30s after the first pass (revisit ghost)
    voxel_first_seen = {} if args.dedup > 0 else None  # voxel_key -> first_timestamp
    DEDUP_TIME_GAP = 5.0  # seconds — only keep points within 5s of first pass per voxel

    print("Processing scans...")

    for topic, log_time, msg in iter_mcap_messages(bag_dir, ['/ouster/points']):
        n_scans += 1
        scan_ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Early exit
        if scan_ts > pose_ts[-1] + 1.0:
            break

        # Parse point cloud
        n_pts = msg.width * msg.height
        if n_pts == 0:
            n_skipped += 1
            continue

        cloud = np.frombuffer(msg.data, dtype=OUSTER_DTYPE, count=n_pts)

        xyz = np.column_stack([
            cloud['x'].astype(np.float32),
            cloud['y'].astype(np.float32),
            cloud['z'].astype(np.float32),
        ])

        # Filter invalid points
        valid = np.isfinite(xyz).all(axis=1) & (
            (xyz[:, 0] != 0) | (xyz[:, 1] != 0) | (xyz[:, 2] != 0)
        )
        if args.max_range > 0:
            point_range = np.sqrt(xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2)
            valid &= point_range <= args.max_range
        if valid.sum() < 100:
            n_skipped += 1
            continue

        # Interpolate submap-corrected pose
        pose = interpolate_pose_matrix(scan_ts, pose_ts, pose_mats)
        if pose is None:
            n_skipped += 1
            continue

        R_world, t_world = pose

        # Displacement gating
        if args.min_displacement > 0:
            if last_acc_pos is not None:
                disp = np.linalg.norm(t_world - last_acc_pos)
                if disp < args.min_displacement:
                    n_disp_skipped += 1
                    continue
            last_acc_pos = t_world.copy()

        n_matched += 1

        # Transform to world
        xyz_valid = xyz[valid].astype(np.float64)
        xyz_world = ((R_world @ xyz_valid.T).T + t_world).astype(np.float32)

        # Extract scalars
        sc_intensity = cloud['intensity'][valid]
        sc_reflectivity = cloud['reflectivity'][valid]
        sc_ambient = cloud['ambient'][valid]
        sc_range = cloud['range'][valid]

        # Time-windowed deduplication: drop points from revisit passes (>30s later)
        if voxel_first_seen is not None:
            vx = np.floor(xyz_world[:, 0] / args.dedup).astype(np.int64)
            vy = np.floor(xyz_world[:, 1] / args.dedup).astype(np.int64)
            vkeys = vx * 1000003 + vy
            unique_scan_keys = set(vkeys.tolist())

            # Check each unique voxel in this scan
            keep_keys = set()
            for k in unique_scan_keys:
                if k not in voxel_first_seen:
                    voxel_first_seen[k] = scan_ts
                    keep_keys.add(k)
                elif scan_ts - voxel_first_seen[k] < DEDUP_TIME_GAP:
                    keep_keys.add(k)  # Same pass — keep
                # else: revisit >30s later — drop

            if len(keep_keys) == 0:
                n_dedup_dropped += len(xyz_world)
                continue
            elif len(keep_keys) < len(unique_scan_keys):
                keep = np.array([int(k) in keep_keys for k in vkeys.tolist()])
                n_dedup_dropped += (~keep).sum()
                xyz_world = xyz_world[keep]
                sc_intensity = sc_intensity[keep]
                sc_reflectivity = sc_reflectivity[keep]
                sc_ambient = sc_ambient[keep]
                sc_range = sc_range[keep]

        # Per-scan subsample
        if args.subsample < 1.0:
            n_keep = max(100, int(len(xyz_world) * args.subsample))
            if n_keep < len(xyz_world):
                idx = np.random.choice(len(xyz_world), size=n_keep, replace=False)
                xyz_world = xyz_world[idx]
                sc_intensity = sc_intensity[idx]
                sc_reflectivity = sc_reflectivity[idx]
                sc_ambient = sc_ambient[idx]
                sc_range = sc_range[idx]

        # Accumulate
        all_xyz.append(xyz_world)
        all_intensity.append(sc_intensity)
        all_reflectivity.append(sc_reflectivity)
        all_ambient.append(sc_ambient)
        all_range.append(sc_range)
        n_points_total += len(xyz_world)

        if n_scans % 100 == 0:
            elapsed = time.time() - t_start
            print(f"  {n_scans} scans, {n_matched} matched, "
                  f"{n_points_total/1e6:.1f}M pts, {elapsed:.0f}s")

    elapsed = time.time() - t_start
    disp_info = f", {n_disp_skipped} disp-skipped" if n_disp_skipped > 0 else ""
    dedup_info = f", {n_dedup_dropped/1e6:.1f}M dedup-dropped" if n_dedup_dropped > 0 else ""
    print(f"\nDone: {n_scans} scans, {n_matched} matched, {n_skipped} skipped{disp_info}{dedup_info}, "
          f"{n_points_total/1e6:.1f}M points in {elapsed:.0f}s")

    if n_points_total == 0:
        print("ERROR: No points accumulated", file=sys.stderr)
        sys.exit(1)

    # Concatenate
    print("Concatenating...")
    xyz_all = np.concatenate(all_xyz)
    intensity_all = np.concatenate(all_intensity)
    reflectivity_all = np.concatenate(all_reflectivity)
    ambient_all = np.concatenate(all_ambient)
    range_all = np.concatenate(all_range)

    # Voxel filter
    if args.voxel > 0:
        print(f"Voxel filtering at {args.voxel:.3f}m...")
        scalars = {
            'intensity': intensity_all,
            'reflectivity': reflectivity_all,
            'ambient': ambient_all,
            'range': range_all,
        }
        xyz_all, scalars = voxel_filter(xyz_all, scalars, args.voxel)
        intensity_all = scalars['intensity']
        reflectivity_all = scalars['reflectivity']
        ambient_all = scalars['ambient']
        range_all = scalars['range']
        print(f"After voxel filter: {len(xyz_all)/1e6:.1f}M points")

    # Write LAZ
    n_out = len(xyz_all)
    print(f"Writing {output_path} ({n_out/1e6:.1f}M points)...")
    header = laspy.LasHeader(point_format=0, version="1.4")
    header.offsets = np.array([
        np.min(xyz_all[:, 0]),
        np.min(xyz_all[:, 1]),
        np.min(xyz_all[:, 2]),
    ], dtype=np.float64)
    header.scales = np.array([0.001, 0.001, 0.001])

    las = laspy.LasData(header)
    las.x = xyz_all[:, 0].astype(np.float64)
    las.y = xyz_all[:, 1].astype(np.float64)
    las.z = xyz_all[:, 2].astype(np.float64)
    las.intensity = intensity_all.astype(np.uint16)

    las.add_extra_dim(laspy.ExtraBytesParams(name="reflectivity", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="ambient", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="lidar_range", type=np.uint32))

    las.reflectivity = reflectivity_all
    las.ambient = ambient_all
    las.lidar_range = range_all

    os.makedirs(os.path.dirname(output_path) or '.', exist_ok=True)
    las.write(output_path)

    file_size = os.path.getsize(output_path) / (1024 * 1024)
    elapsed_total = time.time() - t_start
    print(f"\n=== Complete ===")
    print(f"Output: {output_path} ({file_size:.0f} MB)")
    print(f"Points: {n_out:,} (raw: {n_points_total:,})")
    print(f"Fields: x, y, z, intensity, reflectivity, ambient, range")
    print(f"Scans:  {n_matched}/{n_scans} matched")
    print(f"Time:   {elapsed_total:.0f}s")


if __name__ == '__main__':
    main()
