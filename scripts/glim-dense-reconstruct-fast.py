#!/usr/bin/env python3
"""
FAST dense reconstruction using GLIM's loop-closed submap poses.

Optimized version of glim-dense-reconstruct.py:
- Streaming LAZ writer (constant memory instead of 100GB+)
- Vectorized quaternion SLERP (no scipy per-frame)
- Numpy-only dedup (no Python set operations)
- Pre-computed pose lookup table

Usage:
    glim-dense-reconstruct-fast.py <bag_dir> <glim_dump_dir> [output.laz]
        [--voxel 0.05] [--max-range 50] [--dedup 1.0] [--subsample 0.5]
"""

import sys, os, time, re, struct
import numpy as np
from pathlib import Path

try:
    import laspy
except ImportError:
    sys.exit("pip install laspy lazrs")

try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory
except ImportError:
    sys.exit("pip install mcap mcap-ros2-support")

# Ouster OS-1-128 point struct (48 bytes)
OUSTER_DTYPE = np.dtype([
    ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('_pad1', '<f4'),
    ('intensity', '<u2'), ('t', '<u4'), ('reflectivity', '<u2'),
    ('ring', '<u1'), ('ambient', '<u2'), ('range', '<u4'), ('_pad2', '<u1'),
    ('_pad3', '<f4'), ('_pad4', '<f4'), ('_pad5', '<f4'), ('_pad6', '<u4'),
])


def parse_submap_poses(dump_dir):
    """Parse loop-closed T_world_lidar from all submap data.txt files."""
    dump = Path(dump_dir)
    submap_dirs = sorted(
        [d for d in dump.iterdir() if d.is_dir() and d.name.isdigit()],
        key=lambda d: int(d.name)
    )

    timestamps = []
    rotations = []  # as quaternions for fast SLERP
    translations = []

    for sd in submap_dirs:
        data_file = sd / 'data.txt'
        if not data_file.exists():
            continue

        text = data_file.read_text()
        # Find all T_world_lidar blocks
        blocks = text.split('T_world_lidar:')
        for i, block in enumerate(blocks[1:]):  # skip first (before any T_world_lidar)
            lines = block.strip().split('\n')
            # Parse 4x4 matrix (4 rows of 4 floats)
            try:
                mat = np.zeros((4, 4))
                for row in range(4):
                    mat[row] = [float(x) for x in lines[row].split()]

                # Find timestamp (look backwards for stamp:)
                pre_block = blocks[i]  # text before this T_world_lidar
                stamp_lines = pre_block.strip().split('\n')
                stamp = None
                for sl in reversed(stamp_lines):
                    if 'stamp:' in sl:
                        stamp = float(sl.split('stamp:')[1].strip())
                        break
                if stamp is None:
                    continue

                from scipy.spatial.transform import Rotation
                quat = Rotation.from_matrix(mat[:3, :3]).as_quat()  # [x, y, z, w]
                timestamps.append(stamp)
                rotations.append(quat)
                translations.append(mat[:3, 3])
            except (ValueError, IndexError):
                continue

    order = np.argsort(timestamps)
    timestamps = np.array(timestamps)[order]
    rotations = np.array([rotations[i] for i in order])
    translations = np.array([translations[i] for i in order])

    return timestamps, rotations, translations


def fast_slerp_batch(ts_query, ts_ref, quats_ref, trans_ref):
    """Vectorized pose interpolation for all query timestamps at once.
    Returns (N, 3, 3) rotations and (N, 3) translations."""

    idx = np.searchsorted(ts_ref, ts_query, side='right') - 1
    idx = np.clip(idx, 0, len(ts_ref) - 2)

    t0 = ts_ref[idx]
    t1 = ts_ref[idx + 1]
    dt = t1 - t0
    alpha = np.clip((ts_query - t0) / np.maximum(dt, 1e-9), 0.0, 1.0)

    # Large gaps: snap to nearest
    gap_mask = dt > 5.0
    if gap_mask.any():
        closer_to_next = np.abs(ts_query - t1) < np.abs(ts_query - t0)
        snap_idx = np.where(closer_to_next, idx + 1, idx)
        alpha[gap_mask] = 0.0
        idx[gap_mask] = snap_idx[gap_mask]

    # Lerp translation
    pos = (1.0 - alpha[:, None]) * trans_ref[idx] + alpha[:, None] * trans_ref[idx + 1]

    # Quaternion SLERP (vectorized)
    q0 = quats_ref[idx]
    q1 = quats_ref[idx + 1]

    # Ensure shortest path
    dot = np.sum(q0 * q1, axis=1)
    neg_mask = dot < 0
    q1[neg_mask] *= -1
    dot[neg_mask] *= -1

    # For very similar quaternions, use lerp
    lerp_mask = dot > 0.9995
    # For the rest, proper slerp
    theta = np.arccos(np.clip(dot, -1, 1))
    sin_theta = np.sin(theta)

    # Avoid division by zero
    safe = sin_theta > 1e-6
    s0 = np.where(safe, np.sin((1.0 - alpha) * theta) / sin_theta, 1.0 - alpha)
    s1 = np.where(safe, np.sin(alpha * theta) / sin_theta, alpha)

    q_interp = s0[:, None] * q0 + s1[:, None] * q1

    # Lerp fallback for nearly-identical quaternions
    if lerp_mask.any():
        q_lerp = (1.0 - alpha[lerp_mask, None]) * q0[lerp_mask] + alpha[lerp_mask, None] * q1[lerp_mask]
        q_lerp /= np.linalg.norm(q_lerp, axis=1, keepdims=True)
        q_interp[lerp_mask] = q_lerp

    # Normalize
    q_interp /= np.linalg.norm(q_interp, axis=1, keepdims=True)

    # Convert quaternions to rotation matrices (vectorized)
    x, y, z, w = q_interp[:, 0], q_interp[:, 1], q_interp[:, 2], q_interp[:, 3]
    R = np.zeros((len(q_interp), 3, 3))
    R[:, 0, 0] = 1 - 2*(y*y + z*z)
    R[:, 0, 1] = 2*(x*y - w*z)
    R[:, 0, 2] = 2*(x*z + w*y)
    R[:, 1, 0] = 2*(x*y + w*z)
    R[:, 1, 1] = 1 - 2*(x*x + z*z)
    R[:, 1, 2] = 2*(y*z - w*x)
    R[:, 2, 0] = 2*(x*z - w*y)
    R[:, 2, 1] = 2*(y*z + w*x)
    R[:, 2, 2] = 1 - 2*(x*x + y*y)

    return R, pos


def mcap_sort_key(path):
    m = re.search(r'_(\d+)\.mcap$', str(path))
    return int(m.group(1)) if m else 0


def main():
    import argparse
    parser = argparse.ArgumentParser(description='FAST GLIM dense reconstruction')
    parser.add_argument('bag_dir')
    parser.add_argument('dump_dir')
    parser.add_argument('output', nargs='?', default=None)
    parser.add_argument('--voxel', type=float, default=0.05, help='Final voxel size (m)')
    parser.add_argument('--max-range', type=float, default=50.0)
    parser.add_argument('--min-displacement', type=float, default=0, help='Min displacement between used frames (m). 0.5=one frame per 0.5m')
    parser.add_argument('--dedup', type=float, default=0, help='Deprecated — use voxel filter only')
    parser.add_argument('--subsample', type=float, default=1.0)
    parser.add_argument('--average', action='store_true', help='Average point positions per voxel (reduces noise)')
    parser.add_argument('--chunk-size', type=int, default=5_000_000, help='Points per LAZ chunk')
    args = parser.parse_args()

    bag_dir = Path(args.bag_dir)
    dump_dir = Path(args.dump_dir)
    output_path = args.output or str(bag_dir.parent / f'{bag_dir.name}_dense.laz')

    # Parse poses
    print("Parsing GLIM poses...")
    t0 = time.time()
    pose_ts, pose_quats, pose_trans = parse_submap_poses(dump_dir)
    print(f"  {len(pose_ts)} poses in {time.time()-t0:.1f}s")
    print(f"  Time: {pose_ts[0]:.3f} to {pose_ts[-1]:.3f} ({pose_ts[-1]-pose_ts[0]:.1f}s)")

    # Collect all scan timestamps first (fast pass through mcap headers)
    mcap_files = sorted(
        [f for f in bag_dir.iterdir() if f.suffix == '.mcap'],
        key=mcap_sort_key
    )

    # Process scans in chunks → write LAZ chunks → merge at end
    print(f"\nProcessing {len(mcap_files)} mcap files...")

    # No per-frame dedup — voxel filter per-chunk + at merge handles everything.
    # Previous approaches (2D time-gated, 3D hash) all caused artifacts.
    # The chunked voxel filter is clean and correct.

    chunk_files = []
    chunk_idx = 0
    chunk_xyz = []
    chunk_intensity = []
    chunk_reflectivity = []
    chunk_ambient = []
    chunk_range_vals = []
    chunk_points = 0

    n_scans = 0
    n_matched = 0
    n_points_total = 0
    n_dedup_dropped = 0
    t_start = time.time()

    for mcap_path in mcap_files:
        print(f"  Reading {mcap_path.name}...")
        try:
            with open(mcap_path, 'rb') as f:
                reader = make_reader(f, decoder_factories=[DecoderFactory()])
                for schema, channel, message, msg in reader.iter_decoded_messages(
                    topics=['/ouster/points']
                ):
                    n_scans += 1
                    scan_ts = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

                    if scan_ts < pose_ts[0] - 1.0:
                        continue
                    if scan_ts > pose_ts[-1] + 1.0:
                        break

                    n_pts = msg.width * msg.height
                    if n_pts == 0:
                        continue

                    cloud = np.frombuffer(msg.data, dtype=OUSTER_DTYPE, count=n_pts)
                    xyz = np.column_stack([cloud['x'], cloud['y'], cloud['z']]).astype(np.float32)

                    # Filter
                    valid = np.isfinite(xyz).all(axis=1)
                    valid &= (xyz[:, 0] != 0) | (xyz[:, 1] != 0) | (xyz[:, 2] != 0)
                    if args.max_range > 0:
                        r2 = xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2
                        valid &= r2 <= args.max_range**2

                    if valid.sum() < 100:
                        continue

                    # Subsample
                    if args.subsample < 1.0:
                        valid_idx = np.where(valid)[0]
                        n_keep = max(100, int(len(valid_idx) * args.subsample))
                        if n_keep < len(valid_idx):
                            valid_idx = np.random.choice(valid_idx, n_keep, replace=False)
                            new_valid = np.zeros(len(valid), dtype=bool)
                            new_valid[valid_idx] = True
                            valid = new_valid

                    # Get pose (single frame — use searchsorted)
                    idx = np.searchsorted(pose_ts, scan_ts, side='right') - 1
                    idx = max(0, min(idx, len(pose_ts) - 2))

                    t0_p = pose_ts[idx]
                    t1_p = pose_ts[idx + 1]
                    dt = t1_p - t0_p
                    if dt < 1e-9:
                        alpha = 0.0
                    elif dt > 5.0:
                        alpha = 0.0 if abs(scan_ts - t0_p) < abs(scan_ts - t1_p) else 1.0
                    else:
                        alpha = (scan_ts - t0_p) / dt

                    # Lerp translation
                    pos = (1.0 - alpha) * pose_trans[idx] + alpha * pose_trans[idx + 1]

                    # Fast quaternion slerp (single frame)
                    q0 = pose_quats[idx]
                    q1 = pose_quats[idx + 1]
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

                    # Quaternion to rotation matrix
                    x, y, z, w = q
                    R = np.array([
                        [1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)],
                    ])

                    # Displacement gating — uniform spatial sampling
                    if args.min_displacement > 0:
                        if 'last_pos' not in dir():
                            last_pos = pos.copy()
                        else:
                            disp = np.linalg.norm(pos - last_pos)
                            if disp < args.min_displacement:
                                n_disp_skipped = getattr(main, '_disp_skip', 0) + 1
                                main._disp_skip = n_disp_skipped
                                continue
                            last_pos = pos.copy()

                    n_matched += 1

                    # Transform to world
                    xyz_v = xyz[valid].astype(np.float64)
                    xyz_w = (R @ xyz_v.T).T + pos

                    sc_mask = None  # no per-frame dedup — voxel filter handles it

                    # Extract scalars for kept points
                    if sc_mask is not None:
                        v_idx = np.where(valid)[0]
                        v_idx = v_idx[sc_mask]
                    else:
                        v_idx = np.where(valid)[0]

                    chunk_xyz.append(xyz_w.astype(np.float32))
                    chunk_intensity.append(cloud['intensity'][v_idx])
                    chunk_reflectivity.append(cloud['reflectivity'][v_idx])
                    chunk_ambient.append(cloud['ambient'][v_idx])
                    chunk_range_vals.append(cloud['range'][v_idx])
                    chunk_points += len(xyz_w)
                    n_points_total += len(xyz_w)

                    # Write chunk to temp LAZ when buffer is full
                    if chunk_points >= args.chunk_size:
                        chunk_path = f"{output_path}.chunk_{chunk_idx:04d}.laz"
                        _write_laz_chunk(chunk_path, chunk_xyz, chunk_intensity,
                                        chunk_reflectivity, chunk_ambient, chunk_range_vals,
                                        args.voxel, average=args.average)
                        chunk_files.append(chunk_path)
                        chunk_idx += 1
                        chunk_xyz, chunk_intensity, chunk_reflectivity = [], [], []
                        chunk_ambient, chunk_range_vals = [], []
                        chunk_points = 0

                    if n_scans % 200 == 0:
                        elapsed = time.time() - t_start
                        rate = n_points_total / elapsed / 1e6 if elapsed > 0 else 0
                        print(f"    {n_scans} scans, {n_matched} matched, "
                              f"{n_points_total/1e6:.1f}M pts, {elapsed:.0f}s ({rate:.1f}M/s)")

        except Exception as e:
            print(f"  ERROR reading {mcap_path.name}: {e}")
            continue

    # Write remaining chunk
    if chunk_points > 0:
        chunk_path = f"{output_path}.chunk_{chunk_idx:04d}.laz"
        _write_laz_chunk(chunk_path, chunk_xyz, chunk_intensity,
                        chunk_reflectivity, chunk_ambient, chunk_range_vals,
                        args.voxel)
        chunk_files.append(chunk_path)

    elapsed = time.time() - t_start
    print(f"\nScan processing: {n_scans} scans, {n_matched} matched, "
          f"{n_points_total/1e6:.1f}M pts, {elapsed:.0f}s")
    if n_dedup_dropped > 0:
        print(f"  Dedup dropped: {n_dedup_dropped/1e6:.1f}M pts")
    print(f"  Written {len(chunk_files)} chunk files")

    # Merge chunks into final LAZ
    if len(chunk_files) == 1:
        os.rename(chunk_files[0], output_path)
    elif len(chunk_files) > 1:
        print(f"\nMerging {len(chunk_files)} chunks into {output_path}...")
        _merge_laz_chunks(chunk_files, output_path, args.voxel)

    # Cleanup chunks
    for cf in chunk_files:
        if os.path.exists(cf):
            os.unlink(cf)

    total_time = time.time() - t_start
    if os.path.exists(output_path):
        fsize = os.path.getsize(output_path) / 1e6
        # Count points in output
        las = laspy.read(output_path)
        print(f"\n=== Complete ===")
        print(f"Output: {output_path} ({fsize:.0f} MB, {len(las.points):,} points)")
        print(f"Time:   {total_time:.0f}s ({total_time/60:.1f}m)")
    else:
        print("ERROR: No output produced")


def _write_laz_chunk(path, xyz_list, int_list, refl_list, amb_list, rng_list, voxel, average=False):
    """Write accumulated arrays to a LAZ chunk with voxel filter (averaging or first-point)."""
    xyz = np.concatenate(xyz_list).astype(np.float64)
    intensity = np.concatenate(int_list).astype(np.float64)
    reflectivity = np.concatenate(refl_list).astype(np.float64)
    ambient = np.concatenate(amb_list).astype(np.float64)
    lidar_range = np.concatenate(rng_list).astype(np.float64)

    if voxel > 0 and len(xyz) > 0:
        vk = np.floor(xyz / voxel).astype(np.int32)
        vk_packed = np.ascontiguousarray(vk).view(
            np.dtype((np.void, vk.dtype.itemsize * 3))
        ).ravel()

        if average:
            # Voxel averaging — mean position per voxel (smooths pose noise)
            _, inverse, counts = np.unique(vk_packed, return_inverse=True, return_counts=True)
            n_vox = len(counts)
            xyz = np.column_stack([
                np.bincount(inverse, weights=xyz[:,0], minlength=n_vox) / counts,
                np.bincount(inverse, weights=xyz[:,1], minlength=n_vox) / counts,
                np.bincount(inverse, weights=xyz[:,2], minlength=n_vox) / counts,
            ])
            intensity = np.bincount(inverse, weights=intensity, minlength=n_vox) / counts
            reflectivity = np.bincount(inverse, weights=reflectivity, minlength=n_vox) / counts
            ambient = np.bincount(inverse, weights=ambient, minlength=n_vox) / counts
            lidar_range = np.bincount(inverse, weights=lidar_range, minlength=n_vox) / counts
        else:
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
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = intensity.astype(np.uint16)

    las.add_extra_dim(laspy.ExtraBytesParams(name="reflectivity", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="ambient", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="lidar_range", type=np.uint32))
    las.reflectivity = reflectivity
    las.ambient = ambient
    las.lidar_range = lidar_range

    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    las.write(path)


def _merge_laz_chunks(chunk_files, output_path, voxel):
    """Merge multiple LAZ chunks with cross-chunk voxel dedup."""
    all_xyz = []
    all_int = []
    all_refl = []
    all_amb = []
    all_rng = []

    for cf in chunk_files:
        las = laspy.read(cf)
        xyz = np.column_stack([las.x, las.y, las.z])
        all_xyz.append(xyz)
        all_int.append(np.array(las.intensity))
        all_refl.append(np.array(las.reflectivity))
        all_amb.append(np.array(las.ambient))
        all_rng.append(np.array(las.lidar_range))

    xyz = np.concatenate(all_xyz)
    intensity = np.concatenate(all_int)
    reflectivity = np.concatenate(all_refl)
    ambient = np.concatenate(all_amb)
    lidar_range = np.concatenate(all_rng)

    # Cross-chunk voxel dedup
    if voxel > 0:
        vk = np.floor(xyz / voxel).astype(np.int32)
        vk_packed = np.ascontiguousarray(vk).view(
            np.dtype((np.void, vk.dtype.itemsize * 3))
        ).ravel()
        _, uidx = np.unique(vk_packed, return_index=True)
        print(f"  Cross-chunk dedup: {len(xyz):,} → {len(uidx):,}")
        xyz = xyz[uidx]
        intensity = intensity[uidx]
        reflectivity = reflectivity[uidx]
        ambient = ambient[uidx]
        lidar_range = lidar_range[uidx]

    header = laspy.LasHeader(point_format=0, version="1.4")
    header.offsets = np.min(xyz, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])

    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = intensity.astype(np.uint16)

    las.add_extra_dim(laspy.ExtraBytesParams(name="reflectivity", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="ambient", type=np.uint16))
    las.add_extra_dim(laspy.ExtraBytesParams(name="lidar_range", type=np.uint32))
    las.reflectivity = reflectivity
    las.ambient = ambient
    las.lidar_range = lidar_range

    las.write(output_path)


if __name__ == '__main__':
    main()
