#!/usr/bin/env python3
"""
Merge GLIM offline dump submaps into a single LAZ point cloud.

GLIM's glim_rosbag dumps submaps as numbered directories (000000, 000001, ...),
each containing:
  - points_compact.bin  — float32 XYZ points in local submap frame
  - intensities_compact.bin — float32 intensities
  - data.txt — submap metadata including T_world_origin (4x4 transform)

This script reads all submaps, transforms points to world frame using
T_world_origin, and writes a merged LAZ file.

Usage:
    glim-merge-submaps.py <dump_dir> [output.laz]

Examples:
    glim-merge-submaps.py /var/mnt/nvme2/glim_offline/dense_test/dump
    glim-merge-submaps.py /var/mnt/nvme2/glim_offline/dense_test/dump /tmp/merged_dense.laz
"""

import struct
import sys
import os
import re
import numpy as np
from pathlib import Path

try:
    import laspy
except ImportError:
    print("ERROR: pip install laspy lazrs")
    sys.exit(1)


def read_transform(data_txt):
    """Extract T_world_origin 4x4 matrix from data.txt."""
    with open(data_txt) as f:
        lines = f.readlines()

    # Find T_world_origin block
    T = np.eye(4)
    found = False
    row = 0
    for i, line in enumerate(lines):
        if 'T_world_origin:' in line:
            found = True
            continue
        if found and row < 4:
            vals = [float(x) for x in line.strip().split()]
            if len(vals) == 4:
                T[row] = vals
                row += 1
            if row == 4:
                break

    if not found or row < 4:
        return None
    return T


def read_points_compact(path):
    """Read points_compact.bin — packed float32 XYZ."""
    data = np.fromfile(path, dtype=np.float32)
    return data.reshape(-1, 3)


def read_intensities_compact(path):
    """Read intensities_compact.bin — packed float32."""
    return np.fromfile(path, dtype=np.float32)


def main():
    if len(sys.argv) < 2:
        print("Usage: glim-merge-submaps.py <dump_dir> [output.laz]")
        sys.exit(1)

    dump_dir = Path(sys.argv[1])
    output_path = sys.argv[2] if len(sys.argv) > 2 else str(dump_dir.parent / "glim_offline_map.laz")

    # Find all submap directories
    submap_dirs = sorted([d for d in dump_dir.iterdir()
                          if d.is_dir() and d.name.isdigit()])

    if not submap_dirs:
        print(f"No submap directories found in {dump_dir}")
        sys.exit(1)

    print(f"Found {len(submap_dirs)} submaps")

    all_xyz = []
    all_intensity = []
    total_points = 0

    for sd in submap_dirs:
        pts_file = sd / "points_compact.bin"
        int_file = sd / "intensities_compact.bin"
        data_file = sd / "data.txt"

        if not all(f.exists() for f in [pts_file, int_file, data_file]):
            print(f"  Skipping {sd.name} — missing files")
            continue

        T = read_transform(data_file)
        if T is None:
            print(f"  Skipping {sd.name} — couldn't parse transform")
            continue

        pts_local = read_points_compact(pts_file)
        intensities = read_intensities_compact(int_file)

        if len(pts_local) == 0:
            continue

        # Truncate to match (sometimes slight mismatch)
        n = min(len(pts_local), len(intensities))
        pts_local = pts_local[:n]
        intensities = intensities[:n]

        # Transform to world frame
        R = T[:3, :3]
        t = T[:3, 3]
        pts_world = (R @ pts_local.T).T + t

        all_xyz.append(pts_world)
        all_intensity.append(intensities)
        total_points += n

        if (int(sd.name) + 1) % 10 == 0:
            print(f"  Processed {int(sd.name) + 1}/{len(submap_dirs)} submaps ({total_points:,} points)")

    if total_points == 0:
        print("No points found!")
        sys.exit(1)

    print(f"\nMerging {total_points:,} points from {len(all_xyz)} submaps...")
    xyz = np.vstack(all_xyz)
    intensity = np.concatenate(all_intensity)

    # Voxel deduplication — remove overlapping points from adjacent submaps.
    # Without this, the same surface appears multiple times at slightly different
    # positions (ghosting). Uses grid-based dedup: quantize to voxel grid, keep
    # first occurrence (earliest submap = most loop-closed).
    voxel_size = float(os.environ.get('MERGE_VOXEL', '0.05'))  # 5cm default
    if voxel_size > 0:
        print(f"  Voxel dedup at {voxel_size}m...")
        keys = np.floor(xyz / voxel_size).astype(np.int64)
        # Pack xyz voxel coords into single int64 for fast unique
        # Shift to positive range first
        keys -= keys.min(axis=0)
        mx = keys.max(axis=0) + 1
        flat = keys[:, 0] * (mx[1] * mx[2]) + keys[:, 1] * mx[2] + keys[:, 2]
        _, unique_idx = np.unique(flat, return_index=True)
        unique_idx.sort()  # preserve original order
        before = len(xyz)
        xyz = xyz[unique_idx]
        intensity = intensity[unique_idx]
        print(f"  Dedup: {before:,} → {len(xyz):,} ({len(xyz)/before*100:.0f}% kept)")

    # Scale intensity to uint16 range (LAZ format)
    int_min, int_max = intensity.min(), intensity.max()
    if int_max > int_min:
        intensity_u16 = ((intensity - int_min) / (int_max - int_min) * 65535).astype(np.uint16)
    else:
        intensity_u16 = np.zeros(len(intensity), dtype=np.uint16)

    # Write LAZ
    print(f"Writing LAZ to {output_path}...")
    header = laspy.LasHeader(point_format=0, version="1.4")
    header.offsets = np.min(xyz, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])  # 1mm precision

    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = intensity_u16

    las.write(output_path)
    size_mb = os.path.getsize(output_path) / 1e6
    print(f"Done! {output_path} ({size_mb:.1f} MB, {total_points:,} points)")


if __name__ == "__main__":
    main()
