#!/usr/bin/env python3
"""Extract registered point clouds from offline SLAM MCAP output to PLY.
Handles truncated MCAP files (killed mid-write)."""

import argparse
import struct
import sys
from pathlib import Path

import numpy as np


def parse_pointcloud2_cdr(data):
    """Parse PointCloud2 from CDR-encoded bytes, extract XYZ."""
    # CDR encapsulation: 4 bytes header (0x00 0x01 for LE)
    off = 4
    # header.stamp.sec (int32), header.stamp.nanosec (uint32)
    off += 8
    # header.frame_id (string: uint32 len + chars + padding)
    frame_len = struct.unpack_from('<I', data, off)[0]
    off += 4 + frame_len
    # align to 4
    off = (off + 3) & ~3
    # height (uint32)
    height = struct.unpack_from('<I', data, off)[0]; off += 4
    # width (uint32)
    width = struct.unpack_from('<I', data, off)[0]; off += 4
    n_points = height * width
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)

    # fields array: uint32 count, then each field
    n_fields = struct.unpack_from('<I', data, off)[0]; off += 4

    x_off = y_off = z_off = None
    for _ in range(n_fields):
        # field name (string)
        name_len = struct.unpack_from('<I', data, off)[0]; off += 4
        name = data[off:off+name_len-1].decode('ascii', errors='replace')
        off += name_len
        off = (off + 3) & ~3
        # offset (uint32)
        field_offset = struct.unpack_from('<I', data, off)[0]; off += 4
        # datatype (uint8)
        off += 1
        # padding to 4
        off = (off + 3) & ~3
        # count (uint32)
        off += 4
        if name == 'x': x_off = field_offset
        elif name == 'y': y_off = field_offset
        elif name == 'z': z_off = field_offset

    if x_off is None:
        return np.zeros((0, 3), dtype=np.float32)

    # is_bigendian (bool/uint8)
    off += 1
    off = (off + 3) & ~3
    # point_step (uint32)
    point_step = struct.unpack_from('<I', data, off)[0]; off += 4
    # row_step (uint32)
    off += 4
    # data (byte array: uint32 len + bytes)
    data_len = struct.unpack_from('<I', data, off)[0]; off += 4
    cloud_data = data[off:off+data_len]

    if len(cloud_data) < n_points * point_step:
        n_points = len(cloud_data) // point_step

    points = np.zeros((n_points, 3), dtype=np.float32)
    for i in range(n_points):
        base = i * point_step
        points[i, 0] = struct.unpack_from('<f', cloud_data, base + x_off)[0]
        points[i, 1] = struct.unpack_from('<f', cloud_data, base + y_off)[0]
        points[i, 2] = struct.unpack_from('<f', cloud_data, base + z_off)[0]

    valid = np.isfinite(points).all(axis=1)
    return points[valid]


def write_ply(path, points):
    """Write binary PLY file."""
    with open(path, 'wb') as f:
        header = (
            "ply\n"
            "format binary_little_endian 1.0\n"
            f"element vertex {len(points)}\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "end_header\n"
        )
        f.write(header.encode('ascii'))
        f.write(points.astype(np.float32).tobytes())


def main():
    parser = argparse.ArgumentParser(description='Extract SLAM clouds from MCAP')
    parser.add_argument('bag_path', help='Path to MCAP file or directory')
    parser.add_argument('-o', '--output', default='slam_cloud.ply', help='Output PLY file')
    parser.add_argument('--topic', default='/fast_lio/cloud_registered', help='Cloud topic')
    parser.add_argument('--voxel', type=float, default=0.1, help='Voxel downsample (m)')
    parser.add_argument('--max-scans', type=int, default=0, help='Max scans (0=all)')
    parser.add_argument('--skip', type=int, default=1, help='Use every Nth scan')
    args = parser.parse_args()

    from mcap.stream_reader import StreamReader

    bag_path = Path(args.bag_path)
    mcap_files = sorted(bag_path.glob('*.mcap')) if bag_path.is_dir() else [bag_path]

    all_points = []
    scan_count = 0
    topic_channel_ids = set()
    channels = {}
    target_topic = args.topic

    for mcap_file in mcap_files:
        print(f"Reading {mcap_file.name}...")
        with open(str(mcap_file), 'rb') as f:
            reader = StreamReader(f)
            try:
                for record in reader.records:
                    rtype = type(record).__name__
                    if rtype == 'Channel':
                        channels[record.id] = record
                        if record.topic == target_topic:
                            topic_channel_ids.add(record.id)
                    elif rtype == 'Message':
                        if record.channel_id not in topic_channel_ids:
                            continue
                        scan_count += 1
                        if scan_count % args.skip != 0:
                            continue
                        try:
                            pts = parse_pointcloud2_cdr(record.data)
                            if len(pts) > 0:
                                all_points.append(pts)
                        except Exception as e:
                            if scan_count <= 3:
                                print(f"  Decode error scan {scan_count}: {e}")
                        if scan_count % 100 == 0:
                            total = sum(len(p) for p in all_points)
                            print(f"  {scan_count} scans, {total:,} points...")
                        if args.max_scans > 0 and scan_count >= args.max_scans:
                            break
            except Exception as e:
                print(f"  Stopped reading at scan {scan_count}: {e}")
        if args.max_scans > 0 and scan_count >= args.max_scans:
            break

    if not all_points:
        print(f"ERROR: No points found in {scan_count} scans on {target_topic}")
        sys.exit(1)

    merged = np.vstack(all_points)
    print(f"Total: {scan_count} scans, {len(merged):,} raw points")

    if args.voxel > 0:
        print(f"Voxel downsampling at {args.voxel}m...")
        quantized = np.floor(merged / args.voxel).astype(np.int32)
        _, idx = np.unique(quantized, axis=0, return_index=True)
        merged = merged[idx]
        print(f"After voxel: {len(merged):,} points")

    output = Path(args.output)
    write_ply(str(output), merged)
    print(f"Saved: {output} ({output.stat().st_size / 1e6:.1f} MB)")


if __name__ == '__main__':
    main()
