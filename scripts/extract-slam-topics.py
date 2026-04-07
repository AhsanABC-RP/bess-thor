#!/usr/bin/env python3
"""
Extract SLAM-relevant topics from large rosbag mcap files into slim mcap files.

Full sensor bags are ~17GB per mcap (cameras, all IMU, etc.). SLAM only needs
/ouster/points + /imu/data_guarded (~1.5GB per file). This tool extracts those
topics so reconstruction can read from fast NVMe instead of slow SATA SSDs.

Usage:
    # Extract a single bag (all mcap files)
    extract-slam-topics.py /var/mnt/ssd-c/bags/2026/03/12/bess_20260312_143905

    # Specify output dir (default: /var/mnt/nvme1/slam-cache/<bag_name>)
    extract-slam-topics.py /path/to/bag -o /var/mnt/nvme1/slam-cache/my_bag

    # Extract only mcap files covering a time range (epoch seconds)
    extract-slam-topics.py /path/to/bag --t-start 1773329878 --t-end 1773329998

    # Parallel extraction (default: 1, limited by SSD read)
    extract-slam-topics.py /path/to/bag --jobs 2

Output is a valid rosbag2 directory with metadata.yaml, ready for
glim-offline-replay.sh or glim-dense-reconstruct.py.
"""

import sys
import os
import time
import argparse
import subprocess
from pathlib import Path
from concurrent.futures import ThreadPoolExecutor, as_completed

try:
    from mcap.reader import make_reader
    from mcap.writer import Writer
except ImportError:
    print("ERROR: pip install mcap", file=sys.stderr)
    sys.exit(1)

SLAM_TOPICS = {'/ouster/points', '/imu/data_guarded', '/imu/data',
               '/gnss_1/odometry_earth', '/gnss_1/llh_position',
               '/camera2/camera_driver/image_masked/compressed',
               '/camera3/camera_driver/image_masked/compressed',
               '/thermal1/camera_driver/image_raw',
               '/thermal2/camera_driver/image_raw'}
DEFAULT_CACHE = '/var/mnt/nvme1/slam-cache'


def get_mcap_time_range(mcap_path):
    """Get (start_ns, end_ns) from mcap summary without reading all messages."""
    try:
        with open(mcap_path, 'rb') as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            if summary and summary.statistics:
                return (summary.statistics.message_start_time,
                        summary.statistics.message_end_time)
    except Exception:
        pass
    return None


def extract_one(input_path, output_path):
    """Extract SLAM topics from one mcap file. Returns (n_msgs, size_mb)."""
    try:
        return _extract_one_inner(input_path, output_path)
    except Exception as e:
        print(f"  WARNING: {os.path.basename(input_path)}: {e}", file=sys.stderr)
        # Write empty valid mcap
        with open(output_path, 'wb') as f:
            w = Writer(f); w.start(); w.finish()
        return 0, 0


def _extract_one_inner(input_path, output_path):
    with open(input_path, 'rb') as inf, open(output_path, 'wb') as outf:
        reader = make_reader(inf)
        writer = Writer(outf)
        writer.start()

        summary = reader.get_summary()
        if not summary:
            writer.finish()
            return 0, 0

        # Register only SLAM topic schemas and channels
        schema_map = {}
        channel_map = {}
        for chan_id, chan in summary.channels.items():
            if chan.topic not in SLAM_TOPICS:
                continue
            schema = summary.schemas.get(chan.schema_id)
            if schema and schema.id not in schema_map:
                schema_map[schema.id] = writer.register_schema(
                    name=schema.name, encoding=schema.encoding, data=schema.data)
            new_schema_id = schema_map.get(chan.schema_id, 0)
            channel_map[chan_id] = writer.register_channel(
                topic=chan.topic, message_encoding=chan.message_encoding,
                schema_id=new_schema_id)

        if not channel_map:
            writer.finish()
            return 0, 0

        n = 0
        for schema, channel, message in reader.iter_messages():
            if channel.id in channel_map:
                writer.add_message(
                    channel_id=channel_map[channel.id],
                    log_time=message.log_time,
                    data=message.data,
                    publish_time=message.publish_time,
                    sequence=message.sequence)
                n += 1

        writer.finish()

    sz = os.path.getsize(output_path) / (1024 * 1024)
    return n, sz


def rebuild_metadata(output_dir):
    """Run rebuild-bag-metadata.py on the output dir."""
    script = os.path.join(os.path.dirname(__file__), 'rebuild-bag-metadata.py')
    if os.path.exists(script):
        result = subprocess.run(
            [sys.executable, script, output_dir],
            capture_output=True, text=True)
        if result.returncode == 0:
            return True
        print(f"  metadata rebuild warning: {result.stderr.strip()}", file=sys.stderr)
    return False


def main():
    parser = argparse.ArgumentParser(
        description='Extract SLAM topics from rosbag mcap files')
    parser.add_argument('bag_dir', help='Source bag directory with .mcap files')
    parser.add_argument('-o', '--output', default=None,
                        help=f'Output directory (default: {DEFAULT_CACHE}/<bag_name>)')
    parser.add_argument('--t-start', type=float, default=0,
                        help='Only extract mcap files overlapping this start time (epoch seconds)')
    parser.add_argument('--t-end', type=float, default=0,
                        help='Only extract mcap files overlapping this end time (epoch seconds)')
    parser.add_argument('--jobs', type=int, default=1,
                        help='Parallel extraction jobs (default: 1, SSD is the bottleneck)')
    parser.add_argument('--force', action='store_true',
                        help='Overwrite existing output')
    args = parser.parse_args()

    bag_dir = args.bag_dir.rstrip('/')
    bag_name = os.path.basename(bag_dir)

    # Output directory
    output_dir = args.output or os.path.join(DEFAULT_CACHE, bag_name)

    # Find mcap files
    mcap_files = sorted([
        os.path.join(bag_dir, f) for f in os.listdir(bag_dir) if f.endswith('.mcap')
    ])
    if not mcap_files:
        print(f"ERROR: No .mcap files in {bag_dir}", file=sys.stderr)
        sys.exit(1)

    # Time-range filter: check each mcap's summary to skip irrelevant files
    if args.t_start > 0 or args.t_end > 0:
        t_start_ns = int(args.t_start * 1e9) if args.t_start > 0 else 0
        t_end_ns = int(args.t_end * 1e9) if args.t_end > 0 else float('inf')
        filtered = []
        for mcap_path in mcap_files:
            time_range = get_mcap_time_range(mcap_path)
            if time_range is None:
                # Can't determine time range — include it
                filtered.append(mcap_path)
                continue
            file_start, file_end = time_range
            # Include if any overlap with requested range
            if file_end >= t_start_ns and file_start <= t_end_ns:
                filtered.append(mcap_path)
        print(f"Time filter: {len(filtered)}/{len(mcap_files)} mcap files overlap "
              f"[{args.t_start:.0f}, {args.t_end:.0f}]")
        mcap_files = filtered

    if not mcap_files:
        print("ERROR: No mcap files match the time range", file=sys.stderr)
        sys.exit(1)

    # Check if output already exists
    if os.path.exists(output_dir) and not args.force:
        existing = [f for f in os.listdir(output_dir) if f.endswith('.mcap')]
        if existing:
            print(f"Output already exists: {output_dir} ({len(existing)} mcap files)")
            print(f"Use --force to overwrite, or use the existing extraction.")
            sys.exit(0)

    os.makedirs(output_dir, exist_ok=True)

    print(f"=== SLAM Topic Extraction ===")
    print(f"Source:  {bag_dir} ({len(mcap_files)} mcap files)")
    print(f"Output:  {output_dir}")
    print(f"Topics:  {', '.join(sorted(SLAM_TOPICS))}")
    print()

    t_start = time.time()
    total_msgs = 0
    total_size = 0

    def process_file(mcap_path):
        fname = os.path.basename(mcap_path)
        out_path = os.path.join(output_dir, fname)
        n, sz = extract_one(mcap_path, out_path)
        return fname, n, sz

    if args.jobs > 1:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            futures = {pool.submit(process_file, p): p for p in mcap_files}
            for future in as_completed(futures):
                fname, n, sz = future.result()
                total_msgs += n
                total_size += sz
                print(f"  {fname}: {n:,} msgs, {sz:.0f} MB")
    else:
        for mcap_path in mcap_files:
            fname, n, sz = process_file(mcap_path)
            total_msgs += n
            total_size += sz
            print(f"  {fname}: {n:,} msgs, {sz:.0f} MB")

    elapsed = time.time() - t_start

    # Remove any empty mcap files (no SLAM topics in that file)
    for f in os.listdir(output_dir):
        if f.endswith('.mcap'):
            fpath = os.path.join(output_dir, f)
            if os.path.getsize(fpath) < 100:
                os.unlink(fpath)

    # Rebuild metadata.yaml
    print("\nRebuilding metadata.yaml...")
    if rebuild_metadata(output_dir):
        print("  metadata.yaml written")
    else:
        print("  WARNING: metadata rebuild failed — run rebuild-bag-metadata.py manually")

    print(f"\n=== Complete ===")
    print(f"Output:   {output_dir}")
    print(f"Messages: {total_msgs:,}")
    print(f"Size:     {total_size:.0f} MB (from ~{len(mcap_files) * 17:.0f} GB source)")
    print(f"Time:     {elapsed:.0f}s ({elapsed/len(mcap_files):.0f}s per file)")
    print(f"\nReady for reconstruction:")
    print(f"  glim-dense-reconstruct.py {output_dir} <trajectory> [output.laz]")


if __name__ == '__main__':
    main()
