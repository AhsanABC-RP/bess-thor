#!/usr/bin/env python3
"""
Rebuild missing rosbag2 metadata.yaml from MCAP files.

MCAP files are self-contained — each has its own schema/topic/message index.
The metadata.yaml is just a rosbag2 wrapper. This script reconstructs it
by scanning all mcap files in a bag directory.

Usage:
    rebuild-bag-metadata.py <bag_dir> [--dry-run]
    rebuild-bag-metadata.py --scan-all <ssd_path> [--dry-run]
"""

import json
import os
import sys
import yaml
from pathlib import Path

try:
    from mcap.reader import make_reader
except ImportError:
    print("ERROR: pip install mcap")
    sys.exit(1)


def scan_mcap(path):
    """Extract topic info, message counts, and time range from one mcap file."""
    topics = {}  # topic_name -> {type, count, qos}
    min_ts = None
    max_ts = None

    with open(path, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        if summary is None:
            return None, None, None

        for cid, channel in summary.channels.items():
            schema = summary.schemas.get(channel.schema_id)
            msg_type = schema.name if schema else "unknown"
            count = summary.statistics.channel_message_counts.get(cid, 0)
            topics[channel.topic] = {
                "name": channel.topic,
                "type": msg_type,
                "serialization_format": "cdr",
                "count": count,
            }

        if summary.statistics:
            min_ts = summary.statistics.message_start_time
            max_ts = summary.statistics.message_end_time

    return topics, min_ts, max_ts


def rebuild_metadata(bag_dir):
    """Rebuild metadata.yaml for a bag directory."""
    bag_dir = Path(bag_dir)
    import re
    def _num_key(p):
        m = re.search(r'_(\d+)\.mcap$', str(p))
        return int(m.group(1)) if m else 0
    mcap_files = sorted(bag_dir.glob("*.mcap"), key=_num_key)

    if not mcap_files:
        print(f"  No mcap files in {bag_dir}")
        return None

    all_topics = {}  # topic -> {type, total_count}
    global_min_ts = None
    global_max_ts = None
    total_messages = 0

    for mcap in mcap_files:
        try:
            topics, min_ts, max_ts = scan_mcap(mcap)
        except Exception as e:
            print(f"  WARNING: Failed to read {mcap.name}: {e}")
            continue

        if topics is None:
            continue

        for name, info in topics.items():
            if name not in all_topics:
                all_topics[name] = {
                    "name": name,
                    "type": info["type"],
                    "serialization_format": info["serialization_format"],
                    "count": 0,
                }
            all_topics[name]["count"] += info["count"]
            total_messages += info["count"]

        if min_ts is not None:
            if global_min_ts is None or min_ts < global_min_ts:
                global_min_ts = min_ts
            if global_max_ts is None or max_ts > global_max_ts:
                global_max_ts = max_ts

    if not all_topics:
        print(f"  No valid topics found in {bag_dir}")
        return None

    duration_ns = (global_max_ts - global_min_ts) if (global_min_ts and global_max_ts) else 0

    # Default QoS profile (matches recorder defaults)
    default_qos = (
        "- history: 1\n  depth: 100\n  reliability: 1\n  durability: 2\n"
        "  deadline:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n"
        "  liveliness: 1\n  liveliness_lease_duration:\n"
        "    sec: 9223372036\n    nsec: 854775807\n"
        "  avoid_ros_namespace_conventions: false"
    )

    topics_list = []
    for info in sorted(all_topics.values(), key=lambda x: x["name"]):
        topics_list.append({
            "topic_metadata": {
                "name": info["name"],
                "type": info["type"],
                "serialization_format": info["serialization_format"],
                "offered_qos_profiles": default_qos,
            },
            "message_count": info["count"],
        })

    # Build relative paths for mcap files
    relative_paths = [{"path": f.name, "starting_time": {"nanoseconds_since_epoch": 0},
                       "duration": {"nanoseconds": 0}, "message_count": 0}
                      for f in mcap_files]

    metadata = {
        "rosbag2_bagfile_information": {
            "version": 5,
            "storage_identifier": "mcap",
            "duration": {"nanoseconds": duration_ns},
            "starting_time": {"nanoseconds_since_epoch": global_min_ts or 0},
            "message_count": total_messages,
            "topics_with_message_count": topics_list,
            "compression_format": "",
            "compression_mode": "",
            "relative_file_paths": [f.name for f in mcap_files],
            "files": relative_paths,
        }
    }

    return metadata


def main():
    dry_run = "--dry-run" in sys.argv
    scan_all = "--scan-all" in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith("--")]

    if not args:
        print("Usage: rebuild-bag-metadata.py <bag_dir> [--dry-run]")
        print("       rebuild-bag-metadata.py --scan-all <ssd_path> [--dry-run]")
        sys.exit(1)

    target = Path(args[0])

    if scan_all:
        # Scan all bag dirs under an SSD for missing metadata
        bag_dirs = []
        for root, dirs, files in os.walk(target / "bags"):
            mcaps = [f for f in files if f.endswith(".mcap")]
            if mcaps and "metadata.yaml" not in files:
                bag_dirs.append(Path(root))

        if not bag_dirs:
            print(f"No bags with missing metadata found under {target}")
            return

        print(f"Found {len(bag_dirs)} bags with missing metadata.yaml:")
        for bd in sorted(bag_dirs):
            n_mcap = len(list(bd.glob("*.mcap")))
            print(f"  {bd.name}: {n_mcap} mcap files")

        for bd in sorted(bag_dirs):
            print(f"\nRebuilding {bd.name}...")
            metadata = rebuild_metadata(bd)
            if metadata is None:
                continue
            n_topics = len(metadata["rosbag2_bagfile_information"]["topics_with_message_count"])
            n_msgs = metadata["rosbag2_bagfile_information"]["message_count"]
            print(f"  {n_topics} topics, {n_msgs:,} messages")
            if dry_run:
                print("  [DRY RUN] Would write metadata.yaml")
            else:
                out = bd / "metadata.yaml"
                with open(out, "w") as f:
                    yaml.dump(metadata, f, default_flow_style=False, sort_keys=False)
                print(f"  Wrote {out}")
    else:
        # Single bag directory
        if not target.is_dir():
            print(f"ERROR: {target} is not a directory")
            sys.exit(1)

        if (target / "metadata.yaml").exists():
            print(f"metadata.yaml already exists in {target}")
            if not dry_run:
                print("Use --dry-run to preview without overwriting, or delete it first")
                sys.exit(1)

        print(f"Rebuilding metadata for {target.name}...")
        metadata = rebuild_metadata(target)
        if metadata is None:
            sys.exit(1)

        n_topics = len(metadata["rosbag2_bagfile_information"]["topics_with_message_count"])
        n_msgs = metadata["rosbag2_bagfile_information"]["message_count"]
        n_files = len(metadata["rosbag2_bagfile_information"]["relative_file_paths"])
        print(f"  {n_files} mcap files, {n_topics} topics, {n_msgs:,} messages")

        if dry_run:
            print("  [DRY RUN] Would write metadata.yaml")
            print(yaml.dump(metadata, default_flow_style=False, sort_keys=False)[:500])
        else:
            out = target / "metadata.yaml"
            with open(out, "w") as f:
                yaml.dump(metadata, f, default_flow_style=False, sort_keys=False)
            print(f"  Wrote {out}")


if __name__ == "__main__":
    main()
