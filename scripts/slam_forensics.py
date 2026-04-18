#!/usr/bin/env python3
"""Sanity-audit SLAM odom output.

Reads /fast_lio/odometry or /dlio/odom_node/odom from an MCAP directory and
prints pose count, rate, cumulative distance, max/mean speed, trajectory
extent. Exits non-zero if implausible values are detected (speed > 30 m/s,
extent > 2 km on any axis, gap > 5 s).
"""
import argparse
import glob
import os
import struct
import sys


def parse_args():
    p = argparse.ArgumentParser(description="SLAM odom sanity audit")
    p.add_argument("slam_dir", help="Directory of MCAPs with odom topic")
    p.add_argument("--odom-topic", default="/fast_lio/odometry")
    p.add_argument("--max-speed", type=float, default=30.0, help="Fail if max speed exceeds (m/s)")
    p.add_argument("--max-extent", type=float, default=2000.0, help="Fail if any axis extent exceeds (m)")
    p.add_argument("--max-gap", type=float, default=5.0, help="Warn if pose gap exceeds (s)")
    return p.parse_args()


def parse_odometry_cdr(data):
    body = data[4:]
    off = 0
    sec, nanosec = struct.unpack_from('<iI', body, off); off += 8
    flen = struct.unpack_from('<I', body, off)[0]; off += 4
    off += flen
    off = (off + 3) & ~3
    clen = struct.unpack_from('<I', body, off)[0]; off += 4
    off += clen
    off = (off + 7) & ~7
    px, py, pz, qx, qy, qz, qw = struct.unpack_from('<7d', body, off)
    return sec * 10**9 + nanosec, (px, py, pz)


def main():
    args = parse_args()
    from mcap.stream_reader import StreamReader

    files = sorted(glob.glob(os.path.join(args.slam_dir, "**", "*.mcap"), recursive=True))
    files = [f for f in files if os.path.getsize(f) > 0]
    if not files:
        print(f"ERROR: no mcaps in {args.slam_dir}")
        sys.exit(2)
    print(f"Found {len(files)} MCAP(s) in {args.slam_dir}")

    poses = []
    for f in files:
        target_ids = set()
        with open(f, "rb") as fh:
            reader = StreamReader(fh)
            try:
                for record in reader.records:
                    rtype = type(record).__name__
                    if rtype == 'Channel':
                        if record.topic == args.odom_topic:
                            target_ids.add(record.id)
                    elif rtype == 'Message':
                        if record.channel_id not in target_ids:
                            continue
                        try:
                            ts, xyz = parse_odometry_cdr(record.data)
                        except Exception:
                            continue
                        poses.append((ts, xyz))
            except Exception as e:
                print(f"  {os.path.basename(f)}: stream ended early at {len(poses)} poses ({e})")
    poses.sort(key=lambda x: x[0])

    if not poses:
        print(f"ERROR: no odom messages on topic {args.odom_topic}")
        sys.exit(2)

    n = len(poses)
    t0, t1 = poses[0][0], poses[-1][0]
    span_s = (t1 - t0) / 1e9
    rate = (n - 1) / span_s if span_s > 0 else 0.0

    cumul = 0.0
    speeds = []
    gaps = []
    for i in range(1, n):
        dt = (poses[i][0] - poses[i-1][0]) / 1e9
        if dt <= 0:
            continue
        dx = poses[i][1][0] - poses[i-1][1][0]
        dy = poses[i][1][1] - poses[i-1][1][1]
        dz = poses[i][1][2] - poses[i-1][1][2]
        d = (dx*dx + dy*dy + dz*dz) ** 0.5
        cumul += d
        speeds.append(d / dt)
        gaps.append(dt)

    xs = [p[1][0] for p in poses]
    ys = [p[1][1] for p in poses]
    zs = [p[1][2] for p in poses]
    extent = (max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
    max_speed = max(speeds) if speeds else 0.0
    mean_speed = (sum(speeds) / len(speeds)) if speeds else 0.0
    max_gap = max(gaps) if gaps else 0.0

    print()
    print(f"{'Topic':.<20} {args.odom_topic}")
    print(f"{'Poses':.<20} {n:,}")
    print(f"{'Time span':.<20} {span_s:.1f} s")
    print(f"{'Pose rate':.<20} {rate:.2f} Hz")
    print(f"{'Cumul distance':.<20} {cumul:.1f} m")
    print(f"{'Max speed':.<20} {max_speed:.2f} m/s ({max_speed*3.6:.1f} km/h)")
    print(f"{'Mean speed':.<20} {mean_speed:.2f} m/s")
    print(f"{'Max gap':.<20} {max_gap:.2f} s")
    print(f"{'Extent X':.<20} {min(xs):.1f} .. {max(xs):.1f}  ({extent[0]:.1f} m)")
    print(f"{'Extent Y':.<20} {min(ys):.1f} .. {max(ys):.1f}  ({extent[1]:.1f} m)")
    print(f"{'Extent Z':.<20} {min(zs):.1f} .. {max(zs):.1f}  ({extent[2]:.1f} m)")
    print()

    failed = []
    if max_speed > args.max_speed:
        failed.append(f"max_speed {max_speed:.1f} > {args.max_speed} m/s")
    if max(extent) > args.max_extent:
        failed.append(f"extent {max(extent):.1f} > {args.max_extent} m on axis")
    warned = []
    if max_gap > args.max_gap:
        warned.append(f"max_gap {max_gap:.2f} > {args.max_gap} s")
    if rate < 5.0:
        warned.append(f"pose rate {rate:.2f} Hz < 5 Hz (expected ~10 Hz)")

    if warned:
        print("WARN:", "; ".join(warned))
    if failed:
        print("FAIL:", "; ".join(failed))
        sys.exit(1)
    print("PASS")


if __name__ == "__main__":
    main()
