#!/usr/bin/env python3
"""
Audit per-bag motion on the rolling site-trial bags.

For each bag in /home/thor/nas/bess-bags/rolling/bag/bag_*.mcap:
  - Read /dlio/odom_node/odom (primary motion signal — present in every bag).
  - Compute bag duration, total XYZ path length, net displacement start->end,
    peak linear speed, peak angular speed, and IMU accel magnitude variance
    as a cross-check (uses /ouster/imu linear_acceleration).

Output: a table + a CSV + a JSON summary flagging which bags contain motion
(path_length > --min-path, default 1.0 m) so the offline SLAM rerun can be
restricted to the drive portion.

Usage:
  python3 scripts/audit_bag_motion.py
  python3 scripts/audit_bag_motion.py --bag-dir /home/thor/nas/bess-bags/rolling/bag --min-path 1.0
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from pathlib import Path

try:
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory as Ros2DecoderFactory
except ImportError:
    sys.stderr.write(
        "Missing mcap + mcap-ros2 deps. Install with:\n"
        "  pip install mcap mcap-ros2-support\n"
    )
    sys.exit(1)


ODOM_TOPIC = "/dlio/odom_node/odom"
IMU_TOPIC = "/ouster/imu"


def audit_bag(path: Path) -> dict:
    """Scan a single mcap for odom + imu and return a motion summary."""
    out = {
        "bag": path.name,
        "odom_msgs": 0,
        "imu_msgs": 0,
        "duration_s": 0.0,
        "path_length_m": 0.0,
        "displacement_m": 0.0,
        "peak_speed_mps": 0.0,
        "peak_ang_speed_rps": 0.0,
        "imu_accel_std": 0.0,
        "error": None,
    }

    last_xyz: tuple[float, float, float] | None = None
    first_xyz: tuple[float, float, float] | None = None
    last_t: float | None = None
    first_t: float | None = None
    peak_speed = 0.0
    peak_ang = 0.0
    path_len = 0.0

    imu_accel_mag_sq_sum = 0.0
    imu_accel_mag_sum = 0.0
    imu_count = 0

    try:
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[Ros2DecoderFactory()])
            for schema, channel, message, ros_msg in reader.iter_decoded_messages(
                topics=[ODOM_TOPIC, IMU_TOPIC]
            ):
                t = message.log_time * 1e-9
                if channel.topic == ODOM_TOPIC:
                    p = ros_msg.pose.pose.position
                    xyz = (p.x, p.y, p.z)
                    v = ros_msg.twist.twist.linear
                    w = ros_msg.twist.twist.angular
                    spd = math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
                    ang = math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)
                    if spd > peak_speed:
                        peak_speed = spd
                    if ang > peak_ang:
                        peak_ang = ang
                    if last_xyz is not None:
                        dx = xyz[0] - last_xyz[0]
                        dy = xyz[1] - last_xyz[1]
                        dz = xyz[2] - last_xyz[2]
                        path_len += math.sqrt(dx * dx + dy * dy + dz * dz)
                    if first_xyz is None:
                        first_xyz = xyz
                        first_t = t
                    last_xyz = xyz
                    last_t = t
                    out["odom_msgs"] += 1
                elif channel.topic == IMU_TOPIC:
                    a = ros_msg.linear_acceleration
                    mag = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
                    imu_accel_mag_sum += mag
                    imu_accel_mag_sq_sum += mag * mag
                    imu_count += 1

        if first_xyz and last_xyz:
            dx = last_xyz[0] - first_xyz[0]
            dy = last_xyz[1] - first_xyz[1]
            dz = last_xyz[2] - first_xyz[2]
            out["displacement_m"] = math.sqrt(dx * dx + dy * dy + dz * dz)
        if first_t is not None and last_t is not None:
            out["duration_s"] = last_t - first_t
        out["path_length_m"] = path_len
        out["peak_speed_mps"] = peak_speed
        out["peak_ang_speed_rps"] = peak_ang
        out["imu_msgs"] = imu_count
        if imu_count > 1:
            mean = imu_accel_mag_sum / imu_count
            var = max(0.0, imu_accel_mag_sq_sum / imu_count - mean * mean)
            out["imu_accel_std"] = math.sqrt(var)
    except Exception as e:
        out["error"] = f"{type(e).__name__}: {e}"

    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--bag-dir", default="/home/thor/nas/bess-bags/rolling/bag",
        help="Directory containing bag_N.mcap files",
    )
    ap.add_argument(
        "--out-dir", default="/home/thor/nas/bess-bags/rolling/slam_offline",
        help="Where to write motion_audit.csv + motion_audit.json",
    )
    ap.add_argument(
        "--min-path", type=float, default=1.0,
        help="Path-length threshold (m) to call a bag MOVING",
    )
    ap.add_argument(
        "--min-peak-speed", type=float, default=0.2,
        help="Cross-check: peak twist.linear speed (m/s) to consider moving",
    )
    args = ap.parse_args()

    bag_dir = Path(args.bag_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    bags = sorted(
        bag_dir.glob("bag_*.mcap"),
        key=lambda p: int(p.stem.split("_", 1)[1]),
    )
    if not bags:
        print(f"No bag_*.mcap files in {bag_dir}", file=sys.stderr)
        sys.exit(1)

    print(f"Auditing {len(bags)} bags from {bag_dir}\n")
    print(
        f"{'idx':>4} {'bag':<18} {'dur':>6} {'odom':>5} {'path_m':>8} "
        f"{'disp_m':>8} {'peak_v':>7} {'peak_w':>7} {'imu_std':>8}  moving"
    )
    print("-" * 100)

    results = []
    moving_idx = []
    for path in bags:
        idx = int(path.stem.split("_", 1)[1])
        r = audit_bag(path)
        r["idx"] = idx
        is_moving = (
            r["path_length_m"] >= args.min_path
            and r["peak_speed_mps"] >= args.min_peak_speed
            and r["error"] is None
        )
        r["moving"] = is_moving
        if is_moving:
            moving_idx.append(idx)
        results.append(r)
        marker = "MOV " if is_moving else "stat"
        if r["error"]:
            marker = "ERR "
        print(
            f"{idx:>4} {path.name:<18} {r['duration_s']:>6.1f} "
            f"{r['odom_msgs']:>5} {r['path_length_m']:>8.2f} "
            f"{r['displacement_m']:>8.2f} {r['peak_speed_mps']:>7.2f} "
            f"{r['peak_ang_speed_rps']:>7.2f} {r['imu_accel_std']:>8.3f}  {marker}"
        )

    # CSV
    csv_path = out_dir / "motion_audit.csv"
    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=[
                "idx", "bag", "duration_s", "odom_msgs", "imu_msgs",
                "path_length_m", "displacement_m",
                "peak_speed_mps", "peak_ang_speed_rps",
                "imu_accel_std", "moving", "error",
            ],
        )
        w.writeheader()
        for r in results:
            w.writerow({k: r.get(k, "") for k in w.fieldnames})

    # JSON + suggested contiguous moving range
    ranges: list[tuple[int, int]] = []
    if moving_idx:
        s = e = moving_idx[0]
        for i in moving_idx[1:]:
            if i == e + 1:
                e = i
            else:
                ranges.append((s, e))
                s = e = i
        ranges.append((s, e))

    summary = {
        "total_bags": len(bags),
        "moving_bags": len(moving_idx),
        "moving_indices": moving_idx,
        "moving_ranges": ranges,
        "thresholds": {
            "min_path_m": args.min_path,
            "min_peak_speed_mps": args.min_peak_speed,
        },
    }
    with open(out_dir / "motion_audit.json", "w") as f:
        json.dump(summary, f, indent=2)

    print("\n" + "=" * 100)
    print(f"MOVING bags: {len(moving_idx)} / {len(bags)}")
    if ranges:
        print("Suggested contiguous moving ranges:")
        for s, e in ranges:
            print(f"  bash scripts/offline_slam_replay.sh {s} {e} 1.0")
    print(f"\nWrote: {csv_path}")
    print(f"Wrote: {out_dir / 'motion_audit.json'}")


if __name__ == "__main__":
    main()
