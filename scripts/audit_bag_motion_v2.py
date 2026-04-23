#!/usr/bin/env python3
"""Re-audit rolling bags using /ouster/imu (always present).

Signal: linear_acceleration magnitude minus gravity baseline, gyro RMS.
Classify per bag into: STATIC / MICRO / URBAN / HIGHWAY.

Old audit used /dlio/odom_node/odom which depends on DLIO converging;
on static bags DLIO sometimes publishes junk IMU-propagated deltas which
made the old CSV report 93k m/s peaks. IMU is ground truth — always
there, no SLAM dependency, Ouster-internal BMI085 is on the same clock.

Outputs:
  motion_audit_v2.csv  — per-bag metrics + class
  segments.yaml        — contiguous class runs as (start, end, tag, class)
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import multiprocessing as mp
import os
import sys
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory


IMU_TOPIC = "/ouster/imu"
GRAVITY = 9.8066

# Classification thresholds on accel excess RMS (m/s^2, gravity removed)
# and gyro magnitude RMS (rad/s). Calibrated from the known drive1 / bags
# 40-45 pairs vs known-static bag_0.
CLASS_THRESHOLDS = [
    # (max_accel_excess_rms, max_gyro_rms, class)
    # Calibrated on BMI085 at 640 Hz where IMU noise floor alone yields
    # accel_rms ≈ 0.25 m/s² and gyro_rms ≈ 0.015 rad/s. STATIC == sensor
    # sitting still (no engine idle); MICRO == engine running, people
    # getting in / vibration.
    (0.35, 0.025, "STATIC"),
    (0.80, 0.100, "MICRO"),
    (3.00, 0.400, "URBAN"),
    (float("inf"), float("inf"), "HIGHWAY"),
]


def classify(accel_rms, gyro_rms):
    for acc_t, gyr_t, label in CLASS_THRESHOLDS:
        if accel_rms <= acc_t and gyro_rms <= gyr_t:
            return label
    return "HIGHWAY"


def audit_bag(path):
    accel_mags = []
    gyro_mags = []
    first_ns = None
    last_ns = None
    n = 0
    try:
        with open(path, "rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for _, _, msg, decoded in reader.iter_decoded_messages(topics=[IMU_TOPIC]):
                ax = decoded.linear_acceleration.x
                ay = decoded.linear_acceleration.y
                az = decoded.linear_acceleration.z
                accel_mags.append(math.sqrt(ax * ax + ay * ay + az * az))
                gx = decoded.angular_velocity.x
                gy = decoded.angular_velocity.y
                gz = decoded.angular_velocity.z
                gyro_mags.append(math.sqrt(gx * gx + gy * gy + gz * gz))
                if first_ns is None:
                    first_ns = msg.log_time
                last_ns = msg.log_time
                n += 1
    except Exception as e:
        return {"bag": os.path.basename(path), "error": str(e)}

    if n == 0:
        return {"bag": os.path.basename(path), "error": "no /ouster/imu messages"}

    arr_acc = np.asarray(accel_mags)
    arr_gyr = np.asarray(gyro_mags)
    excess = arr_acc - GRAVITY  # subtract 1g baseline
    accel_rms = float(np.sqrt(np.mean(excess ** 2)))
    accel_peak = float(np.max(np.abs(excess)))
    gyro_rms = float(np.sqrt(np.mean(arr_gyr ** 2)))
    gyro_peak = float(arr_gyr.max())
    dur_s = (last_ns - first_ns) / 1e9 if last_ns and first_ns else 0.0
    return {
        "bag": os.path.basename(path),
        "n_imu": n,
        "duration_s": round(dur_s, 2),
        "accel_rms": round(accel_rms, 3),
        "accel_peak": round(accel_peak, 3),
        "gyro_rms": round(gyro_rms, 4),
        "gyro_peak": round(gyro_peak, 3),
        "class": classify(accel_rms, gyro_rms),
    }


def audit_bag_worker(path):
    """Pool worker: audit one bag and return row dict including idx."""
    size_gb = os.path.getsize(path) / (1024 ** 3)
    if size_gb < 0.001:
        return {"bag": os.path.basename(path),
                "idx": int(os.path.basename(path).split("_")[1].split(".")[0]),
                "error": "empty"}
    r = audit_bag(path)
    r["idx"] = int(os.path.basename(path).split("_")[1].split(".")[0])
    return r


def group_segments(rows, min_bags=1):
    """Collapse contiguous bags of the same class into segments."""
    if not rows:
        return []
    segs = []
    start = 0
    for i in range(1, len(rows)):
        if rows[i]["class"] != rows[start]["class"]:
            segs.append((start, i - 1, rows[start]["class"]))
            start = i
    segs.append((start, len(rows) - 1, rows[start]["class"]))
    return [(a, b, c) for a, b, c in segs if (b - a + 1) >= min_bags]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--bag-dir", default="/home/thor/nas/bess-bags/rolling/bag")
    p.add_argument("--out-csv", default="/home/thor/nas/bess-bags/rolling/slam_offline/motion_audit_v2.csv")
    p.add_argument("--out-yaml", default="/home/thor/nas/bess-bags/rolling/slam_offline/segments.yaml")
    p.add_argument("--limit", type=int, default=0, help="Limit number of bags (0=all)")
    p.add_argument("--skip-broken", action="store_true", help="Skip bags that fail to parse")
    p.add_argument("--jobs", type=int, default=8, help="Parallel workers (default 8)")
    args = p.parse_args()

    files = sorted(
        glob.glob(os.path.join(args.bag_dir, "bag_*.mcap")),
        key=lambda p: int(os.path.basename(p).split("_")[1].split(".")[0]),
    )
    if args.limit:
        files = files[: args.limit]

    print(f"Auditing {len(files)} bags with {args.jobs} workers...")
    rows = []
    with mp.Pool(args.jobs) as pool:
        for i, r in enumerate(pool.imap_unordered(audit_bag_worker, files), 1):
            rows.append(r)
            if "error" in r:
                print(f"  [{i}/{len(files)}] bag_{r['idx']:3d} ERROR: {r['error']}")
            else:
                print(f"  [{i}/{len(files)}] bag_{r['idx']:3d} class={r['class']:7s} "
                      f"acc_rms={r['accel_rms']:5.2f} gyro_rms={r['gyro_rms']:5.3f}",
                      flush=True)

    rows.sort(key=lambda r: r["idx"])

    # CSV
    os.makedirs(os.path.dirname(args.out_csv), exist_ok=True)
    keys = ["idx", "bag", "n_imu", "duration_s", "accel_rms", "accel_peak",
            "gyro_rms", "gyro_peak", "class", "error"]
    with open(args.out_csv, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    print(f"\nWrote {args.out_csv} ({len(rows)} rows)")

    # Segments
    rows_ok = [r for r in rows if "class" in r]
    rows_ok.sort(key=lambda r: r["idx"])
    segs = group_segments(rows_ok)

    with open(args.out_yaml, "w") as fh:
        fh.write("# auto-generated by audit_bag_motion_v2.py\n")
        fh.write(f"# source: {args.bag_dir}\n")
        fh.write(f"# total_bags: {len(rows_ok)}\n")
        fh.write("segments:\n")
        for a, b, cls in segs:
            start_idx = rows_ok[a]["idx"]
            end_idx = rows_ok[b]["idx"]
            n_bags = b - a + 1
            tag = f"{cls.lower()}_{start_idx:03d}_{end_idx:03d}"
            fh.write(f"  - {{start: {start_idx}, end: {end_idx}, tag: {tag}, "
                     f"class: {cls}, n_bags: {n_bags}}}\n")
    print(f"Wrote {args.out_yaml} ({len(segs)} segments)")

    # Class histogram
    from collections import Counter
    print("\nClass histogram:")
    for cls, n in Counter(r["class"] for r in rows_ok).most_common():
        print(f"  {cls:8s}: {n}")
    print("\nSegments (contiguous):")
    for a, b, cls in segs:
        print(f"  bags {rows_ok[a]['idx']:3d}-{rows_ok[b]['idx']:3d}  {cls:8s}  ({b-a+1} bags)")


if __name__ == "__main__":
    main()
