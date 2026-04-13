#!/usr/bin/env python3
"""Re-filter thermal data using segment-based approach.

Instead of per-frame FoV picks, finds time segments where the vehicle
is near the hotel and facing it, then exports ALL frames within those
segments as continuous sequences.

Usage:
    python3 refilter-segments.py --dry-run
    python3 refilter-segments.py
"""

import json
import math
import os
import shutil
import sys
from collections import defaultdict
from pathlib import Path

HOTEL_X, HOTEL_Y = 330470, 432915
MAX_DIST = 200
LEFT_FOV = (30, 150)
RIGHT_FOV = (210, 330)
SEGMENT_GAP_S = 30  # gap > 30s between FoV hits = new segment

PRE_BASE = Path("/var/mnt/nvme2/packages/big_blue_v2/thermal")
OUT_BASE = Path("/var/mnt/nvme2/packages/big_blue_v2/thermal_filtered_v2")

PASSES = [
    "pass01_mar03_2301", "pass02_mar04_1132", "pass03_mar04_1935",
    "pass04_mar05_1903", "pass05_mar05_1912", "pass06_mar05_1917",
    "pass07_mar06_1117", "pass08_mar09_1934", "pass09_mar10_1353",
    "pass10_mar10_1835",
]


def bearing(x1, y1, x2, y2):
    return math.degrees(math.atan2(x2 - x1, y2 - y1)) % 360


def in_fov(rel, side):
    if side == "left_thermal":
        return 30 <= rel <= 150
    if side == "right_thermal":
        return 210 <= rel <= 330
    return False


def main():
    dry_run = "--dry-run" in sys.argv

    for pass_name in PASSES:
        pre_dir = PRE_BASE / pass_name
        if not pre_dir.exists():
            print(f"SKIP: {pass_name} not found")
            continue

        # Load ALL frames with FoV check
        all_frames = []
        for side in ["left_thermal", "right_thermal"]:
            sd = pre_dir / side
            if not sd.exists():
                continue
            for f in sorted(sd.iterdir()):
                if not f.name.endswith(".meta.json"):
                    continue
                with open(f) as fh:
                    meta = json.load(fh)
                x, y = meta["bng"]["x"], meta["bng"]["y"]
                dist = math.sqrt((x - HOTEL_X) ** 2 + (y - HOTEL_Y) ** 2)
                heading = meta.get("heading_deg", 0)
                brg = bearing(x, y, HOTEL_X, HOTEL_Y)
                rel = (brg - heading + 360) % 360

                stem = f.name.replace(".meta.json", "")
                all_frames.append({
                    "side": side,
                    "stem": stem,
                    "ts": meta.get("timestamp_ns", 0),
                    "dist": dist,
                    "in_fov": in_fov(rel, side) and dist <= MAX_DIST,
                })

        all_frames.sort(key=lambda f: f["ts"])

        # Find FoV time segments
        fov_times = [f["ts"] for f in all_frames if f["in_fov"]]
        if not fov_times:
            print(f"{pass_name}: 0 FoV frames, skipping")
            continue

        segments = []
        seg_start = fov_times[0]
        seg_end = fov_times[0]
        for t in fov_times[1:]:
            if (t - seg_end) / 1e9 > SEGMENT_GAP_S:
                segments.append((seg_start, seg_end))
                seg_start = t
            seg_end = t
        segments.append((seg_start, seg_end))

        # Collect all frames within any segment
        keep = set()
        for s, e in segments:
            for f in all_frames:
                if s <= f["ts"] <= e:
                    keep.add((f["side"], f["stem"]))

        print(f"{pass_name}: {len(all_frames)} total, {len(fov_times)} FoV, "
              f"{len(segments)} segments, {len(keep)} to export")

        if dry_run:
            for i, (s, e) in enumerate(segments):
                seg_frames = [(f["side"], f["stem"]) for f in all_frames if s <= f["ts"] <= e]
                dur = (e - s) / 1e9
                print(f"  seg{i}: {dur:.0f}s, {len(seg_frames)} frames")
            continue

        # Copy frames
        out_dir = OUT_BASE / pass_name
        for side in ["left_thermal", "right_thermal"]:
            (out_dir / side).mkdir(parents=True, exist_ok=True)

        copied = 0
        for side, stem in sorted(keep):
            src_dir = pre_dir / side
            dst_dir = out_dir / side
            for ext in [".jpg", ".npy", ".meta.json"]:
                src = src_dir / f"{stem}{ext}"
                if src.exists():
                    dst = dst_dir / f"{stem}{ext}"
                    if not dst.exists():
                        shutil.copy2(src, dst)
            copied += 1

        # Count per side
        left = sum(1 for s, st in keep if s == "left_thermal")
        right = sum(1 for s, st in keep if s == "right_thermal")
        print(f"  Copied: {left} left, {right} right")

    if not dry_run:
        # Write manifest
        manifest = {"passes": {}}
        for pass_name in PASSES:
            out_dir = OUT_BASE / pass_name
            if not out_dir.exists():
                continue
            left = len([f for f in (out_dir / "left_thermal").iterdir()
                       if f.name.endswith(".jpg")]) if (out_dir / "left_thermal").exists() else 0
            right = len([f for f in (out_dir / "right_thermal").iterdir()
                        if f.name.endswith(".jpg")]) if (out_dir / "right_thermal").exists() else 0
            manifest["passes"][pass_name] = {
                "left_thermal": left, "right_thermal": right, "total": left + right
            }
        total = sum(p["total"] for p in manifest["passes"].values())
        manifest["total_frames"] = total
        (OUT_BASE / "package_manifest.json").write_text(json.dumps(manifest, indent=2))
        print(f"\nTotal: {total} frames in {OUT_BASE}")


if __name__ == "__main__":
    main()
