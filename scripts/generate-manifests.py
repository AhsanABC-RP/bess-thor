#!/usr/bin/env python3
"""Generate OSID manifests for Big Blue thermal data and upload to R2.

For each pass:
1. Read all frame meta.json files (BNG coords, camera side)
2. Query spatial DB for UPRNs within 50m of each frame position
3. Apply FoV filter (is UPRN within camera's field of view?)
4. Group visible frames per UPRN → generate manifest.json
5. Upload manifests to R2

Usage:
    python3 generate-manifests.py --dry-run     # preview only
    python3 generate-manifests.py               # generate + upload
    python3 generate-manifests.py --local-only  # generate but don't upload
"""

import argparse
import json
import math
import os
import sqlite3
import sys
import uuid
from collections import defaultdict
from pathlib import Path

import boto3
from botocore.config import Config

# --- R2 config ---
R2_ACCOUNT_ID = "a6da94a2f835df03ba48f3e08cf4d04f"
R2_ACCESS_KEY_ID = "800d811f93c74252b6a25e5a73a71ea1"
R2_SECRET_ACCESS_KEY = "0070be05f8e730afd35ab37efee51f279ae1950eacf9b96613b3cc9f55f89272"
R2_BUCKET = "xridap-master-assets-blackpool"
R2_ENDPOINT = f"https://{R2_ACCOUNT_ID}.eu.r2.cloudflarestorage.com"

# --- Paths ---
FILTERED_BASE = Path("/var/mnt/nvme2/packages/big_blue_v2/thermal_filtered")
SPATIAL_DB = "/opt/bess/data/spatial/bess_uprn.db"

# --- FoV config ---
MAX_DISTANCE_M = 50  # UPRNs must be within 50m of vehicle
LEFT_FOV = (30, 150)   # relative angle range for left camera
RIGHT_FOV = (210, 330)  # relative angle range for right camera


def pass_to_bag_id(pass_name):
    """Convert pass name to bag_id for R2 key structure."""
    parts = pass_name.split("_")
    pass_num = parts[0]
    month_str = parts[1][:3]
    day_str = parts[1][3:]
    months = {"jan": "01", "feb": "02", "mar": "03", "apr": "04", "may": "05",
              "jun": "06", "jul": "07", "aug": "08", "sep": "09", "oct": "10",
              "nov": "11", "dec": "12"}
    month_num = months.get(month_str, "00")
    return f"BESS-big-blue-{pass_num}-2026{month_num}{day_str}"


def bearing_deg(x1, y1, x2, y2):
    """BNG bearing from (x1,y1) to (x2,y2) in degrees [0,360)."""
    dx = x2 - x1
    dy = y2 - y1
    return math.degrees(math.atan2(dx, dy)) % 360


def is_in_fov(relative_angle, camera_side):
    """Check if relative angle is within camera FOV."""
    if camera_side == "left_thermal":
        lo, hi = LEFT_FOV
    elif camera_side == "right_thermal":
        lo, hi = RIGHT_FOV
    else:
        return False
    return lo <= relative_angle <= hi


def load_uprns_near_hotel(db_path):
    """Load all UPRNs within the Big Blue area from spatial DB."""
    # Hotel center BNG (330470, 432915), load everything in ~300m radius
    db = sqlite3.connect(db_path)
    cur = db.cursor()
    cur.execute("""
        SELECT uprn, x, y FROM uprn_spatial
        WHERE x BETWEEN 330170 AND 330770
          AND y BETWEEN 432615 AND 433215
    """)
    uprns = {}
    for uprn, x, y in cur.fetchall():
        uprns[uprn] = (x, y)
    db.close()
    return uprns


def load_frames(pass_dir):
    """Load all frame metadata from a pass directory."""
    frames = []
    for side in ["left_thermal", "right_thermal"]:
        side_dir = pass_dir / side
        if not side_dir.exists():
            continue
        for f in sorted(side_dir.iterdir()):
            if not f.name.endswith(".meta.json"):
                continue
            with open(f) as fh:
                meta = json.load(fh)
            # Frame filename = meta.json name with .meta.json -> .jpg
            frame_name = f.name.replace(".meta.json", ".jpg")
            frames.append({
                "camera": side,
                "frame": frame_name,
                "x": meta["bng"]["x"],
                "y": meta["bng"]["y"],
                "heading_deg": meta.get("heading_deg", 0),
                "timestamp_ns": meta.get("timestamp_ns", 0),
            })
    return frames


def compute_visibility(frames, uprns):
    """For each frame, find which UPRNs are visible. Return uprn -> [frames]."""
    uprn_frames = defaultdict(list)

    for frame in frames:
        veh_x, veh_y = frame["x"], frame["y"]
        heading = frame["heading_deg"]

        for uprn, (ux, uy) in uprns.items():
            dist = math.sqrt((ux - veh_x)**2 + (uy - veh_y)**2)
            if dist > MAX_DISTANCE_M:
                continue

            # Bearing from vehicle to UPRN
            brg = bearing_deg(veh_x, veh_y, ux, uy)
            relative = (brg - heading + 360) % 360

            if is_in_fov(relative, frame["camera"]):
                uprn_frames[uprn].append({
                    "camera": frame["camera"],
                    "frame": frame["frame"],
                    "distance_m": round(dist, 1),
                })

    return uprn_frames


def generate_manifest(bag_id, uprn, visible_frames):
    """Generate manifest.json for one OSID."""
    osid = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{bag_id}_{uprn}"))
    return {
        "osid": osid,
        "uprn": uprn,
        "bag_id": bag_id,
        "visible_frames": [
            {"camera": f["camera"], "frame": f["frame"]}
            for f in visible_frames
        ],
        "hero_rgb": None,
    }


def make_s3_client():
    return boto3.client(
        "s3",
        endpoint_url=R2_ENDPOINT,
        aws_access_key_id=R2_ACCESS_KEY_ID,
        aws_secret_access_key=R2_SECRET_ACCESS_KEY,
        region_name="auto",
        config=Config(signature_version="s3v4", retries={"max_attempts": 3}),
    )


def main():
    parser = argparse.ArgumentParser(description="Generate OSID manifests")
    parser.add_argument("--dry-run", action="store_true", help="Preview only")
    parser.add_argument("--local-only", action="store_true", help="Generate but don't upload")
    parser.add_argument("--passes", nargs="+", help="Specific passes")
    args = parser.parse_args()

    # Load UPRNs
    print("Loading UPRNs from spatial DB...")
    uprns = load_uprns_near_hotel(SPATIAL_DB)
    print(f"  {len(uprns)} UPRNs in hotel area")

    # Find passes
    if args.passes:
        pass_dirs = [FILTERED_BASE / p for p in args.passes]
    else:
        pass_dirs = sorted(
            d for d in FILTERED_BASE.iterdir()
            if d.is_dir() and d.name.startswith("pass")
        )

    total_manifests = 0
    total_frames_matched = 0
    all_pass_stats = []

    s3 = None
    if not args.dry_run and not args.local_only:
        s3 = make_s3_client()

    for pass_dir in pass_dirs:
        if not pass_dir.exists():
            print(f"SKIP: {pass_dir.name} — not found")
            continue

        bag_id = pass_to_bag_id(pass_dir.name)
        print(f"\n{pass_dir.name} -> {bag_id}")

        # Load frames
        frames = load_frames(pass_dir)
        print(f"  {len(frames)} frames loaded")

        if not frames:
            continue

        # Compute visibility
        uprn_frames = compute_visibility(frames, uprns)
        print(f"  {len(uprn_frames)} UPRNs with visible frames")

        frames_matched = sum(len(v) for v in uprn_frames.values())
        total_manifests += len(uprn_frames)
        total_frames_matched += frames_matched

        if args.dry_run:
            # Show top 5 UPRNs by frame count
            top = sorted(uprn_frames.items(), key=lambda x: len(x[1]), reverse=True)[:5]
            for uprn, frs in top:
                osid = str(uuid.uuid5(uuid.NAMESPACE_DNS, f"{bag_id}_{uprn}"))
                print(f"    UPRN {uprn}: {len(frs)} frames, OSID {osid}")
            all_pass_stats.append({
                "pass": pass_dir.name, "bag_id": bag_id,
                "uprns": len(uprn_frames), "frame_matches": frames_matched,
            })
            continue

        # Generate and save/upload manifests
        uploaded = 0
        for uprn, vis_frames in uprn_frames.items():
            manifest = generate_manifest(bag_id, uprn, vis_frames)
            osid = manifest["osid"]
            r2_key = f"bags/{bag_id}/observations/{osid}/manifest.json"
            manifest_json = json.dumps(manifest, indent=2)

            if args.local_only:
                # Save locally
                local_dir = pass_dir / "manifests" / osid
                local_dir.mkdir(parents=True, exist_ok=True)
                (local_dir / "manifest.json").write_text(manifest_json)
            else:
                # Upload to R2
                s3.put_object(
                    Bucket=R2_BUCKET,
                    Key=r2_key,
                    Body=manifest_json.encode(),
                    ContentType="application/json",
                )
            uploaded += 1

        dest = "saved locally" if args.local_only else "uploaded to R2"
        print(f"  {uploaded} manifests {dest}")
        all_pass_stats.append({
            "pass": pass_dir.name, "bag_id": bag_id,
            "uprns": len(uprn_frames), "frame_matches": frames_matched,
            "uploaded": uploaded,
        })

    print(f"\n=== Summary ===")
    print(f"  Total manifests: {total_manifests}")
    print(f"  Total frame-UPRN matches: {total_frames_matched}")
    for s in all_pass_stats:
        print(f"  {s['pass']}: {s['uprns']} UPRNs, {s['frame_matches']} frame matches")


if __name__ == "__main__":
    main()
