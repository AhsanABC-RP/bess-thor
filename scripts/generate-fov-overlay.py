#!/usr/bin/env python3
"""Generate FoV validation overlay GeoJSON per pass and upload to R2.

Produces per-pass GeoJSON with:
- Vehicle trajectory (LineString)
- Heading arrows at sample points (LineStrings)
- Matched UPRN points with visibility stats (Points)

Usage:
    python3 generate-fov-overlay.py --dry-run
    python3 generate-fov-overlay.py
"""

import argparse
import json
import math
import sqlite3
import uuid
from collections import defaultdict
from pathlib import Path

import boto3
from botocore.config import Config

R2_ACCOUNT_ID = "a6da94a2f835df03ba48f3e08cf4d04f"
R2_ACCESS_KEY_ID = "800d811f93c74252b6a25e5a73a71ea1"
R2_SECRET_ACCESS_KEY = "0070be05f8e730afd35ab37efee51f279ae1950eacf9b96613b3cc9f55f89272"
R2_BUCKET = "xridap-master-assets-blackpool"
R2_ENDPOINT = f"https://{R2_ACCOUNT_ID}.eu.r2.cloudflarestorage.com"

FILTERED_BASE = Path("/var/mnt/nvme2/packages/big_blue_v2/thermal_filtered")
SPATIAL_DB = "/opt/bess/data/spatial/bess_uprn.db"

MAX_DISTANCE_M = 50
LEFT_FOV = (30, 150)
RIGHT_FOV = (210, 330)


def pass_to_bag_id(pass_name):
    parts = pass_name.split("_")
    pass_num = parts[0]
    month_str = parts[1][:3]
    day_str = parts[1][3:]
    months = {"jan": "01", "feb": "02", "mar": "03", "apr": "04", "may": "05",
              "jun": "06", "jul": "07", "aug": "08", "sep": "09", "oct": "10",
              "nov": "11", "dec": "12"}
    month_num = months.get(month_str, "00")
    return f"BESS-big-blue-{pass_num}-2026{month_num}{day_str}"


def bng_to_wgs84_approx(x, y):
    """Fast approximate BNG to WGS84 for Blackpool area."""
    # Linear approximation valid within ~1km of hotel center (330470, 432915)
    # Calibrated from: BNG(330470,432915) = WGS84(53.787838, -3.056281)
    ref_x, ref_y = 330470, 432915
    ref_lat, ref_lon = 53.787838, -3.056281
    # Scale factors at this latitude
    lat_per_m = 1 / 111320  # ~8.98e-6 deg/m
    lon_per_m = 1 / (111320 * math.cos(math.radians(ref_lat)))  # ~1.50e-5 deg/m
    lat = ref_lat + (y - ref_y) * lat_per_m
    lon = ref_lon + (x - ref_x) * lon_per_m
    return round(lat, 7), round(lon, 7)


def bearing_deg(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
    return math.degrees(math.atan2(dx, dy)) % 360


def is_in_fov(relative_angle, camera_side):
    if camera_side == "left_thermal":
        return 30 <= relative_angle <= 150
    elif camera_side == "right_thermal":
        return 210 <= relative_angle <= 330
    return False


def make_heading_arrow(lat, lon, heading_deg, length_m=20):
    """Create a short line from vehicle pos in heading direction."""
    lat_per_m = 1 / 111320
    lon_per_m = 1 / (111320 * math.cos(math.radians(lat)))
    # BNG bearing: 0=north, clockwise
    dx = length_m * math.sin(math.radians(heading_deg))
    dy = length_m * math.cos(math.radians(heading_deg))
    end_lat = lat + dy * lat_per_m
    end_lon = lon + dx * lon_per_m
    return [[lon, lat], [round(end_lon, 7), round(end_lat, 7)]]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--passes", nargs="+")
    args = parser.parse_args()

    # Load UPRNs
    db = sqlite3.connect(SPATIAL_DB)
    cur = db.cursor()
    cur.execute("""
        SELECT uprn, x, y, fulladdress, building_use
        FROM uprn_spatial
        WHERE x BETWEEN 330170 AND 330770
          AND y BETWEEN 432615 AND 433215
    """)
    uprn_info = {}
    for uprn, x, y, addr, use in cur.fetchall():
        uprn_info[uprn] = {"x": x, "y": y, "address": addr or "", "use": use or ""}
    db.close()
    print(f"Loaded {len(uprn_info)} UPRNs")

    if args.passes:
        pass_dirs = [FILTERED_BASE / p for p in args.passes]
    else:
        pass_dirs = sorted(
            d for d in FILTERED_BASE.iterdir()
            if d.is_dir() and d.name.startswith("pass")
        )

    s3 = None
    if not args.dry_run:
        s3 = boto3.client(
            "s3", endpoint_url=R2_ENDPOINT,
            aws_access_key_id=R2_ACCESS_KEY_ID,
            aws_secret_access_key=R2_SECRET_ACCESS_KEY,
            region_name="auto",
            config=Config(signature_version="s3v4", retries={"max_attempts": 3}),
        )

    for pass_dir in pass_dirs:
        if not pass_dir.exists():
            continue

        bag_id = pass_to_bag_id(pass_dir.name)
        print(f"\n{pass_dir.name} -> {bag_id}")

        # Load all frames
        frames = []
        for side in ["left_thermal", "right_thermal"]:
            sd = pass_dir / side
            if not sd.exists():
                continue
            for f in sorted(sd.iterdir()):
                if not f.name.endswith(".meta.json"):
                    continue
                with open(f) as fh:
                    meta = json.load(fh)
                frames.append({
                    "camera": side,
                    "frame": f.name.replace(".meta.json", ".jpg"),
                    "x": meta["bng"]["x"],
                    "y": meta["bng"]["y"],
                    "heading": meta.get("heading_deg", 0),
                    "ts": meta.get("timestamp_ns", 0),
                })

        frames.sort(key=lambda f: f["ts"])
        print(f"  {len(frames)} frames")

        features = []

        # 1. Trajectory LineString
        traj_coords = []
        for f in frames:
            lat, lon = bng_to_wgs84_approx(f["x"], f["y"])
            traj_coords.append([lon, lat])

        # Deduplicate consecutive identical coords
        deduped = [traj_coords[0]]
        for c in traj_coords[1:]:
            if c != deduped[-1]:
                deduped.append(c)

        if len(deduped) >= 2:
            features.append({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": deduped},
                "properties": {
                    "name": "trajectory",
                    "session": bag_id,
                    "points": len(deduped),
                    "heading_source": "bng_movement",
                },
            })

        # 2. Heading arrows (every 50th frame)
        for i in range(0, len(frames), 50):
            f = frames[i]
            lat, lon = bng_to_wgs84_approx(f["x"], f["y"])
            arrow = make_heading_arrow(lat, lon, f["heading"])
            features.append({
                "type": "Feature",
                "geometry": {"type": "LineString", "coordinates": arrow},
                "properties": {
                    "name": "heading_arrow",
                    "heading_deg": round(f["heading"], 1),
                },
            })

        # 3. UPRN points with visibility
        uprn_vis = defaultdict(lambda: {"left": 0, "right": 0, "min_dist": 999})
        for f in frames:
            for uprn, info in uprn_info.items():
                dist = math.sqrt((info["x"] - f["x"])**2 + (info["y"] - f["y"])**2)
                if dist > MAX_DISTANCE_M:
                    continue
                brg = bearing_deg(f["x"], f["y"], info["x"], info["y"])
                relative = (brg - f["heading"] + 360) % 360
                if is_in_fov(relative, f["camera"]):
                    uv = uprn_vis[uprn]
                    if f["camera"] == "left_thermal":
                        uv["left"] += 1
                    else:
                        uv["right"] += 1
                    if dist < uv["min_dist"]:
                        uv["min_dist"] = dist

        for uprn, vis in uprn_vis.items():
            info = uprn_info[uprn]
            lat, lon = bng_to_wgs84_approx(info["x"], info["y"])
            features.append({
                "type": "Feature",
                "geometry": {"type": "Point", "coordinates": [lon, lat]},
                "properties": {
                    "name": f"UPRN_{uprn}",
                    "address": info["address"],
                    "distance_m": round(vis["min_dist"], 1),
                    "camera": "both" if vis["left"] > 0 and vis["right"] > 0
                             else "left" if vis["left"] > 0 else "right",
                    "visible_count": vis["left"] + vis["right"],
                    "thermal_left_count": vis["left"],
                    "thermal_right_count": vis["right"],
                    "building_use": info["use"],
                },
            })

        geojson = {
            "type": "FeatureCollection",
            "features": features,
        }

        uprn_count = sum(1 for f in features if f["geometry"]["type"] == "Point")
        arrow_count = sum(1 for f in features if f["properties"].get("name") == "heading_arrow")
        print(f"  {uprn_count} UPRNs, {arrow_count} heading arrows, 1 trajectory")

        if args.dry_run:
            continue

        # Upload to R2
        r2_key = f"bags/{bag_id}/validation_overlay.geojson"
        body = json.dumps(geojson, indent=2).encode()
        s3.put_object(
            Bucket=R2_BUCKET, Key=r2_key,
            Body=body, ContentType="application/geo+json",
        )
        print(f"  Uploaded: {r2_key} ({len(body)/1024:.0f} KB)")

    print("\nDone.")


if __name__ == "__main__":
    main()
