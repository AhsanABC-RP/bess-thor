#!/usr/bin/env python3
"""Upload filtered Big Blue thermal data to R2 bucket.

Resumable: skips files already in R2 (HEAD check).
5G-tolerant: exponential backoff retries on failure.

Usage:
    python3 upload-thermal-r2.py --dry-run          # count files + size only
    python3 upload-thermal-r2.py                     # upload all passes
    python3 upload-thermal-r2.py --passes pass10_mar10_1835  # single pass
"""

import argparse
import json
import os
import sys
import time
import threading
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import boto3
from botocore.config import Config
from botocore.exceptions import ClientError, EndpointConnectionError, ConnectionClosedError

# --- R2 config ---
R2_ACCOUNT_ID = "a6da94a2f835df03ba48f3e08cf4d04f"
R2_ACCESS_KEY_ID = "800d811f93c74252b6a25e5a73a71ea1"
R2_SECRET_ACCESS_KEY = "0070be05f8e730afd35ab37efee51f279ae1950eacf9b96613b3cc9f55f89272"
R2_BUCKET = "xridap-master-assets-blackpool"
R2_ENDPOINT = f"https://{R2_ACCOUNT_ID}.eu.r2.cloudflarestorage.com"

# --- Paths ---
FILTERED_BASE = Path("/var/mnt/nvme2/packages/big_blue_v2/thermal_filtered")

# --- Content types ---
CONTENT_TYPES = {
    ".jpg": "image/jpeg",
    ".npy": "application/octet-stream",
    ".json": "application/json",
}

# --- Retry config ---
MAX_RETRIES = 5
INITIAL_BACKOFF_S = 2


def make_s3_client():
    return boto3.client(
        "s3",
        endpoint_url=R2_ENDPOINT,
        aws_access_key_id=R2_ACCESS_KEY_ID,
        aws_secret_access_key=R2_SECRET_ACCESS_KEY,
        region_name="auto",
        config=Config(
            signature_version="s3v4",
            retries={"max_attempts": 3, "mode": "adaptive"},
            connect_timeout=10,
            read_timeout=30,
        ),
    )


def r2_key_exists(s3, key):
    """Check if key already exists in R2."""
    try:
        s3.head_object(Bucket=R2_BUCKET, Key=key)
        return True
    except ClientError as e:
        if e.response["Error"]["Code"] in ("404", "NoSuchKey"):
            return False
        raise


def upload_with_retry(s3, local_path, r2_key, content_type):
    """Upload file with exponential backoff retry."""
    for attempt in range(MAX_RETRIES):
        try:
            s3.upload_file(
                str(local_path),
                R2_BUCKET,
                r2_key,
                ExtraArgs={"ContentType": content_type},
            )
            return True
        except (EndpointConnectionError, ConnectionClosedError, ClientError, OSError) as e:
            if attempt == MAX_RETRIES - 1:
                print(f"    FAILED after {MAX_RETRIES} attempts: {r2_key} — {e}")
                return False
            wait = INITIAL_BACKOFF_S * (2 ** attempt)
            print(f"    Retry {attempt+1}/{MAX_RETRIES} in {wait}s: {e}")
            time.sleep(wait)
    return False


def pass_to_bag_id(pass_name):
    """Convert pass name to bag_id for R2 key structure."""
    # pass01_mar03_2301 -> BESS-big-blue-pass01-20260303
    parts = pass_name.split("_")
    pass_num = parts[0]  # pass01
    month_str = parts[1][:3]  # mar
    day_str = parts[1][3:]  # 03
    months = {"jan": "01", "feb": "02", "mar": "03", "apr": "04", "may": "05",
              "jun": "06", "jul": "07", "aug": "08", "sep": "09", "oct": "10",
              "nov": "11", "dec": "12"}
    month_num = months.get(month_str, "00")
    return f"BESS-big-blue-{pass_num}-202603{day_str}"


def collect_files(pass_dir, bag_id):
    """Collect all files to upload with their R2 keys."""
    files = []
    for side in ["left_thermal", "right_thermal"]:
        side_dir = pass_dir / side
        if not side_dir.exists():
            continue
        for f in sorted(side_dir.iterdir()):
            ext = f.suffix
            if ext == ".jpg":
                r2_key = f"bags/{bag_id}/images/{side}/{f.name}"
            elif ext == ".npy":
                r2_key = f"bags/{bag_id}/raw_npy/{f.name}"
            elif ext == ".json" and f.name.endswith(".meta.json"):
                r2_key = f"bags/{bag_id}/meta/{side}/{f.name}"
            else:
                continue
            files.append((f, r2_key, CONTENT_TYPES.get(ext, "application/octet-stream")))
    return files


def main():
    parser = argparse.ArgumentParser(description="Upload thermal data to R2")
    parser.add_argument("--dry-run", action="store_true", help="Count files only")
    parser.add_argument("--passes", nargs="+", help="Specific passes to upload")
    parser.add_argument("--no-skip", action="store_true", help="Re-upload existing files")
    args = parser.parse_args()

    # Find passes
    if args.passes:
        pass_dirs = [FILTERED_BASE / p for p in args.passes]
    else:
        pass_dirs = sorted(
            d for d in FILTERED_BASE.iterdir()
            if d.is_dir() and d.name.startswith("pass")
        )

    # Collect all files
    all_files = []
    for pass_dir in pass_dirs:
        if not pass_dir.exists():
            print(f"SKIP: {pass_dir.name} — not found")
            continue
        bag_id = pass_to_bag_id(pass_dir.name)
        files = collect_files(pass_dir, bag_id)
        total_bytes = sum(f[0].stat().st_size for f in files)
        print(f"  {pass_dir.name} -> {bag_id}: {len(files)} files, {total_bytes/1024**2:.0f} MB")
        all_files.extend(files)

    total_size = sum(f[0].stat().st_size for f in all_files)
    print(f"\nTotal: {len(all_files)} files, {total_size/1024**3:.2f} GB")

    if args.dry_run:
        # Test R2 connectivity
        print("\nTesting R2 connectivity...")
        try:
            s3 = make_s3_client()
            resp = s3.list_objects_v2(Bucket=R2_BUCKET, MaxKeys=1)
            count = resp.get("KeyCount", 0)
            print(f"  Connected OK. Bucket has {'objects' if count else 'no objects yet'}.")
        except Exception as e:
            print(f"  CONNECTION FAILED: {e}")
        return

    # Upload with parallel workers
    NUM_WORKERS = 5
    lock = threading.Lock()
    counters = {"uploaded": 0, "skipped": 0, "failed": 0, "bytes": 0, "processed": 0}
    start_time = time.time()

    # Thread-local S3 clients (boto3 clients aren't thread-safe)
    thread_local = threading.local()

    def get_s3():
        if not hasattr(thread_local, "s3"):
            thread_local.s3 = make_s3_client()
        return thread_local.s3

    def process_file(item):
        local_path, r2_key, content_type = item
        s3 = get_s3()
        file_size = local_path.stat().st_size

        # Skip check
        if not args.no_skip:
            try:
                if r2_key_exists(s3, r2_key):
                    with lock:
                        counters["skipped"] += 1
                        counters["processed"] += 1
                        if counters["skipped"] % 500 == 0:
                            print(f"  [{counters['processed']}/{len(all_files)}] "
                                  f"skipped {counters['skipped']} existing...")
                    return
            except Exception:
                pass

        if upload_with_retry(s3, local_path, r2_key, content_type):
            with lock:
                counters["uploaded"] += 1
                counters["bytes"] += file_size
                counters["processed"] += 1
                if counters["uploaded"] % 100 == 0:
                    elapsed = time.time() - start_time
                    rate = (counters["bytes"] / 1024**2) / elapsed
                    remaining = total_size - counters["bytes"]
                    eta_s = remaining / (counters["bytes"] / elapsed) if counters["bytes"] else 0
                    print(f"  [{counters['processed']}/{len(all_files)}] "
                          f"uploaded={counters['uploaded']} skipped={counters['skipped']} "
                          f"failed={counters['failed']} rate={rate:.1f} MB/s "
                          f"ETA={eta_s/60:.0f}min")
        else:
            with lock:
                counters["failed"] += 1
                counters["processed"] += 1

    print(f"\nUploading with {NUM_WORKERS} parallel workers...")
    with ThreadPoolExecutor(max_workers=NUM_WORKERS) as executor:
        futures = [executor.submit(process_file, item) for item in all_files]
        for future in as_completed(futures):
            try:
                future.result()
            except Exception as e:
                print(f"  Worker exception: {e}")
                with lock:
                    counters["failed"] += 1

    elapsed = time.time() - start_time
    print(f"\n=== Upload complete ===")
    print(f"  Uploaded: {counters['uploaded']} ({counters['bytes']/1024**3:.2f} GB)")
    print(f"  Skipped:  {counters['skipped']} (already in R2)")
    print(f"  Failed:   {counters['failed']}")
    print(f"  Time:     {elapsed/60:.1f} min")
    if elapsed > 0 and counters["bytes"] > 0:
        print(f"  Rate:     {counters['bytes']/1024**2/elapsed:.1f} MB/s")


if __name__ == "__main__":
    main()
