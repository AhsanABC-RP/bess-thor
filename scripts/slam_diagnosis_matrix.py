#!/usr/bin/env python3
"""Roll up runs.csv + per-run sanity.txt into a cross-engine comparison matrix.

For each (engine, tag) directory under $SLAM_OFFLINE, reads sanity.txt and
produces:
  diagnosis_matrix.md  — per-segment rows × per-engine cols, verdict + key metrics
  diagnosis_matrix.csv — same data flat
  diagnosis_by_engine.md — per-engine summary: how many PASS / WARN / FAIL by motion class

Requires segments.yaml for motion-class annotation.
"""
from __future__ import annotations

import argparse
import csv
import os
import re
from collections import defaultdict
from pathlib import Path


ENGINES = ["fastlio", "dlio", "glim"]


def parse_sanity(path):
    """Parse key: value lines from sanity.txt, return dict."""
    d = {}
    with open(path) as f:
        for line in f:
            m = re.match(r"^([a-z_0-9]+)\s*:\s*(.*?)\s*$", line)
            if m:
                d[m.group(1)] = m.group(2)
    return d


def parse_segments_yaml(path):
    """Return dict: tag -> (start, end, class, n_bags)."""
    if not os.path.exists(path):
        return {}
    tags = {}
    with open(path) as f:
        for line in f:
            m = re.match(
                r"\s*-\s*\{start:\s*(\d+),\s*end:\s*(\d+),\s*tag:\s*(\S+?),\s*class:\s*(\S+?),\s*n_bags:\s*(\d+)\}",
                line,
            )
            if m:
                s, e, t, c, n = m.groups()
                tags[t] = (int(s), int(e), c, int(n))
    return tags


def discover_runs(slam_offline):
    """Find all <engine>_<tag> dirs, return list of (engine, tag, path)."""
    runs = []
    for d in sorted(os.listdir(slam_offline)):
        full = os.path.join(slam_offline, d)
        if not os.path.isdir(full):
            continue
        for engine in ENGINES:
            prefix = engine + "_"
            if d.startswith(prefix):
                tag = d[len(prefix):]
                runs.append((engine, tag, full))
                break
    return runs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--slam-offline", default="/home/thor/nas/bess-bags/rolling/slam_offline")
    ap.add_argument("--segments-yaml", default="/home/thor/nas/bess-bags/rolling/slam_offline/segments.yaml")
    args = ap.parse_args()

    segments = parse_segments_yaml(args.segments_yaml)
    runs = discover_runs(args.slam_offline)

    # Build table: tag -> {engine -> sanity_dict}
    table = defaultdict(dict)
    for engine, tag, path in runs:
        sanity_path = os.path.join(path, "export", "sanity.txt")
        if os.path.exists(sanity_path):
            table[tag][engine] = parse_sanity(sanity_path)
        else:
            table[tag][engine] = {"verdict": "NO_EXPORT"}

    # Per-tag markdown matrix
    md_lines = ["# SLAM engine × segment diagnosis matrix", ""]
    csv_rows = []
    headers = ["tag", "class", "n_bags", "start", "end"]
    for e in ENGINES:
        for k in ["verdict", "pose_rate_hz", "p99_speed_mps", "cumul_distance_m",
                  "extent_x_m", "extent_y_m", "extent_z_m", "las_size_gb"]:
            headers.append(f"{e}_{k}")
    csv_rows.append(headers)

    md_lines.append("| Tag | Class | Bags | " + " | ".join(
        f"{e.upper()} verdict" for e in ENGINES) + " | "
        + " | ".join(f"{e.upper()} rate" for e in ENGINES) + " | "
        + " | ".join(f"{e.upper()} cumul_m" for e in ENGINES) + " |")
    md_lines.append("|" + "|".join(["---"] * (3 + 3 * len(ENGINES))) + "|")

    # Legacy / hand-named tags that predate the segments.yaml auto-generation.
    # Map to the motion class the bag range is known to belong to so the matrix
    # row has a real class label instead of "?".
    LEGACY_TAG_CLASS = {
        "drive1":       ("URBAN",   7,  16, 22),
        "bags40to45":   ("HIGHWAY", 6,  40, 45),
        "static_005_007": ("STATIC", 3, 5, 7),
        "micro_000_004":  ("MICRO",  5, 0, 4),
        "mixed_008_014":  ("MIXED",  7, 8, 14),
        "mid_023_030":    ("MIXED",  8, 23, 30),
    }

    def lookup_seg(tag):
        s = segments.get(tag)
        if s:
            return s
        legacy = LEGACY_TAG_CLASS.get(tag)
        if legacy:
            cls, n, start, end = legacy
            return (start, end, cls, n)
        return None

    sorted_tags = sorted(table.keys(), key=lambda t: ((lookup_seg(t) or (9999, 9999, "", 0))[0], t))
    for tag in sorted_tags:
        # Skip rows where every engine reported NO_EXPORT — these are stale
        # experiment directories that never completed a LAS export and
        # contribute no signal to the comparison.
        per_engine_verdicts = {e: table[tag].get(e, {}).get("verdict", "NO_EXPORT") for e in ENGINES}
        if all(v in ("NO_EXPORT", "—") for v in per_engine_verdicts.values()):
            continue

        seg = lookup_seg(tag)
        cls = seg[2] if seg else "?"
        n_bags = seg[3] if seg else "?"
        start = seg[0] if seg else "?"
        end = seg[1] if seg else "?"
        row = [tag, cls, str(n_bags), str(start), str(end)]

        verdicts = []
        rates = []
        cumuls = []
        for e in ENGINES:
            s = table[tag].get(e, {})
            v = s.get("verdict", "—")
            verdicts.append(v)
            rate = s.get("pose_rate_hz", "—")
            rates.append(rate)
            cumul = s.get("cumul_distance_m", "—")
            cumuls.append(cumul)
            for k in ["verdict", "pose_rate_hz", "p99_speed_mps", "cumul_distance_m",
                      "extent_x_m", "extent_y_m", "extent_z_m", "las_size_gb"]:
                row.append(s.get(k, ""))

        csv_rows.append(row)
        md_lines.append(f"| {tag} | {cls} | {n_bags} | "
                        + " | ".join(verdicts) + " | "
                        + " | ".join(rates) + " | "
                        + " | ".join(cumuls) + " |")

    md_path = os.path.join(args.slam_offline, "diagnosis_matrix.md")
    with open(md_path, "w") as f:
        f.write("\n".join(md_lines) + "\n")
    csv_path = os.path.join(args.slam_offline, "diagnosis_matrix.csv")
    with open(csv_path, "w", newline="") as f:
        w = csv.writer(f)
        for r in csv_rows:
            w.writerow(r)

    # Per-engine × class summary (use legacy-aware class lookup)
    summary = defaultdict(lambda: defaultdict(lambda: defaultdict(int)))
    for tag, by_engine in table.items():
        seg = lookup_seg(tag)
        cls = seg[2] if seg else "?"
        # Skip NO_EXPORT-only rows — they're stale directories, not real runs.
        if all(by_engine.get(e, {}).get("verdict", "NO_EXPORT") in ("NO_EXPORT", "—")
               for e in ENGINES):
            continue
        for e, s in by_engine.items():
            v = s.get("verdict", "—")
            summary[e][cls][v] += 1

    lines2 = ["# Per-engine verdict summary by motion class", ""]
    for e in ENGINES:
        lines2.append(f"## {e.upper()}")
        lines2.append("| Class | PASS | WARN | FAIL | NO_EXPORT |")
        lines2.append("|---|---|---|---|---|")
        for cls in ["STATIC", "MICRO", "URBAN", "HIGHWAY", "MIXED", "?"]:
            p = summary[e][cls]["PASS"]
            w = summary[e][cls]["WARN"]
            fl = summary[e][cls]["FAIL"]
            ne = summary[e][cls]["NO_EXPORT"]
            if p + w + fl + ne == 0:
                continue
            lines2.append(f"| {cls} | {p} | {w} | {fl} | {ne} |")
        lines2.append("")

    sum_path = os.path.join(args.slam_offline, "diagnosis_by_engine.md")
    with open(sum_path, "w") as f:
        f.write("\n".join(lines2) + "\n")

    print(f"Wrote {md_path} ({len(table)} segments)")
    print(f"Wrote {csv_path}")
    print(f"Wrote {sum_path}")


if __name__ == "__main__":
    main()
