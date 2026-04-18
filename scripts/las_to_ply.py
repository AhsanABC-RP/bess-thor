#!/usr/bin/env python3
"""
las_to_ply.py — convert LAS/LAZ to binary PLY for CloudCompare viewing.

The CloudCompare 2.11.3 build shipped with Ubuntu 24.04 ARM64 was compiled
without qLAS_IO, and PDAL isn't packaged for arm64 either. Rather than build
PDAL from source just to view a LAS, this converts LAS → PLY (binary little-
endian) which every CloudCompare build can open natively.

Preserves:
  - XYZ (float32)
  - intensity (uint16)
  - return_number / number_of_returns (uint8)
  - RGB if present (uint16 → uint8)
  - GPS time if present (float64)

Usage:
  python3 scripts/las_to_ply.py input.las [-o output.ply]
  python3 scripts/las_to_ply.py input.copc.laz
  python3 scripts/las_to_ply.py input.las --downsample 0.1   # 10 % random sample
"""
from __future__ import annotations
import argparse
import os
import sys
import struct
import numpy as np
import laspy


def _header(n: int, has_rgb: bool, has_gps: bool) -> bytes:
    lines = [
        "ply",
        "format binary_little_endian 1.0",
        f"element vertex {n}",
        "property float x",
        "property float y",
        "property float z",
        "property ushort intensity",
        "property uchar return_number",
        "property uchar number_of_returns",
    ]
    if has_rgb:
        lines += ["property uchar red", "property uchar green", "property uchar blue"]
    if has_gps:
        lines += ["property double gps_time"]
    lines += ["end_header", ""]
    return "\n".join(lines).encode("ascii")


def convert(in_path: str, out_path: str, downsample: float | None = None,
            chunk: int = 5_000_000) -> None:
    print(f"Reading: {in_path}")
    las = laspy.open(in_path)
    hdr = las.header
    total = hdr.point_count
    fmt = hdr.point_format
    has_rgb = all(d in fmt.dimension_names for d in ("red", "green", "blue"))
    has_gps = "gps_time" in fmt.dimension_names
    print(f"  Points: {total:,}  format={fmt.id}  rgb={has_rgb}  gps={has_gps}")
    print(f"  Scale:  {hdr.scales}  Offset: {hdr.offsets}")

    if downsample is not None:
        assert 0 < downsample <= 1.0, "downsample must be in (0, 1]"
        n_out = int(total * downsample)
        print(f"  Downsample: keeping ~{n_out:,} / {total:,} ({downsample*100:.1f}%)")
    else:
        n_out = total

    # PLY requires the vertex count in the header, so we write a placeholder and
    # rewrite if downsample discards differ from n_out. Using numpy RNG with a
    # fixed probability → actual count differs slightly; write exact count on finish.
    rng = np.random.default_rng(0)

    # Collect into a temp .raw then prepend header once count is known.
    tmp = out_path + ".raw"
    written = 0
    with open(tmp, "wb") as fout:
        for pts in las.chunk_iterator(chunk):
            x = np.asarray(pts.x, dtype=np.float32)
            y = np.asarray(pts.y, dtype=np.float32)
            z = np.asarray(pts.z, dtype=np.float32)
            inten = np.asarray(pts.intensity, dtype=np.uint16)
            rn = np.asarray(pts.return_number, dtype=np.uint8)
            nr = np.asarray(pts.number_of_returns, dtype=np.uint8)

            if downsample is not None and downsample < 1.0:
                mask = rng.random(x.shape[0]) < downsample
                x, y, z, inten, rn, nr = x[mask], y[mask], z[mask], inten[mask], rn[mask], nr[mask]

            cols = [x, y, z, inten, rn, nr]
            dtype_list = [("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                          ("i", "<u2"), ("rn", "u1"), ("nr", "u1")]

            if has_rgb:
                r = (np.asarray(pts.red,   dtype=np.uint32) >> 8).astype(np.uint8)
                g = (np.asarray(pts.green, dtype=np.uint32) >> 8).astype(np.uint8)
                b = (np.asarray(pts.blue,  dtype=np.uint32) >> 8).astype(np.uint8)
                if downsample is not None and downsample < 1.0:
                    r, g, b = r[mask], g[mask], b[mask]
                cols += [r, g, b]
                dtype_list += [("r", "u1"), ("g", "u1"), ("b", "u1")]

            if has_gps:
                t = np.asarray(pts.gps_time, dtype=np.float64)
                if downsample is not None and downsample < 1.0:
                    t = t[mask]
                cols += [t]
                dtype_list += [("t", "<f8")]

            n = cols[0].shape[0]
            if n == 0:
                continue
            rec = np.empty(n, dtype=dtype_list)
            for (name, _), col in zip(dtype_list, cols):
                rec[name] = col
            fout.write(rec.tobytes())
            written += n

            pct = 100.0 * (written / max(n_out, 1))
            print(f"  ...{written:,} pts ({pct:.1f}%)", end="\r", flush=True)

    print()
    print(f"  Writing PLY header ({written:,} vertices)")
    with open(out_path, "wb") as fout:
        fout.write(_header(written, has_rgb, has_gps))
        with open(tmp, "rb") as fin:
            while True:
                buf = fin.read(64 * 1024 * 1024)
                if not buf:
                    break
                fout.write(buf)
    os.remove(tmp)

    size_gb = os.path.getsize(out_path) / 1e9
    print(f"Done: {out_path} ({size_gb:.2f} GB)")


def main():
    ap = argparse.ArgumentParser(description="Convert LAS/LAZ -> binary PLY")
    ap.add_argument("input", help="input LAS/LAZ path")
    ap.add_argument("-o", "--output", help="output PLY path (default: input .ply)")
    ap.add_argument("--downsample", type=float, default=None,
                    help="random keep fraction in (0,1], e.g. 0.1 for 10%%")
    ap.add_argument("--chunk", type=int, default=5_000_000,
                    help="points per chunk (default 5M)")
    args = ap.parse_args()

    if not os.path.exists(args.input):
        print(f"ERROR: no such file: {args.input}", file=sys.stderr)
        sys.exit(1)

    out = args.output or (os.path.splitext(args.input)[0] + ".ply")
    if args.input.endswith(".copc.laz"):
        out = args.input.replace(".copc.laz", ".ply")
    convert(args.input, out, downsample=args.downsample, chunk=args.chunk)


if __name__ == "__main__":
    main()
