#!/usr/bin/env python3
"""SLAM run diagnostic renderer.

Produces PNGs so the same forensics are visible in auto-mode runs without
asking a human to open CloudCompare or Foxglove. Three outputs per run:

  trajectory.png   top / side / iso view of the odom pose stream
  las_top.png      top-down orthographic scatter of the LAS (hex-binned)
  overview.png     trajectory + las_top + sanity card text, single image

Usage:
  python3 scripts/slam_viz.py <run_dir>
  python3 scripts/slam_viz.py --compare <run_dir1> <run_dir2> ...

Where <run_dir> is like /home/thor/nas/bess-bags/rolling/slam_offline/fastlio_drive1
(contains export/site_trial.las + slam_output/slam_output/*.mcap).

No open3d dep — laspy + matplotlib only, all Agg offscreen.
"""
from __future__ import annotations

import argparse
import glob
import os
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# Make offline_slam_to_las helpers reusable.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from offline_slam_to_las import load_odometry  # noqa: E402


ENGINE_ODOM_TOPIC = {
    "fastlio": "/fast_lio/odometry",
    "dlio": "/dlio/odom_node/odom",
    "glim": "/glim_ros/odom_corrected",
}


def detect_engine(run_dir):
    name = os.path.basename(run_dir.rstrip("/"))
    for prefix, topic in ENGINE_ODOM_TOPIC.items():
        if name.startswith(prefix):
            return prefix, topic
    return None, None


def load_poses_for_run(run_dir, odom_topic=None):
    odom_bags = os.path.join(run_dir, "slam_output", "slam_output")
    if not os.path.isdir(odom_bags):
        odom_bags = os.path.join(run_dir, "slam_output")
    files = sorted(glob.glob(os.path.join(odom_bags, "*.mcap")))
    files = [f for f in files if os.path.getsize(f) > 0]
    if not files:
        return [], odom_topic
    if odom_topic is None:
        _, odom_topic = detect_engine(run_dir)
    if odom_topic is None:
        return [], None
    try:
        poses = load_odometry(files, odom_topic, force=True)
    except SystemExit:
        poses = []
    return poses, odom_topic


def poses_to_arrays(poses):
    if not poses:
        return None
    ts = np.array([p[0] for p in poses], dtype=np.int64)
    xyz = np.array([p[1][:3, 3] for p in poses], dtype=np.float64)
    # Instantaneous speed per segment (not per-pose, so n-1 samples, padded
    # with final for length-match plotting).
    if len(xyz) >= 2:
        dt = np.diff(ts) / 1e9
        dt[dt <= 0] = 1e-6
        dpos = np.linalg.norm(np.diff(xyz, axis=0), axis=1)
        speeds = np.concatenate([[0.0], dpos / dt])
    else:
        speeds = np.zeros(len(xyz))
    return {"ts": ts, "xyz": xyz, "speed": speeds}


def plot_trajectory(poses_arr, out_path, title=""):
    if poses_arr is None or len(poses_arr["xyz"]) < 2:
        _empty_png(out_path, f"{title}\n(no poses)")
        return

    xyz = poses_arr["xyz"]
    v = poses_arr["speed"]
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))

    # Top-down X/Y
    ax = axes[0]
    sc = ax.scatter(xyz[:, 0], xyz[:, 1], c=v, s=2, cmap="viridis",
                    vmin=0, vmax=max(5.0, float(np.percentile(v, 99))))
    ax.plot(xyz[0, 0], xyz[0, 1], "g^", ms=10, label="start")
    ax.plot(xyz[-1, 0], xyz[-1, 1], "rv", ms=10, label="end")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("top-down (X-Y)")
    ax.set_aspect("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    plt.colorbar(sc, ax=ax, label="speed (m/s)", fraction=0.046)

    # X/Z side
    ax = axes[1]
    ax.scatter(xyz[:, 0], xyz[:, 2], c=v, s=2, cmap="viridis",
               vmin=0, vmax=max(5.0, float(np.percentile(v, 99))))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("side (X-Z)")
    ax.grid(alpha=0.3)

    # Speed over time
    ax = axes[2]
    t_rel = (poses_arr["ts"] - poses_arr["ts"][0]) / 1e9
    ax.plot(t_rel, v, "b-", lw=0.5)
    ax.set_xlabel("time (s)")
    ax.set_ylabel("speed (m/s)")
    ax.set_title(f"speed trace (cumul {np.sum(np.linalg.norm(np.diff(xyz, axis=0), axis=1)):.0f} m)")
    ax.grid(alpha=0.3)

    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def plot_las(las_path, out_path, title="", subsample=400_000):
    try:
        import laspy
    except ImportError:
        _empty_png(out_path, "laspy missing")
        return

    if not os.path.exists(las_path):
        _empty_png(out_path, f"{title}\n(no LAS)")
        return

    try:
        las = laspy.read(las_path)
    except Exception as e:
        _empty_png(out_path, f"{title}\nLAS load error: {e}")
        return

    n = len(las.x)
    # Subsample uniformly for plot speed
    if n > subsample:
        idx = np.random.default_rng(0).choice(n, subsample, replace=False)
        x = np.asarray(las.x[idx])
        y = np.asarray(las.y[idx])
        z = np.asarray(las.z[idx])
    else:
        x = np.asarray(las.x)
        y = np.asarray(las.y)
        z = np.asarray(las.z)

    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    ax = axes[0]
    ax.scatter(x, y, c=z, s=0.4, cmap="turbo", alpha=0.6,
               vmin=float(np.percentile(z, 2)), vmax=float(np.percentile(z, 98)))
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"top-down ({n:,} pts, {subsample:,} shown)")
    ax.grid(alpha=0.2)

    ax = axes[1]
    ax.scatter(x, z, c=y, s=0.4, cmap="coolwarm", alpha=0.6,
               vmin=float(np.percentile(y, 2)), vmax=float(np.percentile(y, 98)))
    ax.set_xlabel("x (m)")
    ax.set_ylabel("z (m)")
    ax.set_title("side (X-Z)")
    ax.grid(alpha=0.2)

    fig.suptitle(title, fontsize=11)
    fig.tight_layout()
    fig.savefig(out_path, dpi=110, bbox_inches="tight")
    plt.close(fig)


def _empty_png(out_path, msg):
    fig = plt.figure(figsize=(6, 3))
    fig.text(0.5, 0.5, msg, ha="center", va="center", fontsize=11)
    fig.savefig(out_path, dpi=80)
    plt.close(fig)


def read_sanity(run_dir):
    p = os.path.join(run_dir, "export", "sanity.txt")
    if not os.path.exists(p):
        return "(no sanity.txt)"
    return open(p).read()


def plot_overview(run_dir, out_path):
    poses_arr, odom_topic = None, None
    poses, odom_topic = load_poses_for_run(run_dir)
    poses_arr = poses_to_arrays(poses) if poses else None

    las_path = os.path.join(run_dir, "export", "site_trial.las")
    sanity = read_sanity(run_dir)

    fig = plt.figure(figsize=(20, 10))
    gs = fig.add_gridspec(2, 3, width_ratios=[1.2, 1.2, 1.0])

    # Top-down trajectory
    ax = fig.add_subplot(gs[0, 0])
    if poses_arr is not None and len(poses_arr["xyz"]) >= 2:
        xyz = poses_arr["xyz"]
        v = poses_arr["speed"]
        ax.scatter(xyz[:, 0], xyz[:, 1], c=v, s=2, cmap="viridis",
                   vmin=0, vmax=max(5.0, float(np.percentile(v, 99))))
        ax.plot(xyz[0, 0], xyz[0, 1], "g^", ms=10)
        ax.plot(xyz[-1, 0], xyz[-1, 1], "rv", ms=10)
        ax.set_aspect("equal")
    else:
        ax.text(0.5, 0.5, "no poses", ha="center", transform=ax.transAxes)
    ax.set_title(f"trajectory top-down  ({odom_topic or '?'})")
    ax.grid(alpha=0.3)

    # Speed over time
    ax = fig.add_subplot(gs[0, 1])
    if poses_arr is not None and len(poses_arr["xyz"]) >= 2:
        t_rel = (poses_arr["ts"] - poses_arr["ts"][0]) / 1e9
        ax.plot(t_rel, poses_arr["speed"], "b-", lw=0.5)
        ax.set_xlabel("t (s)")
        ax.set_ylabel("m/s")
    ax.set_title("speed trace")
    ax.grid(alpha=0.3)

    # Sanity card text
    ax = fig.add_subplot(gs[0, 2])
    ax.axis("off")
    ax.text(0.0, 1.0, sanity, family="monospace", fontsize=8, va="top")
    ax.set_title("sanity.txt")

    # LAS top-down + side (span 2 cols)
    import laspy
    try:
        if os.path.exists(las_path):
            las = laspy.read(las_path)
            x = np.asarray(las.x); y = np.asarray(las.y); z = np.asarray(las.z)
            n = len(x)
            sub = 400_000
            if n > sub:
                idx = np.random.default_rng(0).choice(n, sub, replace=False)
                x = x[idx]; y = y[idx]; z = z[idx]
            ax_t = fig.add_subplot(gs[1, 0:2])
            ax_t.scatter(x, y, c=z, s=0.3, cmap="turbo", alpha=0.6,
                         vmin=float(np.percentile(z, 2)),
                         vmax=float(np.percentile(z, 98)))
            ax_t.set_aspect("equal")
            ax_t.set_title(f"LAS top-down  ({n:,} pts shown {len(x):,})")
            ax_t.grid(alpha=0.2)

            ax_s = fig.add_subplot(gs[1, 2])
            ax_s.scatter(x, z, c=y, s=0.3, cmap="coolwarm", alpha=0.6)
            ax_s.set_title("LAS side (X-Z)")
            ax_s.grid(alpha=0.2)
        else:
            ax_t = fig.add_subplot(gs[1, :])
            ax_t.axis("off")
            ax_t.text(0.5, 0.5, "no LAS found", ha="center")
    except Exception as e:
        ax_t = fig.add_subplot(gs[1, :])
        ax_t.axis("off")
        ax_t.text(0.5, 0.5, f"LAS render failed: {e}", ha="center")

    fig.suptitle(os.path.basename(run_dir.rstrip("/")), fontsize=14)
    fig.tight_layout()
    fig.savefig(out_path, dpi=100, bbox_inches="tight")
    plt.close(fig)


def plot_engine_compare(run_dirs, out_path, title="engine comparison"):
    fig, axes = plt.subplots(1, len(run_dirs), figsize=(6 * len(run_dirs), 6),
                             squeeze=False)
    for ax, run_dir in zip(axes[0], run_dirs):
        poses, topic = load_poses_for_run(run_dir)
        arr = poses_to_arrays(poses) if poses else None
        if arr is None or len(arr["xyz"]) < 2:
            ax.text(0.5, 0.5, "no poses", ha="center", transform=ax.transAxes)
        else:
            xyz = arr["xyz"]
            ax.scatter(xyz[:, 0], xyz[:, 1], c=arr["speed"], s=2, cmap="viridis")
            ax.plot(xyz[0, 0], xyz[0, 1], "g^", ms=8)
            ax.plot(xyz[-1, 0], xyz[-1, 1], "rv", ms=8)
            ax.set_aspect("equal")
        ax.set_title(os.path.basename(run_dir.rstrip("/")))
        ax.grid(alpha=0.3)
    fig.suptitle(title, fontsize=14)
    fig.tight_layout()
    fig.savefig(out_path, dpi=100, bbox_inches="tight")
    plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("run_dirs", nargs="+", help="SLAM run dir(s)")
    p.add_argument("--compare", action="store_true",
                   help="Render single engine-compare.png across all run_dirs")
    args = p.parse_args()

    if args.compare and len(args.run_dirs) >= 2:
        out_dir = os.path.commonpath(args.run_dirs) or "."
        out_path = os.path.join(out_dir, "engine_compare.png")
        plot_engine_compare(args.run_dirs, out_path)
        print(f"Wrote {out_path}")
        return

    for rd in args.run_dirs:
        if not os.path.isdir(rd):
            print(f"SKIP {rd}: not a directory")
            continue
        out_dir = os.path.join(rd, "viz")
        os.makedirs(out_dir, exist_ok=True)
        print(f"\n=== {os.path.basename(rd.rstrip('/'))} ===")

        poses, topic = load_poses_for_run(rd)
        arr = poses_to_arrays(poses) if poses else None
        plot_trajectory(arr, os.path.join(out_dir, "trajectory.png"),
                        title=f"{os.path.basename(rd)}  {topic}")
        print(f"  {out_dir}/trajectory.png")

        las_path = os.path.join(rd, "export", "site_trial.las")
        plot_las(las_path, os.path.join(out_dir, "las_top.png"),
                 title=f"{os.path.basename(rd)}  LAS")
        print(f"  {out_dir}/las_top.png")

        plot_overview(rd, os.path.join(out_dir, "overview.png"))
        print(f"  {out_dir}/overview.png")


if __name__ == "__main__":
    main()
