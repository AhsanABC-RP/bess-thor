#!/usr/bin/env python3
"""
BESS Trajectory Evaluation Tool

Computes ATE (Absolute Trajectory Error) and RPE (Relative Pose Error)
between SLAM odometry and ground truth (MicroStrain EKF).

Usage:
    python3 evaluate_trajectory.py <bag_path> [--output <dir>]
"""

import argparse
import os
import sys
from pathlib import Path

import numpy as np
import pandas as pd
from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D
from evo.tools import file_interface, plot
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr


def extract_odometry(bag_path: str, topic: str) -> PoseTrajectory3D:
    """Extract odometry messages from bag to evo trajectory."""
    timestamps = []
    positions = []
    orientations = []

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            raise ValueError(f"Topic {topic} not found in bag")

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = deserialize_cdr(rawdata, connection.msgtype)

            t = timestamp * 1e-9  # nanoseconds to seconds
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation

            timestamps.append(t)
            positions.append([pos.x, pos.y, pos.z])
            orientations.append([ori.x, ori.y, ori.z, ori.w])

    if not timestamps:
        raise ValueError(f"No messages found on {topic}")

    return PoseTrajectory3D(
        positions_xyz=np.array(positions),
        orientations_quat_wxyz=np.array(orientations)[:, [3, 0, 1, 2]],  # xyzw -> wxyz
        timestamps=np.array(timestamps)
    )


def main():
    parser = argparse.ArgumentParser(description="Evaluate SLAM trajectory against ground truth")
    parser.add_argument("bag_path", help="Path to MCAP bag file or directory")
    parser.add_argument("--slam-topic", default="/slam/odometry", help="SLAM odometry topic")
    parser.add_argument("--gt-topic", default="/ekf/odometry_earth", help="Ground truth topic")
    parser.add_argument("--output", "-o", default="./eval_results", help="Output directory")
    parser.add_argument("--align", action="store_true", help="Align trajectories (Umeyama)")
    args = parser.parse_args()

    # Create output directory
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading bag: {args.bag_path}")

    # Extract trajectories
    print(f"Extracting SLAM trajectory from {args.slam_topic}...")
    try:
        traj_slam = extract_odometry(args.bag_path, args.slam_topic)
        print(f"  Found {len(traj_slam.timestamps)} poses")
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)

    print(f"Extracting ground truth from {args.gt_topic}...")
    try:
        traj_gt = extract_odometry(args.bag_path, args.gt_topic)
        print(f"  Found {len(traj_gt.timestamps)} poses")
    except Exception as e:
        print(f"  Error: {e}")
        sys.exit(1)

    # Synchronize trajectories
    print("Synchronizing trajectories...")
    traj_gt_sync, traj_slam_sync = sync.associate_trajectories(
        traj_gt, traj_slam, max_diff=0.1
    )
    print(f"  Matched {len(traj_gt_sync.timestamps)} poses")

    if len(traj_gt_sync.timestamps) < 10:
        print("Error: Not enough matched poses for evaluation")
        sys.exit(1)

    # Align if requested
    if args.align:
        print("Aligning trajectories (Umeyama)...")
        traj_slam_sync.align(traj_gt_sync, correct_scale=False)

    # Compute ATE
    print("\n=== Absolute Trajectory Error (ATE) ===")
    ate_result = metrics.APE(metrics.PoseRelation.translation_part)
    ate_result.process_data((traj_gt_sync, traj_slam_sync))

    print(f"  RMSE:   {ate_result.stats['rmse']:.4f} m")
    print(f"  Mean:   {ate_result.stats['mean']:.4f} m")
    print(f"  Median: {ate_result.stats['median']:.4f} m")
    print(f"  Std:    {ate_result.stats['std']:.4f} m")
    print(f"  Min:    {ate_result.stats['min']:.4f} m")
    print(f"  Max:    {ate_result.stats['max']:.4f} m")

    # Compute RPE
    print("\n=== Relative Pose Error (RPE) ===")
    rpe_result = metrics.RPE(
        metrics.PoseRelation.translation_part,
        delta=1.0,
        delta_unit=metrics.Unit.meters,
        all_pairs=False
    )
    rpe_result.process_data((traj_gt_sync, traj_slam_sync))

    print(f"  RMSE:   {rpe_result.stats['rmse']:.4f} m")
    print(f"  Mean:   {rpe_result.stats['mean']:.4f} m")
    print(f"  Median: {rpe_result.stats['median']:.4f} m")
    print(f"  Std:    {rpe_result.stats['std']:.4f} m")

    # Save results
    results = {
        'ate_rmse': ate_result.stats['rmse'],
        'ate_mean': ate_result.stats['mean'],
        'ate_median': ate_result.stats['median'],
        'ate_std': ate_result.stats['std'],
        'ate_max': ate_result.stats['max'],
        'rpe_rmse': rpe_result.stats['rmse'],
        'rpe_mean': rpe_result.stats['mean'],
        'rpe_median': rpe_result.stats['median'],
        'num_poses': len(traj_gt_sync.timestamps),
        'duration_s': traj_gt_sync.timestamps[-1] - traj_gt_sync.timestamps[0],
    }

    results_file = output_dir / "trajectory_metrics.yaml"
    import yaml
    with open(results_file, 'w') as f:
        yaml.dump(results, f, default_flow_style=False)
    print(f"\nResults saved to: {results_file}")

    # Save trajectories as TUM format
    slam_tum = output_dir / "slam_trajectory.tum"
    gt_tum = output_dir / "gt_trajectory.tum"
    file_interface.write_tum_trajectory_file(str(slam_tum), traj_slam_sync)
    file_interface.write_tum_trajectory_file(str(gt_tum), traj_gt_sync)
    print(f"Trajectories saved to: {slam_tum}, {gt_tum}")

    # Generate plots (non-interactive)
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        # Trajectory plot
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.plot(traj_gt_sync.positions_xyz[:, 0], traj_gt_sync.positions_xyz[:, 1],
                'b-', label='Ground Truth', linewidth=1)
        ax.plot(traj_slam_sync.positions_xyz[:, 0], traj_slam_sync.positions_xyz[:, 1],
                'r-', label='SLAM', linewidth=1)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title('Trajectory Comparison')
        ax.legend()
        ax.axis('equal')
        ax.grid(True)
        fig.savefig(output_dir / "trajectory_2d.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        # ATE over time
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(ate_result.np_arrays['timestamps'] - ate_result.np_arrays['timestamps'][0],
                ate_result.np_arrays['error_array'], 'r-', linewidth=0.5)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('ATE (m)')
        ax.set_title(f'Absolute Trajectory Error (RMSE: {ate_result.stats["rmse"]:.3f} m)')
        ax.grid(True)
        fig.savefig(output_dir / "ate_over_time.png", dpi=150, bbox_inches='tight')
        plt.close(fig)

        print(f"Plots saved to: {output_dir}")

    except Exception as e:
        print(f"Warning: Could not generate plots: {e}")

    print("\n=== Evaluation Complete ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
