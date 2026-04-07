#!/usr/bin/env python3
"""Project thermal + RGB imagery onto SLAM point cloud.

Reads a ROS2 bag with /ouster/points, /kiss/odometry, thermal images, and
RGB camera images. For each LiDAR scan:
  1. Transform to world frame using KISS-ICP pose
  2. Project each point into thermal + RGB cameras using calibrated extrinsics
  3. Sample pixel values at projected locations
  4. Output LAZ with switchable scalar fields

Usage:
  python3 thermal-slam-projection.py /path/to/bag /output/dir [--max_scans 100]
"""
import sys
import json
import argparse
import numpy as np
from pathlib import Path
import time


# --- Calibration data ---
# Thermal camera intrinsics (raw, with distortion)
K_RAW = np.array([
    [334.78419, 0.0,      317.07016],
    [0.0,       332.67743, 234.32726],
    [0.0,       0.0,       1.0],
], dtype=np.float64)
DIST_COEFFS = np.array([-0.164539, 0.026521, -0.004782, -0.004943, 0.0], dtype=np.float64)
IMG_W, IMG_H = 640, 480


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def invert_T_lidar_camera(tx, ty, tz, qx, qy, qz, qw):
    """Convert dvlc T_lidar_camera [t, q] to T_camera_lidar [R, t].

    dvlc T_lidar_camera: p_lidar = R_lc @ p_cam + t_lc
    We need T_camera_lidar: p_cam = R_cl @ p_lidar + t_cl
    Where: R_cl = R_lc^T, t_cl = -R_lc^T @ t_lc
    For quaternions: R^T = conjugate = negate xyz, keep w.
    """
    R_lc = quat_to_rot(qx, qy, qz, qw)
    R_cl = R_lc.T  # = quat_to_rot(-qx, -qy, -qz, qw)
    t_lc = np.array([tx, ty, tz])
    t_cl = -R_cl @ t_lc
    return R_cl, t_cl


# --- Extrinsics from dvlc calibrations (T_lidar_camera format) ---
# Source: /var/mnt/nvme2/slam-output/dvlc_*_target/calib.json (Feb 12)
#         /var/mnt/nvme2/slam-output/dvlc_*_slam/calib.json (Feb 11)

# Thermal2 = RIGHT — from dvlc_thermal2_target
# dvlc T_lidar_camera: t=[0,0,0], q=[-0.7023, -0.0145, 0.0129, 0.6998]
T2_R, T2_t = invert_T_lidar_camera(0, 0, 0, -0.7023300364952526, -0.014474507704056807, 0.01285336171480651, 0.6997966876958116)

# Thermal1 = LEFT — from dvlc_thermal1_target (DIFFERENT from thermal2!)
# dvlc T_lidar_camera: t=[0,0,0], q=[0.0148, -0.6966, 0.7172, -0.0152]
T1_R, T1_t = invert_T_lidar_camera(0, 0, 0, 0.014794891008185233, -0.6965769351428056, 0.7171679262038497, -0.015232222241534442)

# --- RGB Oryx cameras (2x2 binned: 3232x2426) ---
# Intrinsics from dvlc calibrations at binned resolution
# cam2 from dvlc_cam2_target (plumb_bob, zero distortion)
K_CAM2 = np.array([
    [1166.26303, 0.0,        1619.56279],
    [0.0,        1169.02224, 1199.32755],
    [0.0,        0.0,        1.0],
], dtype=np.float64)
DIST_CAM2 = np.zeros(5, dtype=np.float64)
CAM2_W, CAM2_H = 3232, 2426

# cam3 from dvlc_cam3_slam (fisheye model, used as pinhole with zero distortion)
K_CAM3 = np.array([
    [1192.57, 0.0,     1636.47],
    [0.0,     1195.50, 1183.27],
    [0.0,     0.0,     1.0],
], dtype=np.float64)
DIST_CAM3 = np.zeros(5, dtype=np.float64)
CAM3_W, CAM3_H = 3232, 2426

# Cam2 (RIGHT) — from dvlc_cam2_slam (60 frames, includes translation)
# dvlc T_lidar_camera: t=[0.0042, 0.2, -0.1298], q=[0.6891, -0.0018, 0.0278, 0.7241]
CAM2_R, CAM2_t = invert_T_lidar_camera(0.0042, 0.19999999999999998, -0.12980000000000003, 0.6891266360695715, -0.0018080031134361987, 0.027724202260755382, 0.724108126727758)

# Cam3 (LEFT) — from dvlc_cam3_slam (60 frames, includes translation)
# dvlc T_lidar_camera: t=[0.1952, 0.1427, -0.3996], q=[-0.0033, -0.7001, 0.7139, -0.0181]
CAM3_R, CAM3_t = invert_T_lidar_camera(0.19516619667266122, 0.14267676136819465, -0.39962206973777015, -0.0033307642509005065, -0.7000556252924953, 0.7138500245363018, -0.01814304206465452)


def odom_to_matrix(odom_msg):
    """Convert nav_msgs/Odometry to 4x4 transform matrix."""
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    R = quat_to_rot(ori.x, ori.y, ori.z, ori.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pos.x, pos.y, pos.z]
    return T


def project_points_to_camera(points_lidar, cam_R, cam_t, K, dist, img_w, img_h):
    """Project 3D lidar points into camera image plane.

    dvlc's T_lidar_camera is the lidar-to-camera transform:
      p_camera = R * p_lidar + t

    Returns:
        pixels: (N, 2) pixel coordinates
        valid: (N,) bool mask of points that land in the image
    """
    import cv2

    # dvlc T_lidar_camera: use directly as lidar->camera transform
    R_cam_lidar = cam_R
    t_cam_lidar = cam_t

    # Transform points to camera frame
    pts_cam = (R_cam_lidar @ points_lidar.T).T + t_cam_lidar

    # Points must be in front of camera (z > 0)
    in_front = pts_cam[:, 2] > 0.1

    # Project using OpenCV (handles distortion)
    rvec, _ = cv2.Rodrigues(R_cam_lidar)
    tvec = t_cam_lidar.reshape(3, 1)
    pts_2d, _ = cv2.projectPoints(
        points_lidar.astype(np.float64), rvec, tvec, K, dist)
    pixels = pts_2d.reshape(-1, 2)

    # Check bounds
    in_bounds = (
        in_front &
        (pixels[:, 0] >= 0) & (pixels[:, 0] < img_w) &
        (pixels[:, 1] >= 0) & (pixels[:, 1] < img_h)
    )

    return pixels, in_bounds


def decode_compressed(comp_msg):
    """Decode CompressedImage to BGR numpy array."""
    import cv2
    data = comp_msg.data
    if isinstance(data, (bytes, bytearray)):
        buf = np.frombuffer(data, dtype=np.uint8)
    else:
        buf = np.array(data, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    return img


# Build inferno LUT (256 entries, BGR)
def _build_inferno_lut():
    """Build inferno colormap LUT (BGR, 256 entries)."""
    import cv2
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    for i in range(256):
        lut[i, 0] = [i, i, i]
    lut = cv2.applyColorMap(lut, cv2.COLORMAP_INFERNO)
    return lut.reshape(256, 3)

_INFERNO_LUT = None


def enhance_thermal(img_bgr):
    """Convert colormap to intensity, clip cold (sky), stretch warm (buildings), inferno."""
    import cv2
    global _INFERNO_LUT
    if _INFERNO_LUT is None:
        _INFERNO_LUT = _build_inferno_lut()

    # Convert to grayscale (recovers thermal intensity from colormap)
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    # CLAHE for local contrast enhancement
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    eq = clahe.apply(gray)
    # Clip cold end: remap so bottom 30% of intensity → 0 (black),
    # remaining 70% stretches across full 0-255 range.
    # This kills the sky/cold background and gives buildings the full ramp.
    lo = np.percentile(eq, 30)
    stretched = np.clip((eq.astype(np.float32) - lo) / max(255 - lo, 1) * 255, 0, 255).astype(np.uint8)
    # Re-map through inferno colormap
    return _INFERNO_LUT[stretched]  # shape (H, W, 3) BGR


def sample_rgb_image(img_bgr, pixels, valid):
    """Sample RGB values from enhanced thermal image at pixel locations."""
    rgb = np.full((len(pixels), 3), 40, dtype=np.uint8)
    if not valid.any() or img_bgr is None:
        return rgb

    px = np.clip(pixels[valid, 0].astype(int), 0, img_bgr.shape[1] - 1)
    py = np.clip(pixels[valid, 1].astype(int), 0, img_bgr.shape[0] - 1)
    rgb[valid] = img_bgr[py, px]  # BGR
    return rgb


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path', help='Path to ROS2 bag directory')
    parser.add_argument('output_dir', help='Output directory for PLY files')
    parser.add_argument('--max_scans', type=int, default=200,
                        help='Max number of scans to process')
    parser.add_argument('--subsample', type=int, default=4,
                        help='Take every Nth point from each scan')
    parser.add_argument('--camera', choices=['thermal1', 'thermal2', 'both'],
                        default='thermal2', help='Which thermal camera to use')
    parser.add_argument('--max_files', type=int, default=3,
                        help='Max MCAP files to read (limits data volume)')
    args = parser.parse_args()

    import cv2
    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    bag_path = Path(args.bag_path)
    dst = Path(args.output_dir)
    dst.mkdir(parents=True, exist_ok=True)

    # Topics to read
    needed_topics = {
        '/ouster/points', '/kiss/odometry',
        '/thermal1/image_colormap/compressed',
        '/thermal2/image_colormap/compressed',
        '/camera2/camera_driver/image_masked/compressed',
        '/camera3/camera_driver/image_masked/compressed',
    }

    print(f"Reading bag: {bag_path}")
    print(f"Camera: {args.camera} (+ RGB cam2/cam3)")

    odom_list = []       # [(timestamp_s, decoded_msg)]
    scan_list = []       # [(timestamp_s, decoded_msg)]
    thermal1_list = []   # [(timestamp_s, decoded_msg)]
    thermal2_list = []
    cam2_list = []       # [(timestamp_s, decoded_msg)]  RGB RIGHT
    cam3_list = []       # [(timestamp_s, decoded_msg)]  RGB LEFT

    t0 = time.time()
    msg_count = 0

    # Find all MCAP files in bag directory
    if bag_path.is_dir():
        mcap_files = sorted(bag_path.glob('*.mcap'))
    else:
        mcap_files = [bag_path]

    if args.max_files and len(mcap_files) > args.max_files:
        mcap_files = mcap_files[:args.max_files]

    print(f"Reading {len(mcap_files)} MCAP files")

    for mcap_file in mcap_files:
        print(f"  Reading {mcap_file.name}...")
        with open(mcap_file, 'rb') as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            for schema, channel, message, decoded in reader.iter_decoded_messages(
                    topics=list(needed_topics)):
                topic = channel.topic
                t_sec = message.log_time / 1e9
                msg_count += 1

                if topic == '/kiss/odometry':
                    odom_list.append((t_sec, decoded))
                elif topic == '/ouster/points':
                    scan_list.append((t_sec, decoded))
                elif topic == '/thermal1/image_colormap/compressed':
                    thermal1_list.append((t_sec, decoded))
                elif topic == '/thermal2/image_colormap/compressed':
                    thermal2_list.append((t_sec, decoded))
                elif topic == '/camera2/camera_driver/image_masked/compressed':
                    cam2_list.append((t_sec, decoded))
                elif topic == '/camera3/camera_driver/image_masked/compressed':
                    cam3_list.append((t_sec, decoded))

        elapsed = time.time() - t0
        print(f"    Total: {msg_count} msgs ({elapsed:.0f}s) - "
              f"{len(scan_list)} scans, {len(odom_list)} odom, "
              f"{len(thermal1_list)}+{len(thermal2_list)} thermal, "
              f"{len(cam2_list)}+{len(cam3_list)} rgb")

    print(f"\nTotal: {len(scan_list)} scans, {len(odom_list)} odom, "
          f"{len(thermal1_list)} thermal1, {len(thermal2_list)} thermal2, "
          f"{len(cam2_list)} cam2, {len(cam3_list)} cam3")

    if not scan_list or not odom_list:
        print("ERROR: Missing scan or odometry data")
        sys.exit(1)

    # Select scans to process
    n_scans = min(args.max_scans, len(scan_list))
    indices = np.linspace(0, len(scan_list) - 1, n_scans, dtype=int)

    # Build odometry time array for interpolation
    odom_times = np.array([t for t, _ in odom_list])

    # Thermal time arrays
    if thermal1_list:
        t1_times = np.array([t for t, _ in thermal1_list])
    if thermal2_list:
        t2_times = np.array([t for t, _ in thermal2_list])
    # RGB camera time arrays
    if cam2_list:
        cam2_times = np.array([t for t, _ in cam2_list])
    if cam3_list:
        cam3_times = np.array([t for t, _ in cam3_list])

    # Process scans — accumulate per-field lists
    all_xyz = []
    all_rgb = []             # vertex RGB: camera RGB where available, inferno thermal elsewhere
    all_thermal_raw = []     # combined thermal grayscale
    all_thermal1_raw = []    # thermal1 (LEFT) only
    all_thermal2_raw = []    # thermal2 (RIGHT) only
    all_cam2_r = []          # cam2 (RIGHT) RGB channels
    all_cam2_g = []
    all_cam2_b = []
    all_cam3_r = []          # cam3 (LEFT) RGB channels
    all_cam3_g = []
    all_cam3_b = []
    # Ouster lidar fields
    all_lidar = {}           # {field_name: [arrays]}

    print(f"\nProcessing {n_scans} scans...")

    for i, idx in enumerate(indices):
        scan_time, scan_data = scan_list[idx]

        # Find closest odometry
        odom_idx = np.argmin(np.abs(odom_times - scan_time))
        odom_dt = abs(odom_times[odom_idx] - scan_time)
        if odom_dt > 0.5:
            continue  # Skip if no close odometry

        odom_msg = odom_list[odom_idx][1]
        T_world = odom_to_matrix(odom_msg)

        # Point cloud already decoded by mcap
        pc_msg = scan_list[idx][1]
        xyz, lidar_scalars = extract_points(pc_msg)

        if len(xyz) == 0:
            continue

        # Subsample
        step = args.subsample
        xyz = xyz[::step]
        n = len(xyz)
        sub_scalars = {k: v[::step] for k, v in lidar_scalars.items()}

        # Transform to world frame
        xyz_world = (T_world[:3, :3] @ xyz.T).T + T_world[:3, 3]

        # Per-point fields
        rgb = np.full((n, 3), 40, dtype=np.uint8)  # vertex colors
        thermal_raw = np.full(n, -1.0, dtype=np.float32)
        t1_raw = np.full(n, -1.0, dtype=np.float32)
        t2_raw = np.full(n, -1.0, dtype=np.float32)
        c2_r = np.full(n, -1.0, dtype=np.float32)
        c2_g = np.full(n, -1.0, dtype=np.float32)
        c2_b = np.full(n, -1.0, dtype=np.float32)
        c3_r = np.full(n, -1.0, dtype=np.float32)
        c3_g = np.full(n, -1.0, dtype=np.float32)
        c3_b = np.full(n, -1.0, dtype=np.float32)
        has_thermal = np.zeros(n, dtype=bool)
        has_rgb = np.zeros(n, dtype=bool)

        # --- Thermal cameras ---
        if args.camera in ('thermal2', 'both') and thermal2_list:
            t2_idx = np.argmin(np.abs(t2_times - scan_time))
            if abs(t2_times[t2_idx] - scan_time) < 0.2:
                t2_decoded = decode_compressed(thermal2_list[t2_idx][1])
                t2_gray = cv2.cvtColor(t2_decoded, cv2.COLOR_BGR2GRAY)
                t2_img = enhance_thermal(t2_decoded)
                pixels, valid = project_points_to_camera(
                    xyz, T2_R, T2_t, K_RAW, DIST_COEFFS, IMG_W, IMG_H)
                rgb_vals = sample_rgb_image(t2_img, pixels, valid)
                rgb = np.where(valid[:, None], rgb_vals, rgb)
                if valid.any():
                    px = np.clip(pixels[valid, 0].astype(int), 0, IMG_W - 1)
                    py = np.clip(pixels[valid, 1].astype(int), 0, IMG_H - 1)
                    t2_raw[valid] = t2_gray[py, px].astype(np.float32)
                    thermal_raw[valid] = t2_raw[valid]
                has_thermal |= valid

        if args.camera in ('thermal1', 'both') and thermal1_list:
            t1_idx = np.argmin(np.abs(t1_times - scan_time))
            if abs(t1_times[t1_idx] - scan_time) < 0.2:
                t1_decoded = decode_compressed(thermal1_list[t1_idx][1])
                t1_gray = cv2.cvtColor(t1_decoded, cv2.COLOR_BGR2GRAY)
                t1_img = enhance_thermal(t1_decoded)
                pixels, valid = project_points_to_camera(
                    xyz, T1_R, T1_t, K_RAW, DIST_COEFFS, IMG_W, IMG_H)
                rgb_vals = sample_rgb_image(t1_img, pixels, valid)
                new_coverage = valid & ~has_thermal
                rgb = np.where(new_coverage[:, None], rgb_vals, rgb)
                if valid.any():
                    px = np.clip(pixels[valid, 0].astype(int), 0, IMG_W - 1)
                    py = np.clip(pixels[valid, 1].astype(int), 0, IMG_H - 1)
                    t1_raw[valid] = t1_gray[py, px].astype(np.float32)
                    thermal_raw[new_coverage] = t1_raw[new_coverage]
                has_thermal |= valid

        # --- RGB Oryx cameras ---
        if cam2_list:
            c2_idx = np.argmin(np.abs(cam2_times - scan_time))
            if abs(cam2_times[c2_idx] - scan_time) < 0.2:
                c2_img = decode_compressed(cam2_list[c2_idx][1])
                if c2_img is not None:
                    pixels, valid = project_points_to_camera(
                        xyz, CAM2_R, CAM2_t, K_CAM2, DIST_CAM2, CAM2_W, CAM2_H)
                    if valid.any():
                        px = np.clip(pixels[valid, 0].astype(int), 0, CAM2_W - 1)
                        py = np.clip(pixels[valid, 1].astype(int), 0, CAM2_H - 1)
                        sampled = c2_img[py, px]  # BGR
                        c2_r[valid] = sampled[:, 2].astype(np.float32)
                        c2_g[valid] = sampled[:, 1].astype(np.float32)
                        c2_b[valid] = sampled[:, 0].astype(np.float32)
                        # Use camera RGB as vertex color where available
                        rgb[valid, 0] = sampled[:, 0]  # B
                        rgb[valid, 1] = sampled[:, 1]  # G
                        rgb[valid, 2] = sampled[:, 2]  # R
                        has_rgb |= valid

        if cam3_list:
            c3_idx = np.argmin(np.abs(cam3_times - scan_time))
            if abs(cam3_times[c3_idx] - scan_time) < 0.2:
                c3_img = decode_compressed(cam3_list[c3_idx][1])
                if c3_img is not None:
                    pixels, valid = project_points_to_camera(
                        xyz, CAM3_R, CAM3_t, K_CAM3, DIST_CAM3, CAM3_W, CAM3_H)
                    new_rgb = valid & ~has_rgb
                    if valid.any():
                        px = np.clip(pixels[valid, 0].astype(int), 0, CAM3_W - 1)
                        py = np.clip(pixels[valid, 1].astype(int), 0, CAM3_H - 1)
                        sampled = c3_img[py, px]  # BGR
                        c3_r[valid] = sampled[:, 2].astype(np.float32)
                        c3_g[valid] = sampled[:, 1].astype(np.float32)
                        c3_b[valid] = sampled[:, 0].astype(np.float32)
                        # Fill vertex color for points not already covered by cam2
                        if new_rgb.any():
                            px2 = np.clip(pixels[new_rgb, 0].astype(int), 0, CAM3_W - 1)
                            py2 = np.clip(pixels[new_rgb, 1].astype(int), 0, CAM3_H - 1)
                            s2 = c3_img[py2, px2]
                            rgb[new_rgb, 0] = s2[:, 0]
                            rgb[new_rgb, 1] = s2[:, 1]
                            rgb[new_rgb, 2] = s2[:, 2]
                        has_rgb |= valid

        # Points with no camera coverage: dark grey
        no_coverage = ~has_thermal & ~has_rgb
        rgb[no_coverage] = [40, 40, 40]

        all_xyz.append(xyz_world)
        all_rgb.append(rgb)
        all_thermal_raw.append(thermal_raw)
        all_thermal1_raw.append(t1_raw)
        all_thermal2_raw.append(t2_raw)
        all_cam2_r.append(c2_r)
        all_cam2_g.append(c2_g)
        all_cam2_b.append(c2_b)
        all_cam3_r.append(c3_r)
        all_cam3_g.append(c3_g)
        all_cam3_b.append(c3_b)

        # Accumulate lidar scalar fields
        for k, v in sub_scalars.items():
            if k not in all_lidar:
                all_lidar[k] = []
            all_lidar[k].append(v)

        if (i + 1) % 20 == 0 or i == 0:
            t_cov = has_thermal.sum() / n * 100 if n > 0 else 0
            r_cov = has_rgb.sum() / n * 100 if n > 0 else 0
            print(f"  Scan {i+1}/{n_scans}: {n} pts, "
                  f"{t_cov:.0f}% thermal, {r_cov:.0f}% RGB")

    # Concatenate
    all_xyz = np.concatenate(all_xyz)
    all_rgb = np.concatenate(all_rgb)
    all_thermal_raw = np.concatenate(all_thermal_raw)
    all_thermal1_raw = np.concatenate(all_thermal1_raw)
    all_thermal2_raw = np.concatenate(all_thermal2_raw)
    all_cam2_r = np.concatenate(all_cam2_r)
    all_cam2_g = np.concatenate(all_cam2_g)
    all_cam2_b = np.concatenate(all_cam2_b)
    all_cam3_r = np.concatenate(all_cam3_r)
    all_cam3_g = np.concatenate(all_cam3_g)
    all_cam3_b = np.concatenate(all_cam3_b)
    for k in all_lidar:
        all_lidar[k] = np.concatenate(all_lidar[k])

    print(f"\nTotal points: {len(all_xyz):,}")
    has_thermal = (all_thermal_raw >= 0)
    has_cam2 = (all_cam2_r >= 0)
    has_cam3 = (all_cam3_r >= 0)
    print(f"Thermal coverage: {has_thermal.sum():,} / {len(all_xyz):,} "
          f"({has_thermal.sum()/len(all_xyz)*100:.1f}%)")
    print(f"Cam2 (RIGHT) coverage: {has_cam2.sum():,} / {len(all_xyz):,} "
          f"({has_cam2.sum()/len(all_xyz)*100:.1f}%)")
    print(f"Cam3 (LEFT) coverage: {has_cam3.sum():,} / {len(all_xyz):,} "
          f"({has_cam3.sum()/len(all_xyz)*100:.1f}%)")
    print(f"Lidar fields: {list(all_lidar.keys())}")

    # Build scalar fields dict: per-camera RGB + thermal + ouster lidar fields
    scalars = {
        'thermal': all_thermal_raw,
        'thermal1_left': all_thermal1_raw,
        'thermal2_right': all_thermal2_raw,
        'cam2_right_r': all_cam2_r,
        'cam2_right_g': all_cam2_g,
        'cam2_right_b': all_cam2_b,
        'cam3_left_r': all_cam3_r,
        'cam3_left_g': all_cam3_g,
        'cam3_left_b': all_cam3_b,
    }
    scalars.update(all_lidar)

    laz_path = dst / 'thermal_slam.laz'
    save_laz(laz_path, all_xyz, all_rgb, scalars)
    print(f"\nSaved: {laz_path} ({laz_path.stat().st_size / 1e6:.1f} MB)")
    print(f"Scalar fields: {list(scalars.keys())}")


def extract_points(pc_msg):
    """Extract xyz + all Ouster scalar fields from PointCloud2.

    Returns: xyz (N,3), scalars dict {name: (N,) float32}
    """
    point_step = pc_msg.point_step
    n = pc_msg.width * pc_msg.height

    dtype_map = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    dt = np.dtype({
        'names': [f.name for f in pc_msg.fields],
        'formats': [dtype_map.get(f.datatype, 'f4') for f in pc_msg.fields],
        'offsets': [f.offset for f in pc_msg.fields],
        'itemsize': point_step,
    })
    data = pc_msg.data
    if not isinstance(data, (bytes, bytearray)):
        data = bytes(data)
    points = np.frombuffer(data, dtype=dt, count=n)

    xyz = np.stack([
        points['x'].astype(np.float64),
        points['y'].astype(np.float64),
        points['z'].astype(np.float64),
    ], axis=1)

    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)

    # Extract all useful Ouster fields
    scalars = {}
    field_names = points.dtype.names
    # Map ouster field names, avoiding LAS reserved names
    ouster_fields = {
        'reflectivity': 'reflectivity',
        'ambient': 'near_ir',           # Ouster calls near-IR 'ambient'
        'signal': 'signal',
        'range': 'lidar_range',         # 'range' is a Python builtin, use prefix
        'intensity': 'lidar_intensity', # 'intensity' clashes with LAS built-in
    }
    for src, dst_name in ouster_fields.items():
        if src in field_names:
            scalars[dst_name] = points[src][valid].astype(np.float32)

    return xyz[valid], scalars


def save_laz(path, xyz, rgb, scalars=None):
    """Save point cloud as LAZ (compressed LAS 1.4) with RGB + scalar fields."""
    import laspy
    n = len(xyz)

    # LAS 1.4, point format 7 (RGB + extra bytes)
    header = laspy.LasHeader(point_format=7, version="1.4")
    header.offsets = np.min(xyz, axis=0)
    header.scales = [0.01, 0.01, 0.01]  # 1cm precision — 0.001 overflows int32 with BNG coords

    # Add extra dimensions for scalar fields
    if scalars:
        for name in scalars:
            header.add_extra_dim(laspy.ExtraBytesParams(name=name, type=np.float32))

    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]

    # LAS RGB is 16-bit
    las.red = rgb[:, 2].astype(np.uint16) * 256    # BGR→RGB, scale to 16-bit
    las.green = rgb[:, 1].astype(np.uint16) * 256
    las.blue = rgb[:, 0].astype(np.uint16) * 256

    # Write scalar fields
    if scalars:
        for name, arr in scalars.items():
            setattr(las, name, arr.astype(np.float32))

    print(f"Writing LAZ ({n:,} points)...")
    las.write(str(path), laz_backend=laspy.LazBackend.LazrsParallel)


def save_colored_ply(path, xyz, rgb, scalars=None):
    """Save point cloud as PLY with RGB colors + scalar fields.

    scalars: dict of {name: float32_array} — each becomes a switchable
    scalar field in CloudCompare.
    """
    n = len(xyz)
    # Build header
    lines = [
        'ply',
        'format binary_little_endian 1.0',
        f'element vertex {n}',
        'property float x',
        'property float y',
        'property float z',
        'property uchar red',
        'property uchar green',
        'property uchar blue',
    ]
    scalar_names = []
    if scalars:
        for name in scalars:
            lines.append(f'property float {name}')
            scalar_names.append(name)
    lines.append('end_header')
    header = '\n'.join(lines) + '\n'

    # Build structured dtype
    fields = [('x','f4'),('y','f4'),('z','f4'),('r','u1'),('g','u1'),('b','u1')]
    for name in scalar_names:
        fields.append((name, 'f4'))
    dt = np.dtype(fields)

    with open(path, 'wb') as f:
        f.write(header.encode('ascii'))
        data = np.zeros(n, dtype=dt)
        data['x'] = xyz[:, 0].astype(np.float32)
        data['y'] = xyz[:, 1].astype(np.float32)
        data['z'] = xyz[:, 2].astype(np.float32)
        # PLY is RGB, OpenCV colormap is BGR
        data['r'] = rgb[:, 2]
        data['g'] = rgb[:, 1]
        data['b'] = rgb[:, 0]
        if scalars:
            for name, arr in scalars.items():
                data[name] = arr.astype(np.float32)
        f.write(data.tobytes())


if __name__ == '__main__':
    main()
