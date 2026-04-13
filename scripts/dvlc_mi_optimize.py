#!/usr/bin/env python3
"""DVLC Mutual Information Extrinsic Optimizer.

Refines camera-LiDAR extrinsics by maximizing mutual information between
LiDAR reflectivity and camera pixel intensity for synchronized frame pairs.

Reads MCAP bags directly. Supports RGB (compressed) and thermal cameras.

Usage:
    python3 dvlc_mi_optimize.py <bag_dir> --camera thermal2 --n-pairs 20 --output /tmp/dvlc
    python3 dvlc_mi_optimize.py <bag_dir> --camera cam2 --n-pairs 15 --output /tmp/dvlc
"""
import argparse, sys, os, glob, json, time
import numpy as np
import cv2
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation

def log(msg):
    sys.stdout.write(msg + '\n')
    sys.stdout.flush()


# ── Camera configs ──────────────────────────────────────────────────

CAMERAS = {
    'cam2': {
        'topic_img': '/camera2/camera_driver/image_masked/compressed',
        'topic_lidar': '/ouster/points',
        'K': np.array([[1166.26, 0, 1619.56], [0, 1169.02, 1199.33], [0, 0, 1]], dtype=np.float64),
        'D': np.zeros(5, dtype=np.float64),
        'size': (3232, 2426),
        'direction': 'RIGHT',
        # Initial T_lidar_camera (dvlc format: p_lidar = R @ p_cam + t)
        'init_tlc': [0.0042, 0.2, -0.1298, 0.6891, -0.0018, 0.0278, 0.7241],
    },
    'cam3': {
        'topic_img': '/camera3/camera_driver/image_masked/compressed',
        'topic_lidar': '/ouster/points',
        'K': np.array([[1192.57, 0, 1636.47], [0, 1195.50, 1183.27], [0, 0, 1]], dtype=np.float64),
        'D': np.zeros(5, dtype=np.float64),
        'size': (3232, 2426),
        'direction': 'LEFT',
        'init_tlc': [0.1952, -0.1427, 0.3996, 0.0181, 0.7139, 0.7001, -0.0033],
    },
    'thermal1': {
        'topic_img': '/thermal1/camera_driver/image_raw',
        'topic_lidar': '/ouster/points',
        # White-calibrated intrinsics (2026-03-13)
        'K': np.array([[339.28, 0, 321.68], [0, 339.01, 247.97], [0, 0, 1]], dtype=np.float64),
        'D': np.array([-0.1741, 0.0524, -0.001, 0.0012, -0.0157], dtype=np.float64),
        'size': (640, 480),
        'direction': 'LEFT',
        'init_tlc': [0, 0, 0, -0.6651, 0.0430, 0.0316, 0.7448],
        'is_thermal': True,
    },
    'thermal2': {
        'topic_img': '/thermal2/camera_driver/image_raw',
        'topic_lidar': '/ouster/points',
        'K': np.array([[339.28, 0, 321.68], [0, 339.01, 247.97], [0, 0, 1]], dtype=np.float64),
        'D': np.array([-0.1741, 0.0524, -0.001, 0.0012, -0.0157], dtype=np.float64),
        'size': (640, 480),
        'direction': 'RIGHT',
        'init_tlc': [0, 0, 0, 0.6922, -0.0066, 0.0061, 0.7222],
        'is_thermal': True,
    },
}


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),    1-2*(qx*qx+qy*qy)]
    ])


def tlc_to_T_camera_lidar(tx, ty, tz, qx, qy, qz, qw):
    """dvlc T_lidar_camera → 4x4 T_camera_lidar."""
    R_lc = quat_to_rot(qx, qy, qz, qw)
    R_cl = R_lc.T
    t_lc = np.array([tx, ty, tz])
    t_cl = -R_cl @ t_lc
    T = np.eye(4)
    T[:3, :3] = R_cl
    T[:3, 3] = t_cl
    return T


def project_lidar(pts_lidar, T_cl, K, D, w, h):
    """Project lidar points to camera pixels. Returns (N,2) pixels, (N,) valid mask, (N,) depths."""
    pts_cam = (T_cl[:3, :3] @ pts_lidar.T).T + T_cl[:3, 3]
    in_front = pts_cam[:, 2] > 0.5

    rvec, _ = cv2.Rodrigues(T_cl[:3, :3])
    tvec = T_cl[:3, 3].reshape(3, 1)
    pts_2d, _ = cv2.projectPoints(pts_lidar.astype(np.float64), rvec, tvec, K, D)
    pixels = pts_2d.reshape(-1, 2)

    valid = (in_front &
             (pixels[:, 0] >= 0) & (pixels[:, 0] < w) &
             (pixels[:, 1] >= 0) & (pixels[:, 1] < h))

    return pixels, valid, pts_cam[:, 2]


def mutual_information(lidar_vals, image_vals, bins=32):
    """Compute MI between two 1D arrays (higher = better alignment)."""
    if len(lidar_vals) < 100:
        return 0.0

    # Normalize to [0, bins-1]
    lv = lidar_vals.astype(np.float64)
    iv = image_vals.astype(np.float64)

    lv = np.clip((lv - lv.min()) / max(lv.max() - lv.min(), 1e-6) * (bins - 1), 0, bins - 1).astype(int)
    iv = np.clip((iv - iv.min()) / max(iv.max() - iv.min(), 1e-6) * (bins - 1), 0, bins - 1).astype(int)

    # Joint histogram
    joint = np.zeros((bins, bins), dtype=np.float64)
    np.add.at(joint, (lv, iv), 1)
    joint /= joint.sum()

    # Marginals
    p_l = joint.sum(axis=1)
    p_i = joint.sum(axis=0)

    # MI = sum p(l,i) * log(p(l,i) / (p(l)*p(i)))
    nz = joint > 0
    mi = np.sum(joint[nz] * np.log(joint[nz] / (p_l[lv[nz.nonzero()[0]]] * p_i[iv[nz.nonzero()[1]]] + 1e-12) + 1e-12))

    # Simpler: use entropy
    def entropy(p):
        p = p[p > 0]
        return -np.sum(p * np.log(p))

    H_l = entropy(p_l)
    H_i = entropy(p_i)
    H_joint = entropy(joint.ravel())
    return H_l + H_i - H_joint


def evaluate_extrinsic(params, pairs, cam_cfg, base_R, base_t):
    """Evaluate negative MI for a set of extrinsic perturbation parameters.

    params: [drx, dry, drz, dtx, dty, dtz] — small perturbation from initial.
    """
    drx, dry, drz, dtx, dty, dtz = params
    dR = Rotation.from_euler('xyz', [drx, dry, drz], degrees=True).as_matrix()

    R_cl = dR @ base_R
    t_cl = base_t + np.array([dtx, dty, dtz])

    T_cl = np.eye(4)
    T_cl[:3, :3] = R_cl
    T_cl[:3, 3] = t_cl

    K = cam_cfg['K']
    D = cam_cfg['D']
    w, h = cam_cfg['size']

    total_mi = 0.0
    n_valid = 0
    for lidar_xyz, lidar_intensity, img_gray in pairs:
        pixels, valid, depths = project_lidar(lidar_xyz, T_cl, K, D, w, h)
        if valid.sum() < 200:
            continue

        pix_valid = pixels[valid].astype(int)
        # Sample image at projected locations
        img_vals = img_gray[pix_valid[:, 1], pix_valid[:, 0]]
        lid_vals = lidar_intensity[valid]

        mi = mutual_information(lid_vals, img_vals)
        total_mi += mi
        n_valid += 1

    if n_valid == 0:
        return 1e6  # bad
    return -total_mi / n_valid  # minimize negative MI


def extract_pairs(bag_dir, cam_cfg, n_pairs, max_range=60.0):
    """Extract synchronized (LiDAR, camera) frame pairs from bag."""
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_typestore, Stores

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    mcap_files = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')))

    is_thermal = cam_cfg.get('is_thermal', False)
    img_topic = cam_cfg['topic_img']
    lidar_topic = cam_cfg['topic_lidar']

    # Collect timestamps and data
    log(f"Extracting pairs: {img_topic} + {lidar_topic}")
    log(f"  Target: {n_pairs} pairs, max range {max_range}m")

    # Strategy: collect images first, then find nearest LiDAR for each
    images = []   # (timestamp_ns, gray_img)
    lidars = []   # (timestamp_ns, xyz, intensity)

    files_to_scan = mcap_files[:3]  # first 3 files = ~2min, plenty
    for mcap_path in files_to_scan:
        try:
            reader = Reader(mcap_path)
            reader.open()
        except Exception as e:
            log(f"  SKIP {os.path.basename(mcap_path)}: {e}")
            continue
        try:
            for conn, ts, raw in reader.messages():
                if conn.topic == img_topic and len(images) < n_pairs * 3:
                    msg = typestore.deserialize_cdr(raw, conn.msgtype)
                    if is_thermal:
                        # Raw thermal: Mono16 (uint16)
                        data = np.frombuffer(bytes(msg.data), dtype=np.uint16).reshape(msg.height, msg.width)
                        # Normalize to 8-bit for MI
                        p_low, p_high = np.percentile(data, [2, 98])
                        gray = np.clip((data.astype(float) - p_low) / max(p_high - p_low, 1) * 255, 0, 255).astype(np.uint8)
                    else:
                        # Compressed RGB
                        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                        bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
                        if bgr is None:
                            continue
                        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                    images.append((ts, gray))

                elif conn.topic == lidar_topic and len(lidars) < n_pairs * 3:
                    msg = typestore.deserialize_cdr(raw, conn.msgtype)
                    offsets = {f.name: f.offset for f in msg.fields}
                    data = bytes(msg.data)
                    arr = np.frombuffer(data, dtype=np.uint8).reshape(-1, msg.point_step)
                    x = np.frombuffer(arr[:, offsets['x']:offsets['x']+4].tobytes(), dtype=np.float32)
                    y = np.frombuffer(arr[:, offsets['y']:offsets['y']+4].tobytes(), dtype=np.float32)
                    z = np.frombuffer(arr[:, offsets['z']:offsets['z']+4].tobytes(), dtype=np.float32)
                    pts = np.column_stack([x, y, z])

                    intensity = np.zeros(len(x), dtype=np.float32)
                    for fname in ['intensity', 'signal', 'reflectivity']:
                        if fname in offsets:
                            intensity = np.frombuffer(
                                arr[:, offsets[fname]:offsets[fname]+4].tobytes(), dtype=np.float32)
                            break

                    valid = np.all(np.isfinite(pts), axis=1) & (np.linalg.norm(pts, axis=1) > 1.0)
                    rng = np.linalg.norm(pts, axis=1)
                    valid &= rng < max_range
                    pts = pts[valid]
                    intensity = intensity[valid]
                    lidars.append((ts, pts, intensity))

        except Exception as e:
            log(f"  ERROR: {e}")
        finally:
            reader.close()

    log(f"  Collected {len(images)} images, {len(lidars)} LiDAR scans")

    # Match: for each image, find nearest LiDAR within 100ms
    pairs = []
    lidar_times = np.array([t for t, _, _ in lidars])
    for img_ts, gray in images:
        if len(lidar_times) == 0:
            break
        idx = np.argmin(np.abs(lidar_times - img_ts))
        dt_ms = abs(lidar_times[idx] - img_ts) / 1e6
        if dt_ms < 100:
            _, xyz, intens = lidars[idx]
            pairs.append((xyz, intens, gray))
            if len(pairs) >= n_pairs:
                break

    log(f"  Matched {len(pairs)} synchronized pairs")
    return pairs


def main():
    parser = argparse.ArgumentParser(description='DVLC MI Extrinsic Optimizer')
    parser.add_argument('bag_dir', help='Bag directory with MCAP files')
    parser.add_argument('--camera', required=True, choices=list(CAMERAS.keys()),
                        help='Camera to calibrate')
    parser.add_argument('--n-pairs', type=int, default=20, help='Number of frame pairs')
    parser.add_argument('--output', default='/tmp/dvlc', help='Output directory')
    parser.add_argument('--max-range', type=float, default=60.0, help='LiDAR max range for MI')
    parser.add_argument('--n-starts', type=int, default=15, help='Number of random restarts')
    parser.add_argument('--save-overlays', action='store_true', help='Save overlay images')
    args = parser.parse_args()

    cam_cfg = CAMERAS[args.camera]
    os.makedirs(args.output, exist_ok=True)

    # Extract frame pairs
    pairs = extract_pairs(args.bag_dir, cam_cfg, args.n_pairs, args.max_range)
    if len(pairs) < 3:
        log("ERROR: Not enough synchronized pairs")
        sys.exit(1)

    # Initial extrinsic
    init_tlc = cam_cfg['init_tlc']
    T_cl_init = tlc_to_T_camera_lidar(*init_tlc)
    base_R = T_cl_init[:3, :3].copy()
    base_t = T_cl_init[:3, 3].copy()

    # Evaluate initial
    init_params = [0, 0, 0, 0, 0, 0]
    init_cost = evaluate_extrinsic(init_params, pairs, cam_cfg, base_R, base_t)
    log(f"\nInitial MI: {-init_cost:.4f}")

    # Multi-start Nelder-Mead optimization
    log(f"\nRunning {args.n_starts}-start Nelder-Mead optimization...")
    best_cost = init_cost
    best_params = init_params
    best_result = None

    for i in range(args.n_starts):
        if i == 0:
            x0 = np.array(init_params, dtype=np.float64)
        else:
            # Random perturbation: ±5° rotation, ±0.1m translation
            x0 = np.array([
                np.random.uniform(-5, 5),  # drx
                np.random.uniform(-5, 5),  # dry
                np.random.uniform(-5, 5),  # drz
                np.random.uniform(-0.1, 0.1),  # dtx
                np.random.uniform(-0.1, 0.1),  # dty
                np.random.uniform(-0.1, 0.1),  # dtz
            ])

        result = minimize(
            evaluate_extrinsic,
            x0,
            args=(pairs, cam_cfg, base_R, base_t),
            method='Nelder-Mead',
            options={'maxiter': 500, 'xatol': 0.01, 'fatol': 1e-5}
        )

        if result.fun < best_cost:
            best_cost = result.fun
            best_params = result.x
            best_result = result
            log(f"  [{i+1}/{args.n_starts}] MI={-result.fun:.4f} *** NEW BEST ***  "
                f"dR=({result.x[0]:.2f},{result.x[1]:.2f},{result.x[2]:.2f})° "
                f"dt=({result.x[3]:.3f},{result.x[4]:.3f},{result.x[5]:.3f})m")
        else:
            log(f"  [{i+1}/{args.n_starts}] MI={-result.fun:.4f}")

    log(f"\nBest MI: {-best_cost:.4f} (improvement: {(-best_cost) - (-init_cost):.4f})")
    log(f"  dR = ({best_params[0]:.3f}, {best_params[1]:.3f}, {best_params[2]:.3f})°")
    log(f"  dt = ({best_params[3]:.4f}, {best_params[4]:.4f}, {best_params[5]:.4f})m")

    # Compute final T_camera_lidar
    dR = Rotation.from_euler('xyz', best_params[:3], degrees=True).as_matrix()
    final_R_cl = dR @ base_R
    final_t_cl = base_t + best_params[3:6]

    T_cl_final = np.eye(4)
    T_cl_final[:3, :3] = final_R_cl
    T_cl_final[:3, 3] = final_t_cl

    # Convert back to dvlc T_lidar_camera format
    R_lc = final_R_cl.T
    t_lc = -R_lc @ final_t_cl
    q_lc = Rotation.from_matrix(R_lc).as_quat()  # [x,y,z,w]

    log(f"\nFinal T_lidar_camera (dvlc format):")
    log(f"  t = [{t_lc[0]:.6f}, {t_lc[1]:.6f}, {t_lc[2]:.6f}]")
    log(f"  q = [{q_lc[0]:.6f}, {q_lc[1]:.6f}, {q_lc[2]:.6f}, {q_lc[3]:.6f}]")

    log(f"\nFinal T_camera_lidar (for projection):")
    log(f"  R_cl = {final_R_cl.tolist()}")
    log(f"  t_cl = [{final_t_cl[0]:.6f}, {final_t_cl[1]:.6f}, {final_t_cl[2]:.6f}]")

    # Save result
    result_json = {
        'camera': args.camera,
        'direction': cam_cfg['direction'],
        'method': 'MI_optimization',
        'n_pairs': len(pairs),
        'n_starts': args.n_starts,
        'initial_MI': float(-init_cost),
        'final_MI': float(-best_cost),
        'delta_rotation_deg': best_params[:3].tolist(),
        'delta_translation_m': best_params[3:6].tolist(),
        'T_lidar_camera': {
            'translation': t_lc.tolist(),
            'quaternion_xyzw': q_lc.tolist(),
            'flat': [t_lc[0], t_lc[1], t_lc[2], q_lc[0], q_lc[1], q_lc[2], q_lc[3]],
        },
        'T_camera_lidar': {
            'rotation': final_R_cl.tolist(),
            'translation': final_t_cl.tolist(),
        },
        'bag': args.bag_dir,
        'timestamp': time.strftime('%Y-%m-%dT%H:%M:%S'),
    }

    out_path = os.path.join(args.output, f'dvlc_{args.camera}.json')
    with open(out_path, 'w') as f:
        json.dump(result_json, f, indent=2)
    log(f"\nSaved: {out_path}")

    # Save overlay images
    if args.save_overlays:
        log("\nSaving overlay images...")
        K = cam_cfg['K']
        D = cam_cfg['D']
        w, h = cam_cfg['size']
        for pi, (xyz, intens, gray) in enumerate(pairs[:5]):
            # Project with final extrinsic
            pixels, valid, depths = project_lidar(xyz, T_cl_final, K, D, w, h)
            if valid.sum() < 50:
                continue

            overlay = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            pix = pixels[valid].astype(int)
            dep = depths[valid]
            dep_norm = np.clip((dep - dep.min()) / max(dep.max() - dep.min(), 1) * 255, 0, 255).astype(int)
            for j in range(0, len(pix), max(1, len(pix) // 5000)):
                c = cv2.applyColorMap(np.array([[dep_norm[j]]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
                cv2.circle(overlay, (pix[j, 0], pix[j, 1]), 2, (int(c[0]), int(c[1]), int(c[2])), -1)

            overlay_path = os.path.join(args.output, f'overlay_{args.camera}_{pi:02d}.jpg')
            cv2.imwrite(overlay_path, overlay, [cv2.IMWRITE_JPEG_QUALITY, 85])

            # Also save initial overlay for comparison
            pixels_init, valid_init, depths_init = project_lidar(xyz, T_cl_init, K, D, w, h)
            if valid_init.sum() > 50:
                overlay_init = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                pix_i = pixels_init[valid_init].astype(int)
                dep_i = depths_init[valid_init]
                dep_i_norm = np.clip((dep_i - dep_i.min()) / max(dep_i.max() - dep_i.min(), 1) * 255, 0, 255).astype(int)
                for j in range(0, len(pix_i), max(1, len(pix_i) // 5000)):
                    c = cv2.applyColorMap(np.array([[dep_i_norm[j]]], dtype=np.uint8), cv2.COLORMAP_JET)[0, 0]
                    cv2.circle(overlay_init, (pix_i[j, 0], pix_i[j, 1]), 2, (int(c[0]), int(c[1]), int(c[2])), -1)
                init_path = os.path.join(args.output, f'overlay_{args.camera}_{pi:02d}_BEFORE.jpg')
                cv2.imwrite(init_path, overlay_init, [cv2.IMWRITE_JPEG_QUALITY, 85])

        log(f"  Overlays saved to {args.output}/")


if __name__ == '__main__':
    main()
