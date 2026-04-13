#!/usr/bin/env python3
"""ROS2 node: colorize GLIM-aligned point clouds from camera images.

Subscribes to:
  /ouster/points              - raw LiDAR scans (BEST_EFFORT)
  /glim/odom_corrected        - GLIM loop-closure corrected pose
  /camera2/.../compressed     - RIGHT RGB (BEST_EFFORT)
  /camera3/.../compressed     - LEFT RGB (BEST_EFFORT)
  /thermal1/.../compressed    - LEFT thermal colormap
  /thermal2/.../compressed    - RIGHT thermal colormap

For each raw scan:
  1. Get closest GLIM pose
  2. Transform raw points to world frame
  3. Project into each camera using dvlc extrinsics
  4. Sample pixel colors
  5. Accumulate

On shutdown: save accumulated colored points as LAZ.

Usage (inside ROS2 container):
  python3 colorize-glim-node.py --output /data/colorized.laz
"""
import argparse
import signal
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, CompressedImage
from nav_msgs.msg import Odometry


# ── Calibration constants (READ-ONLY from dvlc, do not modify) ──────────

# Thermal intrinsics (raw, with distortion)
K_THERMAL = np.array([
    [334.78419, 0.0,       317.07016],
    [0.0,       332.67743, 234.32726],
    [0.0,       0.0,       1.0],
], dtype=np.float64)
DIST_THERMAL = np.array([-0.164539, 0.026521, -0.004782, -0.004943, 0.0], dtype=np.float64)
THERMAL_W, THERMAL_H = 640, 480

# RGB cam2 (RIGHT) intrinsics — from dvlc_cam2_target
K_CAM2 = np.array([
    [1166.26303, 0.0,        1619.56279],
    [0.0,        1169.02224, 1199.32755],
    [0.0,        0.0,        1.0],
], dtype=np.float64)
DIST_CAM2 = np.zeros(5, dtype=np.float64)
CAM2_W, CAM2_H = 3232, 2426

# RGB cam3 (LEFT) intrinsics — from dvlc_cam3_slam
K_CAM3 = np.array([
    [1192.57, 0.0,     1636.47],
    [0.0,     1195.50, 1183.27],
    [0.0,     0.0,     1.0],
], dtype=np.float64)
DIST_CAM3 = np.zeros(5, dtype=np.float64)
CAM3_W, CAM3_H = 3232, 2426


def quat_to_rot(qx, qy, qz, qw):
    """Quaternion [x,y,z,w] to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def invert_T_lidar_camera(tx, ty, tz, qx, qy, qz, qw):
    """dvlc T_lidar_camera → T_camera_lidar (R, t).

    dvlc: p_lidar = R_lc @ p_cam + t_lc
    Need:  p_cam  = R_cl @ p_lidar + t_cl
    """
    R_lc = quat_to_rot(qx, qy, qz, qw)
    R_cl = R_lc.T
    t_lc = np.array([tx, ty, tz])
    t_cl = -R_cl @ t_lc
    return R_cl, t_cl


# Extrinsics: dvlc T_lidar_camera → inverted to T_camera_lidar
#
# SERIAL FIX (2026-02-14): run_cameras.sh serials corrected so topic labels
# now match physical direction: /thermal1 = LEFT, /thermal2 = RIGHT.
#
# dvlc calibrations were done on OLD (pre-fix) topic data:
#   dvlc_thermal1_target → calibrated OLD /thermal1 (= RIGHT physical) → looks RIGHT
#   dvlc_thermal2_target → calibrated OLD /thermal2 (= LEFT physical) → looks LEFT
# After serial fix, cross-assign: /thermal1 (LEFT) uses dvlc_thermal2, etc.
#
# LEFT extrinsic — MI-optimized from JSON 89901857 base + pitch=+4.1° yaw=+6.0°
# Optimized using LiDAR intensity vs thermal image mutual information (8 pairs, 25-start Nelder-Mead)
LEFT_R = quat_to_rot(-0.665128, 0.043033, 0.031624, 0.744818)
LEFT_t = np.zeros(3)

# RIGHT extrinsic — from calibration JSON 89900593 (T_camera_lidar, direct)
RIGHT_R = quat_to_rot(0.6922, -0.0066, 0.0061, 0.7222)
RIGHT_t = np.zeros(3)

# Cam2 (RIGHT) — dvlc_cam2_slam — looks RIGHT ✓
CAM2_R, CAM2_t = invert_T_lidar_camera(
    0.0042, 0.2, -0.1298,
    0.6891266360695715, -0.0018080031134361987,
    0.027724202260755382, 0.724108126727758)

# Cam3 (LEFT) — from dvlc_cam3_slam, CORRECTED: original dvlc output had
# camera looking RIGHT (-Y). Applied 180deg rotation about lidar X-axis to
# flip to LEFT (+Y). Verified: cam3 now sees points to the LEFT of the vehicle.
CAM3_R, CAM3_t = invert_T_lidar_camera(
    0.19516619667266122, -0.14267676136819465, 0.39962206973777015,
    0.0181430420646545, 0.7138500245363018,
    0.7000556252924953, -0.0033307642509005)

# ── Helpers ──────────────────────────────────────────────────────────────

def extract_xyz(pc_msg):
    """Extract (N,3) float64 xyz from PointCloud2, filtering invalid."""
    dtype_map = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2',
                 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}
    dt = np.dtype({
        'names': [f.name for f in pc_msg.fields],
        'formats': [dtype_map.get(f.datatype, 'f4') for f in pc_msg.fields],
        'offsets': [f.offset for f in pc_msg.fields],
        'itemsize': pc_msg.point_step,
    })
    data = pc_msg.data
    if not isinstance(data, (bytes, bytearray)):
        data = bytes(data)
    n = pc_msg.width * pc_msg.height
    pts = np.frombuffer(data, dtype=dt, count=n)
    xyz = np.stack([pts['x'].astype(np.float64),
                    pts['y'].astype(np.float64),
                    pts['z'].astype(np.float64)], axis=1)
    valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.5)
    return xyz[valid]


def decode_compressed(msg):
    """CompressedImage → BGR numpy array."""
    buf = np.frombuffer(msg.data if isinstance(msg.data, (bytes, bytearray))
                        else bytes(msg.data), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def project_points(xyz_lidar, R, t, K, dist, w, h):
    """Project lidar points into camera image plane.

    Returns: (N,2) pixel coords, (N,) bool valid mask.
    """
    pts_cam = (R @ xyz_lidar.T).T + t
    in_front = pts_cam[:, 2] > 0.1

    rvec, _ = cv2.Rodrigues(R)
    tvec = t.reshape(3, 1)
    pts_2d, _ = cv2.projectPoints(
        xyz_lidar.astype(np.float64), rvec, tvec, K, dist)
    pixels = pts_2d.reshape(-1, 2)

    valid = (in_front &
             (pixels[:, 0] >= 0) & (pixels[:, 0] < w) &
             (pixels[:, 1] >= 0) & (pixels[:, 1] < h))

    return pixels, valid


def sample_image(img_bgr, pixels, valid, w, h):
    """Sample BGR values at projected pixel locations.

    Returns: (N,3) uint8 RGB for valid points (rest zeros).
    """
    rgb = np.zeros((len(pixels), 3), dtype=np.uint8)
    if valid.any() and img_bgr is not None:
        px = np.clip(pixels[valid, 0].astype(int), 0, w - 1)
        py = np.clip(pixels[valid, 1].astype(int), 0, h - 1)
        sampled = img_bgr[py, px]  # BGR
        rgb[valid, 0] = sampled[:, 2]  # R
        rgb[valid, 1] = sampled[:, 1]  # G
        rgb[valid, 2] = sampled[:, 0]  # B
    return rgb


def sample_grayscale(img_gray, pixels, valid, w, h):
    """Sample grayscale values at projected pixel locations.

    Returns: (N,) float32, -1 where not valid.
    """
    vals = np.full(len(pixels), -1.0, dtype=np.float32)
    if valid.any() and img_gray is not None:
        px = np.clip(pixels[valid, 0].astype(int), 0, w - 1)
        py = np.clip(pixels[valid, 1].astype(int), 0, h - 1)
        vals[valid] = img_gray[py, px].astype(np.float32)
    return vals


def rot_to_quat(R):
    """3x3 rotation matrix → quaternion [x,y,z,w]."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 0.5 / np.sqrt(tr + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def slerp_quat(q0, q1, t):
    """Spherical linear interpolation between two quaternions."""
    dot = np.dot(q0, q1)
    if dot < 0:
        q1 = -q1
        dot = -dot
    if dot > 0.9995:
        return q0 + t * (q1 - q0)
    theta = np.arccos(np.clip(dot, -1, 1))
    sin_theta = np.sin(theta)
    return (np.sin((1 - t) * theta) / sin_theta) * q0 + \
           (np.sin(t * theta) / sin_theta) * q1


def interpolate_pose(T0, T1, alpha):
    """Interpolate between two 4x4 poses. alpha in [0,1]."""
    t_interp = (1 - alpha) * T0[:3, 3] + alpha * T1[:3, 3]
    q0 = rot_to_quat(T0[:3, :3])
    q1 = rot_to_quat(T1[:3, :3])
    q_interp = slerp_quat(q0, q1, alpha)
    R_interp = quat_to_rot(q_interp[0], q_interp[1], q_interp[2], q_interp[3])
    T = np.eye(4)
    T[:3, :3] = R_interp
    T[:3, 3] = t_interp
    return T


def odom_to_matrix(msg):
    """nav_msgs/Odometry → 4x4 homogeneous transform."""
    p = msg.pose.pose.position
    o = msg.pose.pose.orientation
    R = quat_to_rot(o.x, o.y, o.z, o.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


# ── ROS2 Node ────────────────────────────────────────────────────────────

class ColorizeGlimNode(Node):
    def __init__(self, output_path, subsample=1, max_scans=0,
                 swap_thermals=False, voxel_size=0.05):
        super().__init__('colorize_glim')
        self.output_path = Path(output_path)
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.subsample = subsample
        self.max_scans = max_scans  # 0 = unlimited
        self.voxel_size = voxel_size  # dedup voxel (0=disable)

        # QoS profiles
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE)
        reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            durability=DurabilityPolicy.VOLATILE)

        # State
        self._lock = threading.Lock()
        self._latest_images = {}  # cam_name → BGR ndarray
        self._odom_buffer = []    # [(stamp_sec, 4x4 matrix)]
        self._scan_count = 0
        self._saving = False
        self._session_locked = False  # True after detecting a session boundary

        # Accumulators
        self._all_xyz = []
        self._all_rgb = []              # vertex RGB (actual camera colors)
        self._all_thermal_left = []     # grayscale scalar
        self._all_thermal_right = []    # grayscale scalar
        self._all_cam_source = []       # 0=none,1=cam2_right,2=cam3_left,3=thermal_left,4=thermal_right
        self._occupied_voxels = set()   # voxel dedup — first-seen wins

        # Camera definitions: name → (R, t, K, dist, W, H, is_thermal, source_id)
        self._cameras = {
            'cam2_right':    (CAM2_R,   CAM2_t,  K_CAM2,    DIST_CAM2,    CAM2_W,    CAM2_H,    False, 1),
            'cam3_left':     (CAM3_R,   CAM3_t,  K_CAM3,    DIST_CAM3,    CAM3_W,    CAM3_H,    False, 2),
            'thermal_left':  (LEFT_R,   LEFT_t,  K_THERMAL, DIST_THERMAL, THERMAL_W, THERMAL_H, True,  3),
            'thermal_right': (RIGHT_R,  RIGHT_t, K_THERMAL, DIST_THERMAL, THERMAL_W, THERMAL_H, True,  4),
        }

        # Image topic → camera name mapping
        # swap_thermals=True for OLD bags (pre-serial-fix: thermal1=RIGHT, thermal2=LEFT)
        if swap_thermals:
            self.get_logger().warn(
                'SWAP THERMALS: thermal1 topic → RIGHT extrinsic, '
                'thermal2 topic → LEFT extrinsic (old bag format)')
            thermal1_cam = 'thermal_right'
            thermal2_cam = 'thermal_left'
        else:
            thermal1_cam = 'thermal_left'
            thermal2_cam = 'thermal_right'

        self._img_topic_to_cam = {
            '/camera2/camera_driver/image_masked/compressed': 'cam2_right',
            '/camera3/camera_driver/image_masked/compressed': 'cam3_left',
            '/thermal1/image_colormap/compressed': thermal1_cam,
            '/thermal2/image_colormap/compressed': thermal2_cam,
        }

        # Subscribe to GLIM odometry (try both corrected and uncorrected)
        self.create_subscription(
            Odometry, '/glim/odom_corrected',
            self._on_odom, reliable)
        self.create_subscription(
            Odometry, '/glim/odom',
            self._on_odom_fallback, reliable)

        # Subscribe to raw point clouds
        self.create_subscription(
            PointCloud2, '/ouster/points',
            self._on_points, best_effort)

        # Subscribe to camera images
        for topic in self._img_topic_to_cam:
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, t=topic: self._on_image(msg, t),
                best_effort)

        self.get_logger().info(
            f'Colorize node started. Output: {self.output_path}')
        self.get_logger().info(
            f'Subsample: {self.subsample}x, voxel_size: '
            f'{self.voxel_size}m, max_scans: '
            f'{"unlimited" if not self.max_scans else self.max_scans}')

    def _on_odom(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        T = odom_to_matrix(msg)
        with self._lock:
            if self._session_locked:
                return  # Ignore poses from subsequent sessions

            # Detect session boundary: position jump > 10m = GLIM restarted
            if self._odom_buffer:
                last_T = self._odom_buffer[-1][1]
                dist = np.linalg.norm(T[:3, 3] - last_T[:3, 3])
                if dist > 10.0:
                    self.get_logger().warn(
                        f'Session boundary: {dist:.1f}m jump. '
                        f'Locking to first session ({len(self._odom_buffer)} poses)')
                    self._session_locked = True
                    return

            self._odom_buffer.append((stamp, T))
            # Keep buffer bounded
            if len(self._odom_buffer) > 10000:
                self._odom_buffer = self._odom_buffer[-5000:]

    def _on_odom_fallback(self, msg):
        """Use uncorrected odom only if no corrected odom received yet."""
        with self._lock:
            if len(self._odom_buffer) == 0:
                stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                self._odom_buffer.append((stamp, odom_to_matrix(msg)))

    def _on_image(self, msg, topic):
        cam_name = self._img_topic_to_cam[topic]
        img = decode_compressed(msg)
        if img is not None:
            with self._lock:
                self._latest_images[cam_name] = img

    def _on_points(self, msg):
        if self._saving:
            return
        if self.max_scans and self._scan_count >= self.max_scans:
            return

        scan_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._lock:
            if len(self._odom_buffer) < 2:
                return

            # Interpolate between bracketing poses
            odom_times = np.array([t for t, _ in self._odom_buffer])
            idx = np.searchsorted(odom_times, scan_stamp)

            if idx == 0:
                # Before first pose — use first pose if close enough
                dt = abs(odom_times[0] - scan_stamp)
                if dt > 0.5:
                    return
                T_world = self._odom_buffer[0][1]
            elif idx >= len(odom_times):
                # After last pose — use last pose if close enough
                dt = abs(odom_times[-1] - scan_stamp)
                if dt > 0.5:
                    return
                T_world = self._odom_buffer[-1][1]
            else:
                # Between two poses — interpolate
                t0 = odom_times[idx - 1]
                t1 = odom_times[idx]
                dt = min(abs(t0 - scan_stamp), abs(t1 - scan_stamp))
                if t1 - t0 > 1.0 or dt > 0.5:
                    return  # Gap too large, skip
                alpha = (scan_stamp - t0) / (t1 - t0)
                alpha = np.clip(alpha, 0.0, 1.0)
                T_world = interpolate_pose(
                    self._odom_buffer[idx - 1][1],
                    self._odom_buffer[idx][1], alpha)

            # Grab latest images
            images = dict(self._latest_images)

        # Extract + subsample points
        xyz = extract_xyz(msg)
        if len(xyz) == 0:
            return
        xyz = xyz[::self.subsample]
        n = len(xyz)

        # Transform to world frame
        xyz_world = (T_world[:3, :3] @ xyz.T).T + T_world[:3, 3]

        # Per-point: vertex RGB, thermal scalars, camera source
        rgb = np.full((n, 3), 40, dtype=np.uint8)  # default dark grey
        thermal_left_val = np.full(n, -1.0, dtype=np.float32)
        thermal_right_val = np.full(n, -1.0, dtype=np.float32)
        cam_source = np.zeros(n, dtype=np.float32)  # 0=none
        has_rgb = np.zeros(n, dtype=bool)

        # Process cameras in order: RGB first (vertex color priority), then thermal
        for cam_name, (R, t, K, dist, w, h, is_thermal, src_id) in self._cameras.items():
            img = images.get(cam_name)
            if img is None:
                continue

            # Resize check
            if img.shape[1] != w or img.shape[0] != h:
                w, h = img.shape[1], img.shape[0]

            pixels, valid = project_points(xyz, R, t, K, dist, w, h)
            if not valid.any():
                continue

            if is_thermal:
                # Thermal: store grayscale scalar + fallback vertex color
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                if 'left' in cam_name:
                    thermal_left_val = sample_grayscale(
                        gray, pixels, valid, w, h)
                else:
                    thermal_right_val = sample_grayscale(
                        gray, pixels, valid, w, h)
                # Fallback vertex color from inferno colormap (where no RGB)
                no_rgb = valid & ~has_rgb
                if no_rgb.any():
                    t_rgb = sample_image(img, pixels, no_rgb, w, h)
                    rgb[no_rgb] = t_rgb[no_rgb]
                    cam_source[no_rgb] = float(src_id)
            else:
                # RGB: highest priority vertex color
                new = valid & ~has_rgb
                if new.any():
                    cam_rgb = sample_image(img, pixels, new, w, h)
                    rgb[new] = cam_rgb[new]
                    cam_source[new] = float(src_id)
                has_rgb |= valid

        has_any = has_rgb | (thermal_left_val >= 0) | (thermal_right_val >= 0)

        # Voxel dedup: keep only points in unoccupied voxels (first-seen wins)
        if self.voxel_size > 0:
            vs = self.voxel_size
            vk = np.floor(xyz_world / vs).astype(np.int64)
            # Pack 3 ints into single int64 key for fast set lookup
            # Shift: x*2^40 + y*2^20 + z (handles coords up to ±500km at 0.05m)
            packed = vk[:, 0] * (1 << 40) + vk[:, 1] * (1 << 20) + vk[:, 2]
            keep = np.ones(n, dtype=bool)
            occ = self._occupied_voxels
            for i in range(n):
                pk = packed[i]
                if pk in occ:
                    keep[i] = False
                else:
                    occ.add(pk)
            xyz_world = xyz_world[keep]
            rgb = rgb[keep]
            thermal_left_val = thermal_left_val[keep]
            thermal_right_val = thermal_right_val[keep]
            cam_source = cam_source[keep]
            has_any = has_any[keep]
            n = keep.sum()

        # Accumulate
        self._all_xyz.append(xyz_world)
        self._all_rgb.append(rgb)
        self._all_thermal_left.append(thermal_left_val)
        self._all_thermal_right.append(thermal_right_val)
        self._all_cam_source.append(cam_source)

        self._scan_count += 1
        coverage = has_any.sum() / n * 100 if n > 0 else 0

        if self._scan_count % 50 == 0 or self._scan_count == 1:
            total_pts = sum(len(x) for x in self._all_xyz)
            dedup_info = ''
            if self.voxel_size > 0:
                dedup_info = (f', dedup: {len(self._occupied_voxels):,} voxels')
            self.get_logger().info(
                f'Scan {self._scan_count}: {n} pts, '
                f'{coverage:.0f}% coverage, '
                f'{total_pts:,} total accumulated, '
                f'odom_dt={dt:.3f}s{dedup_info}')

        # Auto-save and exit when max_scans reached
        if self.max_scans and self._scan_count >= self.max_scans:
            self.get_logger().info(
                f'Reached max_scans={self.max_scans} — saving and exiting')
            self.save_laz()
            raise SystemExit(0)

    def save_laz(self):
        """Save accumulated points as LAZ."""
        self._saving = True
        if not self._all_xyz:
            self.get_logger().warn('No points accumulated — nothing to save')
            return

        import laspy

        self.get_logger().info('Concatenating points...')
        xyz = np.concatenate(self._all_xyz)
        rgb = np.concatenate(self._all_rgb)

        scalars = {
            'thermal_left': np.concatenate(self._all_thermal_left),
            'thermal_right': np.concatenate(self._all_thermal_right),
            'camera_source': np.concatenate(self._all_cam_source),
        }

        self.get_logger().info(
            f'Saving {len(xyz):,} points to {self.output_path}...')

        header = laspy.LasHeader(point_format=7, version="1.4")
        header.offsets = np.min(xyz, axis=0)
        header.scales = [0.01, 0.01, 0.01]  # 1cm precision — 0.001 overflows int32 with BNG coords

        for name in scalars:
            header.add_extra_dim(
                laspy.ExtraBytesParams(name=name, type=np.float32))

        las = laspy.LasData(header)
        las.x = xyz[:, 0]
        las.y = xyz[:, 1]
        las.z = xyz[:, 2]

        # LAS RGB is 16-bit; input rgb is already R,G,B order
        las.red = rgb[:, 0].astype(np.uint16) * 256
        las.green = rgb[:, 1].astype(np.uint16) * 256
        las.blue = rgb[:, 2].astype(np.uint16) * 256

        for name, arr in scalars.items():
            setattr(las, name, arr)

        las.write(str(self.output_path),
                  laz_backend=laspy.LazBackend.LazrsParallel)

        size_mb = self.output_path.stat().st_size / 1e6
        self.get_logger().info(
            f'Saved: {self.output_path} ({size_mb:.1f} MB, '
            f'{len(xyz):,} points)')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True,
                        help='Output LAZ file path')
    parser.add_argument('--subsample', type=int, default=1,
                        help='Take every Nth point (default 1 = keep all)')
    parser.add_argument('--voxel-size', type=float, default=0.05,
                        help='Voxel dedup size in meters (default 0.05, 0=disable)')
    parser.add_argument('--max_scans', type=int, default=0,
                        help='Max scans to process (0=unlimited)')
    parser.add_argument('--swap-thermals', action='store_true',
                        help='Swap thermal topic→extrinsic mapping for OLD '
                             'bags (pre-serial-fix: thermal1=RIGHT, thermal2=LEFT)')

    # ROS2 passes its own args — split them
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = ColorizeGlimNode(args.output, args.subsample, args.max_scans,
                            swap_thermals=args.swap_thermals,
                            voxel_size=args.voxel_size)

    def shutdown_handler(sig, frame):
        node.get_logger().info('Shutdown signal received — saving LAZ...')
        node.save_laz()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_laz()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
