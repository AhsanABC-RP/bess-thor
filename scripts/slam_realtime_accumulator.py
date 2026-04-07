#!/usr/bin/env python3
"""Real-time SLAM map accumulator — FAST-LIO density + GLIM loop-closed correction.

Hybrid approach:
  1. FAST-LIO cloud_registered: 65K pts/frame at 10Hz, deskewed by 200Hz IMU
  2. GLIM odom_corrected: loop-closed pose that eliminates drift/ghosting
  3. FAST-LIO odometry: original drifting pose

For each FAST-LIO cloud at time t:
  - FAST-LIO placed points in world frame using its own (drifting) pose T_fl(t)
  - GLIM's corrected pose at time t is T_glim(t)
  - Correction: T_corr = T_glim(t) @ T_fl(t)^-1
  - Corrected points: p_corrected = T_corr @ p_fl_world

This gives FAST-LIO's full point density with GLIM's loop-closed accuracy.

Height-aware voxel: ground@0.10m, elevated@0.04m
"""

import sys
import os
import time
import atexit
import signal
import numpy as np
from datetime import datetime, timezone
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, NavSatFix
from nav_msgs.msg import Odometry

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
KEYFRAME_INTERVAL = int(os.environ.get("ACC_KEYFRAME_INTERVAL", "3"))
VOXEL_SIZE = float(os.environ.get("ACC_VOXEL_SIZE", "0.10"))
VOXEL_SIZE_ELEVATED = float(os.environ.get("ACC_VOXEL_SIZE_ELEVATED", "0.04"))
GROUND_HEIGHT_MARGIN = float(os.environ.get("ACC_GROUND_MARGIN", "1.5"))
SAVE_EVERY = int(os.environ.get("ACC_SAVE_EVERY", "50"))
MAX_RANGE = float(os.environ.get("ACC_MAX_RANGE", "100.0"))
OUTDIR = os.environ.get("ACC_OUTDIR", "/data/extraction/maps")

# Topics
FL_CLOUD_TOPIC = "/slam/cloud_registered"       # FAST-LIO full-res deskewed cloud
FL_ODOM_TOPIC = "/slam/odometry"                 # FAST-LIO pose (drifts)
GLIM_ODOM_TOPIC = "/glim/odom_corrected"         # GLIM loop-closed pose

# Pose buffer
POSE_BUFFER_SIZE = 200       # ~20s at 10Hz
POSE_MATCH_TOL_NS = 50_000_000  # 50ms tolerance for timestamp matching

# Bomb-proofing
POSE_JUMP_THRESHOLD = 50.0
POSE_STALE_SEC = 2.0
MAX_KEYFRAMES = 2000
MIN_KEYFRAMES_TO_SAVE = 5


def log(msg):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def quat_to_rot(qx, qy, qz, qw):
    return np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw),   2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw),   1-2*(qx*qx+qz*qz),  2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw),   2*(qy*qz+qx*qw),    1-2*(qx*qx+qy*qy)]
    ])


def odom_to_T(msg):
    """Extract 4x4 homogeneous transform from Odometry message."""
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    R = quat_to_rot(q.x, q.y, q.z, q.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def invert_T(T):
    """Invert a 4x4 homogeneous transform (rotation + translation)."""
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv


def apply_T(T, xyz):
    """Apply 4x4 transform to Nx3 point array."""
    R = T[:3, :3].astype(np.float32)
    t = T[:3, 3].astype(np.float32)
    return (xyz @ R.T) + t


def parse_cloud(msg):
    """Parse PointCloud2 — xyz + intensity."""
    n_pts = msg.width * msg.height
    if n_pts == 0:
        return None

    data = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_pts, msg.point_step)

    x = np.frombuffer(data[:, 0:4].tobytes(), dtype=np.float32)
    y = np.frombuffer(data[:, 4:8].tobytes(), dtype=np.float32)
    z = np.frombuffer(data[:, 8:12].tobytes(), dtype=np.float32)
    xyz = np.column_stack([x, y, z])

    intensity = None
    for field in msg.fields:
        if field.name == "intensity":
            intensity = np.frombuffer(
                data[:, field.offset:field.offset+4].tobytes(), dtype=np.float32)
            break

    valid = np.all(np.isfinite(xyz), axis=1)
    if np.sum(valid) > 100:
        centroid = xyz[valid].mean(axis=0)
        dist = np.linalg.norm(xyz - centroid, axis=1)
        valid &= dist < MAX_RANGE

    result = {"xyz": xyz[valid]}
    if intensity is not None:
        result["intensity"] = intensity[valid]

    return result


def render_topdown(all_xyz, img_path):
    """Render top-down bird's eye overview as BMP. ~20cm/px, height-colored."""
    import struct as st

    if len(all_xyz) < 100:
        return

    x, y, z = all_xyz[:, 0], all_xyz[:, 1], all_xyz[:, 2]
    x_min, x_max = float(x.min()), float(x.max())
    y_min, y_max = float(y.min()), float(y.max())
    z_min, z_max = float(z.min()), float(z.max())
    x_span = max(x_max - x_min, 1.0)
    y_span = max(y_max - y_min, 1.0)

    res = 0.2
    w = min(int(x_span / res) + 1, 4096)
    h = min(int(y_span / res) + 1, 4096)
    if w < 10 or h < 10:
        return

    z_sum = np.zeros((h, w), dtype=np.float64)
    count = np.zeros((h, w), dtype=np.int32)

    px = np.clip(((x - x_min) / x_span * (w - 1)).astype(np.int32), 0, w - 1)
    py = np.clip((h - 1) - ((y - y_min) / y_span * (h - 1)).astype(np.int32), 0, h - 1)

    np.add.at(z_sum, (py, px), z.astype(np.float64))
    np.add.at(count, (py, px), 1)

    occupied = count > 0
    z_avg = np.zeros_like(z_sum)
    z_avg[occupied] = z_sum[occupied] / count[occupied]

    z_norm = np.zeros_like(z_avg)
    if z_max > z_min:
        z_norm = (z_avg - z_min) / (z_max - z_min)

    img = np.full((h, w, 3), 30, dtype=np.uint8)
    t = z_norm[occupied]
    r = np.clip((t * 4 - 2) * 255, 0, 255).astype(np.uint8)
    g = np.clip(np.minimum(t * 4, (4 - t * 4)) * 255, 0, 255).astype(np.uint8)
    b = np.clip((1 - t * 4) * 255, 0, 255).astype(np.uint8)

    dens = np.log1p(count[occupied].astype(np.float64))
    dens = dens / max(dens.max(), 1.0)
    brightness = 0.3 + 0.7 * dens

    img[occupied, 0] = np.clip(r * brightness, 0, 255).astype(np.uint8)
    img[occupied, 1] = np.clip(g * brightness, 0, 255).astype(np.uint8)
    img[occupied, 2] = np.clip(b * brightness, 0, 255).astype(np.uint8)

    row_bytes = w * 3
    pad = (4 - row_bytes % 4) % 4
    padded_row = row_bytes + pad
    img_size = padded_row * h

    with open(img_path, 'wb') as f:
        f.write(b'BM')
        f.write(st.pack('<I', 54 + img_size))
        f.write(st.pack('<HH', 0, 0))
        f.write(st.pack('<I', 54))
        f.write(st.pack('<I', 40))
        f.write(st.pack('<i', w))
        f.write(st.pack('<i', h))
        f.write(st.pack('<HH', 1, 24))
        f.write(st.pack('<I', 0))
        f.write(st.pack('<I', img_size))
        f.write(st.pack('<ii', 2835, 2835))
        f.write(st.pack('<II', 0, 0))
        pad_bytes = b'\x00' * pad
        for row_idx in range(h - 1, -1, -1):
            f.write(img[row_idx, :, ::-1].tobytes())
            if pad:
                f.write(pad_bytes)

    size_kb = os.path.getsize(img_path) // 1024
    log(f"  Overview: {os.path.basename(img_path)} ({w}x{h}, {size_kb}KB) "
        f"extent: {x_span:.0f}m x {y_span:.0f}m")


def save_ply(points_list, path):
    """Save PLY with all available fields. Returns point count."""
    if not points_list:
        return 0

    all_xyz = np.vstack([p["xyz"] for p in points_list])
    n = len(all_xyz)
    if n == 0:
        return 0

    available = []
    for name in ["intensity"]:
        if any(name in p for p in points_list):
            available.append(name)

    field_arrays = {}
    for name in available:
        arrays = []
        for p in points_list:
            if name in p:
                arrays.append(p[name])
            else:
                arrays.append(np.zeros(len(p["xyz"]), dtype=np.float32))
        field_arrays[name] = np.concatenate(arrays)

    # Height-aware voxel downsample
    if VOXEL_SIZE > 0:
        z_vals = all_xyz[:, 2]
        ground_z = np.percentile(z_vals, 5)
        elevated_mask = z_vals > (ground_z + GROUND_HEIGHT_MARGIN)
        ground_mask = ~elevated_mask

        keep_indices = []

        if np.any(ground_mask):
            g_xyz = all_xyz[ground_mask]
            g_keys = (g_xyz / VOXEL_SIZE).astype(np.int64)
            _, g_idx = np.unique(
                g_keys[:, 0] * 2000000 + g_keys[:, 1] * 2000 + g_keys[:, 2],
                return_index=True)
            keep_indices.append(np.where(ground_mask)[0][g_idx])

        if np.any(elevated_mask):
            e_xyz = all_xyz[elevated_mask]
            e_keys = (e_xyz / VOXEL_SIZE_ELEVATED).astype(np.int64)
            _, e_idx = np.unique(
                e_keys[:, 0] * 2000000 + e_keys[:, 1] * 2000 + e_keys[:, 2],
                return_index=True)
            keep_indices.append(np.where(elevated_mask)[0][e_idx])

        if keep_indices:
            idx = np.sort(np.concatenate(keep_indices))
            all_xyz = all_xyz[idx]
            for name in available:
                field_arrays[name] = field_arrays[name][idx]
        n = len(all_xyz)
        n_elev = np.sum(all_xyz[:, 2] > (ground_z + GROUND_HEIGHT_MARGIN))
        log(f"  Voxel: ground@{VOXEL_SIZE}m={n - n_elev:,} elevated@{VOXEL_SIZE_ELEVATED}m={n_elev:,} (ground_z={ground_z:.1f}m)")

    # Build PLY
    header = "ply\nformat binary_little_endian 1.0\n"
    header += f"element vertex {n}\n"
    header += "property float x\nproperty float y\nproperty float z\n"
    for name in available:
        header += f"property float scalar_{name}\n"
    header += "end_header\n"

    ncols = 3 + len(available)
    row = np.zeros((n, ncols), dtype=np.float32)
    row[:, :3] = all_xyz.astype(np.float32)
    for i, name in enumerate(available):
        row[:, 3 + i] = field_arrays[name].astype(np.float32)

    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(row.tobytes())

    xr = all_xyz[:, 0].max() - all_xyz[:, 0].min()
    yr = all_xyz[:, 1].max() - all_xyz[:, 1].min()
    zr = all_xyz[:, 2].max() - all_xyz[:, 2].min()
    fields_str = "+".join(available) if available else "xyz-only"
    log(f"SAVED {n:,} pts -> {os.path.basename(path)} | {xr:.0f}m x {yr:.0f}m x {zr:.0f}m | {fields_str}")
    return n


class RealtimeAccumulator(Node):
    def __init__(self):
        super().__init__("slam_accumulator")

        # FAST-LIO pose buffer: stamp_ns -> 4x4 transform
        self.fl_poses = OrderedDict()
        # GLIM corrected pose buffer: stamp_ns -> 4x4 transform
        self.glim_poses = OrderedDict()

        # Pose state for frame reset detection (uses FAST-LIO odom)
        self.prev_fl_pos = None
        self.fl_pose_time = 0.0
        self.fl_pose_count = 0
        self.pose_ok = False
        self.stabilize_until = time.monotonic() + 30.0

        # GLIM readiness
        self.glim_ready = False
        self.glim_pose_time = 0.0

        # Held correction: T_corr persists between fresh matches
        self.held_T_corr = None

        # Correction stats
        self.corr_fresh = 0    # fresh timestamp match
        self.corr_held = 0     # reused held T_corr
        self.corr_skipped = 0  # no correction available, frame skipped

        # Session state
        self.session_id = 0
        self.keyframes = []
        self.chunk_pts = 0
        self.total_pts = 0
        self.scan_count = 0
        self.kf_count = 0
        self.chunk_index = 0
        self.save_count = 0
        self.session_start = time.time()
        self.session_dir = None

        # GNSS
        self.lat = 0.0
        self.lon = 0.0
        self.fix_status = -1

        self.shutting_down = False

        os.makedirs(OUTDIR, exist_ok=True)
        self._init_session_dir()

        qos_be = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        qos_reliable = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # FAST-LIO cloud (BEST_EFFORT)
        self.cloud_sub = self.create_subscription(
            PointCloud2, FL_CLOUD_TOPIC, self.cloud_cb, qos_be)
        # FAST-LIO odom (RELIABLE) — for frame reset detection + pose buffer
        self.fl_odom_sub = self.create_subscription(
            Odometry, FL_ODOM_TOPIC, self.fl_odom_cb, qos_reliable)
        # GLIM corrected odom (RELIABLE) — for loop-closed correction
        self.glim_odom_sub = self.create_subscription(
            Odometry, GLIM_ODOM_TOPIC, self.glim_odom_cb, qos_reliable)
        # GNSS
        self.nav_sub = self.create_subscription(
            NavSatFix, "/gnss_1/llh_position", self.nav_cb, qos_be)

        log(f"Real-time accumulator started (FAST-LIO + GLIM correction, hold T_corr)")
        log(f"  Cloud: {FL_CLOUD_TOPIC} (65K pts, 10Hz, FAST-LIO deskewed)")
        log(f"  FAST-LIO odom: {FL_ODOM_TOPIC} (frame reset + pose buffer)")
        log(f"  GLIM odom: {GLIM_ODOM_TOPIC} (loop-closed correction)")
        log(f"  Skips frames until first GLIM correction; holds T_corr across gaps")
        log(f"  KF interval={KEYFRAME_INTERVAL}, save every {SAVE_EVERY} kf")
        log(f"  Voxel: ground={VOXEL_SIZE}m, elevated={VOXEL_SIZE_ELEVATED}m")
        log(f"  Output: {self.session_dir}/")

    def _init_session_dir(self):
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(OUTDIR, f"session_{ts}_s{self.session_id}")
        os.makedirs(self.session_dir, exist_ok=True)
        latest = os.path.join(OUTDIR, "latest_session")
        try:
            os.remove(latest)
        except OSError:
            pass
        try:
            os.symlink(os.path.basename(self.session_dir), latest)
        except OSError:
            pass

    def _find_nearest_pose(self, buf, stamp_ns, tol_ns):
        """Find nearest pose in buffer by timestamp. Returns 4x4 or None."""
        if not buf:
            return None
        best_key = min(buf.keys(), key=lambda k: abs(k - stamp_ns))
        if abs(best_key - stamp_ns) <= tol_ns:
            return buf[best_key]
        return None

    def fl_odom_cb(self, msg):
        """Buffer FAST-LIO pose + frame reset detection."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if any(not np.isfinite(v) for v in vals):
            return

        qnorm = np.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
        if qnorm < 0.9 or qnorm > 1.1:
            return

        # Buffer pose
        stamp_ns = stamp_to_ns(msg.header.stamp)
        self.fl_poses[stamp_ns] = odom_to_T(msg)
        while len(self.fl_poses) > POSE_BUFFER_SIZE:
            self.fl_poses.popitem(last=False)

        new_pos = np.array([p.x, p.y, p.z])
        now = time.monotonic()

        if now < self.stabilize_until:
            self.prev_fl_pos = new_pos
            self.fl_pose_time = now
            return

        if self.prev_fl_pos is not None:
            jump = np.linalg.norm(new_pos - self.prev_fl_pos)
            if jump > POSE_JUMP_THRESHOLD:
                log(f"FRAME RESET detected: pose jumped {jump:.1f}m — saving and stabilizing 5s")
                self.save_session("frame_reset")
                self.new_session()
                self.held_T_corr = None  # FAST-LIO world frame changed
                self.stabilize_until = now + 5.0
                self.prev_fl_pos = new_pos
                self.fl_pose_time = now
                return

        self.prev_fl_pos = new_pos
        self.fl_pose_time = now
        self.fl_pose_count += 1
        self.pose_ok = True

    def glim_odom_cb(self, msg):
        """Buffer GLIM loop-closed pose."""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        vals = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if any(not np.isfinite(v) for v in vals):
            return

        stamp_ns = stamp_to_ns(msg.header.stamp)
        self.glim_poses[stamp_ns] = odom_to_T(msg)
        while len(self.glim_poses) > POSE_BUFFER_SIZE:
            self.glim_poses.popitem(last=False)

        self.glim_pose_time = time.monotonic()
        if not self.glim_ready:
            self.glim_ready = True
            log("GLIM corrected odom online")

    def nav_cb(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.fix_status = msg.status.status

    def cloud_cb(self, msg):
        self.scan_count += 1

        if not self.pose_ok:
            if self.scan_count == 1:
                log("Waiting for first FAST-LIO odometry...")
            return

        pose_age = time.monotonic() - self.fl_pose_time
        if pose_age > POSE_STALE_SEC:
            if self.scan_count % 50 == 0:
                log(f"FL odom stale ({pose_age:.1f}s) — pausing")
            return

        if self.scan_count % KEYFRAME_INTERVAL != 0:
            return

        try:
            parsed = parse_cloud(msg)
        except Exception as e:
            log(f"Parse error: {e}")
            return

        if parsed is None or len(parsed.get("xyz", [])) == 0:
            return

        # --- GLIM correction ---
        # FAST-LIO cloud is in FAST-LIO's world frame. Apply GLIM correction
        # to move it into GLIM's loop-closed frame.
        stamp_ns = stamp_to_ns(msg.header.stamp)

        # GLIM publishes at ~9Hz, FAST-LIO at 10Hz. 500ms tolerance covers gaps.
        T_fl = self._find_nearest_pose(self.fl_poses, stamp_ns, POSE_MATCH_TOL_NS)
        T_glim = self._find_nearest_pose(self.glim_poses, stamp_ns, 500_000_000)  # 500ms

        if T_fl is not None and T_glim is not None:
            # Fresh correction — update held T_corr
            self.held_T_corr = T_glim @ invert_T(T_fl)
            parsed["xyz"] = apply_T(self.held_T_corr, parsed["xyz"])
            self.corr_fresh += 1
        elif self.held_T_corr is not None:
            # No fresh match — reuse held correction (slowly varying)
            parsed["xyz"] = apply_T(self.held_T_corr, parsed["xyz"])
            self.corr_held += 1
        else:
            # No correction available at all — skip to prevent drift contamination
            self.corr_skipped += 1
            if self.corr_skipped <= 5 or self.corr_skipped % 50 == 0:
                log(f"No GLIM correction yet — skipping frame ({self.corr_skipped} skipped)")
            return

        self.kf_count += 1
        self.keyframes.append(parsed)
        n = len(parsed["xyz"])
        self.chunk_pts += n
        self.total_pts += n

        if self.kf_count % 10 == 0:
            fix_map = {-1: "nofix", 0: "gps", 1: "sbas", 2: "rtk"}
            fn = fix_map.get(self.fix_status, str(self.fix_status))
            elapsed = time.time() - self.session_start
            total_corr = self.corr_fresh + self.corr_held
            corr_pct = (self.corr_fresh / total_corr * 100) if total_corr > 0 else 0
            glim_age = time.monotonic() - self.glim_pose_time if self.glim_ready else -1
            log(f"[{elapsed:.0f}s] s{self.session_id}/c{self.chunk_index} {self.kf_count} kf, "
                f"chunk={self.chunk_pts:,} total={self.total_pts:,} | "
                f"GLIM={corr_pct:.0f}%fresh+{self.corr_held}held({glim_age:.1f}s) skip={self.corr_skipped} | "
                f"{self.lat:.5f},{self.lon:.5f} fix={fn}")

        if self.kf_count % SAVE_EVERY == 0:
            self.save_session("periodic")

        if self.kf_count >= MAX_KEYFRAMES:
            log(f"MAX_KEYFRAMES ({MAX_KEYFRAMES}) — force-save")
            self.save_session("memory_cap")
            self.new_session()

    def new_session(self):
        self.session_id += 1
        self.keyframes = []
        self.chunk_pts = 0
        self.total_pts = 0
        self.kf_count = 0
        self.chunk_index = 0
        self.session_start = time.time()
        self.prev_fl_pos = None
        self.pose_ok = False
        self._init_session_dir()
        log(f"New session {self.session_id} — {self.session_dir}")

    def save_session(self, reason="unknown"):
        if len(self.keyframes) < MIN_KEYFRAMES_TO_SAVE:
            return

        self.save_count += 1
        ts = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        fix_map = {-1: "local", 0: "gps", 1: "sbas", 2: "rtk"}
        fn = fix_map.get(self.fix_status, "local")

        lat_str = f"{self.lat:.4f}" if abs(self.lat) > 0.01 else "local"
        lon_str = f"{self.lon:.4f}" if abs(self.lon) > 0.01 else "local"

        outdir = self.session_dir or OUTDIR
        tmp_path = f"{outdir}/_tmp_save.ply"
        try:
            n = save_ply(self.keyframes, tmp_path)
            if n == 0:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
                return

            fname = (f"chunk_{ts}_c{self.chunk_index:03d}_"
                     f"{lat_str}_{lon_str}_{fn}_{n}pts_{reason}.ply")
            final_path = f"{outdir}/{fname}"
            os.rename(tmp_path, final_path)

            latest = f"{outdir}/latest.ply"
            try:
                os.remove(latest)
            except OSError:
                pass
            try:
                os.symlink(fname, latest)
            except OSError:
                pass

            # Generate top-down overview
            try:
                all_xyz = np.vstack([p["xyz"] for p in self.keyframes])
                img_name = fname.replace(".ply", ".bmp")
                render_topdown(all_xyz, f"{outdir}/{img_name}")
                latest_img = f"{outdir}/latest_overview.bmp"
                try:
                    os.remove(latest_img)
                except OSError:
                    pass
                try:
                    os.symlink(img_name, latest_img)
                except OSError:
                    pass
            except Exception as e:
                log(f"  Overview failed: {e}")

            self.keyframes = []
            self.chunk_pts = 0
            self.chunk_index += 1

        except Exception as e:
            log(f"ERROR saving chunk: {e}")
            try:
                os.remove(tmp_path)
            except OSError:
                pass

    def shutdown_save(self):
        if self.shutting_down:
            return
        self.shutting_down = True
        log("Shutdown — saving final session")
        self.save_session("shutdown")


def main():
    rclpy.init()
    node = RealtimeAccumulator()

    def sig_handler(sig, frame):
        node.shutdown_save()
        sys.exit(0)

    signal.signal(signal.SIGTERM, sig_handler)
    signal.signal(signal.SIGINT, sig_handler)
    atexit.register(node.shutdown_save)

    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except BaseException:
        node.shutdown_save()


if __name__ == "__main__":
    main()
