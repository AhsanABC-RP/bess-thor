#!/usr/bin/env python3
"""
BESS Real-Time Frame Extraction Pipeline

Subscribes to SLAM odometry, GNSS, and multi-sensor topics to produce:
- RGB images (JPEG from PII-masked compressed topics)
- Thermal colormaps (INFERNO JPEG, q=95) + raw radiometric NPY (float32 Celsius)
- LiDAR panoramas (range/reflec/nearir/signal with colormaps, PNG)
- SLAM registered point clouds (NPZ, 1Hz throttle)
- BNG + WGS84 trajectories
- frame_visibility.json (UPRN matching with hero shot scoring + asset filenames)
- extraction_summary.json

Matches MASTER Stage 6/9b output conventions for master.xridap.io API.
Spatial data: /data/spatial/bess_uprn.db (SQLite with 50m grid index).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, CompressedImage, Image, PointCloud2, Temperature, RelativeHumidity
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import json
import os
import sys
import math
import time
import shutil
import sqlite3
import threading
from pathlib import Path
from datetime import datetime, timezone
from dataclasses import dataclass, field
from collections import defaultdict, deque
from concurrent.futures import ThreadPoolExecutor
from typing import Dict, List, Optional, Tuple
from urllib.request import Request, urlopen
from urllib.parse import quote

try:
    from pyproj import Transformer
except ImportError:
    print("ERROR: pyproj not installed. pip install pyproj", file=sys.stderr)
    sys.exit(1)

try:
    from cv_bridge import CvBridge
    import cv2
    HAS_CV = True
except ImportError:
    HAS_CV = False

try:
    import laspy
    HAS_LASPY = True
except ImportError:
    HAS_LASPY = False

    print("WARNING: cv_bridge/cv2 not available. Thermal + LiDAR image processing disabled.",
          file=sys.stderr)


# =============================================================================
# ATOMIC WRITE HELPERS
# =============================================================================

def _tmp_path(path: Path) -> Path:
    """Temp path that preserves original extension (so cv2/numpy see correct format).
    foo.png -> foo.tmp.png, foo.npy -> foo.tmp.npy"""
    return path.parent / (path.stem + '.tmp' + path.suffix)


def atomic_write_bytes(path: Path, data: bytes):
    """Write bytes atomically: write to .tmp, then rename."""
    tmp = _tmp_path(path)
    tmp.write_bytes(data)
    tmp.rename(path)


def atomic_write_file(path: Path, write_fn):
    """Write file atomically using a callback: write_fn(tmp_path).
    write_fn should write the complete file to the given tmp path."""
    tmp = _tmp_path(path)
    write_fn(tmp)
    tmp.rename(path)


def atomic_write_text(path: Path, text: str):
    """Write text atomically: write to .tmp, then rename."""
    tmp = _tmp_path(path)
    tmp.write_text(text)
    tmp.rename(path)


# =============================================================================
# CONSTANTS
# =============================================================================
MAX_VISIBILITY_DISTANCE_M = 75.0
PERPENDICULAR_THRESHOLD_DEG = 15.0
OPTIMAL_DISTANCE_CENTER_M = 32.5
LEFT_FOV_MIN_DEG = 30
LEFT_FOV_MAX_DEG = 150
RIGHT_FOV_MIN_DEG = 210
RIGHT_FOV_MAX_DEG = 330
HEADING_MOVEMENT_THRESHOLD_M = 0.5
HEADING_MOVEMENT_THRESHOLD_NON_RTK_M = 10.0  # need longer baseline without RTK
TRAJECTORY_MOVEMENT_THRESHOLD_M = 2.0  # min movement to add trajectory point
MAX_GPS_SPEED_MS = 30.0   # ~108 km/h — reject GPS jumps (max 60mph + margin)
RTK_FIX_STATUS = 2        # NavSatFix STATUS_GBAS_FIX (RTK fixed/float)
RTK_COVARIANCE_THRESHOLD = 0.1  # m² — max horizontal variance to trust as RTK
HEADING_EMA_ALPHA_RTK = 0.3     # fast convergence from high-quality RTK fixes
HEADING_EMA_ALPHA_NON_RTK = 0.0 # don't update offset from non-RTK (was causing drift)

# Multi-point heading calibration: circular buffer of RTK offset samples
HEADING_BUFFER_SIZE = 20          # keep last N offset samples
HEADING_CONVERGENCE_STD_DEG = 2.0 # converged when circular std < this
HEADING_OUTLIER_SIGMA = 3.0       # reject samples > N*sigma from mean
HEADING_MAX_CONSECUTIVE_REJECTS = 20  # reset buffer after this many consecutive rejections (was 10, too sensitive to GLIM loop closures)
GRID_CELL_SIZE = 50  # meters
FLUSH_INTERVAL_S = 30.0

# Throttle rates (seconds between saves)
THERMAL_THROTTLE_S = 0.2     # 5 Hz (match RGB cameras)
LIDAR_IMG_THROTTLE_S = 1.0   # 1 Hz
SLAM_CLOUD_THROTTLE_S = 1.0  # fallback when no speed available

# Adaptive SLAM cloud capture: distance-based keyframes
# At low speed (residential) → denser clouds for UPRN coverage
# At high speed (highway) → sparser but continuous
SLAM_KEYFRAME_DIST_M = {
    5:  2.0,   # <5 km/h: every 2m (crawling/parking)
    30: 5.0,   # 5-30 km/h: every 5m (residential)
    60: 10.0,  # 30-60 km/h: every 10m (urban arterial)
    999: 20.0, # >60 km/h: every 20m (highway)
}

# FLIR A70 Planck radiometric constants (per-camera, from GenICam Measurement nodes)
# Full FLIR radiometric formula (gige_example_A400_A700.py):
#   Radiance = (raw - J0) / J1
#   Tau = X*exp(-sqrt(d)*(alpha1+beta1*sqrt(H2O))) + (1-X)*exp(-sqrt(d)*(alpha2+beta2*sqrt(H2O)))
#   H2O = humidity * exp(1.5587 + 0.06939*T_atm - 0.00027816*T_atm² + 0.00000068455*T_atm³)
#   raw_refl = R/(exp(B/T_refl)-F);  raw_atm = R/(exp(B/T_atm_k)-F)
#   raw_obj = Radiance/(emiss*Tau) - ((1-emiss)/emiss)*raw_refl - ((1-Tau)/(emiss*Tau))*raw_atm
#   T = B / ln(R/raw_obj + F) - 273.15
# Constants are factory calibration values unique to each camera serial.
THERMAL_PLANCK = {
    '89900594': {  # Thermal1 (White)
        'R': 21324.9, 'B': 1504.2, 'F': 1.05, 'J0': 19831, 'J1': 33.3013,
        'alpha1': 1.239e-08, 'alpha2': 1.1095e-08, 'X': 0.732,
        'beta1': 0.00318, 'beta2': 0.0031802,
    },
    '89902404': {  # Thermal2 (White)
        'R': 23529.5, 'B': 1533.8, 'F': 1.05, 'J0': 19525, 'J1': 35.6463,
        'alpha1': 1.239e-08, 'alpha2': 1.1095e-08, 'X': 0.732,
        'beta1': 0.00318, 'beta2': 0.0031802,
    },
}
THERMAL_EMISSIVITY = 0.95  # Default for building surfaces
THERMAL_OBJECT_DISTANCE = 10.0  # Default distance to buildings (meters)

# PointField datatype constants → numpy dtype
POINTFIELD_DTYPE = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}

# Static asset output directories (LiDAR + SLAM cloud). Camera dirs are
# created dynamically from cameras.yaml keys so the pipeline supports any
# topology (grey/white = 3 RGB + 2 thermal, Thor = 4 RGB + 4 thermal).
STATIC_ASSET_DIRS = [
    'lidar_range', 'lidar_reflec', 'lidar_nearir', 'lidar_signal',
    'slam_cloud', 'slam_cloud_colorized',
]


# =============================================================================
# DATA CLASSES
# =============================================================================

@dataclass
class TrajectoryPoint:
    timestamp_ns: int
    x: float  # BNG easting
    y: float  # BNG northing
    z: float
    lat: float
    lon: float
    alt: float
    heading_deg: float


@dataclass
class VisibilityRecord:
    frame_timestamp_ns: int
    uprn: str
    camera_side: str
    distance_m: float
    bearing_deg: float
    angle_from_perpendicular_deg: float
    is_perpendicular: bool


@dataclass
class HeroTracker:
    """Track best hero shot per UPRN."""
    uprn: str
    best_score: float = float('inf')
    best_timestamp_ns: int = 0
    best_camera_side: str = ''
    best_distance_m: float = 0.0
    best_perp_angle: float = 0.0
    visible_count: int = 0
    perpendicular_count: int = 0
    address: str = ''
    building_height_m: float = 0.0
    building_use: str = ''
    hero_assets: Dict[str, str] = field(default_factory=dict)


# =============================================================================
# SPATIAL DATABASE
# =============================================================================

class SpatialDB:
    """SQLite-backed UPRN spatial lookup with grid index."""

    def __init__(self, db_path: str):
        self.db_path = db_path
        self.conn = None
        self.available = False

        if os.path.exists(db_path):
            try:
                # Use immutable=1 for read-only mounted volumes (avoids journal file creation)
                uri = f"file://{db_path}?mode=ro&immutable=1"
                self.conn = sqlite3.connect(uri, uri=True, check_same_thread=False)
                self.conn.row_factory = sqlite3.Row
                count = self.conn.execute("SELECT COUNT(*) FROM uprn_spatial").fetchone()[0]
                self.available = True
                print(f"Spatial DB loaded: {count:,} UPRNs")
            except Exception as e:
                print(f"WARNING: Failed to open spatial DB: {e}")
        else:
            print(f"WARNING: Spatial DB not found: {db_path}")

    def query_nearby(self, x: float, y: float) -> List[dict]:
        """Find all UPRNs in 3x3 grid neighborhood (~75m radius)."""
        if not self.available:
            return []

        gx = int(x / GRID_CELL_SIZE)
        gy = int(y / GRID_CELL_SIZE)

        cursor = self.conn.execute(
            "SELECT uprn, x, y, fulladdress, postcode, classification, "
            "osid, building_height_m, building_floors, building_use "
            "FROM uprn_spatial WHERE grid_x BETWEEN ? AND ? AND grid_y BETWEEN ? AND ?",
            (gx - 1, gx + 1, gy - 1, gy + 1)
        )
        return [dict(row) for row in cursor]


# =============================================================================
# GEOMETRY HELPERS
# =============================================================================

def calc_distance_bng(x1, y1, x2, y2):
    """Euclidean distance in BNG meters."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


def calc_bearing_bng(from_x, from_y, to_x, to_y):
    """Bearing from point to point in BNG. Returns degrees 0=North clockwise."""
    dx = to_x - from_x
    dy = to_y - from_y
    return math.degrees(math.atan2(dx, dy)) % 360


def is_in_fov(relative_angle, camera_side):
    """Check if relative angle is within camera FOV.

    relative_angle = (heading - bearing + 360) % 360
    Convention: 90° = target is to the LEFT, 270° = target is to the RIGHT.

    LEFT camera FOV:  30-150° (centered on 90°, ±60°)
    RIGHT camera FOV: 210-330° (centered on 270°, ±60°)
    """
    if camera_side == 'left':
        return LEFT_FOV_MIN_DEG <= relative_angle <= LEFT_FOV_MAX_DEG
    elif camera_side == 'right':
        return RIGHT_FOV_MIN_DEG <= relative_angle <= RIGHT_FOV_MAX_DEG
    elif camera_side == 'rear':
        return 120 <= relative_angle <= 240
    return False


def calc_perpendicular_angle(relative_angle, camera_side):
    """Angle from perpendicular. 0 = perfectly perpendicular.

    relative_angle uses (heading - bearing) convention:
      90° = directly LEFT, 270° = directly RIGHT.
    """
    if camera_side == 'left':
        diff = abs(relative_angle - 90)
    elif camera_side == 'right':
        diff = abs(relative_angle - 270)
    else:
        return 180.0  # rear camera — not used for hero shots
    return min(diff, 360 - diff)


def quaternion_to_yaw(q):
    """Extract yaw (heading) from ROS quaternion. Returns radians."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def circular_mean_std_deg(angles_deg):
    """Compute circular mean and std of angles in degrees.

    Returns (mean_deg, std_deg). Uses vector averaging to handle wraparound.
    """
    if not angles_deg:
        return 0.0, 999.0
    rads = [math.radians(a) for a in angles_deg]
    s = sum(math.sin(r) for r in rads)
    c = sum(math.cos(r) for r in rads)
    n = len(rads)
    mean_rad = math.atan2(s / n, c / n)
    # Circular variance: 1 - R (mean resultant length)
    R = math.sqrt((s / n) ** 2 + (c / n) ** 2)
    # Circular std approximation (Mardia & Jupp)
    if R >= 1.0:
        std_rad = 0.0
    else:
        std_rad = math.sqrt(-2.0 * math.log(R))
    return math.degrees(mean_rad) % 360, math.degrees(std_rad)


def timestamp_to_filename(timestamp_ns):
    """Convert nanosecond timestamp to YYYYMMDD_HHMMSS_mmm format."""
    ts_sec = timestamp_ns / 1e9
    dt = datetime.fromtimestamp(ts_sec)
    return dt.strftime('%Y%m%d_%H%M%S_%f')[:-3]


# =============================================================================
# CALIBRATION LOADER — reads per-head calibration from config files
# =============================================================================

def _load_camera_calibration(cam_name, calib_dir='/config/cameras/calibration'):
    """Load camera calibration from JSON file. Returns (K, D, w, h, model) or None."""
    calib_path = os.path.join(calib_dir, cam_name, 'calibration.json')
    if not os.path.exists(calib_path):
        return None
    try:
        with open(calib_path) as f:
            data = json.load(f)
        K = np.array(data['K'], dtype=np.float64)
        D = np.array(data['D'], dtype=np.float64)
        w, h = data.get('image_size', [3232, 2426])
        model = data.get('model', 'pinhole')
        print(f"Loaded calibration for {cam_name}: {w}x{h} {model} fx={K[0,0]:.1f}")
        return K, D, w, h, model
    except Exception as e:
        print(f"WARNING: Failed to load calibration for {cam_name}: {e}")
        return None


# =============================================================================
# PROJECTION HELPERS (from thermal-slam-projection.py — tested calibration)
# =============================================================================

# Thermal camera intrinsics (raw, with distortion)
K_RAW = np.array([
    [334.78419, 0.0,      317.07016],
    [0.0,       332.67743, 234.32726],
    [0.0,       0.0,       1.0],
], dtype=np.float64)
DIST_COEFFS = np.array([-0.164539, 0.026521, -0.004782, -0.004943, 0.0], dtype=np.float64)
THERMAL_IMG_W, THERMAL_IMG_H = 640, 480


def _quat_to_rot(qx, qy, qz, qw):
    """Quaternion to 3x3 rotation matrix."""
    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)],
    ])


def _invert_T_lidar_camera(tx, ty, tz, qx, qy, qz, qw):
    """Convert dvlc T_lidar_camera [t, q] to T_camera_lidar [R, t].

    dvlc T_lidar_camera: p_lidar = R_lc @ p_cam + t_lc
    We need T_camera_lidar: p_cam = R_cl @ p_lidar + t_cl
    Where: R_cl = R_lc^T, t_cl = -R_lc^T @ t_lc
    """
    R_lc = _quat_to_rot(qx, qy, qz, qw)
    R_cl = R_lc.T
    t_lc = np.array([tx, ty, tz])
    t_cl = -R_cl @ t_lc
    return R_cl, t_cl


# --- Extrinsics from dvlc calibrations (inverted to T_camera_lidar) ---
# Source: /var/mnt/nvme2/slam-output/dvlc_*_target/calib.json (Feb 12)
#         /var/mnt/nvme2/slam-output/dvlc_*_slam/calib.json (Feb 11)
#
# SERIAL FIX (2026-02-14): run_cameras.sh serials corrected so topic labels
# now match physical direction: /thermal1 = LEFT, /thermal2 = RIGHT.
# dvlc calibrations were done on OLD (pre-fix) topic data:
#   dvlc_thermal1_target → calibrated OLD /thermal1 (= RIGHT physical) → looks RIGHT
#   dvlc_thermal2_target → calibrated OLD /thermal2 (= LEFT physical) → looks LEFT
# After serial fix, cross-assign: /thermal2 (RIGHT) uses dvlc_thermal1, etc.

# T2 applied to /thermal2 (= RIGHT physical after serial fix)
# Uses dvlc_thermal1_target (looks RIGHT)
T2_R, T2_t = _invert_T_lidar_camera(0, 0, 0, 0.014794891008185233, -0.6965769351428056, 0.7171679262038497, -0.015232222241534442)

# T1 applied to /thermal1 (= LEFT physical after serial fix)
# Uses dvlc_thermal2_target (looks LEFT)
T1_R, T1_t = _invert_T_lidar_camera(0, 0, 0, -0.7023300364952526, -0.014474507704056807, 0.01285336171480651, 0.6997966876958116)

# --- RGB Oryx cameras (2x2 binned: 3232x2426) ---
# Try loading from calibration files first, fall back to hardcoded dvlc values
_CALIB_DIR = os.environ.get('CALIB_DIR', '/config/cameras/calibration')
_cam_a_cal = _load_camera_calibration('cam_a', _CALIB_DIR)
_cam_b_cal = _load_camera_calibration('cam_b', _CALIB_DIR)

# cam_a (RIGHT) — fisheye calibration or fallback to dvlc_cam2_target
if _cam_a_cal:
    K_CAM2, DIST_CAM2, CAM2_W, CAM2_H = _cam_a_cal[0], _cam_a_cal[1], _cam_a_cal[2], _cam_a_cal[3]
else:
    K_CAM2 = np.array([
        [1166.26303, 0.0,        1619.56279],
        [0.0,        1169.02224, 1199.32755],
        [0.0,        0.0,        1.0],
    ], dtype=np.float64)
    DIST_CAM2 = np.zeros(5, dtype=np.float64)
    CAM2_W, CAM2_H = 3232, 2426

# cam_b (LEFT) — fisheye calibration or fallback to dvlc_cam3_slam
if _cam_b_cal:
    K_CAM3, DIST_CAM3, CAM3_W, CAM3_H = _cam_b_cal[0], _cam_b_cal[1], _cam_b_cal[2], _cam_b_cal[3]
else:
    K_CAM3 = np.array([
        [1192.57, 0.0,     1636.47],
        [0.0,     1195.50, 1183.27],
        [0.0,     0.0,     1.0],
    ], dtype=np.float64)
    DIST_CAM3 = np.zeros(5, dtype=np.float64)
    CAM3_W, CAM3_H = 3232, 2426

# Cam2 (RIGHT) — from dvlc_cam2_slam (60 frames, includes translation)
CAM2_R, CAM2_t = _invert_T_lidar_camera(0.0042, 0.19999999999999998, -0.12980000000000003, 0.6891266360695715, -0.0018080031134361987, 0.027724202260755382, 0.724108126727758)

# Cam3 (LEFT) — from dvlc_cam3_slam (60 frames, includes translation)
CAM3_R, CAM3_t = _invert_T_lidar_camera(0.19516619667266122, 0.14267676136819465, -0.39962206973777015, -0.0033307642509005065, -0.7000556252924953, 0.7138500245363018, -0.01814304206465452)


def _odom_to_matrix(odom_msg):
    """Convert nav_msgs/Odometry to 4x4 transform matrix."""
    pos = odom_msg.pose.pose.position
    ori = odom_msg.pose.pose.orientation
    R = _quat_to_rot(ori.x, ori.y, ori.z, ori.w)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [pos.x, pos.y, pos.z]
    return T


def _project_points_to_camera(points_lidar, cam_R, cam_t, K, dist, img_w, img_h):
    """Project 3D lidar points into camera image plane.
    Returns: pixels (N,2), valid mask (N,) bool."""
    pts_cam = (cam_R @ points_lidar.T).T + cam_t
    in_front = pts_cam[:, 2] > 0.1
    rvec, _ = cv2.Rodrigues(cam_R)
    tvec = cam_t.reshape(3, 1)
    pts_2d, _ = cv2.projectPoints(
        points_lidar.astype(np.float64), rvec, tvec, K, dist)
    pixels = pts_2d.reshape(-1, 2)
    in_bounds = (
        in_front &
        (pixels[:, 0] >= 0) & (pixels[:, 0] < img_w) &
        (pixels[:, 1] >= 0) & (pixels[:, 1] < img_h)
    )
    return pixels, in_bounds


def _decode_compressed_bytes(data):
    """Decode JPEG/PNG bytes to BGR numpy array."""
    buf = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


# Build inferno LUT lazily (needs cv2)
_INFERNO_LUT = None


def _build_inferno_lut():
    lut = np.zeros((256, 1, 3), dtype=np.uint8)
    for i in range(256):
        lut[i, 0] = [i, i, i]
    lut = cv2.applyColorMap(lut, cv2.COLORMAP_INFERNO)
    return lut.reshape(256, 3)


def _enhance_thermal_raw(raw_uint16):
    """Raw thermal Mono16 -> (INFERNO BGR image, grayscale uint8).
    Returns (bgr, gray) for vertex coloring and scalar field respectively."""
    global _INFERNO_LUT
    if _INFERNO_LUT is None:
        _INFERNO_LUT = _build_inferno_lut()
    p1, p99 = np.percentile(raw_uint16, [1, 99])
    if p99 > p1:
        gray = np.clip((raw_uint16.astype(np.float32) - p1) / (p99 - p1) * 255,
                        0, 255).astype(np.uint8)
    else:
        gray = np.zeros(raw_uint16.shape, dtype=np.uint8)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    eq = clahe.apply(gray)
    lo = np.percentile(eq, 30)
    stretched = np.clip((eq.astype(np.float32) - lo) / max(255 - lo, 1) * 255,
                         0, 255).astype(np.uint8)
    bgr = _INFERNO_LUT[stretched]
    return bgr, gray


def _sample_rgb(img_bgr, pixels, valid):
    """Sample BGR pixel values at projected coordinates."""
    rgb = np.full((len(pixels), 3), 40, dtype=np.uint8)
    if not valid.any() or img_bgr is None:
        return rgb
    px = np.clip(pixels[valid, 0].astype(int), 0, img_bgr.shape[1] - 1)
    py = np.clip(pixels[valid, 1].astype(int), 0, img_bgr.shape[0] - 1)
    rgb[valid] = img_bgr[py, px]
    return rgb


def _extract_points_full(pc_msg):
    """Extract xyz + all Ouster scalar fields from PointCloud2.
    Returns: xyz (N,3) float64, scalars dict {name: (N,) float32}."""
    point_step = pc_msg.point_step
    n = pc_msg.width * pc_msg.height
    if n == 0:
        return None, {}
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
    scalars = {}
    field_names = points.dtype.names
    ouster_fields = {
        'reflectivity': 'reflectivity',
        'ambient': 'near_ir',
        'signal': 'signal',
        'range': 'lidar_range',
        'intensity': 'lidar_intensity',
    }
    for src, dst_name in ouster_fields.items():
        if src in field_names:
            scalars[dst_name] = points[src][valid].astype(np.float32)
    return xyz[valid], scalars


# =============================================================================
# MAIN NODE
# =============================================================================

class FrameExtractor(Node):
    def __init__(self):
        super().__init__('frame_extractor')

        # Config
        self.config = self._load_config()
        self.rig_id = os.environ.get('VEHICLE_ID', 'bess_grey').replace(' ', '_').lower()
        self.supabase_url = os.environ.get('SUPABASE_URL', '')
        self.supabase_key = os.environ.get('SUPABASE_ANON_KEY', '')
        self.ambient_temp_c = float(os.environ.get('AMBIENT_TEMP_C', '15.0'))
        self.ambient_humidity = 0.5  # relative humidity 0-1, updated from phidget

        # Coordinate transformer (cached)
        self.transformer = Transformer.from_crs("EPSG:4326", "EPSG:27700", always_xy=True)

        # Spatial database
        spatial_db_path = os.environ.get('SPATIAL_DB', '/data/spatial/bess_uprn.db')
        self.spatial_db = SpatialDB(spatial_db_path)

        # CvBridge for Image message conversion
        self.cv_bridge = CvBridge() if HAS_CV else None

        # State
        self.current_gnss = None  # (lat, lon, alt)
        self.current_bng = None   # (x, y)
        self.current_heading = None  # degrees (from GNSS movement)
        self.prev_bng = None
        self.slam_odom = {}  # source -> latest Odometry

        # Heading sources: SLAM quaternion → geographic heading via offset calibration
        self._slam_yaw = {}  # source -> (yaw_rad, wall_time)
        self._slam_heading_offset = None  # geographic_deg - slam_deg (calibrated)
        self._heading_source = 'none'  # 'slam', 'dual_antenna', 'gnss_movement', 'none'

        # Dual antenna heading from GQ7 INS (works even when stationary)
        self._dual_antenna_heading = None  # degrees, geographic
        self._dual_antenna_time = 0.0     # wall time of last update
        self._heading_offset_buffer = deque(maxlen=HEADING_BUFFER_SIZE)  # circular buffer
        self._heading_offset_std = 999.0  # circular std of offset buffer (degrees)
        self._heading_converged = False
        self._heading_consecutive_rejects = 0  # reset buffer when too many rejections

        # RTK quality tracking for heading calibration
        self._gnss_fix_status = -1     # NavSatFix status: -1=none, 0=fix, 1=SBAS, 2=RTK
        self._gnss_h_covariance = 999.0  # horizontal position variance (m²)
        self._rtk_datum_bng = None     # (x, y) of last stationary RTK fix
        self._rtk_datum_time = 0.0     # wall time of RTK datum
        self._heading_offset_n = 0     # number of RTK calibration samples
        self._prev_rtk_bng = None      # last RTK-quality BNG for heading vector
        self.trajectory = []  # List[TrajectoryPoint]
        self.visibility = []  # List[VisibilityRecord]
        self.heroes = {}      # uprn -> HeroTracker
        self.asset_counts = defaultdict(int)
        self.latest_assets = {}  # asset_type -> filename (for hero association)
        self.session_dir = None
        self.session_name = None
        self.last_flush = time.time()
        self.lock = threading.Lock()

        # GPS speed filter state
        self._last_accepted_gnss = None  # (lat, lon, time.time())
        self._last_traj_bng = None       # (x, y) of last trajectory point
        self._is_moving = False          # True when moved > threshold recently
        self._current_speed_ms = 0.0     # estimated GPS speed in m/s
        self._last_cloud_bng = None      # (x, y) of last saved SLAM cloud

        # Throttle timers
        self._last_thermal_save = {}   # camera_name -> timestamp
        self._last_lidar_save = {}     # pano_type -> timestamp
        self._last_cloud_save = 0.0

        # SLAM cloud stats
        self.slam_cloud_count = 0
        self.slam_cloud_total_points = 0
        self._last_session_cloud_write = 0.0
        self._session_cloud_segment_num = 0
        self._session_cloud_last_segment_points = 0
        self._SESSION_CLOUD_SEGMENT_MIN_POINTS = 500_000  # new segment every 500K points

        # SLAM→BNG georeferencing transform (set when heading offset is calibrated)
        self._slam_origin_bng = None       # (x, y, z) first BNG position at SLAM start
        self._slam_origin_set = False

        # Accumulated map buffer (prefer /glim/map, fallback /Laser_map)
        self._accumulated_map = None       # (xyz, intensity, source, timestamp)
        self._accumulated_map_lock = threading.Lock()

        # Image ring buffers for point cloud colorization
        self._img_buffer = {
            'cam2': deque(maxlen=10),      # (timestamp_s, jpeg_bytes)
            'cam3': deque(maxlen=10),      # (timestamp_s, jpeg_bytes)
            'thermal1': deque(maxlen=10),  # (timestamp_s, raw_uint16_ndarray)
            'thermal2': deque(maxlen=10),  # (timestamp_s, raw_uint16_ndarray)
        }
        # SLAM pose buffer for map-frame transform
        self._slam_pose_buffer = deque(maxlen=200)  # (timestamp_s, 4x4_matrix, source)

        # Colorized cloud state (separate from SLAM cloud tracker)
        self._last_colorized_bng = None
        self._last_colorized_save = 0.0

        # Async IO thread pool for LAZ/LAS writes (prevents blocking rclpy callbacks)
        self._io_pool = ThreadPoolExecutor(max_workers=4, thread_name_prefix='bess_io')
        self._io_pending = 0  # track in-flight writes
        self._io_lock = threading.Lock()

        # Create session
        self._init_session()

        # QoS for sensor topics
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # SLAM odometry subscribers (all three sources)
        for topic, source in [
            ('/glim/odom', 'glim'),
            ('/fast_lio/odometry', 'fast_lio'),
            ('/kiss/odometry', 'kiss_icp'),
        ]:
            self.create_subscription(
                Odometry, topic,
                lambda msg, s=source: self._on_slam(msg, s),
                sensor_qos
            )

        # GNSS subscriber — use raw GNSS, not EKF-fused position.
        # EKF /ekf/llh_position has NaN quaternion corruption when filter
        # hasn't initialized heading. Raw /gnss_1/llh_position has proper
        # RTK covariance (~0.0002 m²).
        self.create_subscription(
            NavSatFix, '/gnss_1/llh_position',
            self._on_gnss, sensor_qos
        )

        # Dual antenna heading from GQ7 INS — stable even when stationary.
        self.create_subscription(
            PoseWithCovarianceStamped, '/ekf/dual_antenna_heading',
            self._on_dual_antenna_heading, sensor_qos
        )

        # Environment sensors (phidget) — ambient temp + humidity for Planck correction
        self.create_subscription(
            Temperature, '/environment/temperature',
            self._on_ambient_temp, sensor_qos
        )
        self.create_subscription(
            RelativeHumidity, '/environment/humidity',
            self._on_ambient_humidity, sensor_qos
        )

        # RGB camera subscribers (CompressedImage)
        # Map camera_name -> projection buffer key (based on physical camera)
        self._rgb_to_proj = {}
        for cam_name, topic in self.config['rgb_cameras'].items():
            if '/camera2/' in topic:
                self._rgb_to_proj[cam_name] = 'cam2'
            elif '/camera3/' in topic:
                self._rgb_to_proj[cam_name] = 'cam3'
            self.create_subscription(
                CompressedImage, topic,
                lambda msg, c=cam_name: self._on_image(msg, c),
                sensor_qos
            )

        # Thermal camera subscribers (raw Image, Mono16)
        # Map camera_name → serial for Planck radiometric calibration
        self._thermal_to_proj = {}
        self._thermal_serial = {}
        thermal_serials = self._load_thermal_serials()
        if self.cv_bridge:
            for cam_name, topic in self.config['thermal_cameras'].items():
                if '/thermal1/' in topic:
                    self._thermal_to_proj[cam_name] = 'thermal1'
                    self._thermal_serial[cam_name] = thermal_serials.get('thermal1')
                elif '/thermal2/' in topic:
                    self._thermal_to_proj[cam_name] = 'thermal2'
                    self._thermal_serial[cam_name] = thermal_serials.get('thermal2')
                self.create_subscription(
                    Image, topic,
                    lambda msg, c=cam_name: self._on_thermal(msg, c),
                    sensor_qos
                )

            # LiDAR panorama subscribers (raw Image)
            for pano_name, topic in self.config['lidar_panoramas'].items():
                pano_type = pano_name.replace('lidar_', '')  # range, reflec, nearir, signal
                self.create_subscription(
                    Image, topic,
                    lambda msg, t=pano_type: self._on_lidar_image(msg, t),
                    sensor_qos
                )

        # SLAM per-scan cloud subscribers (PointCloud2, 1Hz throttle → individual LAS)
        for topic, source in [
            ('/fast_lio/cloud_registered', 'fast_lio'),
            ('/glim/aligned_points_corrected', 'glim'),
        ]:
            self.create_subscription(
                PointCloud2, topic,
                lambda msg, s=source: self._on_slam_cloud(msg, s),
                sensor_qos
            )

        # Raw Ouster cloud subscriber for colorized projection — DISABLED
        # Was writing 180K-point colorized LAZ every ~1s, causing thermal shutdown.
        # Colorization can be done offline from bags when needed.
        # if HAS_CV and HAS_LASPY:
        #     self.create_subscription(
        #         PointCloud2, '/ouster/points',
        #         self._on_raw_cloud_colorize,
        #         sensor_qos
        #     )

        # Accumulated map subscribers (for session_cloud.laz — coherent merged map)
        # FAST-LIO /Laser_map uses RELIABLE QoS
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            PointCloud2, '/Laser_map',
            lambda msg: self._on_accumulated_map(msg, 'fast_lio'),
            reliable_qos
        )
        # GLIM /glim/map publishes RELIABLE + TRANSIENT_LOCAL
        glim_map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.create_subscription(
            PointCloud2, '/glim/map',
            lambda msg: self._on_accumulated_map(msg, 'glim'),
            glim_map_qos
        )

        # Periodic flush timer
        self.create_timer(FLUSH_INTERVAL_S, self._flush)

        n_rgb = len(self.config['rgb_cameras'])
        n_thermal = len(self.config['thermal_cameras']) if self.cv_bridge else 0
        n_lidar = len(self.config['lidar_panoramas']) if self.cv_bridge else 0
        colorize = 'yes' if (HAS_CV and HAS_LASPY) else 'no'
        proj_cams = list(self._rgb_to_proj.values()) + list(self._thermal_to_proj.values())
        self.get_logger().info(
            f'FrameExtractor v3: {n_rgb} RGB + {n_thermal} thermal + '
            f'{n_lidar} LiDAR pano + SLAM cloud + colorize={colorize}, '
            f'proj_cams={proj_cams}, '
            f'spatial_db={"yes" if self.spatial_db.available else "no"}, '
            f'ambient_temp={self.ambient_temp_c}C, session={self.session_name}'
        )

    def _load_config(self):
        """Load camera/sensor config from YAML or use defaults."""
        config_path = '/config/cameras.yaml'
        rgb = {}
        thermal = {}
        lidar = {}

        if os.path.exists(config_path):
            entries = self._parse_yaml(config_path)
            for key, topic in entries.items():
                if key.endswith('_raw'):
                    # thermal_raw entries → thermal (strip _raw for output dir)
                    thermal[key.replace('_raw', '')] = topic
                elif key.startswith('lidar_'):
                    lidar[key] = topic
                elif topic.startswith('/'):
                    rgb[key] = topic

        # Defaults if config is empty or missing
        if not rgb:
            rgb = {
                'right_rgb': '/camera2/camera_driver/image_masked/compressed',
                'left_rgb': '/camera3/camera_driver/image_masked/compressed',
            }
        if not thermal:
            thermal = {
                'left_thermal': '/thermal1/camera_driver/image_raw',
                'right_thermal': '/thermal2/camera_driver/image_raw',
            }
        if not lidar:
            lidar = {
                'lidar_range': '/ouster/range_image',
                'lidar_reflec': '/ouster/reflec_image',
                'lidar_nearir': '/ouster/nearir_image',
                'lidar_signal': '/ouster/signal_image',
            }

        return {'rgb_cameras': rgb, 'thermal_cameras': thermal, 'lidar_panoramas': lidar}

    def _load_thermal_serials(self):
        """Load thermal camera serials from discovered_sensors.json or config."""
        serials = {}
        # Try discovery file first
        for path in ('/run/bess/discovered_sensors.json', '/config/discovered_sensors.json'):
            try:
                import json as _json
                with open(path) as f:
                    disco = _json.load(f)
                for key in ('thermal1', 'thermal2'):
                    entry = disco.get('sensors', {}).get(key, {})
                    if entry.get('serial'):
                        serials[key] = entry['serial']
                if serials:
                    break
            except Exception:
                continue
        # Fallback: infer from THERMAL_PLANCK keys (known serials per vehicle)
        if not serials and THERMAL_PLANCK:
            planck_serials = list(THERMAL_PLANCK.keys())
            if len(planck_serials) >= 2:
                serials = {'thermal1': planck_serials[0], 'thermal2': planck_serials[1]}
            elif len(planck_serials) == 1:
                serials = {'thermal1': planck_serials[0], 'thermal2': planck_serials[0]}
        if serials:
            self.get_logger().info(f'Thermal serials: {serials}')
        else:
            self.get_logger().warning('No thermal serials, Planck cal unavailable')
        return serials

    @staticmethod
    def _parse_yaml(path):
        """Simple YAML key:value parser (no PyYAML dependency)."""
        entries = {}
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith('#') or ':' not in line:
                    continue
                parts = line.split(':', 1)
                if len(parts) == 2:
                    key = parts[0].strip()
                    val = parts[1].strip().strip('"').strip("'")
                    if val.startswith('/'):
                        entries[key] = val
        return entries

    def _init_session(self):
        """Create or resume session directory with all asset subdirectories."""
        base_dir = Path(os.environ.get('EXTRACTION_DIR', '/data/extraction'))
        base_dir.mkdir(parents=True, exist_ok=True)

        # Try to resume recent session (< 30 min old)
        if self._try_resume_session(base_dir):
            self.get_logger().info(f'RESUMED session: {self.session_dir}')
        else:
            ts = datetime.now().strftime('%Y%m%d_%H%M%S')
            self.session_name = f"session_bess_{ts}"
            self.session_dir = base_dir / self.session_name
            self.get_logger().info(f'NEW session: {self.session_dir}')

        # Create static asset directories (idempotent)
        for asset_dir in STATIC_ASSET_DIRS:
            (self.session_dir / 'images' / asset_dir).mkdir(parents=True, exist_ok=True)
        # Create per-camera asset directories from config keys
        for cam_name in list(self.config['rgb_cameras'].keys()):
            (self.session_dir / 'images' / cam_name).mkdir(parents=True, exist_ok=True)
        for cam_name in list(self.config['thermal_cameras'].keys()):
            (self.session_dir / 'images' / cam_name).mkdir(parents=True, exist_ok=True)

        # Clean up stale .tmp files from crashed writes
        self._cleanup_tmp_files()

        # Update 'latest' symlink (relative so it works from host mount too)
        latest = base_dir / 'latest'
        tmp_link = base_dir / '.latest_tmp'
        try:
            tmp_link.unlink(missing_ok=True)
            tmp_link.symlink_to(self.session_name)
            tmp_link.rename(latest)
        except Exception:
            pass

    def _try_resume_session(self, base_dir):
        """Try to resume a recent session. Returns True if resumed."""
        latest = base_dir / 'latest'
        if not latest.exists() or not latest.is_symlink():
            return False
        try:
            target = latest.resolve()
            summary = target / 'extraction_summary.json'
            if not summary.exists():
                return False
            age = time.time() - summary.stat().st_mtime
            if age > 1800:  # > 30 min = stale
                self.get_logger().info(f'Previous session too old ({age:.0f}s), starting fresh')
                return False

            # Reload trajectory from trajectory_wgs84.json
            traj_path = target / 'trajectory_wgs84.json'
            if traj_path.exists():
                traj_data = json.loads(traj_path.read_text())
                for p in traj_data:
                    bng_x, bng_y = self.transformer.transform(p['lon'], p['lat'])
                    self.trajectory.append(TrajectoryPoint(
                        timestamp_ns=p['timestamp_ns'],
                        x=bng_x, y=bng_y, z=p.get('alt', 0),
                        lat=p['lat'], lon=p['lon'], alt=p.get('alt', 0),
                        heading_deg=p.get('heading', 0),
                    ))
                self.get_logger().info(f'Resumed {len(self.trajectory)} trajectory points')

            # Reload asset counts from extraction_summary
            summ_data = json.loads(summary.read_text())
            if 'asset_counts' in summ_data:
                for k, v in summ_data['asset_counts'].items():
                    self.asset_counts[k] = v
            if 'slam_clouds' in summ_data:
                self.slam_cloud_count = summ_data['slam_clouds'].get('count', 0)
                self.slam_cloud_total_points = summ_data['slam_clouds'].get('total_points', 0)

            # Reload heroes from frame_visibility.json
            fv_path = target / 'frame_visibility.json'
            if fv_path.exists():
                fv_data = json.loads(fv_path.read_text())
                hero_count = len(fv_data.get('heroes', {}))
                vis_count = len(fv_data.get('visibility', []))
                self.get_logger().info(f'Resumed {hero_count} heroes, {vis_count} visibility records')

            self.session_name = latest.readlink().name if hasattr(latest, 'readlink') else os.readlink(str(latest))
            self.session_dir = target
            return True
        except Exception as e:
            self.get_logger().warn(f'Session resume failed: {e}, starting fresh')
            return False

    def _cleanup_tmp_files(self):
        """Remove stale .tmp files from crashed writes (both *.tmp and *.tmp.*)."""
        count = 0
        try:
            for tmp in self.session_dir.rglob('*.tmp'):
                try:
                    tmp.unlink()
                    count += 1
                except Exception:
                    pass
            for tmp in self.session_dir.rglob('*.tmp.*'):
                try:
                    tmp.unlink()
                    count += 1
                except Exception:
                    pass
        except Exception:
            pass
        if count:
            self.get_logger().info(f'Cleaned up {count} stale .tmp files')

    # ─── HELPERS ───────────────────────────────────────────────────

    _disk_ok = True
    _disk_check_time = 0.0

    def _submit_io(self, fn, *args, **kwargs):
        """Submit a file write to the IO thread pool (non-blocking).
        Logs errors from the background thread. Drops work if pool is saturated."""
        with self._io_lock:
            if self._io_pending >= 8:
                self.get_logger().warn('IO pool saturated — dropping write')
                return
            self._io_pending += 1

        def _wrapper():
            try:
                fn(*args, **kwargs)
            except Exception as e:
                self.get_logger().error(f'Async IO error: {e}')
            finally:
                with self._io_lock:
                    self._io_pending -= 1

        self._io_pool.submit(_wrapper)

    def _check_disk_space(self):
        """Check disk space (cached, checked at most every 30s). Returns False if critical."""
        now = time.time()
        if now - self._disk_check_time < 30:
            return self._disk_ok
        self._disk_check_time = now
        try:
            usage = shutil.disk_usage(self.session_dir)
            if usage.free < 10_000_000_000:  # < 10 GB
                self.get_logger().error(
                    f'DISK CRITICAL: {usage.free / 1e9:.1f} GB free — pausing extraction')
                self._disk_ok = False
            else:
                self._disk_ok = True
        except Exception:
            self._disk_ok = True
        return self._disk_ok

    def _get_timestamp_ns(self, msg):
        """Extract timestamp from message header, fallback to wall clock."""
        ts_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        return ts_ns if ts_ns != 0 else int(time.time() * 1e9)

    def _get_position(self):
        """Return (lat, lon, alt, bng_x, bng_y, heading) or None if no GNSS fix.
        heading may be None if no heading source is available yet."""
        if self.current_gnss is None or self.current_bng is None:
            return None
        lat, lon, alt = self.current_gnss
        bng_x, bng_y = self.current_bng
        heading = self._get_heading()  # None if no source available
        return lat, lon, alt, bng_x, bng_y, heading

    def _camera_side(self, camera_name):
        """Infer camera side from name."""
        if 'left' in camera_name:
            return 'left'
        elif 'right' in camera_name:
            return 'right'
        elif 'rear' in camera_name:
            return 'rear'
        return 'unknown'

    def _get_best_slam_yaw(self):
        """Get most recent SLAM yaw. Priority: GLIM > FAST-LIO > KISS-ICP."""
        for source in ['glim', 'fast_lio', 'kiss_icp']:
            if source in self._slam_yaw:
                return self._slam_yaw[source]
        return None

    def _get_heading(self):
        """Get best available geographic heading.
        Priority: dual antenna > SLAM (calibrated) > GNSS movement > last known.
        NEVER returns None — uses last known heading as final fallback to prevent
        trajectory freezes during heading offset recalibration."""
        # Dual antenna heading from GQ7 (~2° accuracy, immune to SLAM shifts)
        # Widened staleness to 5s — dual antenna publishes at 2Hz, 2s was too tight
        if (self._dual_antenna_heading is not None
                and (time.time() - self._dual_antenna_time) < 5.0):
            self._heading_source = 'dual_antenna'
            self._last_known_heading = self._dual_antenna_heading
            return self._dual_antenna_heading

        # SLAM heading (fallback — may be wrong after loop closure)
        if self._slam_heading_offset is not None:
            best = self._get_best_slam_yaw()
            if best is not None and (time.time() - best[1]) < 5.0:
                heading = (math.degrees(best[0]) + self._slam_heading_offset) % 360
                self._heading_source = 'slam'
                self._last_known_heading = heading
                return heading

        # Fall back to GNSS movement heading
        if self.current_heading is not None:
            self._heading_source = 'gnss_movement'
            self._last_known_heading = self.current_heading
            return self.current_heading

        # Last resort: use last known heading (stale but better than None)
        if hasattr(self, '_last_known_heading') and self._last_known_heading is not None:
            self._heading_source = 'stale'
            return self._last_known_heading

        self._heading_source = 'none'
        return None

    def _write_meta(self, meta_path, ts_ns, lat, lon, bng_x, bng_y, heading,
                    camera_name, **extra):
        """Write .meta.json sidecar file."""
        meta = {
            'timestamp_ns': ts_ns,
            'gps': {'lat': round(lat, 8), 'lon': round(lon, 8)},
            'bng': {'x': round(bng_x, 2), 'y': round(bng_y, 2)},
            'heading_deg': round(heading, 2),
            'camera': camera_name,
            'rig_id': self.rig_id,
        }
        meta.update(extra)
        try:
            atomic_write_text(meta_path, json.dumps(meta))
        except Exception:
            pass

    # ─── CALLBACKS ─────────────────────────────────────────────────

    def _on_ambient_temp(self, msg):
        """Update ambient temperature from phidget sensor."""
        self.ambient_temp_c = msg.temperature

    def _on_ambient_humidity(self, msg):
        """Update relative humidity from phidget sensor."""
        self.ambient_humidity = msg.relative_humidity

    def _on_dual_antenna_heading(self, msg):
        """Update heading from GQ7 dual antenna (works even when stationary)."""
        q = msg.pose.pose.orientation
        if math.isnan(q.w) or (q.x == 0 and q.y == 0 and q.z == 0 and q.w == 0):
            return
        yaw_rad = quaternion_to_yaw(q)
        # GQ7 outputs NED (0°=north), not ENU — direct conversion
        heading_deg = math.degrees(yaw_rad) % 360
        self._dual_antenna_heading = heading_deg
        self._dual_antenna_time = time.time()

        # Use dual antenna heading for SLAM offset calibration
        best = self._get_best_slam_yaw()
        if best is None or (time.time() - best[1]) >= 2.0:
            return
        slam_deg = math.degrees(best[0]) % 360
        new_offset = (heading_deg - slam_deg + 360) % 360

        # After convergence, only monitor for drift — don't update the
        # locked georef offset. This prevents XY scatter from source
        # switching (GLIM→FAST-LIO) or noisy samples.
        if self._heading_converged:
            cur_mean = self._slam_heading_offset
            diff = new_offset - cur_mean
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            if abs(diff) > 25.0:  # was 10° — too sensitive to GLIM loop closures which rotate 10-30°
                self._heading_consecutive_rejects += 1
                if self._heading_consecutive_rejects >= HEADING_MAX_CONSECUTIVE_REJECTS:
                    self.get_logger().warn(
                        f'Heading offset RESET (dual antenna): '
                        f'{self._heading_consecutive_rejects} consecutive rejections '
                        f'(was {cur_mean:.1f}°, new {new_offset:.1f}°)')
                    self._heading_offset_buffer.clear()
                    self._heading_converged = False
                    self._heading_consecutive_rejects = 0
                    # Fall through to accept this sample as seed
                else:
                    return
            else:
                self._heading_consecutive_rejects = 0
                return  # within tolerance, offset stays locked

        # Use same buffer + outlier rejection as RTK path
        if len(self._heading_offset_buffer) >= 5:
            cur_mean, cur_std = circular_mean_std_deg(
                list(self._heading_offset_buffer))
            diff = new_offset - cur_mean
            if diff > 180:
                diff -= 360
            elif diff < -180:
                diff += 360
            threshold = max(HEADING_OUTLIER_SIGMA * cur_std, 5.0)
            if abs(diff) > threshold:
                self._heading_consecutive_rejects += 1
                if self._heading_consecutive_rejects >= HEADING_MAX_CONSECUTIVE_REJECTS:
                    self.get_logger().warn(
                        f'Heading offset RESET (dual antenna): '
                        f'{self._heading_consecutive_rejects} consecutive rejections '
                        f'(was {cur_mean:.1f}°, new {new_offset:.1f}°)')
                    self._heading_offset_buffer.clear()
                    self._heading_converged = False
                    self._heading_consecutive_rejects = 0
                else:
                    return  # skip this sample

        self._heading_offset_buffer.append(new_offset)
        self._heading_offset_n += 1
        self._heading_consecutive_rejects = 0

        # Compute circular mean of buffer as the heading offset
        mean_offset, std_offset = circular_mean_std_deg(
            list(self._heading_offset_buffer))
        self._slam_heading_offset = mean_offset
        self._heading_offset_std = std_offset

        if not self._heading_converged and len(self._heading_offset_buffer) >= 5:
            if std_offset < HEADING_CONVERGENCE_STD_DEG:
                self._heading_converged = True
                self.get_logger().info(
                    f'Heading offset CONVERGED (dual antenna): '
                    f'{mean_offset:.1f}° (std={std_offset:.1f}° n={self._heading_offset_n})')

    def _on_slam(self, msg, source):
        """Track SLAM odometry, extract heading, and buffer full 6DOF pose."""
        self.slam_odom[source] = msg
        yaw_rad = quaternion_to_yaw(msg.pose.pose.orientation)
        self._slam_yaw[source] = (yaw_rad, time.time())
        # Buffer full pose for point cloud colorization
        ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        T = _odom_to_matrix(msg)
        self._slam_pose_buffer.append((ts_sec, T, source))

    def _on_gnss(self, msg):
        """Update current GNSS position and compute BNG.

        Heading offset calibration is RTK-quality-aware:
        - RTK fixed (status=2, low covariance): calibrate heading offset with EMA
        - Non-RTK: don't touch heading offset, rely on SLAM + last known good RTK calibration
        - When stationary with RTK: record datum; first movement from datum gives
          high-confidence heading vector for quick rebaseline.
        """
        if msg.status.status < 0:
            return  # No fix

        lat, lon, alt = msg.latitude, msg.longitude, msg.altitude

        # Reject zero/near-zero coords (EKF not converged)
        if abs(lat) < 0.1 or abs(lon) < 0.0001:
            return

        # Track fix quality
        self._gnss_fix_status = msg.status.status
        if msg.position_covariance_type > 0 and len(msg.position_covariance) >= 5:
            # Horizontal variance = max(σ²_east, σ²_north)
            self._gnss_h_covariance = max(
                msg.position_covariance[0],  # σ²_east
                msg.position_covariance[4],  # σ²_north
            )
        is_rtk = (self._gnss_fix_status >= RTK_FIX_STATUS
                   and self._gnss_h_covariance < RTK_COVARIANCE_THRESHOLD)

        # Speed-based outlier rejection: reject GPS teleportation glitches
        now = time.time()
        if self._last_accepted_gnss is not None:
            prev_lat, prev_lon, prev_t = self._last_accepted_gnss
            dt = now - prev_t
            if dt > 0.01:  # avoid div-by-zero
                dlat = (lat - prev_lat) * 111000  # approx meters
                dlon = (lon - prev_lon) * 111000 * math.cos(math.radians(lat))
                dist_m = math.sqrt(dlat**2 + dlon**2)
                speed = dist_m / dt
                if speed > MAX_GPS_SPEED_MS:
                    return  # reject this point — implied speed too high

        # Compute speed from accepted positions
        if self._last_accepted_gnss is not None:
            prev_lat, prev_lon, prev_t = self._last_accepted_gnss
            dt = now - prev_t
            if dt > 0.01:
                dlat = (lat - prev_lat) * 111000
                dlon = (lon - prev_lon) * 111000 * math.cos(math.radians(lat))
                self._current_speed_ms = math.sqrt(dlat**2 + dlon**2) / dt

        self._last_accepted_gnss = (lat, lon, now)
        self.current_gnss = (lat, lon, alt)

        try:
            easting, northing = self.transformer.transform(lon, lat)
            new_bng = (easting, northing)

            # Capture first BNG position as SLAM origin for georeferencing
            if not self._slam_origin_set:
                self._slam_origin_bng = (easting, northing, alt)
                self._slam_origin_set = True
                self.get_logger().info(
                    f'SLAM origin BNG: E={easting:.2f} N={northing:.2f} Z={alt:.2f}')

            # Track movement for trajectory/UPRN gating
            if self._last_traj_bng is not None:
                dx_t = new_bng[0] - self._last_traj_bng[0]
                dy_t = new_bng[1] - self._last_traj_bng[1]
                self._is_moving = math.sqrt(dx_t**2 + dy_t**2) > TRAJECTORY_MOVEMENT_THRESHOLD_M
            else:
                self._is_moving = False

            # --- RTK datum: record precise position when stationary with RTK ---
            if is_rtk and not self._is_moving:
                self._rtk_datum_bng = new_bng
                self._rtk_datum_time = now
                # Also set prev_rtk_bng so first movement from here is RTK-quality
                self._prev_rtk_bng = new_bng

            # --- Heading from movement vector ---
            if self.prev_bng is not None:
                dx = new_bng[0] - self.prev_bng[0]
                dy = new_bng[1] - self.prev_bng[1]
                dist = math.sqrt(dx*dx + dy*dy)

                # Movement threshold depends on fix quality
                threshold = HEADING_MOVEMENT_THRESHOLD_M if is_rtk else HEADING_MOVEMENT_THRESHOLD_NON_RTK_M

                if dist > threshold:
                    gnss_heading = math.degrees(math.atan2(dx, dy)) % 360
                    self.current_heading = gnss_heading
                    self.prev_bng = new_bng

                    # --- RTK heading offset calibration (multi-point circular mean) ---
                    # Only recalibrate from RTK-quality fixes to prevent drift
                    # After convergence, offset is LOCKED to prevent XY scatter
                    if is_rtk and not self._heading_converged:
                        best = self._get_best_slam_yaw()
                        if best is not None and (now - best[1]) < 2.0:
                            slam_deg = math.degrees(best[0]) % 360
                            new_offset = (gnss_heading - slam_deg + 360) % 360

                            # Use datum-based heading if available (higher confidence)
                            if (self._rtk_datum_bng is not None
                                    and self._prev_rtk_bng is not None):
                                ddx = new_bng[0] - self._prev_rtk_bng[0]
                                ddy = new_bng[1] - self._prev_rtk_bng[1]
                                datum_dist = math.sqrt(ddx*ddx + ddy*ddy)
                                if datum_dist > 2.0:
                                    datum_heading = math.degrees(math.atan2(ddx, ddy)) % 360
                                    new_offset = (datum_heading - slam_deg + 360) % 360
                                    self.get_logger().info(
                                        f'Heading offset RTK datum sample: '
                                        f'{new_offset:.1f}° (from {datum_dist:.1f}m datum)')
                                    self._rtk_datum_bng = None  # consumed

                            # Outlier rejection: if buffer has enough samples,
                            # reject new offset if > 3*sigma from current mean
                            if len(self._heading_offset_buffer) >= 5:
                                cur_mean, cur_std = circular_mean_std_deg(
                                    list(self._heading_offset_buffer))
                                diff = new_offset - cur_mean
                                if diff > 180:
                                    diff -= 360
                                elif diff < -180:
                                    diff += 360
                                threshold = max(HEADING_OUTLIER_SIGMA * cur_std,
                                                5.0)  # min 5° to avoid rejecting during convergence
                                if abs(diff) > threshold:
                                    self._heading_consecutive_rejects += 1
                                    if self._heading_consecutive_rejects >= HEADING_MAX_CONSECUTIVE_REJECTS:
                                        self.get_logger().warn(
                                            f'Heading offset RESET: {self._heading_consecutive_rejects} '
                                            f'consecutive rejections — buffer stale '
                                            f'(was {cur_mean:.1f}°, new sample {new_offset:.1f}°)')
                                        self._heading_offset_buffer.clear()
                                        self._heading_converged = False
                                        self._heading_consecutive_rejects = 0
                                        # Accept this sample as the seed for reconvergence
                                    else:
                                        self.get_logger().warn(
                                            f'Heading offset outlier rejected: {new_offset:.1f}° '
                                            f'(mean={cur_mean:.1f}° std={cur_std:.1f}° diff={diff:.1f}°)')
                                        new_offset = None  # skip this sample

                            if new_offset is not None:
                                self._heading_offset_buffer.append(new_offset)
                                self._heading_offset_n += 1
                                self._heading_consecutive_rejects = 0

                                # Compute circular mean of buffer as the heading offset
                                mean_offset, std_offset = circular_mean_std_deg(
                                    list(self._heading_offset_buffer))
                                self._slam_heading_offset = mean_offset
                                self._heading_offset_std = std_offset

                                # Log convergence state changes
                                # Don't let noisy RTK samples de-converge if dual antenna already locked
                                was_converged = self._heading_converged
                                rtk_converged = (
                                    len(self._heading_offset_buffer) >= 5
                                    and std_offset < HEADING_CONVERGENCE_STD_DEG)
                                if not was_converged:
                                    self._heading_converged = rtk_converged
                                if self._heading_converged and not was_converged:
                                    self.get_logger().info(
                                        f'Heading offset CONVERGED: {mean_offset:.1f}° '
                                        f'(std={std_offset:.1f}° n={len(self._heading_offset_buffer)})')
                                elif not self._heading_converged and self._heading_offset_n <= 1:
                                    self.get_logger().info(
                                        f'Heading offset initial: {mean_offset:.1f}°')

                        # Update RTK-quality reference point
                        self._prev_rtk_bng = new_bng

            if self.prev_bng is None:
                self.prev_bng = new_bng

            self.current_bng = new_bng
        except Exception as e:
            self.get_logger().warn(f'BNG transform failed: {e}')

    # ─── RGB IMAGE (CompressedImage) ───────────────────────────────

    def _on_image(self, msg, camera_name):
        """Process incoming compressed RGB image."""
        # Always buffer for projection (before throttle/position checks)
        proj_key = self._rgb_to_proj.get(camera_name)
        if proj_key:
            ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            self._img_buffer[proj_key].append((ts_sec, bytes(msg.data)))

        if not self._check_disk_space():
            return
        pos = self._get_position()
        if pos is None:
            return

        lat, lon, alt, bng_x, bng_y, heading = pos
        heading_deg = heading if heading is not None else 0.0
        ts_ns = self._get_timestamp_ns(msg)
        camera_side = self._camera_side(camera_name)

        fname = f"{timestamp_to_filename(ts_ns)}_{camera_name}"
        img_dir = self.session_dir / 'images' / camera_name
        img_path = img_dir / f"{fname}.jpg"
        meta_path = img_dir / f"{fname}.meta.json"

        try:
            atomic_write_bytes(img_path, msg.data)
        except Exception as e:
            self.get_logger().error(f'Failed to save RGB image: {e}')
            return

        self._write_meta(meta_path, ts_ns, lat, lon, bng_x, bng_y, heading_deg,
                         camera_name, heading_source=self._heading_source,
                         gnss_fix_status=self._gnss_fix_status,
                         heading_offset_n=self._heading_offset_n)

        with self.lock:
            self.asset_counts[camera_name] += 1
            self.latest_assets[camera_name] = f"{fname}.jpg"

            # Append to trajectory only when vehicle has moved
            if self._is_moving:
                self.trajectory.append(TrajectoryPoint(
                    timestamp_ns=ts_ns,
                    x=bng_x, y=bng_y, z=alt,
                    lat=lat, lon=lon, alt=alt,
                    heading_deg=heading_deg,
                ))
                self._last_traj_bng = (bng_x, bng_y)

                # UPRN visibility matching — only while moving with heading
                if self.spatial_db.available and heading is not None:
                    self._match_uprns(ts_ns, bng_x, bng_y, heading_deg, camera_side)
            elif self._last_traj_bng is None:
                # First point — always add to seed trajectory
                self.trajectory.append(TrajectoryPoint(
                    timestamp_ns=ts_ns,
                    x=bng_x, y=bng_y, z=alt,
                    lat=lat, lon=lon, alt=alt,
                    heading_deg=heading_deg,
                ))
                self._last_traj_bng = (bng_x, bng_y)

    # ─── THERMAL IMAGE (raw Mono16) ───────────────────────────────

    def _on_thermal(self, msg, camera_name):
        """Process raw thermal Mono16 → radiometric NPY + INFERNO colormap JPEG."""
        # Always buffer for projection (before throttle check)
        proj_key = self._thermal_to_proj.get(camera_name)
        if proj_key:
            try:
                raw_buf = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if raw_buf.dtype != np.uint16:
                    raw_buf = raw_buf.astype(np.uint16)
                ts_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                self._img_buffer[proj_key].append((ts_sec, raw_buf.copy()))
            except Exception:
                pass

        if not self._check_disk_space():
            return
        # Throttle
        now = time.time()
        if now - self._last_thermal_save.get(camera_name, 0) < THERMAL_THROTTLE_S:
            return
        self._last_thermal_save[camera_name] = now

        pos = self._get_position()
        if pos is None:
            return

        lat, lon, alt, bng_x, bng_y, heading = pos
        heading = heading if heading is not None else 0.0
        ts_ns = self._get_timestamp_ns(msg)

        try:
            # Decode Mono16 raw → numpy uint16
            raw = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Thermal decode failed: {e}')
            return

        if raw.dtype != np.uint16:
            raw = raw.astype(np.uint16)

        # Full FLIR Planck radiometric conversion with atmospheric correction
        # Uses live ambient temp + humidity from phidget sensors.
        serial = self._thermal_serial.get(camera_name)
        cal = THERMAL_PLANCK.get(serial) if serial else None
        if cal:
            raw_f = raw.astype(np.float64)
            radiance = (raw_f - cal['J0']) / cal['J1']
            emiss = THERMAL_EMISSIVITY
            dist = THERMAL_OBJECT_DISTANCE
            t_atm_c = self.ambient_temp_c
            t_atm_k = t_atm_c + 273.15
            t_refl_k = t_atm_k  # reflected temp ≈ ambient for outdoor survey
            humidity = self.ambient_humidity

            # Atmospheric transmission (FLIR formula)
            h2o = humidity * math.exp(1.5587 + 0.06939 * t_atm_c
                                      - 0.00027816 * t_atm_c**2
                                      + 0.00000068455 * t_atm_c**3)
            sqrt_d = math.sqrt(dist)
            tau = (cal['X'] * math.exp(-sqrt_d * (cal['alpha1'] + cal['beta1'] * math.sqrt(h2o)))
                   + (1 - cal['X']) * math.exp(-sqrt_d * (cal['alpha2'] + cal['beta2'] * math.sqrt(h2o))))

            # Pseudo-radiance of reflected environment and atmosphere
            raw_refl = cal['R'] / (math.exp(cal['B'] / t_refl_k) - cal['F'])
            raw_atm = cal['R'] / (math.exp(cal['B'] / t_atm_k) - cal['F'])

            # Object radiance with atmospheric + emissivity correction
            raw_obj = (radiance / (emiss * tau)
                       - ((1 - emiss) / emiss) * raw_refl
                       - ((1 - tau) / (emiss * tau)) * raw_atm)
            raw_obj = np.clip(raw_obj, 0.001, None)

            temp_c = (cal['B'] / np.log(cal['R'] / raw_obj + cal['F']) - 273.15).astype(np.float32)
        else:
            if not getattr(self, '_planck_warn_logged', False):
                self.get_logger().warning(f'No Planck cal for {camera_name} (serial={serial}), using linear fallback')
                self._planck_warn_logged = True
            temp_c = raw.astype(np.float32) * 0.004 + (self.ambient_temp_c - 84.0)

        fname = f"{timestamp_to_filename(ts_ns)}_{camera_name}"
        out_dir = self.session_dir / 'images' / camera_name

        # Save RAW uint16 Mono16 as NPY — front end applies its own calibration
        # using the Planck constants + ambient conditions from meta.json
        npy_path = out_dir / f"{fname}.npy"
        try:
            atomic_write_file(npy_path, lambda p: np.save(str(p), raw))
        except Exception as e:
            self.get_logger().error(f'Thermal NPY save failed: {e}')
            return

        # Colormap: use Planck-converted temps for visual preview JPEG
        p1, p99 = np.percentile(temp_c, [1, 99])
        if p99 > p1:
            norm = np.clip((temp_c - p1) / (p99 - p1) * 255, 0, 255).astype(np.uint8)
        else:
            norm = np.zeros(temp_c.shape, dtype=np.uint8)

        colored = cv2.applyColorMap(norm, cv2.COLORMAP_INFERNO)

        jpg_path = out_dir / f"{fname}.jpg"
        try:
            atomic_write_file(jpg_path, lambda p: cv2.imwrite(str(p), colored, [cv2.IMWRITE_JPEG_QUALITY, 95]))
        except Exception as e:
            self.get_logger().error(f'Thermal JPEG save failed: {e}')
            return

        # Metadata sidecar — Planck constants + live ambient conditions
        # Front end uses these to convert raw uint16 → temperature
        cal_meta = {}
        if cal:
            cal_meta = {
                'radiometric': {
                    'serial': serial,
                    'emissivity': THERMAL_EMISSIVITY,
                    'object_distance_m': THERMAL_OBJECT_DISTANCE,
                    'ambient_temp_c': round(self.ambient_temp_c, 1),
                    'humidity': round(self.ambient_humidity, 3),
                    'planck': {k: v for k, v in cal.items() if k in ('R', 'B', 'F', 'J0', 'J1')},
                    'atm': {k: v for k, v in cal.items() if k in ('alpha1', 'alpha2', 'X', 'beta1', 'beta2')},
                },
            }
        self._write_meta(
            out_dir / f"{fname}.meta.json",
            ts_ns, lat, lon, bng_x, bng_y, heading, camera_name,
            min_temp_c=round(float(np.min(temp_c)), 2),
            max_temp_c=round(float(np.max(temp_c)), 2),
            mean_temp_c=round(float(np.mean(temp_c)), 2),
            **cal_meta,
        )

        with self.lock:
            self.asset_counts[camera_name] += 1
            self.asset_counts[f"{camera_name}_npy"] += 1
            self.latest_assets[camera_name] = f"{fname}.jpg"
            self.latest_assets[f"{camera_name}_npy"] = f"{fname}.npy"

    # ─── LIDAR PANORAMA (raw Image) ───────────────────────────────

    def _on_lidar_image(self, msg, pano_type):
        """Process LiDAR panorama image → colormapped PNG."""
        if not self._check_disk_space():
            return
        # Throttle
        now = time.time()
        if now - self._last_lidar_save.get(pano_type, 0) < LIDAR_IMG_THROTTLE_S:
            return
        self._last_lidar_save[pano_type] = now

        pos = self._get_position()
        if pos is None:
            return

        lat, lon, alt, bng_x, bng_y, heading = pos
        heading = heading if heading is not None else 0.0
        ts_ns = self._get_timestamp_ns(msg)

        try:
            img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'LiDAR image decode failed ({pano_type}): {e}')
            return

        img_float = img.astype(np.float32)

        # Normalize non-zero pixels (zero = no LiDAR return)
        nonzero = img_float[img_float > 0]
        if len(nonzero) < 10:
            return  # No meaningful data

        p1, p99 = np.percentile(nonzero, [1, 99])
        if p99 > p1:
            norm = np.clip((img_float - p1) / (p99 - p1) * 255, 0, 255).astype(np.uint8)
        else:
            norm = np.zeros(img_float.shape, dtype=np.uint8)

        # Zero pixels stay black
        norm[img_float == 0] = 0

        # Apply per-type colormap
        cmap = {
            'range': cv2.COLORMAP_JET,
            'reflec': cv2.COLORMAP_BONE,
            'nearir': cv2.COLORMAP_VIRIDIS,
            'signal': cv2.COLORMAP_PLASMA,
        }.get(pano_type, cv2.COLORMAP_JET)

        colored = cv2.applyColorMap(norm, cmap)
        colored[img_float == 0] = 0  # Keep no-return pixels black

        asset_name = f"lidar_{pano_type}"
        fname = f"{timestamp_to_filename(ts_ns)}_{asset_name}"
        out_dir = self.session_dir / 'images' / asset_name

        png_path = out_dir / f"{fname}.png"
        try:
            atomic_write_file(png_path, lambda p: cv2.imwrite(str(p), colored))
        except Exception as e:
            self.get_logger().error(f'LiDAR PNG save failed ({pano_type}): {e}')
            return

        # Metadata sidecar
        self._write_meta(
            out_dir / f"{fname}.meta.json",
            ts_ns, lat, lon, bng_x, bng_y, heading, asset_name,
            min_val=round(float(np.min(nonzero)), 2),
            max_val=round(float(np.max(nonzero)), 2),
            mean_val=round(float(np.mean(nonzero)), 2),
        )

        with self.lock:
            self.asset_counts[asset_name] += 1
            self.latest_assets[asset_name] = f"{fname}.png"

    # ─── SLAM POINT CLOUD (PointCloud2) ───────────────────────────

    def _get_keyframe_distance(self):
        """Get adaptive keyframe distance based on current GPS speed."""
        speed_kmh = self._current_speed_ms * 3.6
        for threshold, dist in sorted(SLAM_KEYFRAME_DIST_M.items()):
            if speed_kmh <= threshold:
                return dist
        return 20.0  # fallback

    def _on_slam_cloud(self, msg, source):
        """Save registered SLAM point cloud as LAS 1.4 (adaptive distance-based keyframes)."""
        if not self._check_disk_space():
            return
        pos = self._get_position()
        if pos is None:
            return

        lat, lon, alt, bng_x, bng_y, heading = pos

        # Adaptive distance-based throttle
        if self._last_cloud_bng is not None:
            dx = bng_x - self._last_cloud_bng[0]
            dy = bng_y - self._last_cloud_bng[1]
            dist_since_last = math.sqrt(dx*dx + dy*dy)
            keyframe_dist = self._get_keyframe_distance()
            if dist_since_last < keyframe_dist:
                return  # haven't moved enough for next keyframe
        # Also apply minimum time throttle to prevent burst at startup
        now = time.time()
        if now - self._last_cloud_save < 0.2:
            return
        self._last_cloud_save = now
        self._last_cloud_bng = (bng_x, bng_y)

        heading = heading if heading is not None else 0.0
        ts_ns = self._get_timestamp_ns(msg)

        xyz, intensity = self._parse_pointcloud2(msg)
        if xyz is None or len(xyz) < 10:
            return

        # Georeference to BNG if transform is calibrated
        if self._is_georeferenced():
            xyz = self._slam_to_bng(xyz)
            crs_wkt = self._BNG_WKT
        else:
            crs_wkt = None

        fname = f"{timestamp_to_filename(ts_ns)}_slam_cloud"
        out_dir = self.session_dir / 'images' / 'slam_cloud'

        if HAS_LASPY:
            las_path = out_dir / f"{fname}.las"
            # Async write — don't block the SLAM callback
            xyz_copy = xyz.copy()
            int_copy = intensity.copy() if intensity is not None else None
            acc_meta = self._get_accuracy_meta()
            self._submit_io(
                lambda p=las_path, x=xyz_copy, i=int_copy, c=crs_wkt, a=acc_meta:
                    atomic_write_file(p, lambda tp: self._write_las(tp, x, i, crs_wkt=c, accuracy_meta=a)))
            saved_name = f"{fname}.las"
        else:
            # Fallback to NPZ if laspy not available
            npz_path = out_dir / f"{fname}.npz"
            save_dict = {'xyz': xyz.copy()}
            if intensity is not None:
                save_dict['intensity'] = intensity.copy()
            self._submit_io(
                lambda p=npz_path, sd=save_dict:
                    atomic_write_file(p, lambda tp: np.savez_compressed(str(tp), **sd)))
            saved_name = f"{fname}.npz"

        # Metadata sidecar
        self._write_meta(
            out_dir / f"{fname}.meta.json",
            ts_ns, lat, lon, bng_x, bng_y, heading, f'slam_cloud_{source}',
            num_points=len(xyz),
        )

        with self.lock:
            self.asset_counts['slam_cloud'] += 1
            self.slam_cloud_count += 1
            self.slam_cloud_total_points += len(xyz)
            self.latest_assets['slam_cloud'] = saved_name

    def _on_accumulated_map(self, msg, source):
        """Buffer latest accumulated SLAM map (prefer /glim/map, fallback /Laser_map)."""
        xyz, intensity = self._parse_pointcloud2(msg)
        if xyz is None or len(xyz) < 100:
            return
        with self._accumulated_map_lock:
            # Prefer GLIM map (GPU-accelerated, loop-closed); FAST-LIO as fallback
            if self._accumulated_map is not None:
                _, _, prev_source, _ = self._accumulated_map
                # Don't replace GLIM map with FAST-LIO map
                if prev_source == 'glim' and source == 'fast_lio':
                    return
            self._accumulated_map = (xyz, intensity, source, time.time())

    # ─── RAW CLOUD COLORIZATION ──────────────────────────────────

    def _find_nearest_pose(self, target_ts, max_dt=0.1):
        """Find nearest SLAM pose in buffer within max_dt seconds.
        Returns 4x4 matrix or None."""
        best = None
        best_dt = max_dt
        for ts, T, source in self._slam_pose_buffer:
            dt = abs(ts - target_ts)
            if dt < best_dt:
                best_dt = dt
                best = T
        return best

    def _find_nearest_img(self, buffer_name, target_ts, max_dt=0.2):
        """Find nearest image in ring buffer within max_dt seconds.
        Returns raw data (bytes for RGB, uint16 ndarray for thermal) or None."""
        buf = self._img_buffer.get(buffer_name)
        if not buf:
            return None
        best = None
        best_dt = max_dt
        for ts, data in buf:
            dt = abs(ts - target_ts)
            if dt < best_dt:
                best_dt = dt
                best = data
        return best

    def _on_raw_cloud_colorize(self, msg):
        """Colorize raw Ouster points with thermal + RGB camera projection."""
        if not self._check_disk_space():
            return
        pos = self._get_position()
        if pos is None:
            return

        lat, lon, alt, bng_x, bng_y, heading = pos

        # Distance-based keyframe throttle (separate from SLAM cloud tracker)
        if self._last_colorized_bng is not None:
            dx = bng_x - self._last_colorized_bng[0]
            dy = bng_y - self._last_colorized_bng[1]
            dist_since_last = math.sqrt(dx*dx + dy*dy)
            if dist_since_last < self._get_keyframe_distance():
                return
        # Burst protection
        now = time.time()
        if now - self._last_colorized_save < 0.5:
            return

        ts_ns = self._get_timestamp_ns(msg)
        ts_sec = ts_ns / 1e9

        # Find nearest SLAM pose within 100ms
        T_world = self._find_nearest_pose(ts_sec, max_dt=0.1)
        if T_world is None:
            return

        # Parse raw cloud with full Ouster scalar fields
        xyz, lidar_scalars = _extract_points_full(msg)
        if xyz is None or len(xyz) < 100:
            return

        n = len(xyz)

        # Per-point output arrays
        rgb = np.full((n, 3), 40, dtype=np.uint8)  # vertex colors (BGR)
        thermal_raw = np.full(n, -1.0, dtype=np.float32)
        t1_raw = np.full(n, -1.0, dtype=np.float32)
        t2_raw = np.full(n, -1.0, dtype=np.float32)
        rgb_r = np.full(n, -1.0, dtype=np.float32)
        rgb_g = np.full(n, -1.0, dtype=np.float32)
        rgb_b = np.full(n, -1.0, dtype=np.float32)
        has_thermal = np.zeros(n, dtype=bool)
        has_rgb = np.zeros(n, dtype=bool)

        # --- Thermal2 (RIGHT) ---
        t2_data = self._find_nearest_img('thermal2', ts_sec, 0.2)
        if t2_data is not None:
            t2_bgr, t2_gray = _enhance_thermal_raw(t2_data)
            pixels, valid = _project_points_to_camera(
                xyz, T2_R, T2_t, K_RAW, DIST_COEFFS, THERMAL_IMG_W, THERMAL_IMG_H)
            rgb_vals = _sample_rgb(t2_bgr, pixels, valid)
            rgb = np.where(valid[:, None], rgb_vals, rgb)
            if valid.any():
                px = np.clip(pixels[valid, 0].astype(int), 0, THERMAL_IMG_W - 1)
                py = np.clip(pixels[valid, 1].astype(int), 0, THERMAL_IMG_H - 1)
                t2_raw[valid] = t2_gray[py, px].astype(np.float32)
                thermal_raw[valid] = t2_raw[valid]
            has_thermal |= valid

        # --- Thermal1 (LEFT) ---
        t1_data = self._find_nearest_img('thermal1', ts_sec, 0.2)
        if t1_data is not None:
            t1_bgr, t1_gray = _enhance_thermal_raw(t1_data)
            pixels, valid = _project_points_to_camera(
                xyz, T1_R, T1_t, K_RAW, DIST_COEFFS, THERMAL_IMG_W, THERMAL_IMG_H)
            new_coverage = valid & ~has_thermal
            rgb_vals = _sample_rgb(t1_bgr, pixels, valid)
            rgb = np.where(new_coverage[:, None], rgb_vals, rgb)
            if valid.any():
                px = np.clip(pixels[valid, 0].astype(int), 0, THERMAL_IMG_W - 1)
                py = np.clip(pixels[valid, 1].astype(int), 0, THERMAL_IMG_H - 1)
                t1_raw[valid] = t1_gray[py, px].astype(np.float32)
                thermal_raw[new_coverage] = t1_raw[new_coverage]
            has_thermal |= valid

        # --- Cam2 (RIGHT RGB) ---
        c2_data = self._find_nearest_img('cam2', ts_sec, 0.2)
        if c2_data is not None:
            c2_img = _decode_compressed_bytes(c2_data)
            if c2_img is not None:
                pixels, valid = _project_points_to_camera(
                    xyz, CAM2_R, CAM2_t, K_CAM2, DIST_CAM2, CAM2_W, CAM2_H)
                if valid.any():
                    px = np.clip(pixels[valid, 0].astype(int), 0, CAM2_W - 1)
                    py = np.clip(pixels[valid, 1].astype(int), 0, CAM2_H - 1)
                    sampled = c2_img[py, px]  # BGR
                    rgb_r[valid] = sampled[:, 2].astype(np.float32)
                    rgb_g[valid] = sampled[:, 1].astype(np.float32)
                    rgb_b[valid] = sampled[:, 0].astype(np.float32)
                    # RGB overrides thermal for vertex color
                    rgb[valid, 0] = sampled[:, 0]
                    rgb[valid, 1] = sampled[:, 1]
                    rgb[valid, 2] = sampled[:, 2]
                    has_rgb |= valid

        # --- Cam3 (LEFT RGB) ---
        c3_data = self._find_nearest_img('cam3', ts_sec, 0.2)
        if c3_data is not None:
            c3_img = _decode_compressed_bytes(c3_data)
            if c3_img is not None:
                pixels, valid = _project_points_to_camera(
                    xyz, CAM3_R, CAM3_t, K_CAM3, DIST_CAM3, CAM3_W, CAM3_H)
                new_rgb = valid & ~has_rgb
                if valid.any():
                    px = np.clip(pixels[valid, 0].astype(int), 0, CAM3_W - 1)
                    py = np.clip(pixels[valid, 1].astype(int), 0, CAM3_H - 1)
                    sampled = c3_img[py, px]  # BGR
                    rgb_r[valid] = sampled[:, 2].astype(np.float32)
                    rgb_g[valid] = sampled[:, 1].astype(np.float32)
                    rgb_b[valid] = sampled[:, 0].astype(np.float32)
                    if new_rgb.any():
                        px2 = np.clip(pixels[new_rgb, 0].astype(int), 0, CAM3_W - 1)
                        py2 = np.clip(pixels[new_rgb, 1].astype(int), 0, CAM3_H - 1)
                        s2 = c3_img[py2, px2]
                        rgb[new_rgb, 0] = s2[:, 0]
                        rgb[new_rgb, 1] = s2[:, 1]
                        rgb[new_rgb, 2] = s2[:, 2]
                    has_rgb |= valid

        # Transform to map frame
        xyz_map = (T_world[:3, :3] @ xyz.T).T + T_world[:3, 3]

        # Georeference to BNG if calibrated
        if self._is_georeferenced():
            xyz_out = self._slam_to_bng(xyz_map)
            crs_wkt = self._BNG_WKT
            crs_label = 'BNG'
        else:
            xyz_out = xyz_map
            crs_wkt = None
            crs_label = 'SLAM'

        # Build scalar fields
        scalars = {
            'thermal': thermal_raw,
            'thermal1_left': t1_raw,
            'thermal2_right': t2_raw,
            'rgb_r': rgb_r,
            'rgb_g': rgb_g,
            'rgb_b': rgb_b,
        }
        scalars.update(lidar_scalars)

        # Save as LAZ — full raw density + all LiDAR values + camera colorization
        heading_val = heading if heading is not None else 0.0
        fname = f"{timestamp_to_filename(ts_ns)}_slam_colorized"
        out_dir = self.session_dir / 'images' / 'slam_cloud_colorized'
        laz_path = out_dir / f"{fname}.laz"

        # Async write — copy arrays to prevent mutation during background write
        xyz_copy = xyz_out.copy()
        rgb_copy = rgb.copy()
        scalars_copy = {k: v.copy() for k, v in scalars.items()}
        acc_meta = self._get_accuracy_meta()
        self._submit_io(
            lambda p=laz_path, x=xyz_copy, r=rgb_copy, s=scalars_copy, c=crs_wkt, a=acc_meta:
                atomic_write_file(p, lambda tp: self._write_laz_colorized(tp, x, r, s, crs_wkt=c, accuracy_meta=a)))

        # Update state (immediately, don't wait for write)
        self._last_colorized_bng = (bng_x, bng_y)
        self._last_colorized_save = now

        t_pct = has_thermal.sum() / n * 100 if n > 0 else 0
        r_pct = has_rgb.sum() / n * 100 if n > 0 else 0
        self.get_logger().info(
            f'Dense cloud ({crs_label}): {n:,} pts, {t_pct:.0f}% thermal, '
            f'{r_pct:.0f}% RGB -> {fname}.laz [async]')

        # Metadata sidecar (includes accuracy tags)
        self._write_meta(
            out_dir / f"{fname}.meta.json",
            ts_ns, lat, lon, bng_x, bng_y, heading_val, 'slam_cloud_colorized',
            num_points=n,
            thermal_coverage_pct=round(t_pct, 1),
            rgb_coverage_pct=round(r_pct, 1),
            accuracy=acc_meta,
        )

        with self.lock:
            self.asset_counts['slam_cloud_colorized'] += 1
            self.latest_assets['slam_cloud_colorized'] = f"{fname}.laz"

    @staticmethod
    def _write_laz_colorized(path, xyz, rgb, scalars=None, crs_wkt=None,
                             accuracy_meta=None):
        """Write colorized point cloud as LAZ (LAS 1.4 format 7) with RGB + scalars.

        accuracy_meta: optional dict with accuracy tags (heading_std, gnss_fix, etc.)
        """
        header = laspy.LasHeader(point_format=7, version="1.4")
        header.offsets = np.min(xyz, axis=0)
        header.scales = [0.01, 0.01, 0.01]  # 1cm precision — 0.001 overflows int32 with BNG coords
        if crs_wkt:
            try:
                vlr = laspy.VLR(
                    user_id="LASF_Projection",
                    record_id=2112,
                    description="OGC Coordinate System WKT",
                    record_data=crs_wkt.encode('utf-8') + b'\x00',
                )
                header.vlrs.append(vlr)
            except Exception:
                pass
        # Accuracy metadata as JSON VLR (BESS custom)
        if accuracy_meta:
            try:
                import json as _json
                vlr = laspy.VLR(
                    user_id="BESS",
                    record_id=1,
                    description="BESS accuracy metadata",
                    record_data=_json.dumps(accuracy_meta).encode('utf-8') + b'\x00',
                )
                header.vlrs.append(vlr)
            except Exception:
                pass
        if scalars:
            for name in scalars:
                header.add_extra_dim(laspy.ExtraBytesParams(name=name, type=np.float32))
        las = laspy.LasData(header)
        las.x = xyz[:, 0]
        las.y = xyz[:, 1]
        las.z = xyz[:, 2]
        # LAS RGB is 16-bit; input is BGR uint8
        las.red = rgb[:, 2].astype(np.uint16) * 256
        las.green = rgb[:, 1].astype(np.uint16) * 256
        las.blue = rgb[:, 0].astype(np.uint16) * 256
        if scalars:
            for name, arr in scalars.items():
                setattr(las, name, arr.astype(np.float32))
        las.write(str(path), laz_backend=laspy.LazBackend.LazrsParallel)

    def _slam_to_bng(self, xyz):
        """Transform SLAM-frame points to BNG (EPSG:27700) using heading offset + origin.

        Returns transformed xyz array, or original if transform not yet calibrated.
        """
        if (self._slam_heading_offset is None
                or self._slam_origin_bng is None):
            return xyz  # not calibrated yet

        theta = math.radians(self._slam_heading_offset)
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        ox, oy, oz = self._slam_origin_bng

        out = np.empty_like(xyz)
        out[:, 0] = cos_t * xyz[:, 0] - sin_t * xyz[:, 1] + ox
        out[:, 1] = sin_t * xyz[:, 0] + cos_t * xyz[:, 1] + oy
        out[:, 2] = xyz[:, 2] + oz
        return out

    def _is_georeferenced(self):
        """True if SLAM→BNG transform is calibrated."""
        return (self._slam_heading_offset is not None
                and self._slam_origin_bng is not None)

    def _get_accuracy_meta(self):
        """Build per-cloud accuracy metadata dict for VLR embedding."""
        return {
            'heading_offset_deg': round(self._slam_heading_offset, 2) if self._slam_heading_offset is not None else None,
            'heading_std_deg': round(self._heading_offset_std, 2),
            'heading_converged': self._heading_converged,
            'heading_samples': self._heading_offset_n,
            'heading_source': self._heading_source,
            'gnss_fix_status': self._gnss_fix_status,
            'gnss_h_covariance_m2': round(self._gnss_h_covariance, 4),
            'georeferenced': self._is_georeferenced(),
            'slam_source': next(
                (s for s in ['glim', 'fast_lio', 'kiss_icp']
                 if s in self._slam_yaw), 'none'),
            'vehicle': self.rig_id,
            'pipeline_version': 'v11',
        }

    # BNG WKT for LAS CRS tagging (EPSG:27700 — OSGB 1936 / British National Grid)
    _BNG_WKT = (
        'PROJCS["OSGB 1936 / British National Grid",'
        'GEOGCS["OSGB 1936",DATUM["OSGB_1936",'
        'SPHEROID["Airy 1830",6377563.396,299.3249646]],'
        'PRIMEM["Greenwich",0],UNIT["degree",0.0174532925199433]],'
        'PROJECTION["Transverse_Mercator"],'
        'PARAMETER["latitude_of_origin",49],'
        'PARAMETER["central_meridian",-2],'
        'PARAMETER["scale_factor",0.9996012717],'
        'PARAMETER["false_easting",400000],'
        'PARAMETER["false_northing",-100000],'
        'UNIT["metre",1]]'
    )

    @staticmethod
    def _write_las(path, xyz, intensity=None, crs_wkt=None, accuracy_meta=None):
        """Write point cloud as LAS 1.4 with optional intensity and CRS."""
        # Filter NaN/inf points
        valid = np.isfinite(xyz).all(axis=1)
        if not valid.all():
            xyz = xyz[valid]
            if intensity is not None:
                intensity = intensity[valid]
        if len(xyz) == 0:
            return

        header = laspy.LasHeader(point_format=6, version="1.4")
        header.offsets = np.min(xyz, axis=0).astype(np.float64)
        # Use 1cm scale to avoid int32 overflow with large BNG coordinates
        header.scales = np.array([0.01, 0.01, 0.01])

        if crs_wkt:
            try:
                vlr = laspy.VLR(
                    user_id="LASF_Projection",
                    record_id=2112,
                    description="OGC Coordinate System WKT",
                    record_data=crs_wkt.encode('utf-8') + b'\x00',
                )
                header.vlrs.append(vlr)
            except Exception:
                pass
        if accuracy_meta:
            try:
                import json as _json
                vlr = laspy.VLR(
                    user_id="BESS",
                    record_id=1,
                    description="BESS accuracy metadata",
                    record_data=_json.dumps(accuracy_meta).encode('utf-8') + b'\x00',
                )
                header.vlrs.append(vlr)
            except Exception:
                pass

        las = laspy.LasData(header)
        las.x = xyz[:, 0].astype(np.float64)
        las.y = xyz[:, 1].astype(np.float64)
        las.z = xyz[:, 2].astype(np.float64)

        if intensity is not None:
            # LAS intensity is uint16; scale float to 0-65535
            i_min, i_max = intensity.min(), intensity.max()
            if i_max > i_min:
                las.intensity = ((intensity - i_min) / (i_max - i_min) * 65535).astype(np.uint16)
            else:
                las.intensity = np.zeros(len(intensity), dtype=np.uint16)

        las.write(str(path))

    def _write_session_cloud(self):
        """Write session_cloud.laz from the latest accumulated SLAM map.

        If SLAM→BNG transform is calibrated, writes in BNG (EPSG:27700).
        Otherwise falls back to SLAM local frame.
        """
        with self._accumulated_map_lock:
            if self._accumulated_map is None:
                return
            all_xyz, all_int, source, ts = self._accumulated_map

        if len(all_xyz) < 100:
            return

        # Apply SLAM→BNG transform if calibrated
        georef = self._is_georeferenced()
        if georef:
            out_xyz = self._slam_to_bng(all_xyz)
            crs_wkt = self._BNG_WKT
            suffix = 'BNG'
        else:
            out_xyz = all_xyz
            crs_wkt = None
            suffix = 'SLAM'

        session_laz = self.session_dir / 'session_cloud.laz'
        tmp = session_laz.with_suffix('.tmp.laz')
        try:
            self._write_las(tmp, out_xyz, all_int, crs_wkt=crs_wkt)
            tmp.rename(session_laz)
            self.get_logger().info(
                f'Session cloud ({source}, {suffix}): {len(out_xyz):,} points -> {session_laz.name}'
            )

            # Write numbered segment if enough new points accumulated
            new_points = len(out_xyz) - self._session_cloud_last_segment_points
            if georef and new_points >= self._SESSION_CLOUD_SEGMENT_MIN_POINTS:
                self._session_cloud_segment_num += 1
                seg_name = f'session_cloud_{self._session_cloud_segment_num:03d}_{suffix}.laz'
                seg_path = self.session_dir / seg_name
                seg_tmp = seg_path.with_suffix('.tmp.laz')
                self._write_las(seg_tmp, out_xyz, all_int, crs_wkt=crs_wkt)
                seg_tmp.rename(seg_path)
                self._session_cloud_last_segment_points = len(out_xyz)
                self.get_logger().info(
                    f'Session cloud segment {self._session_cloud_segment_num}: '
                    f'{len(out_xyz):,} points -> {seg_name}'
                )
        except Exception as e:
            self.get_logger().error(f'Session cloud write failed: {e}')
            if tmp.exists():
                tmp.unlink()


    @staticmethod
    def _parse_pointcloud2(msg):
        """Extract xyz (and optionally intensity) from PointCloud2 message."""
        fields = {f.name: f for f in msg.fields}
        if 'x' not in fields or 'y' not in fields or 'z' not in fields:
            return None, None

        n_points = msg.width * msg.height
        if n_points == 0:
            return None, None

        # Build structured numpy dtype from field descriptors
        sorted_fields = sorted(msg.fields, key=lambda f: f.offset)
        dtype_list = []
        prev_end = 0
        for f in sorted_fields:
            # Insert padding bytes if there's a gap between fields
            if f.offset > prev_end:
                dtype_list.append((f'_pad{prev_end}', np.uint8, f.offset - prev_end))
            np_dtype = POINTFIELD_DTYPE.get(f.datatype, np.float32)
            dtype_list.append((f.name, np_dtype))
            prev_end = f.offset + np.dtype(np_dtype).itemsize

        # Pad to match point_step
        if msg.point_step > prev_end:
            dtype_list.append(('_pad_end', np.uint8, msg.point_step - prev_end))

        try:
            cloud = np.frombuffer(msg.data, dtype=np.dtype(dtype_list), count=n_points)
        except Exception:
            return None, None

        xyz = np.column_stack([
            cloud['x'].astype(np.float32),
            cloud['y'].astype(np.float32),
            cloud['z'].astype(np.float32),
        ])

        intensity = None
        if 'intensity' in fields:
            intensity = cloud['intensity'].astype(np.float32)

        # Filter out NaN/inf points
        valid = np.isfinite(xyz).all(axis=1)
        if not valid.all():
            xyz = xyz[valid]
            if intensity is not None:
                intensity = intensity[valid]

        return xyz, intensity

    # ─── UPRN MATCHING ─────────────────────────────────────────────

    def _match_uprns(self, ts_ns, vx, vy, heading, camera_side):
        """Find visible UPRNs from current position and camera."""
        nearby = self.spatial_db.query_nearby(vx, vy)

        for uprn_row in nearby:
            ux, uy = uprn_row['x'], uprn_row['y']
            dist = calc_distance_bng(vx, vy, ux, uy)

            if dist > MAX_VISIBILITY_DISTANCE_M:
                continue

            bearing = calc_bearing_bng(vx, vy, ux, uy)
            # (heading - bearing): 90° = LEFT of vehicle, 270° = RIGHT
            relative_angle = (heading - bearing + 360) % 360

            if not is_in_fov(relative_angle, camera_side):
                continue

            perp_angle = calc_perpendicular_angle(relative_angle, camera_side)
            is_perp = perp_angle < PERPENDICULAR_THRESHOLD_DEG

            uprn_id = str(uprn_row['uprn'])

            self.visibility.append(VisibilityRecord(
                frame_timestamp_ns=ts_ns,
                uprn=uprn_id,
                camera_side=camera_side,
                distance_m=round(dist, 2),
                bearing_deg=round(bearing, 2),
                angle_from_perpendicular_deg=round(perp_angle, 2),
                is_perpendicular=is_perp,
            ))

            # Update hero tracker
            score = perp_angle + abs(dist - OPTIMAL_DISTANCE_CENTER_M) * 0.3
            if uprn_id not in self.heroes:
                self.heroes[uprn_id] = HeroTracker(
                    uprn=uprn_id,
                    address=uprn_row.get('fulladdress') or '',
                    building_height_m=uprn_row.get('building_height_m') or 0.0,
                    building_use=uprn_row.get('building_use') or '',
                )

            hero = self.heroes[uprn_id]
            hero.visible_count += 1
            if is_perp:
                hero.perpendicular_count += 1
            if score < hero.best_score:
                hero.best_score = score
                hero.best_timestamp_ns = ts_ns
                hero.best_camera_side = camera_side
                hero.best_distance_m = round(dist, 2)
                hero.best_perp_angle = round(perp_angle, 2)
                # Snapshot current asset filenames for hero association
                hero.hero_assets = dict(self.latest_assets)

    # ─── FLUSH ─────────────────────────────────────────────────────

    def _flush(self):
        """Periodically write outputs to disk."""
        if not self.session_dir:
            return

        with self.lock:
            traj = list(self.trajectory)
            vis = list(self.visibility)
            heroes = dict(self.heroes)
            counts = dict(self.asset_counts)
            cloud_count = self.slam_cloud_count
            cloud_points = self.slam_cloud_total_points

        if not traj:
            return

        try:
            self._write_trajectory_bng(traj)
            self._write_trajectory_wgs84(traj)
            self._write_frame_visibility(traj, vis, heroes, counts)
            self._write_extraction_summary(traj, counts, cloud_count, cloud_points)
            self._write_validation_overlay(traj, heroes)
            self._update_supabase(traj, counts, heroes)
        except Exception as e:
            self.get_logger().error(f'Flush failed: {e}')

        # Write session cloud every 2 minutes from accumulated SLAM map
        now = time.time()
        if HAS_LASPY and (now - self._last_session_cloud_write) > 120:
            try:
                self._write_session_cloud()
                self._last_session_cloud_write = now
            except Exception as e:
                self.get_logger().error(f'Session cloud failed: {e}')

        self.last_flush = time.time()

    def _write_trajectory_bng(self, traj):
        """Write BNG trajectory: timestamp_ns x y z heading_deg"""
        path = self.session_dir / 'trajectory_bng.txt'
        tmp = path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            for p in traj:
                if not self._valid_coord(p):
                    continue
                f.write(f"{p.timestamp_ns} {p.x:.2f} {p.y:.2f} {p.z:.2f} {p.heading_deg:.2f}\n")
        tmp.rename(path)

    @staticmethod
    def _valid_coord(p):
        """Return True if trajectory point has valid (non-zero) coordinates."""
        return abs(p.lat) > 0.1 and abs(p.lon) > 0.0001

    def _write_trajectory_wgs84(self, traj):
        """Write WGS84 trajectory as JSON."""
        path = self.session_dir / 'trajectory_wgs84.json'
        tmp = path.with_suffix('.tmp')
        records = [{
            'timestamp_ns': p.timestamp_ns,
            'lat': round(p.lat, 8),
            'lon': round(p.lon, 8),
            'alt': round(p.alt, 2),
            'heading': round(p.heading_deg, 2),
        } for p in traj if self._valid_coord(p)]
        with open(tmp, 'w') as f:
            json.dump(records, f)
        tmp.rename(path)

    def _write_frame_visibility(self, traj, vis, heroes, counts):
        """Write frame_visibility.json with asset filenames per hero."""
        path = self.session_dir / 'frame_visibility.json'
        tmp = path.with_suffix('.tmp')

        # Trajectory bounding box
        xs = [p.x for p in traj]
        ys = [p.y for p in traj]
        bbox = {
            'xmin': round(min(xs), 2), 'ymin': round(min(ys), 2),
            'xmax': round(max(xs), 2), 'ymax': round(max(ys), 2),
        }

        # Unique UPRNs seen
        unique_uprns = set(v.uprn for v in vis)

        # Frame info (deduplicate by timestamp)
        seen_ts = set()
        frames = []
        for p in traj:
            if p.timestamp_ns in seen_ts:
                continue
            seen_ts.add(p.timestamp_ns)
            frames.append({
                'timestamp_ns': p.timestamp_ns,
                'x': round(p.x, 2),
                'y': round(p.y, 2),
                'heading_deg': round(p.heading_deg, 2),
            })

        # Hero shots with asset filenames
        hero_records = {}
        for uprn_id, h in heroes.items():
            if h.best_timestamp_ns == 0:
                continue
            record = {
                'uprn': uprn_id,
                'hero_rgb_timestamp_ns': h.best_timestamp_ns,
                'camera_side': h.best_camera_side,
                'distance_m': h.best_distance_m,
                'angle_from_perpendicular_deg': h.best_perp_angle,
                'visible_frame_count': h.visible_count,
                'perpendicular_frame_count': h.perpendicular_count,
                'address': h.address,
                'building_height_m': h.building_height_m,
                'building_use': h.building_use,
            }
            # Add hero asset filenames (hero_left_rgb, hero_left_thermal, etc.)
            for asset_type, filename in h.hero_assets.items():
                record[f'hero_{asset_type}'] = filename
            hero_records[f"UPRN_{uprn_id}"] = record

        # Visibility records
        vis_records = [{
            'frame_timestamp_ns': v.frame_timestamp_ns,
            'uprn': v.uprn,
            'camera_side': v.camera_side,
            'distance_m': v.distance_m,
            'bearing_deg': v.bearing_deg,
            'angle_from_perpendicular_deg': v.angle_from_perpendicular_deg,
            'is_perpendicular': v.is_perpendicular,
        } for v in vis]

        output = {
            'metadata': {
                'session_name': self.session_name,
                'rig_id': self.rig_id,
                'generated_at': datetime.now(timezone.utc).isoformat(),
                'generator': 'bess-extraction-v2',
            },
            'statistics': {
                'total_frames': len(frames),
                'total_uprns': len(unique_uprns),
                'total_visibility_records': len(vis),
                'uprns_with_hero_shots': len(hero_records),
                'assets': dict(counts),
            },
            'trajectory_bbox': bbox,
            'frames': frames,
            'heroes': hero_records,
            'visibility': vis_records,
        }

        # Use compact JSON for large datasets to save memory during serialization
        indent = 2 if len(frames) < 50000 else None
        with open(tmp, 'w') as f:
            json.dump(output, f, indent=indent)
        tmp.rename(path)

        self.get_logger().info(
            f'Flush: {len(frames)} frames, {len(unique_uprns)} UPRNs, '
            f'{len(hero_records)} heroes, assets={dict(counts)}'
        )

    def _write_validation_overlay(self, traj, heroes):
        """Write GeoJSON validation overlay + HTML viewer for trajectory/UPRN QA."""
        # GeoJSON with trajectory LineString + UPRN hero Points
        features = []

        # Trajectory as LineString (WGS84: [lon, lat])
        if len(traj) >= 2:
            # Subsample to avoid huge files (max 2000 points for the line)
            step = max(1, len(traj) // 2000)
            coords = [[round(p.lon, 7), round(p.lat, 7)] for p in traj[::step]
                       if self._valid_coord(p)]
            features.append({
                'type': 'Feature',
                'geometry': {'type': 'LineString', 'coordinates': coords},
                'properties': {
                    'name': 'trajectory',
                    'session': self.session_name,
                    'points': len(traj),
                    'heading_source': self._heading_source,
                },
            })

        # Heading arrows every ~50 trajectory points (shows FOV direction)
        arrow_step = max(1, len(traj) // 100)
        for p in traj[::arrow_step]:
            if p.heading_deg == 0.0 or not self._valid_coord(p):
                continue
            # Arrow endpoint ~20m in heading direction
            arrow_len = 0.0002  # ~20m in degrees
            end_lon = p.lon + arrow_len * math.sin(math.radians(p.heading_deg))
            end_lat = p.lat + arrow_len * math.cos(math.radians(p.heading_deg))
            features.append({
                'type': 'Feature',
                'geometry': {
                    'type': 'LineString',
                    'coordinates': [
                        [round(p.lon, 7), round(p.lat, 7)],
                        [round(end_lon, 7), round(end_lat, 7)],
                    ],
                },
                'properties': {
                    'name': 'heading_arrow',
                    'heading_deg': round(p.heading_deg, 1),
                },
            })

        # UPRN hero points
        for uprn_id, h in heroes.items():
            if h.best_timestamp_ns == 0:
                continue
            # Find the trajectory point closest to the hero timestamp
            best_traj = min(traj, key=lambda p: abs(p.timestamp_ns - h.best_timestamp_ns))
            # Get UPRN location from spatial DB
            uprn_info = None
            if self.spatial_db.available:
                rows = self.spatial_db.query_nearby(best_traj.x, best_traj.y)
                for row in rows:
                    if str(row['uprn']) == uprn_id:
                        uprn_info = row
                        break

            if uprn_info:
                # Convert BNG to approx WGS84 for GeoJSON
                try:
                    lon, lat = self.transformer.transform(
                        uprn_info['x'], uprn_info['y'], direction='INVERSE'
                    )
                except Exception:
                    continue

                features.append({
                    'type': 'Feature',
                    'geometry': {'type': 'Point', 'coordinates': [round(lon, 7), round(lat, 7)]},
                    'properties': {
                        'name': f'UPRN_{uprn_id}',
                        'address': h.address,
                        'distance_m': h.best_distance_m,
                        'perp_angle': h.best_perp_angle,
                        'camera': h.best_camera_side,
                        'visible_count': h.visible_count,
                        'building_use': h.building_use,
                    },
                })

        geojson = {'type': 'FeatureCollection', 'features': features}

        path = self.session_dir / 'validation_overlay.geojson'
        tmp = path.with_suffix('.tmp')
        with open(tmp, 'w') as f:
            json.dump(geojson, f)
        tmp.rename(path)

        # Write HTML viewer (once per session)
        html_path = self.session_dir / 'validation.html'
        if not html_path.exists():
            self._write_validation_html(html_path)

    def _write_validation_html(self, html_path):
        """Write Leaflet.js HTML viewer for the validation GeoJSON overlay."""
        html = """<!DOCTYPE html>
<html><head>
<meta charset="utf-8">
<title>BESS Extraction Validation</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9/dist/leaflet.css"/>
<script src="https://unpkg.com/leaflet@1.9/dist/leaflet.js"></script>
<style>
body{margin:0;font-family:monospace}
#map{height:100vh}
#info{position:absolute;top:10px;right:10px;z-index:1000;background:rgba(0,0,0,0.8);
      color:#fff;padding:12px;border-radius:6px;font-size:13px;max-width:350px}
#info h3{margin:0 0 8px 0;color:#4fc3f7}
.legend{position:absolute;bottom:30px;left:10px;z-index:1000;background:rgba(0,0,0,0.8);
        color:#fff;padding:10px;border-radius:6px;font-size:12px}
.legend i{display:inline-block;width:14px;height:14px;margin-right:6px;vertical-align:middle}
</style>
</head><body>
<div id="map"></div>
<div id="info"><h3>BESS Extraction QA</h3><div id="stats">Loading...</div></div>
<div class="legend">
  <div><i style="background:#e53935"></i> Trajectory</div>
  <div><i style="background:#ffb300"></i> Heading arrows</div>
  <div><i style="background:#43a047;border-radius:50%"></i> UPRN hero</div>
</div>
<script>
const map = L.map('map');
L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: 'OSM', maxZoom: 20
}).addTo(map);

// Satellite layer toggle
const sat = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {attribution: 'Esri', maxZoom: 20}
);
L.control.layers({'Street': L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png').addTo(map),
                   'Satellite': sat}).addTo(map);

fetch('./validation_overlay.geojson')
  .then(r => r.json())
  .then(data => {
    let trajPoints = 0, uprns = 0;
    const layer = L.geoJSON(data, {
      style: function(f) {
        if (f.properties.name === 'trajectory')
          return {color: '#e53935', weight: 3, opacity: 0.9};
        if (f.properties.name === 'heading_arrow')
          return {color: '#ffb300', weight: 2, opacity: 0.7};
        return {};
      },
      pointToLayer: function(f, ll) {
        uprns++;
        return L.circleMarker(ll, {
          radius: 7, fillColor: '#43a047', color: '#fff',
          weight: 2, fillOpacity: 0.9
        });
      },
      onEachFeature: function(f, l) {
        if (f.properties.name === 'trajectory') {
          trajPoints = f.properties.points;
          l.bindPopup('<b>Trajectory</b><br>Points: ' + f.properties.points +
                      '<br>Heading: ' + f.properties.heading_source);
        } else if (f.properties.name && f.properties.name.startsWith('UPRN_')) {
          const p = f.properties;
          l.bindPopup(
            '<b>' + p.name + '</b><br>' +
            p.address + '<br>' +
            'Distance: ' + p.distance_m + 'm<br>' +
            'Perp angle: ' + p.perp_angle + '&deg;<br>' +
            'Camera: ' + p.camera + '<br>' +
            'Visible: ' + p.visible_count + ' frames<br>' +
            'Use: ' + (p.building_use || 'unknown')
          );
        }
      }
    }).addTo(map);
    map.fitBounds(layer.getBounds(), {padding: [20, 20]});
    document.getElementById('stats').innerHTML =
      'Trajectory: ' + trajPoints + ' pts<br>' +
      'UPRNs matched: ' + uprns + '<br>' +
      '<small>Click features for details</small>';
  })
  .catch(e => {
    document.getElementById('stats').innerHTML = 'Error loading GeoJSON: ' + e;
  });
</script>
</body></html>"""
        try:
            html_path.write_text(html)
        except Exception:
            pass

    def _write_extraction_summary(self, traj, counts, cloud_count, cloud_points):
        """Write extraction_summary.json with all asset type counts."""
        path = self.session_dir / 'extraction_summary.json'
        tmp = path.with_suffix('.tmp')

        xs = [p.x for p in traj]
        ys = [p.y for p in traj]

        summary = {
            'session_name': self.session_name,
            'rig_id': self.rig_id,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'generator': 'bess-extraction-v2',
            'assets': dict(counts),
            'total_images': sum(v for k, v in counts.items() if k != 'slam_cloud'),
            'total_frames': len(traj),
            'uprns_matched': len(self.heroes),
            'slam_cloud_count': cloud_count,
            'slam_cloud_total_points': cloud_points,
            'bbox_bng': {
                'xmin': round(min(xs), 2), 'ymin': round(min(ys), 2),
                'xmax': round(max(xs), 2), 'ymax': round(max(ys), 2),
            },
            'spatial_db_available': self.spatial_db.available,
            'ambient_temp_c': self.ambient_temp_c,
            'heading_source': self._heading_source,
            'gnss_fix_status': self._gnss_fix_status,
            'gnss_h_covariance_m2': round(self._gnss_h_covariance, 4),
            'heading_offset_deg': round(self._slam_heading_offset, 2) if self._slam_heading_offset is not None else None,
            'heading_offset_std_deg': round(self._heading_offset_std, 2),
            'heading_converged': self._heading_converged,
            'heading_offset_rtk_samples': self._heading_offset_n,
            'georeferenced': self._is_georeferenced(),
            'crs': 'EPSG:27700' if self._is_georeferenced() else 'SLAM_local',
            'slam_origin_bng': {
                'easting': round(self._slam_origin_bng[0], 3),
                'northing': round(self._slam_origin_bng[1], 3),
                'altitude': round(self._slam_origin_bng[2], 3),
            } if self._slam_origin_bng else None,
        }

        with open(tmp, 'w') as f:
            json.dump(summary, f, indent=2)
        tmp.rename(path)

    def _update_supabase(self, traj, counts, heroes):
        """Update Supabase recordings table with extraction progress."""
        if not self.supabase_url or not self.supabase_key:
            return

        try:
            xs = [p.x for p in traj]
            ys = [p.y for p in traj]

            payload = {
                'vehicle_id': self.rig_id,
                'session_name': self.session_name,
                'extraction_images': sum(counts.values()),
                'extraction_frames': len(traj),
                'extraction_uprns': len(heroes),
                'extraction_bbox': json.dumps({
                    'xmin': round(min(xs), 2), 'ymin': round(min(ys), 2),
                    'xmax': round(max(xs), 2), 'ymax': round(max(ys), 2),
                }),
                'extraction_updated_at': datetime.now(timezone.utc).isoformat(),
            }

            data = json.dumps(payload).encode('utf-8')
            url = f"{self.supabase_url}/rest/v1/extraction_sessions"
            req = Request(url, data=data, method='POST')
            req.add_header('apikey', self.supabase_key)
            req.add_header('Authorization', f'Bearer {self.supabase_key}')
            req.add_header('Content-Type', 'application/json')
            req.add_header('Prefer', 'resolution=merge-duplicates')

            urlopen(req, timeout=5)
        except Exception as e:
            self.get_logger().debug(f'Supabase update failed: {e}')


# =============================================================================
# MAIN
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = FrameExtractor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Drain IO thread pool before final flush
        try:
            node.get_logger().info(f'Draining IO pool ({node._io_pending} pending writes)...')
            node._io_pool.shutdown(wait=True, cancel_futures=False)
            node.get_logger().info('IO pool drained')
        except Exception as e:
            node.get_logger().error(f'IO pool drain failed: {e}')
        # Final flush — try full flush first, fallback to trajectory-only
        try:
            node._flush()
            node.get_logger().info('Final flush completed')
        except Exception as e:
            node.get_logger().error(f'FINAL FLUSH FAILED: {e}')
            # Try minimal save — just trajectory (most critical data)
            try:
                with node.lock:
                    traj = list(node.trajectory)
                if traj:
                    node._write_trajectory_wgs84(traj)
                    node.get_logger().info(f'Emergency trajectory save: {len(traj)} points')
            except Exception as e2:
                node.get_logger().error(f'EMERGENCY SAVE ALSO FAILED: {e2}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
