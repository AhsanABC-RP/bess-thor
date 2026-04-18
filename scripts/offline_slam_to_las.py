#!/usr/bin/env python3
"""
Offline SLAM point cloud exporter — reads recorded bags, uses FAST-LIO2
odometry poses to transform raw Ouster scans into map frame, outputs
LAS 1.4 (local viewing) and COPC (web frontend).

Preserves ALL Ouster fields: x,y,z, intensity, reflectivity, ambient (near-IR),
range, ring, t (per-point timestamp).

Usage:
    python3 offline_slam_to_las.py /mnt/bess-usb/bags/rolling/bag \
        --output /mnt/bess-usb/bags/rolling/site_trial \
        --voxel 0.05 \
        --max-range 80.0

Requires: mcap, mcap-ros2-decoder, laspy, copclib, numpy, scipy
"""

import argparse
import glob
import os
import re
import struct
import sys
import time

import numpy as np
from scipy.spatial.transform import Rotation

def parse_args():
    p = argparse.ArgumentParser(description="Offline SLAM → LAS/COPC exporter")
    p.add_argument("bag_dir", help="Directory containing bag_*.mcap files")
    p.add_argument("--output", "-o", default=None, help="Output directory (default: <bag_dir>/export)")
    p.add_argument("--voxel", type=float, default=0.0, help="Voxel downsample size in meters (0=no downsample)")
    p.add_argument("--max-range", type=float, default=100.0, help="Max range filter in meters")
    p.add_argument("--min-range", type=float, default=1.0, help="Min range filter in meters")
    p.add_argument("--skip-copc", action="store_true", help="Skip COPC generation")
    p.add_argument("--max-files", type=int, default=0, help="Max mcap files to process (0=all)")
    p.add_argument("--odom-topic", default="/fast_lio/odometry", help="Odometry topic")
    p.add_argument("--cloud-topic", default="/ouster/points", help="Raw point cloud topic")
    p.add_argument("--odom-bag-dir", default=None, help="Separate dir for odometry bags (default: same as bag_dir). Use when clean offline SLAM output is in a different dir than the raw /ouster/points bags.")
    # Body<-LiDAR extrinsic. Odometry child_frame_id is the SLAM body/IMU frame;
    # raw /ouster/points is in LiDAR frame. p_world = T_odom @ T_body_lidar @ p_lidar.
    # Default below matches containers/fast-lio/fast_lio_single.yaml (180 deg Z flip
    # for Ouster BMI085). DLIO uses identity — pass --extrinsic-preset dlio.
    p.add_argument("--extrinsic-preset", choices=["fast_lio", "dlio", "identity"], default="fast_lio",
                   help="Body<-LiDAR extrinsic preset (fast_lio=180degZ flip, dlio/identity=no flip)")
    p.add_argument("--extrinsic-R", type=float, nargs=9, default=None,
                   help="Override 3x3 rotation body<-lidar (row-major, 9 floats)")
    p.add_argument("--extrinsic-T", type=float, nargs=3, default=None,
                   help="Override 3x1 translation body<-lidar (3 floats, meters)")
    p.add_argument("--max-speed", type=float, default=30.0,
                   help="Audit-gate p99 speed threshold in m/s (default 30). Raise to 50 for highway bags (UK motorway ~31 m/s).")
    p.add_argument("--max-extent", type=float, default=2000.0,
                   help="Audit-gate per-axis extent threshold in m (default 2000).")
    p.add_argument("--force", action="store_true",
                   help="Downgrade audit FAIL to WARN and proceed with LAS export anyway. Use only when you already understand the divergence (e.g. known FAST-LIO2 Z drift).")
    p.add_argument("--deskew-bins", type=int, default=64,
                   help="Number of time buckets per 100 ms scan for per-point "
                        "de-skew (default 64 → ~1.5 ms bucket → ~2 cm residual "
                        "at 15 m/s peak; 16 → 6 ms bucket → 9 cm residual, "
                        "visible as lamp-post/thin-vertical ghosting).")
    return p.parse_args()


def build_body_lidar_extrinsic(args):
    """Return 4x4 T_body_lidar from CLI args/preset. body = SLAM odom child frame."""
    if args.extrinsic_R is not None or args.extrinsic_T is not None:
        R = np.array(args.extrinsic_R if args.extrinsic_R else [1,0,0, 0,1,0, 0,0,1]).reshape(3,3)
        t = np.array(args.extrinsic_T if args.extrinsic_T else [0.0, 0.0, 0.0])
    elif args.extrinsic_preset == "fast_lio":
        # From containers/fast-lio/fast_lio_single.yaml
        R = np.array([[-1.0, 0.0, 0.0],
                      [ 0.0,-1.0, 0.0],
                      [ 0.0, 0.0, 1.0]])
        t = np.array([0.0024, 0.0097, 0.0307])
    elif args.extrinsic_preset == "dlio":
        # From config/dlio/dlio_thor.yaml extrinsics/baselink2lidar after the
        # 2026-04-18 Rz(180°) fix. DLIO's baselink = os_sensor; /ouster/points
        # arrives in os_lidar frame, which is os_sensor rotated 180° about Z
        # with +38.2 mm Z offset. Identity here = 180° residual in the first
        # pose = world-frame yaw divergence from FAST-LIO2.
        R = np.array([[-1.0, 0.0, 0.0],
                      [ 0.0,-1.0, 0.0],
                      [ 0.0, 0.0, 1.0]])
        t = np.array([0.0, 0.0, 0.0382])
    else:  # identity
        R = np.eye(3)
        t = np.zeros(3)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def _parse_odometry_cdr(data):
    """Parse nav_msgs/Odometry CDR payload → (ts_ns, xyz, qxyzw).

    Uses StreamReader-compatible raw bytes, so works on truncated MCAPs
    where make_reader + get_summary() fail (recorder SIGTERM'd mid-write).
    """
    body = data[4:]  # skip 4-byte CDR encapsulation header
    off = 0
    sec, nanosec = struct.unpack_from('<iI', body, off); off += 8
    # header.frame_id string (length + bytes, align length field to 4)
    flen = struct.unpack_from('<I', body, off)[0]; off += 4
    off += flen
    off = (off + 3) & ~3
    # child_frame_id string
    clen = struct.unpack_from('<I', body, off)[0]; off += 4
    off += clen
    # align to 8 for float64 (Point.x)
    off = (off + 7) & ~7
    px, py, pz, qx, qy, qz, qw = struct.unpack_from('<7d', body, off)
    return sec * 10**9 + nanosec, (px, py, pz), (qx, qy, qz, qw)


def load_odometry(files, odom_topic, max_speed_mps=30.0, max_extent_m=2000.0, force=False):
    """Extract all odometry poses as (timestamp_ns, 4x4 matrix) pairs.

    Streaming read via StreamReader so truncated/un-indexed MCAP files
    (e.g. ros2 bag record SIGTERM'd during shutdown) still yield their
    recorded odom messages up to the truncation point.
    """
    from mcap.stream_reader import StreamReader

    poses = []
    for f in files:
        channels = {}
        target_ids = set()
        with open(f, "rb") as fh:
            reader = StreamReader(fh)
            try:
                for record in reader.records:
                    rtype = type(record).__name__
                    if rtype == 'Channel':
                        channels[record.id] = record
                        if record.topic == odom_topic:
                            target_ids.add(record.id)
                    elif rtype == 'Message':
                        if record.channel_id not in target_ids:
                            continue
                        try:
                            ts, (x, y, z), (qx, qy, qz, qw) = _parse_odometry_cdr(record.data)
                        except Exception:
                            continue
                        rot = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()
                        T = np.eye(4)
                        T[:3, :3] = rot
                        T[:3, 3] = [x, y, z]
                        poses.append((ts, T))
            except Exception as e:
                print(f"  Stream stopped early at {len(poses)} poses in {os.path.basename(f)}: {e}")
    poses.sort(key=lambda x: x[0])
    print(f"  Loaded {len(poses)} odometry poses")
    poses = _decimate_burst_poses(poses)
    _audit_poses(poses, max_speed_mps=max_speed_mps, max_extent_m=max_extent_m, force=force)
    return poses


def _decimate_burst_poses(poses, min_dt_ms=5.0):
    """Drop back-to-back pose samples within min_dt_ms — keep the latest.

    DLIO publishes the geometric-observer state at its IMU callback rate
    (~117 Hz) with bursts where consecutive samples land 1-4 ms apart right
    after a GICP correction. The samples are individually correct (each is
    the up-to-the-microsecond state), but SLERPing between two samples that
    differ by 0.5-1 deg in 1-4 ms produces an interpolated rotation-rate
    of 100-700 deg/s — far above any car's physical turn rate. 16-bin
    per-point deskew lands bin centers at ~6 ms intervals, so adjacent
    bins can draw from opposite sides of a burst and end up 0.5 deg
    rotated relative to each other — the visible LAS 'rotation snap' in
    spite of a clean DLIO MCAP (Foxglove shows keyframe clouds at their
    scan-time pose, no interpolation between state updates, so the bursts
    are invisible in viz).

    Keeping the LATEST sample in each min_dt_ms window preserves the
    most-corrected state (after GICP has pulled the IMU-propagated state
    back onto the scan measurement) while guaranteeing SLERP always
    operates on stable >=5 ms intervals.

    Observed on 2026-04-18 drive1 DLIO run: 16,310 raw poses → ~10,000 after
    decimation; 61 rotation spikes >100 deg/s → 0 in decimated stream.
    FAST-LIO2 at 10 Hz is unaffected (all dt >=100 ms, nothing to drop)."""
    if len(poses) < 2:
        return poses
    min_dt_ns = int(min_dt_ms * 1e6)
    out = [poses[0]]
    dropped = 0
    for ts, T in poses[1:]:
        prev_ts, _ = out[-1]
        if ts - prev_ts < min_dt_ns:
            out[-1] = (ts, T)  # replace with later sample
            dropped += 1
        else:
            out.append((ts, T))
    if dropped > 0:
        print(f"  Burst-decimated {dropped} poses within {min_dt_ms:.1f} ms "
              f"of their predecessor ({len(poses)} → {len(out)})")
    return out


def _audit_poses(poses, max_speed_mps=30.0, max_extent_m=2000.0, force=False):
    """Sanity-check SLAM odom before LAS export. Prints PASS/WARN/FAIL card
    and raises on divergent trajectories — catches garbage poses before 2+
    minutes of LAS write time is spent on them.

    Gate on p99 speed, not raw max: DLIO at ~117 Hz publishes IMU-propagated
    pose bursts where occasional sample pairs are <1 ms apart with small
    spatial jumps — the sample is fine, the dt is noise, so speed = d/dt
    spikes to hundreds of m/s on up to ~0.5% of pairs while cumul + extent
    are correct. The 13 km / 354 km/h divergence we actually want to catch
    has sustained overspeed across the whole trajectory, so p99 catches it
    (would be hundreds of m/s) while tolerating stream noise (p99=19 m/s on
    the known-good 2026-04-18 drive1)."""
    if len(poses) < 2:
        print(f"  AUDIT: SKIPPED — only {len(poses)} poses")
        return
    t0, t1 = poses[0][0], poses[-1][0]
    span = (t1 - t0) / 1e9
    cumul = 0.0
    speeds = []
    for i in range(1, len(poses)):
        dt = (poses[i][0] - poses[i-1][0]) / 1e9
        if dt <= 0:
            continue
        dp = poses[i][1][:3, 3] - poses[i-1][1][:3, 3]
        d = float(np.linalg.norm(dp))
        cumul += d
        speeds.append(d / dt)
    xs = np.array([p[1][0, 3] for p in poses])
    ys = np.array([p[1][1, 3] for p in poses])
    zs = np.array([p[1][2, 3] for p in poses])
    extent = (xs.max() - xs.min(), ys.max() - ys.min(), zs.max() - zs.min())
    rate = (len(poses) - 1) / span if span > 0 else 0
    if speeds:
        speeds_arr = np.asarray(speeds)
        p99_speed = float(np.percentile(speeds_arr, 99.0))
        max_speed = float(speeds_arr.max())
    else:
        p99_speed = 0.0
        max_speed = 0.0
    print(f"  AUDIT: {len(poses):,} poses / {span:.1f}s / {rate:.2f} Hz / "
          f"{cumul:.0f} m cumul / p99 {p99_speed:.2f} m/s (max {max_speed:.1f}) / "
          f"extent {extent[0]:.0f}x{extent[1]:.0f}x{extent[2]:.0f} m")
    if p99_speed > max_speed_mps or max(extent) > max_extent_m:
        verdict = "WARN (forced)" if force else "FAIL"
        print(f"  AUDIT: {verdict} — p99_speed={p99_speed:.1f} m/s "
              f"max_extent={max(extent):.0f} m. Trajectory looks divergent; "
              f"re-tune SLAM before wasting LAS write time.")
        if not force:
            raise SystemExit(2)
        return
    if p99_speed > max_speed_mps * 0.5 or rate < 5.0:
        print(f"  AUDIT: WARN — p99_speed={p99_speed:.1f} m/s rate={rate:.2f} Hz")
    else:
        print(f"  AUDIT: PASS")


def interpolate_pose(poses, ts):
    """Linearly interpolate pose at timestamp ts (nanoseconds)."""
    if not poses:
        return None
    if ts <= poses[0][0]:
        return poses[0][1]
    if ts >= poses[-1][0]:
        return poses[-1][1]

    # Binary search
    lo, hi = 0, len(poses) - 1
    while lo < hi - 1:
        mid = (lo + hi) // 2
        if poses[mid][0] <= ts:
            lo = mid
        else:
            hi = mid

    t0, T0 = poses[lo]
    t1, T1 = poses[hi]
    if t1 == t0:
        return T0

    alpha = (ts - t0) / (t1 - t0)

    # Interpolate translation
    trans = T0[:3, 3] * (1 - alpha) + T1[:3, 3] * alpha

    # SLERP rotation
    r0 = Rotation.from_matrix(T0[:3, :3])
    r1 = Rotation.from_matrix(T1[:3, :3])
    from scipy.spatial.transform import Slerp
    slerp = Slerp([0, 1], Rotation.concatenate([r0, r1]))
    rot = slerp(alpha).as_matrix()

    T = np.eye(4)
    T[:3, :3] = rot
    T[:3, 3] = trans
    return T


def parse_ouster_cloud(decoded, min_range, max_range):
    """Parse Ouster PointCloud2 into structured arrays, applying range filter."""
    fields = {f.name: (f.offset, f.datatype, f.count) for f in decoded.fields}
    step = decoded.point_step
    data = bytes(decoded.data)
    n = decoded.width * decoded.height

    # Ouster field layout (from bag inspection):
    # x(0,f32) y(4,f32) z(8,f32) intensity(16,f32) t(20,u32)
    # reflectivity(24,u16) ring(26,u16) ambient(28,u16) range(32,u32)
    dtype = np.dtype([
        ('x', '<f4'), ('_pad1', 'V4'),
        ('y', '<f4'), ('_pad1b', 'V0'),
        ('z', '<f4'), ('_pad2', 'V4'),
        ('intensity', '<f4'),
        ('t', '<u4'),
        ('reflectivity', '<u2'),
        ('ring', '<u2'),
        ('ambient', '<u2'), ('_pad3', 'V2'),
        ('range', '<u4'), ('_pad4', 'V' + str(step - 36)),
    ])

    # Simpler: just unpack what we need directly
    xyz = np.zeros((n, 3), dtype=np.float32)
    intensity = np.zeros(n, dtype=np.float32)
    reflectivity = np.zeros(n, dtype=np.uint16)
    ring = np.zeros(n, dtype=np.uint16)
    ambient = np.zeros(n, dtype=np.uint16)
    range_mm = np.zeros(n, dtype=np.uint32)
    t_ns = np.zeros(n, dtype=np.uint32)

    ox, oy, oz = fields['x'][0], fields['y'][0], fields['z'][0]
    oi = fields['intensity'][0]
    ot = fields['t'][0]
    oref = fields['reflectivity'][0]
    oring = fields['ring'][0]
    oamb = fields['ambient'][0]
    orng = fields['range'][0]

    arr = np.frombuffer(data, dtype=np.uint8).reshape(n, step)
    xyz[:, 0] = np.frombuffer(arr[:, ox:ox+4].tobytes(), dtype='<f4')
    xyz[:, 1] = np.frombuffer(arr[:, oy:oy+4].tobytes(), dtype='<f4')
    xyz[:, 2] = np.frombuffer(arr[:, oz:oz+4].tobytes(), dtype='<f4')
    intensity[:] = np.frombuffer(arr[:, oi:oi+4].tobytes(), dtype='<f4')
    t_ns[:] = np.frombuffer(arr[:, ot:ot+4].tobytes(), dtype='<u4')
    reflectivity[:] = np.frombuffer(arr[:, oref:oref+2].tobytes(), dtype='<u2')
    ring[:] = np.frombuffer(arr[:, oring:oring+2].tobytes(), dtype='<u2')
    ambient[:] = np.frombuffer(arr[:, oamb:oamb+2].tobytes(), dtype='<u2')
    range_mm[:] = np.frombuffer(arr[:, orng:orng+4].tobytes(), dtype='<u4')

    # Range filter (range is in mm)
    range_m = range_mm.astype(np.float64) / 1000.0
    mask = (range_m >= min_range) & (range_m <= max_range) & np.isfinite(xyz[:, 0])

    return {
        'xyz': xyz[mask],
        'intensity': intensity[mask],
        'reflectivity': reflectivity[mask],
        'ring': ring[mask],
        'ambient': ambient[mask],
        'range_m': range_m[mask].astype(np.float32),
        't': t_ns[mask],
    }


def parse_dlio_deskewed_cloud(decoded):
    """Parse DLIO /dlio/odom_node/pointcloud/deskewed — already in world frame.

    Fields: x f32 @0, y f32 @4, z f32 @8, intensity f32 @16, t f32 @24, ...
    DLIO's PointType is pcl::PointXYZI plus Ouster time variants. We only
    need xyz + intensity — the cloud is already transformed to the `dlio_odom`
    frame by DLIO's deskewPointcloud() using the full IMU-integrated pose
    (not snapshot-interpolated), so no pose lookup or extrinsic is needed."""
    fields = {f.name: f.offset for f in decoded.fields}
    step = decoded.point_step
    data = bytes(decoded.data)
    n = decoded.width * decoded.height
    if n == 0:
        return None

    arr = np.frombuffer(data, dtype=np.uint8).reshape(n, step)
    ox, oy, oz = fields['x'], fields['y'], fields['z']
    oi = fields.get('intensity', ox)  # fallback — always defined on DLIO cloud
    xyz = np.empty((n, 3), dtype=np.float32)
    xyz[:, 0] = np.frombuffer(arr[:, ox:ox+4].tobytes(), dtype='<f4')
    xyz[:, 1] = np.frombuffer(arr[:, oy:oy+4].tobytes(), dtype='<f4')
    xyz[:, 2] = np.frombuffer(arr[:, oz:oz+4].tobytes(), dtype='<f4')
    intensity = np.frombuffer(arr[:, oi:oi+4].tobytes(), dtype='<f4').copy()
    mask = np.isfinite(xyz).all(axis=1)
    return {
        'xyz': xyz[mask],
        'intensity': intensity[mask],
        'reflectivity': np.zeros(int(mask.sum()), dtype=np.uint16),
        'ring': np.zeros(int(mask.sum()), dtype=np.uint16),
        'ambient': np.zeros(int(mask.sum()), dtype=np.uint16),
        'range_m': np.linalg.norm(xyz[mask], axis=1).astype(np.float32),
        't': np.zeros(int(mask.sum()), dtype=np.uint32),
    }


def transform_points(xyz, T):
    """Apply 4x4 transform to Nx3 points."""
    ones = np.ones((xyz.shape[0], 1), dtype=np.float32)
    pts_h = np.hstack([xyz, ones])
    transformed = (T @ pts_h.T).T
    return transformed[:, :3].astype(np.float32)


def voxel_downsample(points, voxel_size):
    """Simple voxel grid downsample — keeps first point per voxel."""
    if voxel_size <= 0:
        return np.arange(len(points['xyz']))

    coords = (points['xyz'] / voxel_size).astype(np.int32)
    _, idx = np.unique(
        coords[:, 0].astype(np.int64) * 1000000000 +
        coords[:, 1].astype(np.int64) * 1000000 +
        coords[:, 2].astype(np.int64),
        return_index=True
    )
    return idx


def write_las(output_path, all_points):
    """Write LAS 1.4 with extra Ouster fields."""
    import laspy

    xyz = all_points['xyz']
    # Drop non-finite AND extreme-outlier points. DLIO/FAST-LIO2 can emit huge
    # finite poses during init (e.g. 1e10 metres) that overflow int32 storage
    # at scale=0.001 (range only ±2147 km). Clamp to median ± 10 km which
    # still covers any sensible drive.
    finite = np.isfinite(xyz).all(axis=1)
    med = np.median(xyz[finite], axis=0) if finite.any() else np.zeros(3)
    sane = finite & (np.abs(xyz - med) < 10000).all(axis=1)
    if not sane.all():
        dropped = int((~sane).sum())
        print(f"  Dropping {dropped:,} non-finite/outlier points (|p - median| > 10km)")
        xyz = xyz[sane]
        for k in list(all_points.keys()):
            if k != 'xyz' and hasattr(all_points[k], '__len__') and len(all_points[k]) == len(sane):
                all_points[k] = all_points[k][sane]
        all_points['xyz'] = xyz
    n = len(xyz)
    print(f"  Writing LAS: {n:,} points to {output_path}")

    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    offsets = mins
    scales = np.array([0.001, 0.001, 0.001])

    header = laspy.LasHeader(point_format=6, version="1.4")
    header.offsets = offsets
    header.scales = scales
    # LAS 1.4 requires WKT CRS bit (global_encoding bit 4) for point formats >= 6
    # COPC conversion fails with "WKT bit must be set" otherwise
    header.global_encoding.wkt = True

    # Extra dimensions for Ouster fields
    header.add_extra_dim(laspy.ExtraBytesParams(name="reflectivity", type=np.uint16))
    header.add_extra_dim(laspy.ExtraBytesParams(name="ambient", type=np.uint16))
    header.add_extra_dim(laspy.ExtraBytesParams(name="range_m", type=np.float32))
    header.add_extra_dim(laspy.ExtraBytesParams(name="ring", type=np.uint16))

    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = all_points['intensity'].astype(np.uint16)
    las.reflectivity = all_points['reflectivity']
    las.ambient = all_points['ambient']
    las.range_m = all_points['range_m']
    las.ring = all_points['ring']

    # GPS time from scan index (relative seconds)
    if 'gps_time' in all_points:
        las.gps_time = all_points['gps_time']

    las.write(output_path)
    size_gb = os.path.getsize(output_path) / (1024**3)
    print(f"  LAS written: {size_gb:.2f} GB")


PDAL_BIN = "/usr/local/bin/pdal"


def write_copc(las_path, copc_path):
    """Convert LAS → COPC via PDAL CLI.

    copclib's FileWriter requires a source that's already COPC-ordered, so it
    can't convert a plain LAS — use pdal translate. forward=all preserves LAS
    VLRs/metadata; extra_dims=all preserves Ouster reflectivity/ambient/ring/t.
    """
    import subprocess

    if not os.path.exists(PDAL_BIN):
        raise FileNotFoundError(
            f"pdal binary not at {PDAL_BIN} — install via micromamba/conda-forge"
        )

    print(f"  Converting to COPC via pdal: {copc_path}")
    result = subprocess.run(
        [PDAL_BIN, "translate", las_path, copc_path,
         "--writers.copc.forward=all",
         "--writers.copc.extra_dims=all"],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"pdal translate failed ({result.returncode}): "
            f"{result.stderr.strip() or result.stdout.strip()}"
        )
    size_gb = os.path.getsize(copc_path) / (1024**3)
    print(f"  COPC written: {size_gb:.2f} GB")


def main():
    args = parse_args()

    from mcap.reader import make_reader
    from mcap_ros2.decoder import DecoderFactory

    # Find mcap files
    files = sorted(
        glob.glob(os.path.join(args.bag_dir, "bag_*.mcap")),
        key=lambda f: int(re.search(r'bag_(\d+)\.mcap', f).group(1))
    )
    files = [f for f in files if os.path.getsize(f) > 0]
    if args.max_files > 0:
        files = files[:args.max_files]
    print(f"Found {len(files)} bag files")

    # Output dir
    out_dir = args.output or os.path.join(args.bag_dir, "export")
    os.makedirs(out_dir, exist_ok=True)

    # Step 1: Load all odometry poses (optionally from a separate dir)
    print("\n[1/3] Loading odometry poses...")
    if args.odom_bag_dir:
        odom_files = sorted(glob.glob(os.path.join(args.odom_bag_dir, "*.mcap")))
        odom_files = [f for f in odom_files if os.path.getsize(f) > 0]
        print(f"  Reading odom from {len(odom_files)} bag(s) in {args.odom_bag_dir}")
    else:
        odom_files = files
    poses = load_odometry(odom_files, args.odom_topic,
                          max_speed_mps=args.max_speed,
                          max_extent_m=args.max_extent,
                          force=args.force)
    if not poses:
        print("ERROR: No odometry poses found. Cannot proceed.")
        sys.exit(1)

    # Step 2: Transform raw clouds using interpolated poses
    # If cloud topic is a SLAM-deskewed stream (already in world frame), skip
    # our own deskew + extrinsic — consume DLIO's internal deskewPointcloud()
    # output directly. DLIO integrates IMU continuously inside each 100 ms scan
    # to deskew, so the result is far more faithful than SLERPing the published
    # pose stream across scan-internal time bins.
    # GLIM's /glim_ros/aligned_points_corrected is also in the world frame
    # (rviz_viewer.cpp:466 transforms points by T_world_sensor before publish),
    # same pattern as DLIO's deskewed topic — bypass exporter SLERP-deskew.
    cloud_already_world = (
        "deskewed" in args.cloud_topic
        or "aligned_points" in args.cloud_topic
    )
    T_body_lidar = build_body_lidar_extrinsic(args)
    if cloud_already_world:
        print(f"\n[2/3] Concatenating SLAM-deskewed clouds from {args.cloud_topic} "
              f"(already in world frame — skipping exporter deskew + extrinsic)...")
    else:
        print(f"\n[2/3] Transforming raw Ouster clouds (range {args.min_range}-{args.max_range}m)...")
        print(f"  Extrinsic preset: {args.extrinsic_preset}")
        print(f"  T_body_lidar R:\n    {T_body_lidar[0,:3]}\n    {T_body_lidar[1,:3]}\n    {T_body_lidar[2,:3]}")
        print(f"  T_body_lidar t: {T_body_lidar[:3,3]}")
    all_xyz = []
    all_intensity = []
    all_reflectivity = []
    all_ring = []
    all_ambient = []
    all_range = []
    all_gps_time = []

    total_points = 0
    total_scans = 0
    t0 = time.time()

    N_BINS = args.deskew_bins

    # If cloud_already_world, we read from the SLAM-deskewed bag (not the
    # raw-Ouster bag). Use odom_files if available — those are the bags
    # containing the SLAM output topics.
    cloud_files = sorted(glob.glob(os.path.join(args.odom_bag_dir, "*.mcap"))) \
        if (cloud_already_world and args.odom_bag_dir) else files
    cloud_files = [f for f in cloud_files if os.path.getsize(f) > 0]

    for fi, f in enumerate(cloud_files):
        with open(f, "rb") as fh:
            reader = make_reader(fh, decoder_factories=[DecoderFactory()])
            for _, channel, message, decoded in reader.iter_decoded_messages(topics=[args.cloud_topic]):
                ts_scan = decoded.header.stamp.sec * 10**9 + decoded.header.stamp.nanosec

                if cloud_already_world:
                    # DLIO/FAST-LIO2 deskewed topic — already in world frame,
                    # deskewed per-point by SLAM's IMU-integrated continuous
                    # pose. No exporter deskew or extrinsic needed.
                    cloud = parse_dlio_deskewed_cloud(decoded)
                    if cloud is None or len(cloud['xyz']) == 0:
                        continue
                    xyz_map = cloud['xyz']
                else:
                    # Sanity-check the pose stream covers the scan stamp; if the
                    # whole scan is outside pose coverage, skip it.
                    if interpolate_pose(poses, ts_scan) is None:
                        continue

                    cloud = parse_ouster_cloud(decoded, args.min_range, args.max_range)
                    if len(cloud['xyz']) == 0:
                        continue

                    # Per-point de-skew. The Ouster driver fills `t` with ns
                    # offsets from the scan header stamp (0 .. ~100 ms). Applying
                    # a single pose to the full 131k-pt scan smears 0.5-2 m of
                    # wall-thickening per scan at drive speeds — baked into the
                    # LAS independent of SLAM quality. Bucket the scan into
                    # N_BINS time windows, SLERP the body pose per bin, transform
                    # each bin's points with its own T_world_lidar.
                    t_abs = ts_scan + cloud['t'].astype(np.int64)
                    # Use np.linspace(min, max, N+1). If the scan has zero time
                    # span (all points at same t), edges collapse and all points
                    # fall in bin 0 — fall through to single-pose path.
                    t_min = int(t_abs.min())
                    t_max = int(t_abs.max())
                    xyz_map = np.empty_like(cloud['xyz'])
                    if t_max == t_min:
                        T_world_lidar = interpolate_pose(poses, t_min) @ T_body_lidar
                        xyz_map = transform_points(cloud['xyz'], T_world_lidar)
                    else:
                        bin_edges = np.linspace(t_min, t_max, N_BINS + 1)
                        bin_idx = np.clip(
                            np.searchsorted(bin_edges, t_abs, side='right') - 1,
                            0, N_BINS - 1,
                        )
                        for b in range(N_BINS):
                            mask = bin_idx == b
                            if not mask.any():
                                continue
                            t_bin = int(0.5 * (bin_edges[b] + bin_edges[b + 1]))
                            T_world_body = interpolate_pose(poses, t_bin)
                            if T_world_body is None:
                                # Out of pose range (edge of bag); fall back to
                                # scan-stamp pose rather than dropping points.
                                T_world_body = interpolate_pose(poses, ts_scan)
                            T_world_lidar = T_world_body @ T_body_lidar
                            xyz_map[mask] = transform_points(cloud['xyz'][mask], T_world_lidar)

                all_xyz.append(xyz_map)
                all_intensity.append(cloud['intensity'])
                all_reflectivity.append(cloud['reflectivity'])
                all_ring.append(cloud['ring'])
                all_ambient.append(cloud['ambient'])
                all_range.append(cloud['range_m'])

                # Relative time in seconds from first pose
                rel_time = (ts_scan - poses[0][0]) / 1e9
                all_gps_time.append(np.full(len(xyz_map), rel_time, dtype=np.float64))

                total_scans += 1
                total_points += len(xyz_map)

        elapsed = time.time() - t0
        rate = total_scans / elapsed if elapsed > 0 else 0
        print(f"  File {fi+1}/{len(cloud_files)}: {total_scans} scans, {total_points:,} pts, {rate:.1f} scans/s")

    if total_points == 0:
        print("ERROR: No points accumulated.")
        sys.exit(1)

    # Concatenate
    print(f"\n  Total: {total_scans} scans, {total_points:,} points")
    points = {
        'xyz': np.vstack(all_xyz),
        'intensity': np.concatenate(all_intensity),
        'reflectivity': np.concatenate(all_reflectivity),
        'ring': np.concatenate(all_ring),
        'ambient': np.concatenate(all_ambient),
        'range_m': np.concatenate(all_range),
        'gps_time': np.concatenate(all_gps_time),
    }

    # Free intermediates
    del all_xyz, all_intensity, all_reflectivity, all_ring, all_ambient, all_range, all_gps_time

    # Voxel downsample
    if args.voxel > 0:
        print(f"  Voxel downsample at {args.voxel}m...")
        idx = voxel_downsample(points, args.voxel)
        for k in points:
            points[k] = points[k][idx]
        print(f"  After downsample: {len(points['xyz']):,} points")

    # Step 3: Write outputs
    print("\n[3/3] Writing output files...")
    las_path = os.path.join(out_dir, "site_trial.las")
    write_las(las_path, points)

    if not args.skip_copc:
        copc_path = os.path.join(out_dir, "site_trial.copc.laz")
        try:
            write_copc(las_path, copc_path)
        except Exception as e:
            print(f"  COPC conversion failed: {e}")

    total_time = time.time() - t0
    print(f"\nDone in {total_time:.0f}s ({total_time/60:.1f} min)")
    print(f"Output: {out_dir}")


if __name__ == "__main__":
    main()
