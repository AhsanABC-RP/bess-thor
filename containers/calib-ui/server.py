#!/usr/bin/env python3
"""BESS Calibration UI backend.

FastAPI + rclpy hybrid. Subscribes to a camera image topic, runs AprilGrid
detection on every frame, builds a live coverage map, serves MJPEG preview +
JSON state via websocket, and drives ros2 bag record + Kalibr as subprocesses.

Design notes:
- rclpy spins on a dedicated thread using SingleThreadedExecutor; FastAPI
  event loop on the main thread. Shared state is a single dataclass behind a
  lock.
- Detection uses the `apriltag` (Swatbotics) package already in bess-calib-
  util. Tag family hard-wired to tag36h11 (calib.io + Kalibr's only
  supported family).
- Coverage scoring: image is divided into GRID_ROWS x GRID_COLS cells.
  Each cell is "hit" when a detected tag center lands in it. Plus three
  pose-variety metrics derived cheaply from tag geometry: distance (mean
  tag side length in px), tilt (aspect ratio of the tag bounding rect),
  roll (angle of the top edge).
- Recording: spawn `ros2 bag record` subprocess in MCAP mode. On Finish,
  kill the subprocess, move the resulting bag to /data/<session>/raw.mcap,
  convert via bess-calib-util (rosbags-convert) -> raw.bag.
- Calibrate button: spawn bess-calib-kalibr container via docker socket,
  with KALIBR_MANUAL_FOCAL_LENGTH_INIT=1 + a focal seed. Stream stdout
  back over websocket.
"""

from __future__ import annotations

import asyncio
import json
import os
import shutil
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from pupil_apriltags import Detector as _PupilDetector
import rclpy
import uvicorn
import yaml
from cv_bridge import CvBridge
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

# --------------------------------------------------------------------- Config

GRID_ROWS = 8
GRID_COLS = 10           # 80 cells; aim for >= 70% hit
MIN_GRID_HIT_PCT = 0.70
MIN_DISTANCE_BINS = 3    # close / mid / far — each needs MIN_FRAMES_PER_BIN
MIN_TILT_BINS = 5        # tag aspect ratio quantized
MIN_ROLL_BINS = 5        # tag top-edge angle quantized
MIN_FRAMES_PER_BIN = 5
TARGET_DETECTED_FRAMES = 80

# Thermal foil-grid target (A70 boards): 16 cols × 12 rows of 50mm squares.
THERMAL_GRID_ROWS = 12
THERMAL_GRID_COLS = 16
THERMAL_SPACING_M = 0.05

# Thermal circle-grid target — asymmetric dot pattern, 60 mm dots on 60 mm
# offset. In OpenCV's asymmetric layout each row has `cols` dots and rows
# alternate horizontal-offset by `square_size`; object-point spacing is
# `square_size` in both axes. Defaults are probed via CIRCLES_PROBE_SIZES
# when pattern_size is not pinned via env, so a new board shape doesn't
# require a rebuild.
THERMAL_CIRCLES_SPACING_M = float(os.environ.get("THERMAL_CIRCLES_SPACING_M", "0.06"))
THERMAL_CIRCLES_ASYMMETRIC = os.environ.get("THERMAL_CIRCLES_ASYMMETRIC", "1") not in ("0", "false", "False")
# Pinned (cols, rows) if set — otherwise the detector probes a list of common
# asymmetric sizes and picks the first that registers.
_pinned_cols = os.environ.get("THERMAL_CIRCLES_COLS")
_pinned_rows = os.environ.get("THERMAL_CIRCLES_ROWS")
THERMAL_CIRCLES_PINNED: Optional[Tuple[int, int]] = (
    (int(_pinned_cols), int(_pinned_rows)) if _pinned_cols and _pinned_rows else None
)
# Common sizes to probe, largest-first so a partial-view match on a big board
# beats a full-view match on a small one. (cols, rows) in OpenCV's sense.
CIRCLES_PROBE_SIZES: List[Tuple[int, int]] = [
    (4, 11), (4, 13), (4, 9), (5, 11), (5, 13), (5, 9),
    (3, 11), (3, 13), (3, 9), (4, 7), (5, 7), (6, 11), (6, 9),
]

CALIB_BASE = Path(os.environ.get("CALIB_BASE", "/data"))
CALIB_HOST_BASE = Path(os.environ.get("CALIB_HOST_BASE", "/home/thor/calib"))
CONFIG_DIR = Path("/config")

# Camera presets — topic + model + rough focal seed for KALIBR_MANUAL init.
# Copied from scripts/calib_capture.sh ns_to_image_topic so the UI matches the
# CLI workflow.
CAMERA_PRESETS: Dict[str, Dict[str, object]] = {
    "lucid_right": {
        "ns": "lucid2",
        "image_topic": "/lucid2/camera_driver/image_raw",
        "info_topic": "/lucid2/camera_driver/camera_info",
        "model": "pinhole-equi",
        "focal_seed": 1200,
    },
    "lucid_left": {
        "ns": "lucid1",
        "image_topic": "/lucid1/camera_driver/image_raw",
        "info_topic": "/lucid1/camera_driver/camera_info",
        "model": "pinhole-equi",
        "focal_seed": 1200,
    },
    "blackfly_right": {
        "ns": "blackfly1",
        "image_topic": "/blackfly/camera1/blackfly_camera/image_raw",
        "info_topic": "/blackfly/camera1/blackfly_camera/camera_info",
        # Blackfly S narrow-FOV lens: pinhole-equi (Kannala-Brandt fisheye)
        # diverges — 2026-04-20 attempt drove focal init to fx=179 on a
        # 3000px-wide image and OptimizationDiverged. Radtan is the correct
        # model; equi is only for >120° FOV fisheyes (A70, Lucid wide).
        "model": "pinhole-radtan",
        "focal_seed": 900,
    },
    "blackfly_left": {
        "ns": "blackfly2",
        "image_topic": "/blackfly/camera2/blackfly_camera/image_raw",
        "info_topic": "/blackfly/camera2/blackfly_camera/camera_info",
        "model": "pinhole-radtan",
        "focal_seed": 900,
    },
    "a6701_right": {
        "ns": "thermal1",
        "image_topic": "/thermal/camera1/image_raw",
        "info_topic": "/thermal/camera1/camera_info",
        "model": "pinhole-radtan",
        # A6701sc: 640x512 InSb, 15 µm pitch, 17 mm lens → fx ≈ 17/0.015 ≈ 1133 px
        "focal_seed": 1133,
        "target": "thermal_circles_asym",
    },
    "a6701_left": {
        "ns": "thermal2",
        "image_topic": "/thermal/camera2/image_raw",
        "info_topic": "/thermal/camera2/camera_info",
        "model": "pinhole-radtan",
        "focal_seed": 1133,
        "target": "thermal_circles_asym",
    },
    "a70_right": {
        "ns": "thermal3",
        "image_topic": "/thermal/camera3/image_raw",
        "info_topic": "/thermal/camera3/camera_info",
        "model": "pinhole-equi",
        "focal_seed": 350,
        "target": "thermal_circles_asym",
    },
    "a70_left": {
        "ns": "thermal4",
        "image_topic": "/thermal/camera4/image_raw",
        "info_topic": "/thermal/camera4/camera_info",
        "model": "pinhole-equi",
        "focal_seed": 350,
        "target": "thermal_circles_asym",
    },
    "zed_rgb": {
        "ns": "zed",
        "image_topic": "/zed/zed/rgb/color/rect/image",
        "info_topic": "/zed/zed/rgb/camera_info",
        "model": "pinhole-radtan",
        "focal_seed": 530,
    },
}

# ---------------------------------------------------------------- Shared state

@dataclass
class CoverageState:
    grid: np.ndarray = field(default_factory=lambda: np.zeros((GRID_ROWS, GRID_COLS), dtype=bool))
    dist_bins: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=int))
    tilt_bins: np.ndarray = field(default_factory=lambda: np.zeros(MIN_TILT_BINS, dtype=int))
    roll_bins: np.ndarray = field(default_factory=lambda: np.zeros(MIN_ROLL_BINS, dtype=int))
    frame_total: int = 0
    frame_detected: int = 0
    last_tag_count: int = 0

    def reset(self):
        self.grid.fill(False)
        self.dist_bins.fill(0)
        self.tilt_bins.fill(0)
        self.roll_bins.fill(0)
        self.frame_total = 0
        self.frame_detected = 0
        self.last_tag_count = 0


@dataclass
class SessionState:
    session_name: Optional[str] = None
    preset_key: Optional[str] = None
    image_topic: Optional[str] = None
    info_topic: Optional[str] = None
    image_width: int = 0
    image_height: int = 0
    recording: bool = False
    record_proc: Optional[subprocess.Popen] = None
    bag_tmp_dir: Optional[Path] = None
    kalibr_proc: Optional[subprocess.Popen] = None
    kalibr_log: List[str] = field(default_factory=list)
    kalibr_result: Optional[str] = None
    # Accumulated (object_points, image_points) pairs for thermal calib
    # (only populated when preset target == thermal_foil_16x12 AND recording).
    thermal_captures: List[tuple] = field(default_factory=list)


coverage = CoverageState()
session = SessionState()
state_lock = threading.Lock()
last_jpeg: Optional[bytes] = None
last_jpeg_lock = threading.Lock()
# Raw pre-overlay BGR frame (matches preview rotation). Kept so /rectified.jpg
# can undistort a fresh frame without needing its own ROS subscription.
last_bgr: Optional[np.ndarray] = None
last_bgr_lock = threading.Lock()

# Per-preset preview rotation, degrees clockwise in {0, 90, 180, 270}.
# Applied to the decoded image BEFORE tag detection so tilt/roll coverage
# bins stay meaningful to the operator looking at the rotated preview.
preview_rotation: Dict[str, int] = {k: 0 for k in CAMERA_PRESETS}
preview_rotation_lock = threading.Lock()

# Per-preset target override (UX dropdown). None = use the preset's built-in target.
# Valid values: "apriltag_calibio", "checker_18x25_30mm", "thermal_foil_16x12".
preview_target_override: Dict[str, Optional[str]] = {k: None for k in CAMERA_PRESETS}
preview_target_lock = threading.Lock()

AVAILABLE_TARGETS = [
    "apriltag_calibio",
    "checker_18x25_30mm",
    "thermal_foil_16x12",
    "thermal_circles_asym",
]


def _apply_rotation(img: np.ndarray, deg: int) -> np.ndarray:
    if deg == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if deg == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if deg == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img

# ------------------------------------------------------------ AprilTag setup

detector = _PupilDetector(
    families="tag36h11",
    nthreads=4,
    quad_decimate=2.0,   # detect on a /2 image for speed
    refine_edges=True,
)
detector_lock = threading.Lock()

# Raw-frame dump trigger: POST /debug/dump sets this; the next image callback
# writes the pre-detection gray + bgr to /tmp so we can replay detection off-line.
_debug_dump_request = threading.Event()


# ------------------------------------------------------- Checkerboard
# Classic chessboard target. cv2.findChessboardCornersSB is robust + fast enough
# for live preview. We wrap the detection as a single pseudo-"tag" (quad = outer
# corners of the inner-corner grid) so update_from_detections() can reuse the
# apriltag coverage/pose path unchanged.
#
# INNER-corner counts (not squares) — 18x25 SQUARES board = 17x24 inner corners.
CHECKER_TARGETS: Dict[str, Tuple[int, int]] = {
    "checker_18x25_30mm": (17, 24),
}


def detect_checkerboard(gray: np.ndarray, pattern: Tuple[int, int]) -> list:
    """Return one pseudo-detection per inner corner so the coverage grid lights up
    across the board as it moves. All dets share the board's outer-quad as
    `corners` so pose proxies (dist/tilt/roll, which pick the largest det) still
    see the full board geometry. `pattern` is (inner_rows, inner_cols)."""
    rows, cols = pattern
    # Same saturation clip as the apriltag path — findChessboardCornersSB needs
    # sharp black/white boundaries and degrades badly when the white squares
    # blow out to 255.
    if float((gray > 250).mean()) > 0.05:
        det_gray = np.minimum(gray, 110)
        det_gray = cv2.normalize(det_gray, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    else:
        det_gray = gray
    # Downsample 2x for the live path — SB on 2336x1750 w/ EXHAUSTIVE is 1.5 s.
    # Results are scaled back to full resolution at the end.
    h, w = det_gray.shape
    small = cv2.resize(det_gray, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
    # NORMAL mode only (default). EXHAUSTIVE tripled runtime w/o a hit-rate win.
    found, corners = cv2.findChessboardCornersSB(small, (cols, rows))
    if not found or corners is None or len(corners) != rows * cols:
        return []
    pts = corners.reshape(-1, 2) * 2.0  # back to full-res coords
    # Outer-quad of the inner-corner grid — row-major from findChessboardCornersSB
    tl = pts[0]
    tr = pts[cols - 1]
    br = pts[rows * cols - 1]
    bl = pts[(rows - 1) * cols]
    board_quad = np.array([tl, tr, br, bl], dtype=np.float32)
    # One det per corner; shared corners=board_quad means _area() is identical
    # across dets, so max(...) just returns the first — pose proxies unaffected.
    return [_ThermalDet(pt.astype(np.float32), board_quad) for pt in pts]


# ------------------------------------------------------- Thermal foil-grid
# Bespoke detector for a 16×12 foil-square grid on thermal substrate. No
# per-cell IDs, so: (1) each frame extracts dark-blob centers as pseudo
# "detections" for coverage/preview — same interface as pupil-apriltags,
# (2) at calibrate time, partial-view-tolerant lattice RANSAC assigns (r,c)
# to each blob and feeds cv2.calibrateCamera / cv2.fisheye.calibrate.


class _ThermalDet:
    __slots__ = ("center", "corners")

    def __init__(self, center, corners):
        self.center = center
        self.corners = corners


def _order_clockwise(pts: np.ndarray) -> np.ndarray:
    """Order 4 points clockwise from top-left. Matches AprilTag corner order."""
    s = pts.sum(axis=1)
    diff = pts[:, 0] - pts[:, 1]
    out = np.zeros((4, 2), dtype=pts.dtype)
    out[0] = pts[np.argmin(s)]        # top-left (min x+y)
    out[2] = pts[np.argmax(s)]        # bottom-right (max x+y)
    out[1] = pts[np.argmax(diff)]     # top-right (max x-y)
    out[3] = pts[np.argmin(diff)]     # bottom-left (min x-y)
    return out


# ------------------------------------------------------- Circle-grid target
# OpenCV's purpose-built asymmetric circle-grid detector. For the new A70
# board: 60 mm dark dots on a bright-substrate 60 mm offset grid. The
# findCirclesGrid + CALIB_CB_CLUSTERING combo handles perspective and
# partial views far better than the generic blob+lattice approach — it
# was written specifically for this use case. SimpleBlobDetector is tuned
# for thermal imagery: dark blobs on bright background, wide size range,
# relaxed circularity because steep perspective turns circles into ellipses.


def _make_thermal_blob_detector() -> "cv2.SimpleBlobDetector":
    params = cv2.SimpleBlobDetector_Params()
    params.filterByColor = True
    params.blobColor = 0          # dark blobs on bright background
    params.filterByArea = True
    params.minArea = 25           # far dots ~ 5×5 px
    params.maxArea = 50000        # near dots can get huge on closeup
    params.filterByCircularity = True
    params.minCircularity = 0.5   # strong perspective squashes circles
    params.filterByInertia = True
    params.minInertiaRatio = 0.1  # tolerate strong ellipse foreshortening
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.minDistBetweenBlobs = 5.0
    return cv2.SimpleBlobDetector_create(params)


_circle_blob_detector: Optional["cv2.SimpleBlobDetector"] = None


def detect_circle_grid(
    gray: np.ndarray,
    pattern_size: Optional[Tuple[int, int]] = None,
    asymmetric: bool = True,
) -> Tuple[List[_ThermalDet], Optional[Tuple[int, int]]]:
    """Detect an asymmetric circle grid. Returns (detections, matched_size).

    matched_size is the (cols, rows) that registered. If None is returned,
    nothing matched. Each _ThermalDet has `center` at the circle center and
    `corners` set to a small 4-point box around it — render_overlay draws
    the box + center dot, and downstream fit_thermal_lattice is BYPASSED
    because we already have the full correspondence from findCirclesGrid.
    """
    global _circle_blob_detector
    if _circle_blob_detector is None:
        _circle_blob_detector = _make_thermal_blob_detector()

    H, W = gray.shape[:2]

    # Blob detection on RAW gray. CLAHE amplified sky/tree noise → 85 blobs
    # vs 55 raw, and findCirclesGrid couldn't cluster through the noise.
    keypoints = _circle_blob_detector.detect(gray)
    if not keypoints:
        return [], None

    kp_xy = np.array([[k.pt[0], k.pt[1]] for k in keypoints], dtype=np.float32)
    kp_size = np.array([k.size for k in keypoints], dtype=np.float32)
    # Estimate dot pitch from median blob size; use it to spatially cluster.
    med_size = float(np.median(kp_size))
    link = max(4.0 * med_size, 30.0)

    # Union-find spatial clustering to find the board (densest blob cluster).
    n = len(kp_xy)
    diffs = kp_xy[:, None, :] - kp_xy[None, :, :]
    d = np.sqrt(np.einsum("ijk,ijk->ij", diffs, diffs))
    np.fill_diagonal(d, np.inf)
    parent = list(range(n))
    def _find(x: int) -> int:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    ii, jj = np.where(d < link)
    for i, j in zip(ii.tolist(), jj.tolist()):
        if i >= j:
            continue
        ri, rj = _find(i), _find(j)
        if ri != rj:
            parent[ri] = rj
    groups: Dict[int, List[int]] = {}
    for i in range(n):
        groups.setdefault(_find(i), []).append(i)
    sorted_groups = sorted(groups.values(), key=len, reverse=True)

    sizes: List[Tuple[int, int]]
    if pattern_size is not None:
        sizes = [pattern_size]
    elif THERMAL_CIRCLES_PINNED is not None:
        sizes = [THERMAL_CIRCLES_PINNED]
    else:
        sizes = list(CIRCLES_PROBE_SIZES)

    grid_flag = cv2.CALIB_CB_ASYMMETRIC_GRID if asymmetric else cv2.CALIB_CB_SYMMETRIC_GRID

    # Try up to the 3 densest clusters. For each, ROI-crop around the cluster
    # and run findCirclesGrid on just that window — keeps sky/tree blobs out
    # of the grid-fit stage. This is the key to robust detection in cluttered
    # scenes. Upscale 2x when the ROI is small: findCirclesGrid's clustering
    # heuristics break down when dots are <~16 px apart.
    matched: Optional[Tuple[int, int]] = None
    centers: Optional[np.ndarray] = None
    for grp in sorted_groups[:3]:
        if len(grp) < 8:
            break
        grp_xy = kp_xy[grp]
        pad = int(2.0 * med_size)
        x0 = max(0, int(grp_xy[:, 0].min()) - pad)
        y0 = max(0, int(grp_xy[:, 1].min()) - pad)
        x1 = min(W, int(grp_xy[:, 0].max()) + pad)
        y1 = min(H, int(grp_xy[:, 1].max()) + pad)
        if x1 - x0 < 20 or y1 - y0 < 20:
            continue
        roi = gray[y0:y1, x0:x1]
        # Probe at multiple scales (1x, 2x, 3x) — pick first that registers.
        # Also probe with and without CLUSTERING; CLUSTERING helps in noise,
        # hurts when the board fills the ROI cleanly.
        scales = [1.0]
        max_dim = max(roi.shape[:2])
        if max_dim < 200:
            scales.append(2.0)
        if max_dim < 120:
            scales.append(3.0)
        for scale in scales:
            if scale == 1.0:
                scaled = roi
            else:
                scaled = cv2.resize(
                    roi,
                    (int(roi.shape[1] * scale), int(roi.shape[0] * scale)),
                    interpolation=cv2.INTER_LINEAR,
                )
            for cluster_flag in (cv2.CALIB_CB_CLUSTERING, 0):
                flags = grid_flag | cluster_flag
                for sz in sizes:
                    found, pts = cv2.findCirclesGrid(
                        scaled, sz, flags=flags,
                        blobDetector=_circle_blob_detector,
                    )
                    if found and pts is not None:
                        matched = sz
                        c = pts.reshape(-1, 2).copy()
                        if scale != 1.0:
                            c /= scale
                        c[:, 0] += x0
                        c[:, 1] += y0
                        centers = c
                        break
                if matched is not None:
                    break
            if matched is not None:
                break
        if matched is not None:
            break

    radius = max(3.0, min(med_size * 0.5, 30.0))

    if centers is None:
        # Grid fit failed — but we have candidate blobs on what looks like a
        # board. Show them as yellow markers so the user sees that SOMETHING
        # is being detected, and can adjust distance/angle until grid matches.
        fallback_grp = sorted_groups[0] if sorted_groups else []
        if len(fallback_grp) < 8:
            return [], None
        dets: List[_ThermalDet] = []
        for i in fallback_grp:
            cx, cy = float(kp_xy[i, 0]), float(kp_xy[i, 1])
            box = np.array(
                [[cx - radius, cy - radius],
                 [cx + radius, cy - radius],
                 [cx + radius, cy + radius],
                 [cx - radius, cy + radius]],
                dtype=np.float32,
            )
            dets.append(_ThermalDet(np.array([cx, cy], dtype=np.float32), box))
        return dets, None

    dets: List[_ThermalDet] = []
    for (cx, cy) in centers:
        box = np.array(
            [[cx - radius, cy - radius],
             [cx + radius, cy - radius],
             [cx + radius, cy + radius],
             [cx - radius, cy + radius]],
            dtype=np.float32,
        )
        dets.append(_ThermalDet(np.array([cx, cy], dtype=np.float32), box))
    return dets, matched


def circle_grid_object_points(
    pattern_size: Tuple[int, int],
    spacing_m: float,
    asymmetric: bool = True,
) -> np.ndarray:
    """Object points in metres for an asymmetric/symmetric circle grid.

    Asymmetric layout (OpenCV convention): point at (col, row) maps to
    ((2*col + row%2) * spacing, row * spacing, 0). Symmetric: (col*spacing, row*spacing, 0).
    Returned shape: (cols*rows, 3) float32.
    """
    cols, rows = pattern_size
    pts = np.zeros((cols * rows, 3), dtype=np.float32)
    for r in range(rows):
        for c in range(cols):
            if asymmetric:
                pts[r * cols + c] = ((2 * c + r % 2) * spacing_m, r * spacing_m, 0.0)
            else:
                pts[r * cols + c] = (c * spacing_m, r * spacing_m, 0.0)
    return pts


def detect_thermal_grid(gray: np.ndarray) -> List[_ThermalDet]:
    """Find foil-square blobs, scored by lattice-fit inliers.

    Runs adaptive threshold at 4 block sizes × 2 polarities (the board is
    bright foil on dark substrate in some lighting and the inverse in
    others). For each (scale, polarity) and each of the top-3 area clusters,
    feeds the candidate blobs through fit_thermal_lattice() and keeps
    whichever combination produces the most grid inliers. Raw kept-cluster
    count loses to clutter — a backlit tree or warm body produces hundreds
    of same-size blobs but they don't sit on a regular lattice, so the
    lattice fit returns None on noise and accepts only the board.
    """
    h, w = gray.shape
    blur = cv2.GaussianBlur(gray, (3, 3), 0.8)
    min_area = 8                        # ~3×3 px — far-distance floor
    max_area = (w * h) / 50             # at most 1/50 of frame per blob
    best_dets: List[_ThermalDet] = []
    best_inliers = 0
    for block, polarity in ((13, cv2.THRESH_BINARY_INV), (21, cv2.THRESH_BINARY_INV),
                             (31, cv2.THRESH_BINARY_INV), (45, cv2.THRESH_BINARY_INV),
                             (13, cv2.THRESH_BINARY), (21, cv2.THRESH_BINARY),
                             (31, cv2.THRESH_BINARY), (45, cv2.THRESH_BINARY)):
        bw = cv2.adaptiveThreshold(
            blur, 255,
            cv2.ADAPTIVE_THRESH_MEAN_C, polarity, block, 3,
        )
        # RETR_LIST (not RETR_EXTERNAL) — the board squares sit inside the
        # board outline contour, so EXTERNAL silently hides every one.
        contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        cands: List[tuple] = []
        for c in contours:
            a = float(cv2.contourArea(c))
            if a < min_area or a > max_area:
                continue
            rect = cv2.minAreaRect(c)
            (cx, cy), (rw, rh), _ = rect
            if rw < 2 or rh < 2:
                continue
            ar = max(rw, rh) / max(min(rw, rh), 1e-6)
            if ar > 1.8:
                continue
            hull_a = cv2.contourArea(cv2.convexHull(c))
            if hull_a <= 0 or a / hull_a < 0.70:
                continue
            cands.append((a, cx, cy, rect))
        if len(cands) < 6:
            continue

        # Sweep populated area bins — gravel/foliage owns the top histogram
        # bins, so top-3 misses the board; we need a wider sweep but capped
        # to stay <200 ms. 8 populated bins × top-3 clusters × 8 scales ≈
        # 200 lattice-fit calls, bounded runtime.
        areas = np.array([c[0] for c in cands])
        log_a = np.log(areas)
        hist, edges = np.histogram(log_a, bins=20)
        ranked_bins = np.argsort(hist)[::-1][:8]
        for bin_idx in ranked_bins:
            if hist[bin_idx] < 4:
                continue
            mode_log = 0.5 * (edges[bin_idx] + edges[bin_idx + 1])
            mode_area = float(np.exp(mode_log))
            keep_mask = np.abs(areas - mode_area) / mode_area < 0.6
            if int(keep_mask.sum()) < 6:
                continue
            dets_all: List[_ThermalDet] = []
            for (a, cx, cy, rect), ok in zip(cands, keep_mask):
                if not ok:
                    continue
                box = cv2.boxPoints(rect).astype(np.float32)
                box = _order_clockwise(box)
                dets_all.append(_ThermalDet(np.array([cx, cy], dtype=np.float32), box))
            # Spatial pre-clustering — the board is a physically compact
            # 100-300 px cluster; gravel/foliage produces hundreds of blobs
            # scattered across the frame. Feeding all candidates to
            # fit_thermal_lattice lets the gravel NN distribution dominate
            # the pitch histogram and the basis is fit to noise. Instead,
            # group candidates by proximity using a scale-adaptive threshold
            # (the filter block size is a proxy for the board cell pitch:
            # the board is only selected when the block size is close to
            # a cell's apparent width) and run the lattice fit on each
            # cluster that has enough members.
            if len(dets_all) < 6:
                continue
            centers_all = np.stack([d.center for d in dets_all])
            # Link distance must follow CELL size, not block size. Cell edge
            # ≈ sqrt(mode_area); spacing between adjacent cells on the board
            # is ~1.8-2.2× cell edge (dark cell + bright gap). Use 2.6×edge
            # for some slack. Block size is a poor proxy — large-cell boards
            # (40+ px) need block=45 to threshold but 45*1.4 is still not
            # always enough to link 40 px cells across an 80 px spacing.
            cell_edge = float(np.sqrt(mode_area))
            link = max(2.6 * cell_edge, 18.0)
            # Vectorized union-find — compute all pairwise distances once
            # with broadcast (cheap: n≤200 means ≤40k pairs in one numpy
            # op). A Python per-pair np.linalg.norm loop is orders of
            # magnitude slower — the pre-fix detector hit 9 s/frame with
            # Python-loop distances.
            n_all = len(dets_all)
            diffs_all = centers_all[:, None, :] - centers_all[None, :, :]
            d_all = np.sqrt(np.einsum("ijk,ijk->ij", diffs_all, diffs_all))
            np.fill_diagonal(d_all, np.inf)
            near_ii, near_jj = np.where(d_all < link)
            parent = list(range(n_all))
            def _find(x):
                while parent[x] != x:
                    parent[x] = parent[parent[x]]
                    x = parent[x]
                return x
            for i, j in zip(near_ii.tolist(), near_jj.tolist()):
                if i >= j:
                    continue
                ri, rj = _find(i), _find(j)
                if ri != rj:
                    parent[ri] = rj
            groups: Dict[int, List[int]] = {}
            for i in range(n_all):
                groups.setdefault(_find(i), []).append(i)
            # Top-3 clusters only — typical frame produces 20+ tiny
            # 1-2-element clusters from clutter, plus the board. Board is
            # usually one of the largest.
            sorted_groups = sorted(groups.values(), key=len, reverse=True)
            for grp in sorted_groups[:3]:
                if len(grp) < 6:
                    break
                dets = [dets_all[i] for i in grp]
                try:
                    fit = fit_thermal_lattice(dets)
                except Exception:
                    fit = None
                if fit is None:
                    continue
                _, _, inlier_idxs = fit
                n_inl = len(inlier_idxs)
                if n_inl > best_inliers:
                    best_inliers = n_inl
                    best_dets = [dets[i] for i in inlier_idxs]
    return best_dets


def fit_thermal_lattice(
    blobs: List[_ThermalDet],
    rows: int = THERMAL_GRID_ROWS,
    cols: int = THERMAL_GRID_COLS,
    spacing_m: float = THERMAL_SPACING_M,
    residual_tol: float = 0.30,
) -> Optional[tuple]:
    """Partial-view RANSAC-ish lattice fit.

    Returns (object_points (N,3) float32, image_points (N,2) float32) or None.
    Object-point coords are in metres (col * spacing, row * spacing, 0).

    Algorithm: nearest-neighbor displacements → two dominant angles 90° apart
    → lattice basis (e_a, e_b) → express each blob in that basis → round to
    nearest integer lattice position → keep inliers within residual_tol
    lattice units → reject if span exceeds grid.
    """
    n = len(blobs)
    if n < 6:
        return None
    centers = np.stack([b.center for b in blobs]).astype(np.float32)

    # Pairwise distances (small n ≤ ~200, so O(n²) is fine)
    diffs = centers[:, None, :] - centers[None, :, :]
    dists = np.linalg.norm(diffs, axis=-1)
    np.fill_diagonal(dists, np.inf)
    nn = dists.min(axis=1)
    # Median NN can collapse to noise-cluster scale when env clutter is present.
    # On strongly angled boards the nearest-neighbour distribution is bimodal
    # (near side vs far side pitch can differ 2-3×), so try each of the top
    # histogram peaks as a seed pitch and keep whichever produces the best
    # lattice fit below. Single-peak mode-picking misses angled boards.
    valid_nn = nn[(nn > 3) & (nn < 80)]
    if len(valid_nn) < 4:
        return None
    hist_p, edges_p = np.histogram(valid_nn, bins=24, range=(3, 80))
    # Candidate pitches: top-3 peaks + median (safety net if histogram is flat)
    top_pitch_bins = np.argsort(hist_p)[::-1][:3]
    pitch_candidates = []
    for b in top_pitch_bins:
        if hist_p[b] < 2:
            continue
        pitch_candidates.append(0.5 * (edges_p[b] + edges_p[b + 1]))
    pitch_candidates.append(float(np.median(valid_nn)))
    # Deduplicate (within 1 px)
    pitch_candidates = sorted({round(p, 1) for p in pitch_candidates if p >= 4.0})
    if not pitch_candidates:
        return None

    tol_ang = np.radians(20)
    # Max plausible span: board dimensions + small slack for partial views that
    # wrap or blobs that round into a boundary cell.
    max_span = max(rows, cols) + 2
    best_result = None  # (score, inliers_mask, grid_coords, resid, Binv, origin)
    best_pitch = None

    for pitch in pitch_candidates:
        # Wider band than before (0.4..2.2×pitch) so one seed pitch still catches
        # the neighbour pairs on the opposite-perspective side of an angled
        # board, where apparent pitch varies 2-3× between near and far sides.
        mask = (dists > 0.4 * pitch) & (dists < 2.2 * pitch)
        ii, jj = np.where(mask)
        sel = ii < jj
        ii, jj = ii[sel], jj[sel]
        if len(ii) < 4:
            continue
        deltas = centers[jj] - centers[ii]
        fold = deltas[:, 1] < 0
        deltas[fold] = -deltas[fold]
        angles = np.arctan2(deltas[:, 1], deltas[:, 0])
        hist, edges = np.histogram(angles, bins=36, range=(0, np.pi))
        # Try the top-K angle peaks (+90°) as basis candidates.
        top_idx = np.argsort(hist)[::-1][:6]

        def _avg_near(angle_c: float):
            d = np.abs(((angles - angle_c + np.pi / 2) % np.pi) - np.pi / 2)
            sel = d < tol_ang
            if not sel.any():
                return None
            return deltas[sel].mean(axis=0)

        for p_idx in top_idx:
            if hist[p_idx] < 2:
                continue
            peak1 = 0.5 * (edges[p_idx] + edges[p_idx + 1])
            peak2 = (peak1 + np.pi / 2) % np.pi
            e_a = _avg_near(peak1)
            e_b = _avg_near(peak2)
            if e_a is None or e_b is None:
                continue
            B = np.stack([e_a, e_b], axis=1)
            if abs(np.linalg.det(B)) < 1e-3:
                continue
            Binv_try = np.linalg.inv(B)
            for o_idx in range(n):
                origin_try = centers[o_idx]
                rel_try = (centers - origin_try) @ Binv_try.T
                gc_try = np.round(rel_try).astype(int)
                resid_try = np.linalg.norm(rel_try - gc_try, axis=1)
                # Loosened rigid-basis residual — the basis is only a seed for
                # the homography expansion pass below, which handles strong
                # perspective. A too-tight residual here rejects the near/far
                # side of an angled board from even becoming a seed.
                mask_try = resid_try < max(residual_tol, 0.45)
                n_inl = int(mask_try.sum())
                if n_inl < 6:
                    continue
                inl_gc = gc_try[mask_try]
                span_a = int(inl_gc[:, 0].max() - inl_gc[:, 0].min()) + 1
                span_b = int(inl_gc[:, 1].max() - inl_gc[:, 1].min()) + 1
                if span_a > max_span or span_b > max_span:
                    continue
                # Score by pixel-area density, not integer-lattice density.
                # Gravel/tree clutter can form accidental integer-lattice
                # matches that look "dense" in grid-unit space but span
                # hundreds of pixels in image space. The board's cells are
                # physically compact — even a steeply-tilted 150×150 px board
                # has bounding-pixel-area << a 400×400 px scattered-clutter
                # lattice. n_inl² / pixel_area rewards compactness so the
                # board wins regardless of how many extra clutter blobs the
                # gate lets through.
                inl_pix = centers[mask_try]
                px_a = float(inl_pix[:, 0].max() - inl_pix[:, 0].min()) + 1.0
                px_b = float(inl_pix[:, 1].max() - inl_pix[:, 1].min()) + 1.0
                score = (n_inl * n_inl) / (px_a * px_b)
                if best_result is None or score > best_result[0]:
                    best_result = (score, mask_try, gc_try, resid_try, Binv_try, origin_try)
                    best_pitch = pitch

    if best_result is None:
        return None
    pitch = best_pitch
    _, inliers_mask, grid_coords, resid, Binv, origin = best_result

    # Deduplicate (best-residual wins per integer cell)
    best: Dict[tuple, int] = {}
    for i in np.where(inliers_mask)[0]:
        key = (int(grid_coords[i, 0]), int(grid_coords[i, 1]))
        if key not in best or resid[i] < resid[best[key]]:
            best[key] = int(i)
    if len(best) < 4:
        return None

    # Rigid-basis RANSAC grabs only the region where perspective is small. A
    # tilted board's far edge has smaller cells → rigid basis misses them. Fit
    # a homography from the seed correspondences and iterate to grow the set,
    # because H maps integer grid coords to image pixels under any perspective.
    try:
        seed_keys = np.array(list(best.keys()), dtype=np.float32)
        seed_idxs = np.array(list(best.values()))
        seed_pix = centers[seed_idxs].astype(np.float32)
        H = None
        if len(seed_keys) >= 4:
            H, _ = cv2.findHomography(seed_keys, seed_pix, cv2.RANSAC, 2.0)
        expanded: Dict[tuple, int] = dict(best)
        if H is not None:
            Hinv = np.linalg.inv(H)
            for _pass in range(3):
                # Project every blob back to grid coords via current H
                pix = np.concatenate([centers, np.ones((n, 1), dtype=np.float32)], axis=1)
                grid_h = pix @ Hinv.T
                grid_h = grid_h[:, :2] / grid_h[:, 2:3]
                g_round = np.round(grid_h).astype(int)
                g_resid = np.linalg.norm(grid_h - g_round, axis=1)
                new_map: Dict[tuple, tuple] = {}   # key -> (resid, idx)
                for i in range(n):
                    if g_resid[i] > residual_tol:
                        continue
                    k = (int(g_round[i, 0]), int(g_round[i, 1]))
                    if k not in new_map or g_resid[i] < new_map[k][0]:
                        new_map[k] = (float(g_resid[i]), i)
                if len(new_map) < 4:
                    break
                new_keys = np.array(list(new_map.keys()), dtype=np.float32)
                new_idxs = np.array([v[1] for v in new_map.values()])
                new_pix = centers[new_idxs].astype(np.float32)
                H_new, _ = cv2.findHomography(new_keys, new_pix, cv2.RANSAC, 2.0)
                if H_new is None:
                    break
                # Converge when inlier set stabilizes
                if len(new_map) == len(expanded) and set(new_map.keys()) == set(expanded.keys()):
                    expanded = {k: v[1] for k, v in new_map.items()}
                    H = H_new
                    Hinv = np.linalg.inv(H)
                    break
                expanded = {k: v[1] for k, v in new_map.items()}
                H = H_new
                Hinv = np.linalg.inv(H)
        best = expanded
    except Exception:
        pass
    if len(best) < 6:
        return None

    # Inliers can include scattered far-lattice false matches from env noise
    # (tree branches that happen to land on an integer lattice coord). Keep
    # only the dominant connected cluster in image space — the board's blobs
    # are physically contiguous; spurious matches are not.
    idxs_all = np.array(list(best.values()))
    keys_all = np.array(list(best.keys()))
    pts = centers[idxs_all]
    # Union-find by pairwise distance < 2*pitch — vectorized
    n_pts = len(pts)
    diffs_pts = pts[:, None, :] - pts[None, :, :]
    d_pts = np.sqrt(np.einsum("ijk,ijk->ij", diffs_pts, diffs_pts))
    np.fill_diagonal(d_pts, np.inf)
    pi_ii, pi_jj = np.where(d_pts < 2.0 * pitch)
    parent = list(range(n_pts))
    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x
    for i, j in zip(pi_ii.tolist(), pi_jj.tolist()):
        if i >= j:
            continue
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[ri] = rj
    groups: Dict[int, list] = {}
    for i in range(len(pts)):
        groups.setdefault(find(i), []).append(i)
    biggest = max(groups.values(), key=len)
    if len(biggest) < 6:
        return None
    keys_all = keys_all[biggest]
    idxs = idxs_all[biggest]

    mins = keys_all.min(axis=0)
    keys = keys_all - mins
    span = keys.max(axis=0) + 1                          # (span_a, span_b)

    # Axis assignment: pick whichever (rows, cols) orientation fits, with
    # +2 slack on each dim to tolerate off-by-one from lattice-edge blobs.
    if span[0] <= cols + 2 and span[1] <= rows + 2:
        col_of = keys[:, 0]
        row_of = keys[:, 1]
    elif span[0] <= rows + 2 and span[1] <= cols + 2:
        row_of = keys[:, 0]
        col_of = keys[:, 1]
    else:
        return None

    obj_pts = np.stack(
        [col_of * spacing_m, row_of * spacing_m, np.zeros_like(col_of)],
        axis=1,
    ).astype(np.float32)
    img_pts = centers[idxs].astype(np.float32)
    return obj_pts, img_pts, idxs.tolist()

# ---------------------------------------------------------------- ROS bridge

bridge = CvBridge()


def decode_any(msg: Image) -> Optional[np.ndarray]:
    """Decode a sensor_msgs/Image into a BGR uint8 for visualisation.
    Handles bayer_gbrg8 (Lucid), mono8/mono16 (thermal), bgr8/rgb8 (blackfly).
    """
    enc = (msg.encoding or "").lower()
    try:
        if enc in ("bgr8", "rgb8"):
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        elif enc.startswith("bayer_"):
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # bayer_gbrg8 -> OpenCV COLOR_BAYER_GR2BGR (swatbotics libs expect GR)
            bayer_map = {
                "bayer_gbrg8": cv2.COLOR_BAYER_GR2BGR,
                "bayer_grbg8": cv2.COLOR_BAYER_GB2BGR,
                "bayer_rggb8": cv2.COLOR_BAYER_BG2BGR,
                "bayer_bggr8": cv2.COLOR_BAYER_RG2BGR,
            }
            code = bayer_map.get(enc, cv2.COLOR_BAYER_GR2BGR)
            img = cv2.cvtColor(raw, code)
        elif enc == "mono8":
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
            img = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        elif enc == "mono16":
            raw = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
            # Stretch to 8-bit for display + detection
            lo, hi = np.percentile(raw, (1, 99))
            if hi <= lo:
                hi = lo + 1
            scaled = np.clip((raw - lo) * 255.0 / (hi - lo), 0, 255).astype(np.uint8)
            img = cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)
        else:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        return img
    except Exception as e:  # noqa: BLE001
        print(f"[decode] failed for encoding={enc}: {e}", flush=True)
        return None


# ---------------------------------------------------- Detection + coverage

def update_from_detections(detections, img_w: int, img_h: int) -> None:
    with state_lock:
        coverage.frame_total += 1
        coverage.last_tag_count = len(detections)
        if not detections:
            return
        coverage.frame_detected += 1

        # Image-space coverage
        for d in detections:
            cx, cy = d.center
            col = int(cx / img_w * GRID_COLS)
            row = int(cy / img_h * GRID_ROWS)
            if 0 <= row < GRID_ROWS and 0 <= col < GRID_COLS:
                coverage.grid[row, col] = True

        # Pose-variety proxies using only tag geometry (no intrinsics)
        # Use the LARGEST detected tag per frame for a single sample.
        def _area(d):
            c = d.corners.astype(np.float32)
            return cv2.contourArea(c)

        largest = max(detections, key=_area)
        c = largest.corners.astype(np.float32)

        # Distance proxy: mean side length in pixels (bigger = closer)
        sides = [np.linalg.norm(c[(i + 1) % 4] - c[i]) for i in range(4)]
        mean_side = float(np.mean(sides))
        # 3 bins — thresholds tuned to lucid binning=4 / thermal sensor scale.
        # Use relative thresholds against image diagonal so they work across
        # sensor resolutions (thermal 640x512, lucid 2336x1750).
        diag = float(np.hypot(img_w, img_h))
        frac = mean_side / diag
        if frac < 0.035:
            coverage.dist_bins[0] += 1  # far
        elif frac < 0.07:
            coverage.dist_bins[1] += 1  # medium
        else:
            coverage.dist_bins[2] += 1  # close

        # Tilt proxy: ratio of diagonals — symmetric-square tag projects
        # to a quad whose diagonal lengths equalize only at 0° tilt.
        d1 = np.linalg.norm(c[2] - c[0])
        d2 = np.linalg.norm(c[3] - c[1])
        tilt_ratio = min(d1, d2) / max(d1, d2, 1e-6)  # 1.0 = frontal, 0.5 = heavy tilt
        # Map [0.5, 1.0] -> bins [0..MIN_TILT_BINS-1]
        tb = int(np.clip((1.0 - tilt_ratio) * 2.0 * MIN_TILT_BINS, 0, MIN_TILT_BINS - 1))
        coverage.tilt_bins[tb] += 1

        # Roll proxy: angle of top edge
        top = c[1] - c[0]
        ang = float(np.degrees(np.arctan2(top[1], top[0])))  # -180..+180
        # Fold to [-90, 90] since tag is square (90° rotation is symmetric)
        ang = ((ang + 90) % 180) - 90
        rb = int(np.clip((ang + 90) / 180 * MIN_ROLL_BINS, 0, MIN_ROLL_BINS - 1))
        coverage.roll_bins[rb] += 1


def render_overlay(
    img: np.ndarray,
    detections,
    chessboard_shape: Optional[Tuple[int, int]] = None,
) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]

    # Coverage grid overlay
    with state_lock:
        grid = coverage.grid.copy()

    overlay = out.copy()
    for r in range(GRID_ROWS):
        for cc in range(GRID_COLS):
            y0 = int(r * h / GRID_ROWS)
            y1 = int((r + 1) * h / GRID_ROWS)
            x0 = int(cc * w / GRID_COLS)
            x1 = int((cc + 1) * w / GRID_COLS)
            color = (0, 200, 0) if grid[r, cc] else (60, 60, 200)
            cv2.rectangle(overlay, (x0, y0), (x1 - 1, y1 - 1), color, 1)
    cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)

    # Rainbow chessboard overlay when a circle-grid lock is active.
    if chessboard_shape is not None and detections:
        pts = np.stack([d.center for d in detections]).astype(np.float32)
        expected = int(chessboard_shape[0]) * int(chessboard_shape[1])
        if pts.shape[0] == expected:
            try:
                cv2.drawChessboardCorners(out, chessboard_shape, pts, True)
            except Exception:
                pass

    # Tag outlines
    for d in detections:
        pts = d.corners.astype(int)
        cv2.polylines(out, [pts], True, (0, 255, 255), 2)
        cv2.circle(out, tuple(d.center.astype(int)), 4, (0, 255, 255), -1)

    # Top-line HUD
    with state_lock:
        txt = (
            f"frames {coverage.frame_total}  detected {coverage.frame_detected}  "
            f"tags {coverage.last_tag_count}  coverage {coverage.grid.mean()*100:.0f}%"
        )
    cv2.rectangle(out, (0, 0), (w, 28), (0, 0, 0), -1)
    cv2.putText(out, txt, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------- ROS node

class CalibNode(Node):
    def __init__(self) -> None:
        super().__init__("calib_ui_node")
        self._sub = None
        self._topic = None

    def switch_topic(self, topic: str) -> None:
        if self._sub is not None:
            self.destroy_subscription(self._sub)
            self._sub = None
        if not topic:
            self._topic = None
            return
        # BEST_EFFORT matches sensor-data publishers (lucid/blackfly/thermal)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._sub = self.create_subscription(Image, topic, self._on_image, qos)
        self._topic = topic
        print(f"[ros] subscribed topic={topic}", flush=True)

    def _on_image(self, msg: Image) -> None:
        img = decode_any(msg)
        if img is None:
            return
        with state_lock:
            preset_key = session.preset_key
            recording = session.recording
        with preview_rotation_lock:
            deg = preview_rotation.get(preset_key or "", 0)
        if deg:
            img = _apply_rotation(img, deg)
        h, w = img.shape[:2]
        with state_lock:
            session.image_width, session.image_height = w, h
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        target = "apriltag_calibio"
        if preset_key and preset_key in CAMERA_PRESETS:
            target = str(CAMERA_PRESETS[preset_key].get("target", "apriltag_calibio"))
        with preview_target_lock:
            ov = preview_target_override.get(preset_key or "")
        if ov:
            target = ov

        dets: list = []
        circle_matched: Optional[Tuple[int, int]] = None
        if target == "thermal_foil_16x12":
            try:
                dets = detect_thermal_grid(gray)
            except Exception as e:  # noqa: BLE001
                print(f"[thermal-detect] {e}", flush=True)
                dets = []
            # Raw contour detection picks up anything dark — the warm body, the
            # cold floor, environment clutter. Filter through the lattice fit
            # so preview only shows blobs that land on the 16×12 grid; anything
            # off-lattice is discarded before render_overlay and coverage update.
            if dets:
                try:
                    fit = fit_thermal_lattice(dets)
                except Exception as e:  # noqa: BLE001
                    print(f"[thermal-fit] {e}", flush=True)
                    fit = None
                if fit is not None:
                    obj_pts, img_pts, inlier_idxs = fit
                    dets = [dets[i] for i in inlier_idxs]
                    if recording:
                        with state_lock:
                            session.thermal_captures.append((obj_pts, img_pts))
                else:
                    dets = []
        elif target == "thermal_circles_asym":
            try:
                dets, matched = detect_circle_grid(gray, asymmetric=True)
            except Exception as e:  # noqa: BLE001
                print(f"[circles-detect] {e}", flush=True)
                dets, matched = [], None
            circle_matched = matched
            if dets and matched is not None and recording:
                obj_pts = circle_grid_object_points(
                    matched, THERMAL_CIRCLES_SPACING_M, asymmetric=True,
                )
                img_pts = np.stack([d.center for d in dets]).astype(np.float32)
                with state_lock:
                    session.thermal_captures.append((obj_pts, img_pts))
        elif target in CHECKER_TARGETS:
            try:
                dets = detect_checkerboard(gray, CHECKER_TARGETS[target])
            except Exception as e:  # noqa: BLE001
                print(f"[checker-detect] {e}", flush=True)
                dets = []
        else:
            # When the Lucid bayer sensor blows out white squares to 255, apriltag3's
            # quad-threshold step can't find tag boundaries and returns 0. If > 5% of
            # the frame is clipped, fold highlights down to the 85th percentile and
            # renormalize — empirically recovers 12-19 tags on frames where the raw
            # input detects 0 (see /tmp/detector_clip_sweep.py for the evidence).
            sat_frac = float((gray > 250).mean())
            if sat_frac > 0.05:
                # Fixed cap: on heavily-blown-out frames (p85 often already 255)
                # a percentile-derived cap collapses to no-op. Empirical sweep
                # showed cap=100-160 recovers 12-19 tags (0 without).
                cap = 110
                clipped = np.minimum(gray, cap)
                det_input = cv2.normalize(clipped, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            else:
                det_input = gray
            try:
                # pupil_apriltags.Detector is not thread-safe; serialize calls
                with detector_lock:
                    dets = detector.detect(det_input)
            except Exception as e:  # noqa: BLE001
                print(f"[detect] {e}", flush=True)
                dets = []
            # One-shot raw-frame dump for off-line detector debugging.
            # Trigger via: curl -XPOST http://localhost:8091/debug/dump
            if _debug_dump_request.is_set():
                try:
                    cv2.imwrite("/tmp/calib_raw_gray.png", gray)
                    cv2.imwrite("/tmp/calib_raw_bgr.png", img)
                    print(f"[debug] dumped raw frame {w}x{h}, dets={len(dets)}", flush=True)
                except Exception as e:  # noqa: BLE001
                    print(f"[debug] dump failed: {e}", flush=True)
                _debug_dump_request.clear()

        update_from_detections(dets, w, h)
        with last_bgr_lock:
            global last_bgr
            last_bgr = img.copy()
        over = render_overlay(img, dets, chessboard_shape=circle_matched)

        # Downsize very large frames (lucid) before JPEG to keep MJPEG snappy.
        if over.shape[1] > 1280:
            scale = 1280.0 / over.shape[1]
            over = cv2.resize(over, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

        ok, enc = cv2.imencode(".jpg", over, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
        if ok:
            global last_jpeg
            with last_jpeg_lock:
                last_jpeg = enc.tobytes()


ros_node: Optional[CalibNode] = None
ros_stop_event = threading.Event()


def ros_thread() -> None:
    global ros_node
    rclpy.init(args=None)
    ros_node = CalibNode()
    executor = SingleThreadedExecutor()
    executor.add_node(ros_node)
    while not ros_stop_event.is_set():
        try:
            executor.spin_once(timeout_sec=0.1)
        except Exception as e:  # noqa: BLE001
            print(f"[ros] {e}", flush=True)
            time.sleep(0.1)
    executor.shutdown()
    ros_node.destroy_node()
    rclpy.shutdown()


# --------------------------------------------------------------- Recording

def start_recording(preset_key: str, session_name: str) -> Dict[str, object]:
    preset = CAMERA_PRESETS[preset_key]
    sess_dir = CALIB_BASE / session_name
    sess_dir.mkdir(parents=True, exist_ok=True)
    bag_tmp = sess_dir / "raw_mcap_tmp"
    if bag_tmp.exists():
        shutil.rmtree(bag_tmp)

    topics = [preset["image_topic"], preset["info_topic"]]
    cmd = [
        "bash", "-lc",
        (
            "source /opt/ros/jazzy/setup.bash && "
            f"exec ros2 bag record --storage mcap "
            f"--output {bag_tmp} "
            f"--qos-profile-overrides-path /config/rosbag_qos.yaml "
            + " ".join(topics)
        ),
    ]
    # rosbag_qos.yaml may be absent; drop that flag if the file doesn't exist
    qos_path = Path("/config/rosbag_qos.yaml")
    if not qos_path.exists():
        cmd[2] = cmd[2].replace("--qos-profile-overrides-path /config/rosbag_qos.yaml ", "")

    proc = subprocess.Popen(
        cmd,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid,
    )
    with state_lock:
        session.recording = True
        session.record_proc = proc
        session.bag_tmp_dir = bag_tmp
        session.session_name = session_name
        session.preset_key = preset_key
        session.image_topic = str(preset["image_topic"])
        session.info_topic = str(preset["info_topic"])
        session.thermal_captures = []
        coverage.reset()
    if ros_node:
        ros_node.switch_topic(str(preset["image_topic"]))
    return {"ok": True, "session": session_name, "topic": preset["image_topic"]}


def stop_recording() -> Dict[str, object]:
    with state_lock:
        proc = session.record_proc
        bag_tmp = session.bag_tmp_dir
        sess_name = session.session_name
    if proc is None or bag_tmp is None or sess_name is None:
        return {"ok": False, "err": "not recording"}
    # SIGINT so rosbag2 flushes its cache cleanly
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        proc.wait(timeout=15)
    except Exception as e:  # noqa: BLE001
        print(f"[stop] {e}", flush=True)
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass

    # Move MCAP out of tmp dir into sess_dir/raw.mcap
    sess_dir = CALIB_BASE / sess_name
    mcaps = list(bag_tmp.glob("*.mcap")) if bag_tmp.exists() else []
    if mcaps:
        final = sess_dir / "raw.mcap"
        shutil.move(str(mcaps[0]), str(final))
        shutil.rmtree(bag_tmp, ignore_errors=True)
        size_mb = final.stat().st_size / 1e6
    else:
        size_mb = 0.0

    # Manifest
    with state_lock:
        manifest = {
            "session": sess_name,
            "preset": session.preset_key,
            "image_topic": session.image_topic,
            "info_topic": session.info_topic,
            "frame_total": coverage.frame_total,
            "frame_detected": coverage.frame_detected,
            "coverage_pct": float(coverage.grid.mean()),
            "image_width": session.image_width,
            "image_height": session.image_height,
        }
    (sess_dir / "manifest.yaml").write_text(yaml.safe_dump(manifest))

    # Convert MCAP -> ROS1 bag for Kalibr in-place (we are already the util container)
    if (sess_dir / "raw.mcap").exists():
        # rosbags-convert refuses to overwrite; purge any stale artefact from a prior run
        # so re-recording into the same session dir doesn't silently reuse the old bag.
        stale = sess_dir / "raw.bag"
        if stale.exists():
            try:
                stale.unlink()
            except Exception as e:  # noqa: BLE001
                print(f"[convert] could not remove stale raw.bag: {e}", flush=True)
        try:
            subprocess.run(
                ["rosbags-convert", "--src", str(sess_dir / "raw.mcap"),
                 "--dst", str(sess_dir / "raw.bag")],
                check=True,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[convert] {e}", flush=True)

    with state_lock:
        session.recording = False
        session.record_proc = None
        session.bag_tmp_dir = None

    return {"ok": True, "mcap_mb": size_mb, "session_dir": str(sess_dir)}


# --------------------------------------------------------------- Kalibr

def run_kalibr_background() -> Dict[str, object]:
    with state_lock:
        sess_name = session.session_name
        preset_key = session.preset_key
        topic = session.image_topic
    if not sess_name or not preset_key or not topic:
        return {"ok": False, "err": "no session"}
    preset = CAMERA_PRESETS[preset_key]
    sess_dir = CALIB_BASE / sess_name
    if not (sess_dir / "raw.bag").exists():
        return {"ok": False, "err": "raw.bag missing — did you stop recording?"}

    # Pick target yaml: UX override > preset default > aprilgrid
    with preview_target_lock:
        ov = preview_target_override.get(preset_key)
    active_target = ov or str(preset.get("target") or "apriltag_calibio")
    target_yaml = {
        "apriltag_calibio": "aprilgrid_calibio.yaml",
        "checker_18x25_30mm": "checker_18x25_30mm.yaml",
    }.get(active_target, "aprilgrid_calibio.yaml")

    host_sess_dir = CALIB_HOST_BASE / sess_name
    cmd = [
        "docker", "run", "--rm",
        "-v", f"{host_sess_dir}:/data",
        "-v", "/home/thor/bess/config/calib:/config:ro",
        "-e", "KALIBR_MANUAL_FOCAL_LENGTH_INIT=1",
        "localhost/bess-calib-kalibr:noetic",
        "bash", "-lc",
        (
            ". /opt/ros/noetic/setup.bash && . /catkin_ws/devel/setup.bash && cd /data && "
            f"echo '{preset['focal_seed']}' | rosrun kalibr kalibr_calibrate_cameras "
            f"--target /config/{target_yaml} "
            f"--bag /data/raw.bag "
            f"--models {preset['model']} "
            f"--topics {topic} "
            "--dont-show-report 2>&1 | tee /data/kalibr.log"
        ),
    ]
    with state_lock:
        session.kalibr_log = [f"$ {' '.join(cmd)}"]
        session.kalibr_result = None

    def pump():
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        with state_lock:
            session.kalibr_proc = proc
        for line in iter(proc.stdout.readline, ""):
            with state_lock:
                session.kalibr_log.append(line.rstrip())
                if len(session.kalibr_log) > 1000:
                    session.kalibr_log = session.kalibr_log[-1000:]
        proc.wait()
        with state_lock:
            session.kalibr_proc = None
            # Pull key result lines if camchain yaml was written
            camchain = sess_dir / "raw-camchain.yaml"
            if camchain.exists():
                session.kalibr_result = camchain.read_text()

    threading.Thread(target=pump, daemon=True).start()
    return {"ok": True}


# ----------------------------------------------- Thermal (OpenCV direct)

def run_thermal_calibrate_background() -> Dict[str, object]:
    """Calibrate thermal cam intrinsics from captures accumulated during record.

    Kalibr has no first-class 16x12 foil target, so we go direct to OpenCV:
    - pinhole-equi  -> cv2.fisheye.calibrate (Kannala-Brandt)
    - pinhole-radtan -> cv2.calibrateCamera (Brown-Conrady / radtan)
    Output written to <sess>/thermal_intrinsics.yaml; streamed to kalibr_log
    so the UI log pane shows progress identically to a Kalibr run.
    """
    with state_lock:
        sess_name = session.session_name
        preset_key = session.preset_key
        captures = list(session.thermal_captures)
        img_w = session.image_width
        img_h = session.image_height
    if not sess_name or not preset_key:
        return {"ok": False, "err": "no session"}
    if len(captures) < 10:
        return {"ok": False, "err": f"need ≥10 good lattice frames, have {len(captures)}"}
    if img_w <= 0 or img_h <= 0:
        return {"ok": False, "err": "image size not observed"}

    preset = CAMERA_PRESETS[preset_key]
    model = str(preset.get("model", "pinhole-radtan"))
    target_name = str(preset.get("target", "thermal_foil_16x12"))
    # Foil grid spacing = 0.05; asymmetric circle grid spacing = 0.06.
    # The min-grid-span gate scales to whichever target captured these frames.
    spacing_for_gate = (
        THERMAL_CIRCLES_SPACING_M if target_name == "thermal_circles_asym"
        else THERMAL_SPACING_M
    )
    sess_dir = CALIB_BASE / sess_name

    def _log(line: str) -> None:
        with state_lock:
            session.kalibr_log.append(line)
            if len(session.kalibr_log) > 1000:
                session.kalibr_log = session.kalibr_log[-1000:]

    with state_lock:
        session.kalibr_log = [
            f"$ opencv calibrate model={model} frames={len(captures)} size={img_w}x{img_h}"
        ]
        session.kalibr_result = None

    def worker() -> None:
        try:
            obj_all = [c[0].astype(np.float32) for c in captures]
            img_all = [c[1].astype(np.float32) for c in captures]

            # Frame-quality gates. fisheye.calibrate's InitExtrinsics calls
            # findHomography internally on each view; any rank-deficient
            # homography (near-collinear board corners OR near-degenerate
            # image projection) crashes the decomposition with
            # fabs(norm_u1) > 0. Screen out those frames BEFORE solve.
            pairs: list = []
            drop_pts, drop_grid, drop_pix, drop_hom = 0, 0, 0, 0
            for o, i in zip(obj_all, img_all):
                if len(o) < 12:
                    drop_pts += 1
                    continue
                grid_x_span = float(o[:, 0].max() - o[:, 0].min())
                grid_y_span = float(o[:, 1].max() - o[:, 1].min())
                min_grid_span = 3.5 * spacing_for_gate  # ≥4 cells each axis
                if grid_x_span < min_grid_span or grid_y_span < min_grid_span:
                    drop_grid += 1
                    continue
                pix_x_span = float(i[:, 0].max() - i[:, 0].min())
                pix_y_span = float(i[:, 1].max() - i[:, 1].min())
                if pix_x_span < 80.0 or pix_y_span < 80.0:
                    drop_pix += 1
                    continue
                # Per-view homography conditioning check. Same math
                # fisheye.InitExtrinsics does; we just refuse to hand it a
                # view it can't decompose. Ratio of first/last singular
                # value > 1e4 = rank-deficient.
                try:
                    H, _ = cv2.findHomography(o[:, :2], i, 0)
                    if H is None:
                        drop_hom += 1
                        continue
                    sv = np.linalg.svd(H, compute_uv=False)
                    if sv[-1] < 1e-8 or sv[0] / sv[-1] > 1e4:
                        drop_hom += 1
                        continue
                except cv2.error:
                    drop_hom += 1
                    continue
                pairs.append((o, i))
            _log(
                f"frames: {len(obj_all)} total, {len(pairs)} kept "
                f"(dropped: pts={drop_pts} grid={drop_grid} pix={drop_pix} hom={drop_hom})"
            )
            if len(pairs) < 10:
                _log(f"FAIL: only {len(pairs)} usable frames, need ≥10")
                return
            obj_kept = [p[0] for p in pairs]
            img_kept = [p[1] for p in pairs]
            # Cap at ~60 frames — fisheye/calibrateCamera solvers don't benefit
            # from more and grind linearly. Uniform subsample preserves pose
            # diversity.
            MAX_CALIB_FRAMES = 60
            if len(obj_kept) > MAX_CALIB_FRAMES:
                idx = np.linspace(0, len(obj_kept) - 1, MAX_CALIB_FRAMES, dtype=int)
                obj_kept = [obj_kept[k] for k in idx]
                img_kept = [img_kept[k] for k in idx]
                _log(f"subsampled to {len(obj_kept)} frames (uniform)")
            model_used = model

            # Focal seed: prefer the preset's per-camera value (derived from
            # known FOV), else fall back to 0.6*max(W,H) (~80° FOV heuristic).
            # For the A70 95° thermal the preset gives 350; without it the
            # solver converges on f≈2400 (absurd telephoto).
            f_seed_cfg = preset.get("focal_seed")
            f_seed = float(f_seed_cfg) if f_seed_cfg else float(max(img_w, img_h)) * 0.6
            K_seed = np.array([
                [f_seed, 0.0, img_w / 2.0],
                [0.0, f_seed, img_h / 2.0],
                [0.0, 0.0, 1.0],
            ], dtype=np.float64)

            if model_used == "pinhole-equi":
                _log(f"running cv2.fisheye.calibrate (f_seed={f_seed:.0f}) …")
                D = np.zeros((4, 1), dtype=np.float64)
                obj_fisheye = [o.reshape(1, -1, 3).astype(np.float64) for o in obj_kept]
                img_fisheye = [i.reshape(1, -1, 2).astype(np.float64) for i in img_kept]
                term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 60, 1e-6)
                # Try in descending strictness. CALIB_CHECK_COND aborts on
                # any ill-conditioned frame (raises, reports index); without
                # it bad frames can crash InitExtrinsics. USE_INTRINSIC_GUESS
                # forces K_seed which stabilizes the per-view pose fit.
                attempts = [
                    ("CHECK_COND + GUESS",
                     cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                     | cv2.fisheye.CALIB_FIX_SKEW
                     | cv2.fisheye.CALIB_CHECK_COND
                     | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS),
                    ("GUESS only",
                     cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                     | cv2.fisheye.CALIB_FIX_SKEW
                     | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS),
                ]
                rms = None
                last_err: Optional[Exception] = None
                for label, flags in attempts:
                    try:
                        K = K_seed.copy()
                        D = np.zeros((4, 1), dtype=np.float64)
                        rms, K, D, _, _ = cv2.fisheye.calibrate(
                            obj_fisheye, img_fisheye, (img_w, img_h),
                            K, D, flags=flags, criteria=term,
                        )
                        _log(f"fisheye {label}: rms={rms:.3f}")
                        break
                    except cv2.error as e:
                        last_err = e
                        _log(f"fisheye {label} failed, trying next")
                if rms is None:
                    # Last resort: iteratively drop the single worst-
                    # conditioned remaining frame and retry. fisheye
                    # CALIB_CHECK_COND reports via exception which frame
                    # is offending; simplest robust approach is to keep
                    # trimming until it converges.
                    obj_list = list(obj_fisheye)
                    img_list = list(img_fisheye)
                    # Sort by homography conditioning worst-first so the
                    # first drop is most likely to fix it.
                    conds = []
                    for o3, i2 in zip(obj_kept, img_kept):
                        try:
                            H, _ = cv2.findHomography(o3[:, :2], i2, 0)
                            sv = np.linalg.svd(H, compute_uv=False) if H is not None else [1, 0]
                            conds.append(sv[0] / max(sv[-1], 1e-12))
                        except Exception:
                            conds.append(float("inf"))
                    order = np.argsort(conds)[::-1]
                    for drop_n in range(1, min(20, len(order))):
                        drop_set = set(order[:drop_n].tolist())
                        sub_obj = [obj_list[k] for k in range(len(obj_list)) if k not in drop_set]
                        sub_img = [img_list[k] for k in range(len(img_list)) if k not in drop_set]
                        if len(sub_obj) < 10:
                            break
                        try:
                            K = K_seed.copy()
                            D = np.zeros((4, 1), dtype=np.float64)
                            rms, K, D, _, _ = cv2.fisheye.calibrate(
                                sub_obj, sub_img, (img_w, img_h),
                                K, D,
                                flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
                                      | cv2.fisheye.CALIB_FIX_SKEW
                                      | cv2.fisheye.CALIB_USE_INTRINSIC_GUESS,
                                criteria=term,
                            )
                            _log(f"fisheye succeeded after dropping {drop_n} worst-conditioned frames: rms={rms:.3f}")
                            obj_kept = [obj_kept[k] for k in range(len(obj_kept)) if k not in drop_set]
                            img_kept = [img_kept[k] for k in range(len(img_kept)) if k not in drop_set]
                            break
                        except cv2.error as e:
                            last_err = e
                if rms is None:
                    _log("fisheye failed after trim retries; falling back to pinhole-radtan")
                    model_used = "pinhole-radtan"
                    K = K_seed.copy()
                    D = np.zeros((5, 1), dtype=np.float64)
                    rms, K, D, _, _ = cv2.calibrateCamera(
                        obj_kept, img_kept, (img_w, img_h), K, D,
                        flags=cv2.CALIB_USE_INTRINSIC_GUESS,
                    )
                D_flat = D.flatten().tolist()
            else:
                _log(f"running cv2.calibrateCamera (radtan, f_seed={f_seed:.0f}) …")
                K = K_seed.copy()
                D = np.zeros((5, 1), dtype=np.float64)
                # For narrow-FOV thermal lenses with modest frame counts,
                # letting the solver wander over 9 intrinsic params (fx/fy/cx/cy
                # + k1..k3, p1, p2) lets it drift into nonsense (cx outside
                # image, fx 5× sensor width). Pin principal point to image
                # center and drop k3 — a 17 mm lens is well within a 2-term
                # radial model.
                flags = (
                    cv2.CALIB_USE_INTRINSIC_GUESS
                    | cv2.CALIB_FIX_PRINCIPAL_POINT
                    | cv2.CALIB_FIX_K3
                )
                rms, K, D, _, _ = cv2.calibrateCamera(
                    obj_kept, img_kept, (img_w, img_h), K, D, flags=flags,
                )
                # Post-solve sanity. Reject obviously broken results so the
                # user knows to recapture, rather than writing a YAML that'll
                # silently corrupt downstream rectification / SLAM extrinsics.
                fx_val = float(K[0, 0])
                cx_val = float(K[0, 2])
                cy_val = float(K[1, 2])
                if not (0.25 * img_w <= fx_val <= 5.0 * img_w):
                    _log(f"FAIL: fx={fx_val:.0f} outside [0.25w, 5w] for w={img_w}")
                    return
                if not (0 <= cx_val <= img_w and 0 <= cy_val <= img_h):
                    _log(f"FAIL: principal point ({cx_val:.0f},{cy_val:.0f}) outside image")
                    return
                if rms > 2.0:
                    _log(f"WARN: high rms={rms:.2f}px — result kept but re-capture recommended")
                D_flat = D.flatten().tolist()

            out = {
                "camera_model": model_used,
                "image_size": [int(img_w), int(img_h)],
                "rms_reproj_px": float(rms),
                "n_frames_used": len(obj_kept),
                "n_frames_captured": len(captures),
                "intrinsics": {
                    "fx": float(K[0, 0]),
                    "fy": float(K[1, 1]),
                    "cx": float(K[0, 2]),
                    "cy": float(K[1, 2]),
                },
                "K": K.tolist(),
                "distortion_coeffs": D_flat,
                "target": (
                    {
                        "kind": "thermal_circles_asym",
                        "spacing_m": THERMAL_CIRCLES_SPACING_M,
                        "asymmetric": True,
                    }
                    if target_name == "thermal_circles_asym"
                    else {
                        "kind": "thermal_foil_16x12",
                        "rows": THERMAL_GRID_ROWS,
                        "cols": THERMAL_GRID_COLS,
                        "spacing_m": THERMAL_SPACING_M,
                    }
                ),
            }
            yaml_path = sess_dir / "thermal_intrinsics.yaml"
            yaml_path.write_text(yaml.safe_dump(out, sort_keys=False))
            _log(f"OK  rms={rms:.3f}px  fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
                 f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")
            _log(f"wrote {yaml_path}")
            with state_lock:
                session.kalibr_result = yaml_path.read_text()
        except Exception as e:  # noqa: BLE001
            _log(f"FAIL: {e!r}")

    threading.Thread(target=worker, daemon=True).start()
    return {"ok": True, "frames": len(captures)}


# --------------------------------------------------------------- FastAPI

app = FastAPI()


@app.middleware("http")
async def _no_store(request, call_next):
    # Stop the browser cache from serving stale app.js/index.html during UX
    # iteration. JSON/MJPEG endpoints already vary per-request; this is cheap
    # and covers everything.
    resp = await call_next(request)
    resp.headers["Cache-Control"] = "no-store, no-cache, must-revalidate"
    resp.headers["Pragma"] = "no-cache"
    return resp


app.mount("/static", StaticFiles(directory="/app/static"), name="static")


@app.get("/")
async def index():
    return FileResponse("/app/static/index.html")


@app.get("/presets")
async def presets():
    # Probe each preset's image topic for active publishers so the UI can dim
    # presets whose camera isn't on the wire (most common: one A6701 physically
    # absent while its partner is streaming).
    out: Dict[str, Dict[str, object]] = {}
    for k, v in CAMERA_PRESETS.items():
        topic = str(v["image_topic"])
        # DDS publisher-count is unreliable here: CycloneDDS leaves ghost
        # entries when a publisher dies unclean, AND cross-distro humble→jazzy
        # discovery reports real publishers as _NODE_NAME_UNKNOWN_. So we
        # always return live=true and surface "no frames" via the preview
        # instead (see _last_image_at in the WS state).
        live = True
        out[k] = {"image_topic": topic, "model": v["model"], "live": live}
    return out


class StartReq(BaseModel):
    preset: str
    session: str


@app.post("/start")
async def start(req: StartReq):
    if req.preset not in CAMERA_PRESETS:
        return JSONResponse({"ok": False, "err": "unknown preset"}, status_code=400)
    # Even if not recording, user can just "preview" by calling /preview; /start always records
    return start_recording(req.preset, req.session)


class PreviewReq(BaseModel):
    preset: str


@app.post("/preview")
async def preview(req: PreviewReq):
    if req.preset not in CAMERA_PRESETS:
        return JSONResponse({"ok": False, "err": "unknown preset"}, status_code=400)
    if ros_node:
        ros_node.switch_topic(str(CAMERA_PRESETS[req.preset]["image_topic"]))
    with state_lock:
        session.preset_key = req.preset
        session.image_topic = str(CAMERA_PRESETS[req.preset]["image_topic"])
        coverage.reset()
    # Default to auto-exposure on preset switch so framing isn't "wild" from whatever
    # value the operator locked in during the last session. Kalibr prefers a fixed
    # exposure, but the operator should dial that in explicitly before /start recording.
    ns = str(CAMERA_PRESETS[req.preset].get("ns", ""))
    if ns.startswith("lucid"):
        try:
            subprocess.run(
                ["docker", "exec", ns, "bash", "-lc",
                 "source /opt/ros/jazzy/setup.bash && "
                 "ros2 param set /lucid_camera_node exposure_us -1.0 && "
                 "ros2 param set /lucid_camera_node gain_db -1.0"],
                capture_output=True, text=True, timeout=10,
            )
        except Exception as e:  # noqa: BLE001
            print(f"[preview] auto-exposure restore failed: {e}", flush=True)
    return {"ok": True, "topic": CAMERA_PRESETS[req.preset]["image_topic"]}


@app.post("/stop")
async def stop():
    return stop_recording()


class ExposureReq(BaseModel):
    exposure_us: float
    gain_db: float
    preset: Optional[str] = None


@app.post("/exposure")
async def set_exposure(req: ExposureReq):
    with state_lock:
        key = req.preset or session.preset_key
    if not key or key not in CAMERA_PRESETS:
        return JSONResponse({"ok": False, "err": "no active preset"}, status_code=400)
    ns = str(CAMERA_PRESETS[key].get("ns", ""))
    # For Lucid we shell `docker exec <ns> ros2 param set` — the lucid node's
    # on_set_parameters callback applies the change to the Arena SDK live.
    if ns.startswith("lucid"):
        cmds = [
            ["docker", "exec", ns, "bash", "-lc",
             f"source /opt/ros/jazzy/setup.bash && "
             f"ros2 param set /lucid_camera_node exposure_us {req.exposure_us} && "
             f"ros2 param set /lucid_camera_node gain_db {req.gain_db}"],
        ]
        try:
            for c in cmds:
                out = subprocess.run(c, capture_output=True, text=True, timeout=15)
                if out.returncode != 0:
                    return JSONResponse({
                        "ok": False, "err": out.stderr.strip() or out.stdout.strip()
                    }, status_code=500)
        except subprocess.TimeoutExpired:
            return JSONResponse({"ok": False, "err": "ros2 param set timed out"}, status_code=504)
        return {"ok": True, "ns": ns, "exposure_us": req.exposure_us, "gain_db": req.gain_db}
    # For Blackfly, spinnaker_camera_driver exposes GenICam params as dynamic
    # ROS 2 parameters via /config/blackfly_s_thor.yaml. `ros2 param set` CLI
    # precheck fails because the node isn't enumerated via humble discovery in
    # this container's DDS view, but the set_parameters service *is* reachable.
    # We call it directly. Semantics match Lucid: -1 → auto, else explicit.
    if ns.startswith("blackfly"):
        cam_idx = "1" if ns == "blackfly1" else "2"
        node_fqn = f"/blackfly/camera{cam_idx}/blackfly_camera"
        params = []
        exp_auto = "Continuous" if req.exposure_us < 0 else "Off"
        gain_auto = "Continuous" if req.gain_db < 0 else "Off"
        params.append(f'{{name: exposure_auto, value: {{type: 4, string_value: \\"{exp_auto}\\"}}}}')
        if req.exposure_us >= 0:
            params.append(f'{{name: exposure_time, value: {{type: 3, double_value: {req.exposure_us}}}}}')
        params.append(f'{{name: gain_auto, value: {{type: 4, string_value: \\"{gain_auto}\\"}}}}')
        if req.gain_db >= 0:
            params.append(f'{{name: gain, value: {{type: 3, double_value: {req.gain_db}}}}}')
        payload = f'{{parameters: [{", ".join(params)}]}}'
        cmd = ["docker", "exec", ns, "bash", "-lc",
               f"source /opt/ros/humble/setup.bash && "
               f"export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
               f"export CYCLONEDDS_URI=file:///config/cyclonedds.xml && "
               f"timeout 10 ros2 service call {node_fqn}/set_parameters "
               f"rcl_interfaces/srv/SetParameters \"{payload}\" 2>&1 | tail -5"]
        try:
            out = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
        except subprocess.TimeoutExpired:
            return JSONResponse({"ok": False, "err": "blackfly set_parameters timed out"}, status_code=504)
        blob = (out.stdout or "") + (out.stderr or "")
        if "successful=True" not in blob:
            return JSONResponse({"ok": False, "err": blob.strip()[:500]}, status_code=500)
        return {"ok": True, "ns": ns, "exposure_us": req.exposure_us, "gain_db": req.gain_db,
                "node": node_fqn}
    return JSONResponse({
        "ok": False, "err": f"live exposure not wired for preset ns={ns}; edit compose and restart"
    }, status_code=501)


class RotateReq(BaseModel):
    preset: Optional[str] = None
    deg: Optional[int] = None  # if None, cycle 0→90→180→270→0


@app.post("/rotate")
async def rotate(req: RotateReq):
    # Default target is whatever the preview is currently bound to.
    with state_lock:
        key = req.preset or session.preset_key
    if not key or key not in CAMERA_PRESETS:
        return JSONResponse({"ok": False, "err": "no active preset"}, status_code=400)
    with preview_rotation_lock:
        cur = preview_rotation.get(key, 0)
        if req.deg is None:
            new = (cur + 90) % 360
        else:
            new = int(req.deg) % 360
            if new not in (0, 90, 180, 270):
                return JSONResponse(
                    {"ok": False, "err": "deg must be 0/90/180/270"}, status_code=400
                )
        preview_rotation[key] = new
    # Reset coverage since rotated image has different tilt/roll geometry
    with state_lock:
        coverage.reset()
    return {"ok": True, "preset": key, "rotation": new}


@app.post("/calibrate")
async def calibrate():
    with state_lock:
        preset_key = session.preset_key
    target = CAMERA_PRESETS.get(preset_key, {}).get("target") if preset_key else None
    if target in ("thermal_foil_16x12", "thermal_circles_asym"):
        return run_thermal_calibrate_background()
    return run_kalibr_background()


@app.get("/stream.mjpg")
async def mjpeg():
    boundary = b"--frame"

    async def gen():
        while True:
            with last_jpeg_lock:
                frame = last_jpeg
            if frame is not None:
                yield boundary + b"\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            await asyncio.sleep(1 / 15.0)

    return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")


def _load_calib_from_session(sess_dir: Path) -> Optional[tuple]:
    """Return (K, D, model) loaded from either the thermal YAML or a Kalibr
    camchain under sess_dir, or None if neither is present.
    """
    thermal_yaml = sess_dir / "thermal_intrinsics.yaml"
    camchain = sess_dir / "raw-camchain.yaml"
    if thermal_yaml.exists():
        c = yaml.safe_load(thermal_yaml.read_text()) or {}
        K = np.array(c.get("K"), dtype=np.float64)
        D = np.array(c.get("distortion_coeffs"), dtype=np.float64)
        model = str(c.get("camera_model", "pinhole-radtan"))
        return K, D, model
    if camchain.exists():
        c = yaml.safe_load(camchain.read_text()) or {}
        cam = c.get("cam0") or (next(iter(c.values())) if c else None)
        if not cam:
            return None
        fx, fy, cx, cy = cam["intrinsics"]
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        D = np.array(cam["distortion_coeffs"], dtype=np.float64)
        dist_model = str(cam.get("distortion_model", "radtan")).lower()
        # Kalibr uses "equidistant" for fisheye / Kannala-Brandt
        model = "pinhole-equi" if ("equi" in dist_model or "fisheye" in dist_model) else "pinhole-radtan"
        return K, D, model
    return None


def _latest_session_dir(preset_key: Optional[str]) -> Optional[Path]:
    """Most recent session directory for a preset that has a calibration yaml.

    Used to re-adopt a completed calibration after a calib-ui restart (which
    wipes in-memory session state). Matches directories by prefix
    `<preset_key>_intrinsics_` so a70_right doesn't pick up lucid_right.
    """
    if not preset_key or not CALIB_BASE.exists():
        return None
    prefix = f"{preset_key}_intrinsics_"
    candidates = [
        p for p in CALIB_BASE.iterdir()
        if p.is_dir() and p.name.startswith(prefix)
        and (p / "thermal_intrinsics.yaml").exists()
    ]
    if not candidates:
        # Fall back to any camchain.yaml (Kalibr-path calibrations).
        candidates = [
            p for p in CALIB_BASE.iterdir()
            if p.is_dir() and p.name.startswith(prefix)
            and (p / "camchain.yaml").exists()
        ]
    if not candidates:
        return None
    candidates.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return candidates[0]


def _latest_session_yaml_text(preset_key: Optional[str]) -> Optional[str]:
    sess_dir = _latest_session_dir(preset_key)
    if sess_dir is None:
        return None
    for name in ("thermal_intrinsics.yaml", "camchain.yaml"):
        p = sess_dir / name
        if p.exists():
            try:
                return p.read_text()
            except Exception:
                return None
    return None


@app.get("/rectified.jpg")
async def rectified_jpg():
    """Side-by-side RAW | RECTIFIED JPEG using the active session's calibration.
    Falls back to the most recent completed session for the active preset so the
    rectified view works after a calib-ui restart. Returns 404 until a calibration
    yaml exists, 503 if no preview frame yet.
    """
    with state_lock:
        sess_name = session.session_name
        preset_key = session.preset_key
    sess_dir: Optional[Path] = None
    if sess_name:
        sess_dir = CALIB_BASE / sess_name
    if sess_dir is None or not (sess_dir / "thermal_intrinsics.yaml").exists():
        sess_dir = _latest_session_dir(preset_key)
    if sess_dir is None:
        return JSONResponse({"ok": False, "err": "no session with calibration"}, status_code=404)
    calib = _load_calib_from_session(sess_dir)
    if calib is None:
        return JSONResponse({"ok": False, "err": "no calibration yaml in session"}, status_code=404)
    K, D, model = calib
    with last_bgr_lock:
        frame = None if last_bgr is None else last_bgr.copy()
    if frame is None:
        return JSONResponse({"ok": False, "err": "no live frame yet"}, status_code=503)
    h, w = frame.shape[:2]
    try:
        if model == "pinhole-equi":
            D4 = D[:4].reshape(-1, 1).astype(np.float64)
            K_new = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
                K, D4, (w, h), np.eye(3), balance=0.0
            )
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                K, D4, np.eye(3), K_new, (w, h), cv2.CV_16SC2
            )
        else:
            K_new, _ = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 0.0, (w, h))
            map1, map2 = cv2.initUndistortRectifyMap(
                K, D, None, K_new, (w, h), cv2.CV_16SC2
            )
        rect = cv2.remap(frame, map1, map2, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    except Exception as e:  # noqa: BLE001
        return JSONResponse({"ok": False, "err": f"undistort failed: {e!r}"}, status_code=500)
    # Label bars above each half so the operator can tell which is which
    bar_h = 28
    label_raw = np.zeros((bar_h, w, 3), dtype=np.uint8)
    label_rect = np.zeros((bar_h, w, 3), dtype=np.uint8)
    cv2.putText(label_raw, "RAW", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(label_rect, f"RECTIFIED  ({model})", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    side = np.hstack([np.vstack([label_raw, frame]), np.vstack([label_rect, rect])])
    ok, enc = cv2.imencode(".jpg", side, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if not ok:
        return JSONResponse({"ok": False, "err": "jpeg encode failed"}, status_code=500)
    return Response(
        content=enc.tobytes(),
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store"},
    )


def _state_snapshot() -> Dict[str, object]:
    with state_lock:
        grid_hit = int(coverage.grid.sum())
        grid_total = int(coverage.grid.size)
        dist_hit = int((coverage.dist_bins >= MIN_FRAMES_PER_BIN).sum())
        tilt_hit = int((coverage.tilt_bins >= MIN_FRAMES_PER_BIN).sum())
        roll_hit = int((coverage.roll_bins >= MIN_FRAMES_PER_BIN).sum())
        return {
            "frame_total": coverage.frame_total,
            "frame_detected": coverage.frame_detected,
            "last_tag_count": coverage.last_tag_count,
            "grid": coverage.grid.astype(int).tolist(),
            "grid_hit": grid_hit,
            "grid_total": grid_total,
            "grid_pct": grid_hit / max(grid_total, 1),
            "dist_bins": coverage.dist_bins.tolist(),
            "dist_hit": dist_hit,
            "tilt_bins": coverage.tilt_bins.tolist(),
            "tilt_hit": tilt_hit,
            "roll_bins": coverage.roll_bins.tolist(),
            "roll_hit": roll_hit,
            "min_bin_frames": MIN_FRAMES_PER_BIN,
            "min_dist": MIN_DISTANCE_BINS,
            "min_tilt": MIN_TILT_BINS,
            "min_roll": MIN_ROLL_BINS,
            "min_grid_pct": MIN_GRID_HIT_PCT,
            "target_frames": TARGET_DETECTED_FRAMES,
            "recording": session.recording,
            "session": session.session_name,
            "preset": session.preset_key,
            "image_topic": session.image_topic,
            "image_width": session.image_width,
            "image_height": session.image_height,
            "rotation": preview_rotation.get(session.preset_key or "", 0),
            "target": (
                preview_target_override.get(session.preset_key or "")
                or (CAMERA_PRESETS.get(session.preset_key or "", {}).get("target") if session.preset_key else None)
                or "apriltag_calibio"
            ),
            "targets_available": AVAILABLE_TARGETS,
            "kalibr_running": session.kalibr_proc is not None,
            "kalibr_log_tail": session.kalibr_log[-40:],
            "kalibr_result": (
                session.kalibr_result
                or _latest_session_yaml_text(session.preset_key)
            ),
            "ready_to_finish": (
                grid_hit / max(grid_total, 1) >= MIN_GRID_HIT_PCT
                and dist_hit >= MIN_DISTANCE_BINS
                and tilt_hit >= MIN_TILT_BINS
                and roll_hit >= MIN_ROLL_BINS
                and coverage.frame_detected >= TARGET_DETECTED_FRAMES
            ),
        }


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(_state_snapshot())
            await asyncio.sleep(0.3)
    except WebSocketDisconnect:
        return


@app.get("/state")
async def state_http():
    return _state_snapshot()


@app.post("/debug/dump")
async def debug_dump():
    _debug_dump_request.set()
    return {"ok": True, "hint": "check /tmp/calib_raw_{gray,bgr}.png after next frame"}


class TargetReq(BaseModel):
    preset: Optional[str] = None
    target: Optional[str] = None  # null = clear override (use preset default)


@app.post("/target")
async def set_target(req: TargetReq):
    with state_lock:
        preset_key = req.preset or session.preset_key
    if not preset_key or preset_key not in CAMERA_PRESETS:
        return {"ok": False, "err": "no active preset"}
    if req.target is not None and req.target not in AVAILABLE_TARGETS:
        return {"ok": False, "err": f"unknown target {req.target}"}
    with preview_target_lock:
        preview_target_override[preset_key] = req.target
    # Reset coverage so bins reflect the new target
    with state_lock:
        coverage.grid[:] = False
        coverage.dist_bins[:] = 0
        coverage.tilt_bins[:] = 0
        coverage.roll_bins[:] = 0
        coverage.frame_total = 0
        coverage.frame_detected = 0
        coverage.last_tag_count = 0
    return {"ok": True, "preset": preset_key, "target": req.target, "available": AVAILABLE_TARGETS}


# ---------------------------------------------------------------- main

def main() -> None:
    t = threading.Thread(target=ros_thread, daemon=True)
    t.start()
    uvicorn.run(app, host="0.0.0.0", port=8091, log_level="warning")


if __name__ == "__main__":
    main()
