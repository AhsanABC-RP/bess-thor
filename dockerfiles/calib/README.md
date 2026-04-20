# BESS Calibration Containers

Two off-stack containers for sensor calibration work. Never started by the
sequenced launcher; run ad-hoc via `docker run` when you have capture data to
process.

| Image | Purpose | Base |
|-------|---------|------|
| `localhost/bess-calib-kalibr:noetic` | Multi-cam intrinsics (pinhole-radtan + pinhole-equi + double-sphere), IMU-camera spatiotemporal, AprilGrid target. ROS Noetic only | `arm64v8/ros:noetic` |
| `localhost/bess-calib-util:jazzy`    | MCAP ↔ ROS 1 `.bag` conversion (`rosbags-convert`), OpenCV fisheye scripts | `bess-base:jazzy` |

Separate `bess-calib-dvlc` (koide3/direct_visual_lidar_calibration, ROS 2
Humble) will be added when we start lidar↔camera extrinsic calibration.

## Camera L/R identity — READ BEFORE ORDERING STEREO ARGS

Container names (`blackflyN`, `thermalN`, `lucidN`) **do NOT track left/right**.
The identity-of-truth is `config/camera_rig.yaml` (hand-wave test, 2026-04-18).
Container renames are a deferred follow-up.

| Side | Lucid | Blackfly S | A6701 (MWIR) | A70 (LWIR) |
|------|-------|------------|--------------|------------|
| **left**  | `lucid1` | `blackfly2` | `thermal2` | `thermal4` |
| **right** | `lucid2` | `blackfly1` | `thermal1` | `thermal3` |

3 of 4 pairs are crossed — only Lucid has container# matching side.
When a stereo-pair example below reads `cam-stereo <sess> blackfly1 blackfly2`,
that is `right left` order. Pass the pair in whatever order you want (Kalibr
does not care), but label the output yaml accordingly so the `T_cn_cnm1`
extrinsic is not read upside-down downstream.

## Building

```bash
cd /home/thor/bess/dockerfiles/calib
docker build -t localhost/bess-calib-kalibr:noetic -f Containerfile.kalibr .
docker build -t localhost/bess-calib-util:jazzy    -f Containerfile.util    .

# Web UI (FastAPI + live preview + OpenCV solver for thermal circles)
docker build -t localhost/bess-calib-ui:jazzy \
    -f /home/thor/bess/containers/calib-ui/Containerfile \
    /home/thor/bess/containers/calib-ui
```

The Kalibr build is ~30–45 min on Thor (aslam_optimizer + aslam_cameras are
the long catkin packages). The util container and UI are a few minutes each.

## Web UI workflow (preferred)

For per-camera intrinsics, use the browser UI at `http://<thor>:8091`. It runs
live preview, coverage tracking (grid/dist/tilt/roll bins), and — depending on
the target — calls either Kalibr (AprilGrid) or an in-process OpenCV solver
(thermal asymmetric circles).

```bash
cd /home/thor/bess
docker compose -f docker-compose.thor.yml --profile calib up -d calib-ui
# open http://<thor-host>:8091
```

1. Pick camera from the `Camera` dropdown (`lucid_right`, `blackfly_left`,
   `a6701_left`, `a70_right`, …). The preset carries the topic, model, and
   focal seed — all three MUST match the lens.
2. Pick target (`apriltag_calibio` for visible cams, `thermal_circles_asym`
   for thermal cams).
3. Type a session name or leave blank to auto-generate
   `<preset>_intrinsics_YYYY-MM-DD`.
4. `Start` — live preview with yellow dot overlay (detected blobs) and rainbow
   chessboard (grid lock) for circle-grid targets, or AprilTag outlines for
   AprilGrid.
5. Capture when all coverage bars are green (framerate bar ≥ 30 usually
   suffices for OpenCV solve; Kalibr wants 80+).
6. `Stop`, then `Kalibr` for AprilGrid or the in-process solve for thermal
   circles. Output lands in `/home/thor/calib/<session>/thermal_intrinsics.yaml`
   or `camchain.yaml`.
7. `Rectified` — shows pre/post undistort split. The button auto-enables once
   a YAML exists in the session dir (survives calib-ui container restarts —
   `/status` auto-loads the most recent session for the current preset).

Presets are defined in `containers/calib-ui/server.py:CAMERA_PRESETS`. Add a
new camera by editing that dict and restarting `calib-ui`.

### Model + focal-seed rule

`server.py` post-solve has a sanity gate that rejects `fx` outside
`[0.25·width, 5·width]` or a principal point outside the image. If you get
`FAIL: fx=… outside [0.25w, 5w]`, either the lens is wildly different from
`focal_seed` or the wrong model was chosen. Recompute the seed from:

```
focal_seed ≈ focal_length_mm / pixel_pitch_mm
```

Today's verified seeds:

| Camera | Sensor | Lens | Pixel pitch | Seed | Verified fx |
|--------|--------|------|-------------|------|-------------|
| FLIR A6701 (cooled MWIR) | 640×512 InSb | 17 mm | 15 µm | 1133 | **1116** (0.32 px rms) |
| FLIR A70 (uncooled LWIR, wide) | 640×480 VOx | ~4.9 mm (95° FOV) | 12 µm | 350 | **342–357** (0.65–0.88 px rms) |
| Blackfly S (24 MP) | 3000×4096 | TBD narrow | 2.74 µm | 900 | solve pending |
| Lucid Atlas (64 MP) | 9568×6376 | TBD | 2.0 µm | 1200 | solve pending |

## Rig geometry — NO overlapping stereo pairs

Thor's 8 cameras are mounted in **opposing-side pairs** (one facing each side
of the vehicle), not forward-facing stereo. There is no inter-camera FOV
overlap, so Kalibr's `cam-stereo` multi-cam mode does NOT apply — it solves
for `T_cnm1_cn` from shared target observations, which cannot exist here.

Workflow instead:

1. **Intrinsics per camera** — one `cam-mono` capture per lens, 8 total.
2. **Camera → lidar extrinsic per camera** — shared target in overlap with
   Ouster, computed via koide3/direct_visual_lidar_calibration (bess-calib-dvlc,
   not yet built) or per-camera `kalibr_calibrate_imu_camera` against an IMU
   that's already pinned to the lidar frame.
3. **IMU noise priors** — `imu-static` + allan_variance_ros for each IMU
   (Ouster BMI085 + GQ7).
4. **IMU → camera time offset** — `imu-cam` per camera against one IMU.

The lidar becomes the common reference frame; inter-camera extrinsics come
out by composition through it.

## Capture workflow

1. **Print AprilGrid target** — use the calib.io PDF that shipped with your
   printed AprilGrid. Confirm tag family `t36h11` (the only one Kalibr
   supports). Flat mount, good lighting, no reflections.
2. **Edit** `/home/thor/bess/config/calib/aprilgrid_calibio.yaml` so
   `tagRows / tagCols / tagSize / tagSpacing` match your physical target.
3. **Capture** a bag of one camera seeing the target through a range of
   poses (tilt, yaw, distance, all quadrants of the image):
   ```bash
   # lucid_right (lucid2) intrinsics, 120 s
   bash /home/thor/bess/scripts/calib_capture.sh \
       cam-mono lucid_right_intrinsics_2026-04-19 lucid2 --duration 120
   ```
4. The capture script records via the `foxglove` container, pulls the MCAP
   out to `/home/thor/calib/<session>/raw.mcap`, then converts to
   `raw.bag` with the util container.

## Running Kalibr

Once `raw.bag` exists:

### Single-camera intrinsics

```bash
docker run --rm -it \
    -v /home/thor/calib/lucid_right_intrinsics_2026-04-19:/data \
    -v /home/thor/bess/config/calib:/config:ro \
    localhost/bess-calib-kalibr:noetic \
    kalibr_calibrate_cameras \
        --target /config/aprilgrid_calibio.yaml \
        --bag /data/raw.bag \
        --models pinhole-equi \
        --topics /lucid2/camera_driver/image_raw \
        --bag-from-to 5 115
```

**Camera models for Thor sensors (evidence-backed 2026-04-20):**

| Sensor | FOV / lens | Model | Why |
|--------|-----------|-------|-----|
| **Blackfly S (24 MP)** | narrow ~70–80° | `pinhole-radtan` | `pinhole-equi` diverges — 2026-04-20 attempt on `blackfly_right` drove focal init to fx=179 on a 3000 px image and OptimizationDiverged. Equi is for true fisheyes (>120° FOV) only |
| **Lucid Atlas (64 MP)** | wide | `pinhole-equi` | default for wide-lens 64 MP — verify fx falls in `[0.25w, 5w]` post-solve |
| **ZED 2i RGB** (4 mm ≈ 110° FOV) | fisheye | `pinhole-equi` | matches Stereolabs' published model |
| **FLIR A6701 MWIR** (17 mm on 15 µm pitch) | ~33° HFOV narrow | `pinhole-radtan` | `focal_seed=1133` (= 17 mm / 15 µm); verified fx=1116, rms=0.32 px, principal point centred |
| **FLIR A70 LWIR** (95° HFOV wide) | fisheye | `pinhole-equi` | `focal_seed=350` (4.9 mm / 12 µm); verified fx=342–357, rms=0.65–0.88 px, pinhole-radtan won't converge at corners |

Rule of thumb:

- `pinhole-radtan` — narrow-to-normal FOV (up to ~80°). Brown-Conrady polynomial; 5 distortion coeffs; the web UI auto-sets `CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_K3` to keep the solver stable on sparse thermal captures.
- `pinhole-equi` — wide FOV / fisheye (>~90°). Kannala-Brandt 4-param. Will NOT converge on narrow lenses — initial focal estimation explodes, the BA diverges, and you get `Focal length needs to be positive [0 > 0]`.
- `ds` (double-sphere) — ultra-wide >180° (not currently used on Thor).

**Failure signature** — if Kalibr (or the UI's OpenCV solver) returns
`OptimizationDiverged`, projection init with `fx < 0.1 * image_width`, or a
sanity-gate `fx outside [0.25w, 5w]`, the model is wrong for the lens before
it's a "bad capture" problem. Switch models first, re-run on the existing
bag — don't re-capture.

### IMU noise characterization (prerequisite for IMU-cam)

```bash
# 1. Record a ≥ 3-hour static IMU bag (sensors motionless, flat surface):
bash /home/thor/bess/scripts/calib_capture.sh imu-static imu_static_$(date +%F) --duration 10800

# 2. Run allan_variance_ros inside the Kalibr container:
docker run --rm -it \
    -v /home/thor/calib/imu_static_YYYY-MM-DD:/data \
    localhost/bess-calib-kalibr:noetic \
    bash -lc '. /opt/ros/noetic/setup.bash && \
              . /catkin_ws/devel/setup.bash && \
              rosrun allan_variance_ros allan_variance /data /data/imu_config.yaml'
```

The Allan deviation plot yields σ_g, σ_a, σ_bg, σ_ba which go into the
`imu.yaml` that Kalibr IMU-cam calibration consumes.

### IMU-camera spatiotemporal

```bash
docker run --rm -it \
    -v /home/thor/calib/<session>:/data \
    -v /home/thor/bess/config/calib:/config:ro \
    localhost/bess-calib-kalibr:noetic \
    kalibr_calibrate_imu_camera \
        --target /config/aprilgrid_calibio.yaml \
        --bag /data/raw.bag \
        --cam /data/camchain.yaml \
        --imu /data/imu.yaml \
        --bag-from-to 5 175
```

Output: `camchain-imucam-*.yaml` with T_cam_imu + time offset per camera.
That's the source of truth for the TF tree + offline de-skew.

## Lucid Atlas native-resolution calibration mode

Both Lucid Atlas 64MP cameras run with `binning:=4` + `fps:=3.0` in the default
stack (`docker-compose.thor.yml:379,419`). Binning drops the sensor from its
native `9568×6376` down to `2392×1594` — a 16× pixel-count reduction. The
default exists because two native-resolution Lucids publishing concurrently
would push ~360 MB/s through the sfp28-7/8 pair, saturate the compress-lucid
CPU budget, and make the live iPad Foxglove viz unusable.

For calibration we want the highest-resolution frames Kalibr can see (AprilGrid
corner detection is pixel-limited at typical capture distances). The calibration
override lives at `/home/thor/bess/docker-compose.calib-lucid.yml` and flips the
Lucids to `binning:=1` + `fps:=1.0` (~60 MB/s per camera, well inside link
budget).

### Automatic override (preferred)

`scripts/calib_capture.sh` auto-detects whenever `lucid1` or `lucid2` appears in
the capture target list. Before recording it runs:

```bash
docker compose -f docker-compose.thor.yml -f docker-compose.calib-lucid.yml \
    up -d --force-recreate --no-deps lucid1 lucid2   # (only those in target)
sleep 45   # Arena SDK re-init at full resolution
```

After recording completes (OR on CTRL-C / error — a `trap` handles this) it
restores the defaults:

```bash
docker compose -f docker-compose.thor.yml up -d --force-recreate --no-deps lucid1 lucid2
```

You should never need to touch the override file manually for a normal capture.

**Verify override took effect**:

```bash
docker logs lucid1 --tail 20 | grep "Image size"   # expect: 9568x6376
docker logs lucid2 --tail 20 | grep "Image size"   # expect: 9568x6376
```

### Manual override (for ad-hoc debugging only)

If you're poking at the Lucids without going through `calib_capture.sh`:

```bash
cd /home/thor/bess

# Flip to calibration mode
docker compose -f docker-compose.thor.yml -f docker-compose.calib-lucid.yml \
    up -d --force-recreate --no-deps lucid1 lucid2

# ...do your thing — live Foxglove, manual bag record, arv-tool queries, etc.

# CRITICAL: restore BEFORE leaving the stack running unattended
docker compose -f docker-compose.thor.yml \
    up -d --force-recreate --no-deps lucid1 lucid2
```

### Why not leave the override active permanently

- `/lucidN/camera_driver/image_raw` at native res is ~180 MB/s at 3 fps — two
  cameras saturate the sfp28-7/8 10GigE pair.
- `compress-lucid1` / `compress-lucid2` GPU-JPEG encode budget is sized for
  binning=4 frames; native frames stall the pipeline and drop.
- Foxglove iPad viz over WiFi (~80 Mbps practical) can't handle the compressed
  native streams either.

Override is a capture-time tool only. Put it back.

## Thermal calibration — asymmetric circle grid

AprilGrid doesn't work on thermal below body temp — the paper disappears into
the scene noise. For Thor's thermal cameras we use a **heat-contrast asymmetric
circle grid**: dark 60 mm circles printed/cut on a thermally-different
substrate with 60 mm offset spacing (4×11 OpenCV pattern, `CALIB_CB_ASYMMETRIC_GRID`).

The UI's `thermal_circles_asym` target runs an in-process OpenCV solver
instead of shelling out to Kalibr:

- `cv2.SimpleBlobDetector` on raw gray (no CLAHE — it amplifies sky/tree
  blobs), tuned for dark-blob-on-bright-background.
- Union-find spatial clustering to isolate the densest blob group.
- ROI crop + 2× / 3× upscale if the grid is <200 px wide (close-range / small
  boards).
- `cv2.findCirclesGrid` with `CALIB_CB_CLUSTERING` fallback across probe sizes
  in `CIRCLES_PROBE_SIZES` (4×11 first).
- Fallback to yellow marker visualization if grid fit fails — user can still
  see the detector found something before re-capturing.
- Solve is `cv2.fisheye.calibrate` (for `pinhole-equi`) or `cv2.calibrateCamera`
  with `CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_K3`
  (for `pinhole-radtan`). The `FIX_PRINCIPAL_POINT` flag matters on sparse
  captures — without it, fewer than ~30 frames lets the solver drift cx/cy
  outside the image.
- Post-solve sanity gate refuses to write YAML if `fx ∉ [0.25·width, 5·width]`
  or principal point is outside the image. Logs `FAIL:` in the UI log panel.

Output: `<session>/thermal_intrinsics.yaml` (not `camchain.yaml`).

Adjust the board physical spacing via `THERMAL_CIRCLES_SPACING_M` env
(default 0.06 m). Pattern rows × cols are auto-probed via
`CIRCLES_PROBE_SIZES` — ordered to try the common calib.io 4×11 first.

## Session results (2026-04-20)

| Preset | Model | Frames | RMS (px) | Status |
|--------|-------|--------|----------|--------|
| `a6701_left` | pinhole-radtan | 11 / 11 | 0.32 | clean — re-capture with 30+ frames for production robustness |
| `a70_left` | pinhole-equi | 60 / 290 | 0.65 | clean |
| `a70_right` | pinhole-equi | 60 / 341 | 0.88 | clean |
| `blackfly_right` | pinhole-equi | — | — | **FAILED**: OptimizationDiverged — wrong model. Preset now flipped to `pinhole-radtan`; re-run solve on existing `raw.bag` |

Outputs live under `/home/thor/calib/<session>/thermal_intrinsics.yaml` (OpenCV
path) or `/home/thor/calib/<session>/camchain.yaml` (Kalibr path). When the
full 8-camera intrinsics set is done, copy the per-camera YAMLs into
`/home/thor/bess/config/cameras/calibration/` so the extraction container
picks them up at launch.

## Notes

- **Kalibr runs offline only** — it's not a ROS 2 node. Never start it as
  part of the live stack. The `calib` profile keeps `calib-ui` off the
  `full`/`sensors` paths by design.
- **Model choice trumps capture quality** — a diverged solve on a good bag
  usually means wrong model for the lens FOV, not a bad capture. See model
  table above. Switch and re-solve on the existing bag before re-capturing.
- **Focal seed matters** — `focal_seed ≈ focal_length_mm / pixel_pitch_mm`.
  For narrow-FOV radtan solves, a seed that's >2× off the true fx lets the
  bundle adjust wander to `cx, cy` outside the image (A6701 LEFT hit this
  earlier 2026-04-20 with seed=600 on a 17 mm lens; fix was 1133).
- **Calib.io AprilGrid** — the printed SKU's datasheet lists tagSize (edge of
  black square in metres) and tagSpacing (gap ratio). Do not eyeball.
- **Thermal AprilGrid is a dead path** — use the asymmetric circle grid above.
  A room-temp AprilGrid is invisible in LWIR; heating the target is fragile
  and time-limited.
- **Never bake intrinsic YAMLs into containers** — drop solver output into
  `/home/thor/bess/config/cameras/calibration/` so the extraction container
  picks them up at launch.
- **Container restarts don't erase session state** — `calib-ui` auto-loads
  the most recent YAML for the current preset from disk on `/status`, so the
  `Rectified` button stays enabled after a `docker restart calib-ui`.
- **Bind-mounted server.py inode trap** — editing `server.py` on the host
  breaks the single-file bind mount (Edit replaces the inode). Restart
  `calib-ui` after every host edit: `docker restart calib-ui`.
