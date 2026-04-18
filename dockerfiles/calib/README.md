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

## Building

```bash
cd /home/thor/bess/dockerfiles/calib
docker build -t localhost/bess-calib-kalibr:noetic -f Containerfile.kalibr .
docker build -t localhost/bess-calib-util:jazzy    -f Containerfile.util    .
```

The Kalibr build is ~30–45 min on Thor (aslam_optimizer + aslam_cameras are
the long catkin packages). The util container is a few minutes.

## Capture workflow

1. **Print AprilGrid target** — use the calib.io PDF that shipped with your
   printed AprilGrid. Confirm tag family `t36h11` (the only one Kalibr
   supports). Flat mount, good lighting, no reflections.
2. **Edit** `/home/thor/bess/config/calib/aprilgrid_calibio.yaml` so
   `tagRows / tagCols / tagSize / tagSpacing` match your physical target.
3. **Capture** a bag of the sensor(s) seeing the target through a range of
   poses (tilt, yaw, distance, all quadrants of the image):
   ```bash
   # Example: stereo-pair Blackfly intrinsics + extrinsics, 180 s
   bash /home/thor/bess/scripts/calib_capture.sh \
       cam-stereo blackfly_stereo_2026-04-18 blackfly1 blackfly2 \
       --duration 180
   ```
4. The capture script records via the `foxglove` container, pulls the MCAP
   out to `/mnt/bess-usb/calib/<session>/raw.mcap`, then converts to
   `raw.bag` with the util container.

## Running Kalibr

Once `raw.bag` exists:

### Multi-camera intrinsics + extrinsics

```bash
docker run --rm -it \
    -v /mnt/bess-usb/calib/<session>:/data \
    -v /home/thor/bess/config/calib:/config:ro \
    localhost/bess-calib-kalibr:noetic \
    kalibr_calibrate_cameras \
        --target /config/aprilgrid_calibio.yaml \
        --bag /data/raw.bag \
        --models pinhole-equi pinhole-equi \
        --topics /blackfly1/image_raw /blackfly2/image_raw \
        --bag-from-to 5 175
```

**Camera models for Thor sensors:**

| Sensor | FOV / lens | Model | Why |
|--------|-----------|-------|-----|
| Blackfly S 2.1 MP (2.8mm) | wide-ish | `pinhole-equi` | 2.8 mm on 1/1.8" needs equi distortion, radtan fails at edges |
| Blackfly S (8mm)  | normal | `pinhole-radtan` | narrow FOV, radtan sufficient |
| Lucid Atlas (unknown) | TBC | `pinhole-equi` | default for any wide lens |
| ZED 2i RGB (4mm ≈ 110° FOV) | fisheye | `pinhole-equi` | matches Stereolabs' published model |
| FLIR A6701 thermal | normal | `pinhole-radtan` | |
| FLIR A70 thermal | wide | `pinhole-equi` | 95° HFOV on A70 |

If you have an 8mm fisheye with barrel distortion, that's `pinhole-equi`
(Kannala-Brandt 4-param) — radtan will not converge at the image corners.
For truly ultra-wide >180° FOV use `ds` (double-sphere) instead.

### IMU noise characterization (prerequisite for IMU-cam)

```bash
# 1. Record a ≥ 3-hour static IMU bag (sensors motionless, flat surface):
bash /home/thor/bess/scripts/calib_capture.sh imu-static imu_static_$(date +%F) --duration 10800

# 2. Run allan_variance_ros inside the Kalibr container:
docker run --rm -it \
    -v /mnt/bess-usb/calib/imu_static_YYYY-MM-DD:/data \
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
    -v /mnt/bess-usb/calib/<session>:/data \
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

## Notes

- **Kalibr runs offline only** — it's not a ROS 2 node. Never start it as
  part of the live stack.
- **8mm fisheye barrel distortion** — use `pinhole-equi`, not `pinhole-radtan`.
  Radtan's k1+k2 polynomial saturates around 60° half-angle.
- **Calib.io AprilGrid** — the printed SKU's datasheet lists tagSize (edge of
  black square in metres) and tagSpacing (gap ratio). Do not eyeball.
- **Thermal calibration** — a room-temp AprilGrid is invisible in LWIR. Heat
  the target (hairdryer → 50–60 °C) or use calib.io's thermal-visible target
  with embedded resistive heating. Otherwise tag detection fails.
- **Never bake intrinsic YAMLs into containers** — drop Kalibr output into
  `/home/thor/bess/config/cameras/calibration/` so the extraction container
  picks them up at launch.
