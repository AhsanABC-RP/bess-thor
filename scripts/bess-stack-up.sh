#!/bin/bash
# BESS Thor sequenced stack launcher
#
# Replaces `docker compose up --profile full` which race-starts ~30
# containers in parallel. That parallel stampede was the root cause of the
# 2026-04-14/15 boot loop: 27 containers racing at docker.service startup
# for GPU init / USB probe / GigE SDK discovery / host-net / PTP clients
# wedged the kernel long enough for /dev/watchdog0 (Tegra HW watchdog) to
# fire. Fix: start containers in small ordered batches with settle sleeps.
#
# Invariants:
#   * Never more than 4 containers starting at any instant.
#   * Camera SDKs (Spinnaker C++, Spinnaker PySpin, Aravis, Arena) NEVER
#     run their network discovery in parallel with another camera SDK.
#   * GPU-touching containers (ouster CUDA driver init, fast-lio, dlio,
#     pii-mask) staggered at least 5s apart.
#   * --no-deps: we sequence explicitly, don't let compose resolve it.
#   * Idempotent: re-running is safe, already-up containers are left alone.
#
# This script assumes boot-sensors.sh has already run (dnsmasq live,
# enP2p1s0 flushed, 169.254.100.1/16 on mgbe0_0). The systemd unit
# bess-stack.service orders After=bess-network.service to guarantee this.

set -u
cd /home/thor/bess

COMPOSE="docker compose -f docker-compose.thor.yml"
UP="$COMPOSE up -d --no-deps"

LOG() { echo "[$(date +%H:%M:%S)] bess-stack-up: $*"; }

LOG "=== starting sequenced stack startup ==="

# --- Batch 1: infrastructure (lightweight, no hardware) ---------------------
LOG "batch 1/10: foxglove bridges + WAN uplink helpers"
$UP foxglove foxglove-local rutx ntrip || LOG "WARN batch 1 partial"
sleep 10

# --- Batch 2: LiDAR (GigE + CUDA driver init) -------------------------------
# Ouster is the heaviest single-container init on the box: CUDA context +
# libos_ros_sdk GigE discovery + PTP slave handshake. Alone in its batch so
# nothing else competes for the kernel while it probes.
LOG "batch 2/10: ouster LiDAR"
$UP ouster || LOG "WARN ouster failed to start"
sleep 15

# --- Batch 3: IMU (USB) -----------------------------------------------------
# microstrain resolves /dev/ttyACM* via udev symlinks and reads sysfs for
# main/aux disambiguation — quick but USB-bound.
LOG "batch 3/10: microstrain GQ7 IMU"
$UP microstrain || LOG "WARN microstrain failed (GQ7 may be unplugged)"
sleep 5

# --- Batch 4: Blackfly S (Spinnaker C++ driver) -----------------------------
# Heaviest camera init: spinnaker_camera_driver loads the full libspinnaker
# node tree and claims PTP SlaveOnly. Must be alone. 20s to let GigE Vision
# heartbeat stabilize before the next SDK starts its discovery.
LOG "batch 4/10: blackfly1 + blackfly2 (Spinnaker C++)"
$UP blackfly1 blackfly2 || LOG "WARN blackfly batch partial"
sleep 20

# --- Batch 5: A6701 thermal (Spinnaker PySpin) ------------------------------
# Same SDK family as Blackfly but different container image. Staggered from
# Blackfly by a full settle window so Spinnaker initiator heartbeats don't
# cross.
LOG "batch 5/10: thermal1 + thermal2 (A6701, PySpin)"
$UP thermal1 thermal2 || LOG "WARN thermal A6701 batch partial"
sleep 15

# --- Batch 6: A70 thermal (patched PySpin, different GenICam) ---------------
# A70 uses the same Spinnaker PySpin image as A6701 but hits a different
# GenICam XML tree (MAX GevSCPD=512 vs A6701's larger value). Keep in its
# own batch to avoid initializer-side races inside PySpin.
LOG "batch 6/10: thermal3 + thermal4 (A70, PySpin patched)"
$UP thermal3 thermal4 || LOG "WARN thermal A70 batch partial"
sleep 15

# --- Batch 7: Lucid Atlas (Arena SDK) ---------------------------------------
# Completely separate SDK (arena_api) — co-running with Spinnaker doesn't
# cause SDK-level conflicts but doubling up GigE Vision discovery traffic
# on mgbe0_0 at the same moment as 4 thermal cams coming online makes
# switch L2 tables churn. Give it its own window.
LOG "batch 7/10: lucid1 + lucid2 (Arena SDK)"
$UP lucid1 lucid2 || LOG "WARN lucid batch partial"
sleep 15

# --- Batch 8: compression + colormap (lightweight, 9 containers) ------------
# These only subscribe to already-publishing camera topics; all of them are
# safe to start together. No kernel contention, just ROS 2 subs.
LOG "batch 8/10: compression pipeline + thermal colormap"
$UP compress-blackfly1 compress-blackfly2 \
    compress-thermal1 compress-thermal2 compress-thermal3 compress-thermal4 \
    compress-lucid1 compress-lucid2 \
    thermal-colormap || LOG "WARN compression batch partial"
sleep 5

# --- Batch 9: SLAM (GPU, depends on ouster + imu) ---------------------------
# fast-lio is the heavier start (laserMapping + 8 static_transform_publisher
# children + tf republisher + camera_tf_broadcaster). Start it first, let
# it settle, then start dlio + accumulator together.
LOG "batch 9/10: SLAM stack (fast-lio first, then dlio + accumulator)"
$UP fast-lio || LOG "WARN fast-lio failed to start"
sleep 10
$UP dlio slam-map-accumulator || LOG "WARN dlio/accumulator batch partial"
sleep 5

# --- Batch 10: inference + extraction ---------------------------------------
# pii-mask is GPU (shares CUDA 0 with segformer but segformer is opt-in now
# after the 2026-04-15 iGPU instability). extraction + glim consume SLAM
# output and are downstream-only, safe together.
LOG "batch 10/10: pii-mask + extraction + glim"
$UP pii-mask || LOG "WARN pii-mask failed to start"
sleep 5
$UP extraction glim || LOG "WARN extraction/glim batch partial"

LOG "=== sequenced stack startup COMPLETE ==="
LOG "run 'docker ps' to verify; topic sweeps via foxglove bridge"
exit 0
