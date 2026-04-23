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
# foxglove-web is the nginx-static Lichtblick UI on :8080 (iPad LAN viewer).
# Was missing from the launcher pre-2026-04-23 — only foxglove + foxglove-local
# (the WS bridges) were started; the HTTP bundle never came up post-boot.
LOG "batch 1/11: foxglove bridges + Lichtblick web + WAN uplink helpers"
$UP foxglove foxglove-local foxglove-web rutx ntrip || LOG "WARN batch 1 partial"
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
# after the 2026-04-15 iGPU instability). extraction is downstream-only.
# glim removed from boot path 2026-04-23: OOM-prone (Issue #25-related),
# deprecated for OS1-128 (too sparse). Profile `glim-deprecated` only.
LOG "batch 10/11: pii-mask + extraction"
$UP pii-mask || LOG "WARN pii-mask failed to start"
sleep 5
# extraction also writes to F8 NAS (/home/thor/nas/bess-bags/extraction) so
# trigger autofs first, same reasoning as batch 11.
ls /home/thor/nas/bess-bags >/dev/null 2>&1
mkdir -p /home/thor/nas/bess-bags/extraction 2>/dev/null || true
$UP extraction || LOG "WARN extraction failed to start"

# --- Batch 11: recorder (F8 NAS soak) ---------------------------------------
# Added to boot path 2026-04-23 — user requirement is auto-soak post-boot.
# Output is the F8 NAS (10G sfp28-16) with 3 TB cap; --profile record gates
# it so manual `docker compose up -d --no-deps recorder` still works without
# the profile.
#
# NFS mount: the home-thor-nas-bess\x2dbags.mount unit is enabled and pulled
# in at boot via multi-user.target. The .automount remains as a fallback for
# on-demand re-trigger. Touching the path here forces autofs to actually
# mount before docker tries the bind, since pre-2026-04-23 boots showed
# docker bind failing with "no such device" because autofs hadn't been
# triggered yet (extraction + recorder both died, exit 255).
LOG "batch 11/11: recorder (F8 NAS soak)"
sleep 5
ls /home/thor/nas/bess-bags >/dev/null 2>&1   # force autofs trigger
mkdir -p /home/thor/nas/bess-bags/extraction 2>/dev/null || true
if mount | grep -q '169.254.100.30:.* on /home/thor/nas/bess-bags '; then
    $UP recorder || LOG "WARN recorder failed to start"
else
    LOG "WARN F8 NAS NFS not mounted at /home/thor/nas/bess-bags — recorder NOT started"
fi

LOG "=== sequenced stack startup COMPLETE ==="
LOG "run 'docker ps' to verify; topic sweeps via foxglove bridge"
exit 0
