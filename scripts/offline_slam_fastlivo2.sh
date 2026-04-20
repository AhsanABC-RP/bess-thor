#!/bin/bash
# offline_slam_fastlivo2.sh — FAST-LIVO2 offline visual-inertial-lidar SLAM.
#
# FAST-LIVO2 is ROS 1 Noetic only. This script:
#  1. Converts our ROS 2 MCAP bags to ROS 1 bags via ternaris/rosbags (cached).
#  2. Spawns FAST-LIVO2 inside Noetic with our Ouster+Blackfly configs.
#  3. Plays the ROS 1 bags in another Noetic container via rosbag play --clock.
#  4. Records output poses/cloud to a ROS 1 bag for LAS export.
#
# Usage:
#   bash scripts/offline_slam_fastlivo2.sh <start_bag> <end_bag> [rate] [tag]
# Example:
#   bash scripts/offline_slam_fastlivo2.sh 16 22 1.0 drive1

set -euo pipefail

START=${1:?missing start_bag}
END=${2:?missing end_bag}
RATE=${3:-1.0}
TAG=${4:-run}

BAG_DIR="/mnt/bess-usb/bags/rolling/bag"
ROS1_BAG_DIR="/mnt/bess-usb/bags/rolling/bag_ros1"   # cached converted bags
OUT_BASE="/mnt/bess-usb/bags/rolling/slam_offline/fastlivo2_${TAG}"
LOG_DIR="${OUT_BASE}/logs"
BAG_OUT="${OUT_BASE}/slam_output"

BESS_ROOT="/home/thor/bess"
LIVO_MAIN_CFG="${BESS_ROOT}/config/fastlivo2/ouster_blackfly.yaml"
LIVO_CAM_CFG="${BESS_ROOT}/config/fastlivo2/camera_blackfly1.yaml"

mkdir -p "$LOG_DIR" "$ROS1_BAG_DIR"
sudo rm -rf "$BAG_OUT" 2>/dev/null || true

BAG_LIST=""
BAG_COUNT=0
for i in $(seq "$START" "$END"); do
    f="$BAG_DIR/bag_${i}.mcap"
    if [ -f "$f" ] && [ -s "$f" ]; then
        BAG_LIST="$BAG_LIST $i"
        BAG_COUNT=$((BAG_COUNT + 1))
    fi
done
[ "$BAG_COUNT" -eq 0 ] && { echo "ERROR: no bags in $START..$END"; exit 1; }

echo "=== FAST-LIVO2 Offline Replay ==="
echo "  Tag:    $TAG"
echo "  Bags:   $BAG_COUNT (bag_${START}..bag_${END})"
echo "  Rate:   ${RATE}x"
echo "  Output: $OUT_BASE"
echo ""

for c in fastlivo2-off recorder-ros1 player-ros1; do
    docker rm -f "$c" >/dev/null 2>&1 || true
done

# --- Step 1: convert MCAP -> ROS 1 bags (cached) ---------------------------
echo "[1/4] Converting MCAP -> ROS 1 bags (cached in $ROS1_BAG_DIR)..."
for IDX in $BAG_LIST; do
    SRC="$BAG_DIR/bag_${IDX}.mcap"
    DST_DIR="$ROS1_BAG_DIR/bag_${IDX}"
    DST="$DST_DIR/bag_${IDX}.bag"
    if [ -f "$DST" ] && [ -s "$DST" ]; then
        echo "  bag_${IDX}: cached"
        continue
    fi
    mkdir -p "$DST_DIR"
    echo "  bag_${IDX}: converting..."
    docker run --rm \
        -v "$BAG_DIR":/src:ro \
        -v "$DST_DIR":/dst:rw \
        localhost/bess-fastlivo2:noetic \
        bash -c "rosbags-convert --src /src/bag_${IDX}.mcap --dst /dst/bag_${IDX}.bag 2>&1"
done

ROS1_BAGS=""
for IDX in $BAG_LIST; do
    ROS1_BAGS="$ROS1_BAGS /bags/bag_${IDX}/bag_${IDX}.bag"
done

# --- Step 2: start roscore + FAST-LIVO2 ------------------------------------
echo "[2/4] Starting FAST-LIVO2 (roscore + LIVMapper)..."
mkdir -p "$BAG_OUT"
docker run -d --rm \
    --name fastlivo2-off \
    --network host --ipc host \
    -v "$LIVO_MAIN_CFG":/cfg/main.yaml:ro \
    -v "$LIVO_CAM_CFG":/cfg/cam.yaml:ro \
    -v "$BAG_OUT":/rec:rw \
    localhost/bess-fastlivo2:noetic \
    bash -c 'roscore &
             sleep 3
             rosparam set use_sim_time true
             roslaunch fast_livo mapping_ouster64.launch \
                 config_path:=/cfg/main.yaml \
                 cam_config_path:=/cfg/cam.yaml 2>&1 | tee /rec/fastlivo2.log' \
    >/dev/null

echo "  Giving FAST-LIVO2 15s to init..."
sleep 15

if ! docker ps --format '{{.Names}}' | grep -q "^fastlivo2-off$"; then
    echo "ERROR: fastlivo2-off exited during startup"
    docker logs fastlivo2-off --tail 60 2>&1 || true
    exit 1
fi

# --- Step 3: start recorder for FAST-LIVO2 output topics -------------------
echo "[3/4] Recording /Odometry + /cloud_registered..."
docker run -d --rm \
    --name recorder-ros1 \
    --network host --ipc host \
    -v "$BAG_OUT":/rec:rw \
    localhost/bess-fastlivo2:noetic \
    bash -c 'sleep 2 && exec rosbag record \
        -O /rec/slam_output.bag \
        /Odometry /path /cloud_registered /aft_mapped_to_init /tf /tf_static' \
    >/dev/null
sleep 3

# --- Step 4: play bags -----------------------------------------------------
echo "[4/4] Playing ROS 1 bags at ${RATE}x..."
docker run --rm \
    --name player-ros1 \
    --network host --ipc host \
    -v "$ROS1_BAG_DIR":/bags:ro \
    localhost/bess-fastlivo2:noetic \
    bash -c "rosbag play --clock -r $RATE $ROS1_BAGS"

echo ""
echo "Playback done. Flushing for 20s..."
sleep 20
docker stop -t 10 recorder-ros1 fastlivo2-off 2>/dev/null || true

echo ""
echo "=== FAST-LIVO2 offline replay complete ==="
echo "Output: $BAG_OUT/"
ls -lh "$BAG_OUT/" 2>/dev/null || true
