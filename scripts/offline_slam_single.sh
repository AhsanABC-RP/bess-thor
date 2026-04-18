#!/bin/bash
# Offline SLAM reprocessing — single container, sequential bag playback.
# Bags are played one at a time to avoid ros2 bag play multi-input reordering.
#
# Usage: bash scripts/offline_slam_single.sh [start_bag] [end_bag] [rate]
#
# Clean shutdown: docker stop -t 30 fastlio-offline
#   → MCAP recorder flushes, PCD saved by FAST-LIO2 (if SIGINT propagates)
#   → Use extract_slam_cloud.py to get PLY from partial MCAP regardless
#
# Quick preview after any run:
#   python3 scripts/extract_slam_cloud.py \
#       /mnt/bess-usb/bags/rolling/slam_offline/slam_output/ \
#       -o slam_preview.ply --voxel 0.1

set -e

BAG_DIR="/mnt/bess-usb/bags/rolling/bag"
OUT_DIR="/mnt/bess-usb/bags/rolling/slam_offline"
START=${1:-0}
END=${2:-144}
RATE=${3:-0.5}

mkdir -p "$OUT_DIR"
sudo rm -rf "$OUT_DIR/slam_output" 2>/dev/null || true

# Clean up any stale container
docker rm -f fastlio-offline 2>/dev/null || true

# Build list of valid bag indices
BAG_LIST=""
BAG_COUNT=0
for i in $(seq $START $END); do
    f="$BAG_DIR/bag_${i}.mcap"
    if [ -f "$f" ] && [ -s "$f" ]; then
        BAG_LIST="$BAG_LIST $i"
        BAG_COUNT=$((BAG_COUNT + 1))
    fi
done

if [ $BAG_COUNT -eq 0 ]; then
    echo "ERROR: No valid bag files found in range $START-$END"
    exit 1
fi

echo "=== Offline SLAM Replay (single container, sequential) ==="
echo "  Bags: $BAG_COUNT files, bag_${START}.mcap → bag_${END}.mcap"
echo "  Rate: ${RATE}x"
echo "  Output: ${OUT_DIR}/slam_output/"
echo "  Stop cleanly: docker stop -t 30 fastlio-offline"
echo ""

docker run --rm \
    --name fastlio-offline \
    --network host \
    --init \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///config/cyclonedds.xml \
    -e ROS_DOMAIN_ID=42 \
    -e ENABLE_SCANCONTEXT=0 \
    -e FAST_LIO_CONFIG=/ros2_ws/config/fast_lio_single_offline.yaml \
    -v /home/thor/bess/containers/fast-lio/fast_lio_single_offline.yaml:/ros2_ws/config/fast_lio_single_offline.yaml:ro \
    -v /home/thor/bess/containers/fast-lio/launch_fast_lio_single.py:/ros2_ws/launch_fast_lio_single.py:ro \
    -v /home/thor/bess/containers/fast-lio/ouster_imu_guard.py:/ros2_ws/ouster_imu_guard.py:ro \
    -v /home/thor/bess/containers/fast-lio/imu_guard.py:/ros2_ws/imu_guard.py:ro \
    -v /home/thor/bess/containers/fast-lio/scancontext_node.py:/ros2_ws/scancontext_node.py:ro \
    -v /home/thor/bess/containers/fast-lio/camera_tf_broadcaster.py:/ros2_ws/camera_tf_broadcaster.py:ro \
    -v /home/thor/bess/containers/fast-lio/tf_republisher.py:/ros2_ws/tf_republisher.py:ro \
    -v /home/thor/bess/config/cyclonedds.xml:/config/cyclonedds.xml:ro \
    -v "$BAG_DIR":/bags:ro \
    -v "$OUT_DIR":/output:rw \
    localhost/bess-fast-lio:jazzy \
    bash -c '
trap "echo Shutting down...; kill $REC_PID $SLAM_PID $GUARD_PID 2>/dev/null; wait; exit 0" SIGTERM SIGINT

source /opt/ros/jazzy/setup.bash

echo "Starting Ouster IMU Guard..."
python3 /ros2_ws/ouster_imu_guard.py --ros-args --param use_sim_time:=true &
GUARD_PID=$!
sleep 2

echo "Starting FAST-LIO2 (sim time, offline config)..."
ros2 launch /ros2_ws/launch_fast_lio_single.py use_sim_time:=true &
SLAM_PID=$!
sleep 8

echo "Starting recorder..."
ros2 bag record \
    -o /output/slam_output \
    -s mcap \
    --use-sim-time \
    /fast_lio/odometry \
    /fast_lio/cloud_registered \
    /fast_lio/cloud_registered_body \
    /fast_lio/path \
    /tf \
    /tf_static &
REC_PID=$!
sleep 3

PLAYED=0
for IDX in '"$BAG_LIST"'; do
    BAG_FILE="/bags/bag_${IDX}.mcap"
    if [ -f "$BAG_FILE" ]; then
        PLAYED=$((PLAYED + 1))
        echo "[${PLAYED}/'"$BAG_COUNT"'] Playing bag_${IDX}.mcap..."
        ros2 bag play \
            -i "$BAG_FILE" mcap \
            --topics /ouster/points /ouster/imu \
            --clock 200 \
            -r '"$RATE"' \
            --read-ahead-queue-size 1000 \
            --disable-keyboard-controls
    fi
done

echo "Playback done ($PLAYED bags). Flushing..."
sleep 15

kill $REC_PID 2>/dev/null || true
sleep 3
kill $SLAM_PID 2>/dev/null || true
kill $GUARD_PID 2>/dev/null || true
wait

echo "=== Complete ==="
'
