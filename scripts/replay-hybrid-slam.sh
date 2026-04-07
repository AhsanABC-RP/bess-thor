#!/bin/bash
##############################################################################
# BESS Hybrid SLAM Bag Replay
#
# Replays a rosbag through FAST-LIO + GLIM + Extraction to produce
# hybrid maps (GLIM loop-closed poses + raw Ouster density + all fields).
#
# This is the proper way to test the hybrid accumulator on recorded data
# without driving.
#
# Usage:
#   replay-hybrid-slam.sh <bag_dir> [--rate <float>]
#
# Example:
#   replay-hybrid-slam.sh /var/mnt/nvme1/bag-cache/bess_20260312_143905
#   replay-hybrid-slam.sh /var/mnt/nvme1/bag-cache/bess_20260312_143905 --rate 0.5
#
# Prerequisites:
#   - FAST-LIO, GLIM, and extraction containers must exist
#   - Bag must contain /ouster/points, /imu/data, /imu/data_guarded
#   - GPU available (GLIM needs RTX 2000 Ada)
#
# What happens:
#   1. Stops live FAST-LIO, GLIM (if running)
#   2. Starts GLIM with FILTER_MAX_AGE=9999 (accept old timestamps)
#   3. Starts FAST-LIO
#   4. Extraction already running — subscribes to SLAM topics + raw points
#   5. Plays bag at specified rate (default 1.0)
#   6. Waits for playback to finish
#   7. Extraction writes hybrid maps to current session dir
#
# Output:
#   /var/mnt/nvme2/extraction/session_bess_<timestamp>/fastlio_map_*.laz
##############################################################################
set -eo pipefail

BAG_DIR=""
RATE="1.0"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --rate) RATE="$2"; shift 2 ;;
        --help|-h) head -30 "$0" | tail -28; exit 0 ;;
        *) BAG_DIR="$1"; shift ;;
    esac
done

if [[ -z "$BAG_DIR" ]]; then
    echo "Usage: replay-hybrid-slam.sh <bag_dir> [--rate <float>]"
    exit 1
fi

BAG_DIR=$(realpath "$BAG_DIR")
if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
    echo "ERROR: No metadata.yaml in $BAG_DIR"
    exit 1
fi

echo "=== BESS Hybrid SLAM Replay ==="
echo "Bag:  $BAG_DIR"
echo "Rate: ${RATE}x"
echo ""

# Step 1: Stop live SLAM services
echo "Stopping live FAST-LIO and GLIM..."
sudo systemctl stop bess-fast-lio bess-glim 2>/dev/null || true
sleep 2

# Step 2: Restart extraction to get a fresh session
echo "Restarting extraction (fresh session)..."
sudo systemctl restart bess-extraction
sleep 3

# Step 3: Start GLIM with replay-safe filter (FILTER_MAX_AGE=9999)
echo "Starting GLIM (replay mode: FILTER_MAX_AGE=9999)..."
sudo podman run -d --rm \
    --name bess-glim-replay \
    --device nvidia.com/gpu=1 \
    --security-opt label=disable \
    --shm-size=8gb \
    --memory=24G \
    --cpuset-cpus=6-17 \
    --ulimit "stack=4294967296:4294967296" \
    --network=host \
    -e "ROS_DOMAIN_ID=0" \
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    -e "GLIM_CONFIG=/ros2_ws/config/config_realtime.json" \
    -e "GLIM_SENSOR_CONFIG=/ros2_ws/config/config_ouster.json" \
    -e "CUDA_VISIBLE_DEVICES=0" \
    -e "NVIDIA_VISIBLE_DEVICES=1" \
    -e "NVIDIA_DRIVER_CAPABILITIES=compute,utility" \
    -e "SPDLOG_LEVEL=error" \
    -e "CYCLONEDDS_URI=file:///etc/cyclonedds.xml" \
    -e "FILTER_SKIP_N=1" \
    -e "FILTER_MAX_AGE=999999" \
    -v "/opt/bess/config/cyclonedds.xml:/etc/cyclonedds.xml:ro" \
    -v "/opt/bess/config/glim:/ros2_ws/config:ro" \
    -v "/opt/bess/config/glim/config_ros.json:/ros2_ws/install/glim/share/glim/config/config_ros.json:ro" \
    -v "/opt/bess/config/glim/config_global_mapping_gpu.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro" \
    -v "/opt/bess/config/glim/config_sub_mapping_gpu.json:/ros2_ws/install/glim/share/glim/config/config_sub_mapping_gpu.json:ro" \
    -v "/opt/bess/config/glim/config_odometry_gpu.json:/ros2_ws/install/glim/share/glim/config/config_odometry_gpu.json:ro" \
    -v "/opt/bess/config/glim/config_sensors.json:/ros2_ws/install/glim/share/glim/config/config_sensors.json:ro" \
    -v "/opt/bess/config/glim/config_preprocess.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro" \
    -v "/opt/bess/dockerfiles/glim/launch_glim.py:/ros2_ws/launch_glim.py:ro" \
    -v "/opt/bess/data/maps:/data/maps:rw" \
    localhost/bess-glim:humble \
    bash /ros2_ws/config/drain_and_launch.sh ros2 launch /ros2_ws/launch_glim.py

echo "  Waiting for GLIM to initialize (30s)..."
sleep 30

# Step 4: Start FAST-LIO
echo "Starting FAST-LIO (replay mode)..."
sudo systemctl start bess-fast-lio
sleep 10

# Verify both SLAM systems are running
echo ""
echo "=== Status Check ==="
for name in bess-glim-replay bess-fast-lio bess-extraction; do
    if [[ "$name" == "bess-glim-replay" ]]; then
        status=$(sudo podman ps --filter name=$name --format "{{.Status}}" 2>/dev/null)
        [[ -n "$status" ]] && echo "  $name: $status" || echo "  $name: NOT RUNNING"
    else
        status=$(systemctl is-active $name 2>/dev/null)
        echo "  $name: $status"
    fi
done
echo ""

# Step 5: Play the bag
echo "=== Starting bag playback at ${RATE}x ==="
echo "  Topics: /ouster/points, /imu/data, /imu/data_guarded"
echo "  Press Ctrl+C to stop early"
echo ""

# Play from bess-recorder container (ros2 not installed on host)
# Only replay SLAM-relevant topics to reduce bandwidth
sudo podman run --rm \
    --name bess-bag-player \
    --network=host \
    --security-opt label=disable \
    -e "ROS_DOMAIN_ID=0" \
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    -e "CYCLONEDDS_URI=file:///etc/cyclonedds.xml" \
    -v "/opt/bess/config/cyclonedds.xml:/etc/cyclonedds.xml:ro" \
    -v "${BAG_DIR}:/bag:ro" \
    localhost/bess-recorder:humble \
    bash -c "source /opt/ros/humble/setup.bash && \
        ros2 bag play /bag \
        --rate $RATE \
        --topics /ouster/points /imu/data /imu/data_guarded \
        --read-ahead-queue-size 100"

echo ""
echo "=== Bag playback complete ==="

# Step 6: Wait for extraction to flush
echo "Waiting 30s for extraction to flush accumulated maps..."
sleep 30

# Step 7: Find the session output
SESSION_DIR=$(ls -td /var/mnt/nvme2/extraction/session_bess_* 2>/dev/null | head -1)
if [[ -n "$SESSION_DIR" ]]; then
    echo ""
    echo "=== Hybrid Map Output ==="
    ls -lh "$SESSION_DIR"/fastlio_map_*.laz 2>/dev/null || echo "  No hybrid maps yet (may need more processing time)"
    ls -lh "$SESSION_DIR"/session_cloud*.laz 2>/dev/null || true
    echo ""
    echo "Session dir: $SESSION_DIR"
fi

# Step 8: Cleanup
echo ""
echo "Stopping replay containers..."
sudo podman stop bess-glim-replay 2>/dev/null || true
sudo systemctl stop bess-fast-lio 2>/dev/null || true

echo ""
echo "To restart live services:"
echo "  sudo systemctl start bess-fast-lio bess-glim"
echo ""
echo "To download hybrid maps:"
echo "  scp bessgrey@100.116.195.78:${SESSION_DIR}/fastlio_map_001_*.laz ."
