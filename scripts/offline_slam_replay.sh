#!/bin/bash
# Offline SLAM reprocessing — replays raw Ouster bags through FAST-LIO2
# then exports to LAS/COPC with full Ouster field preservation.
#
# Usage: bash scripts/offline_slam_replay.sh [start_bag] [end_bag] [rate]
#   start_bag: first bag index (default: 0)
#   end_bag:   last bag index (default: 144)
#   rate:      playback rate (default: 1.0)

set -e

BAG_DIR="/home/thor/nas/bess-bags/rolling/bag"
OUT_DIR="/home/thor/nas/bess-bags/rolling/slam_offline"
START=${1:-0}
END=${2:-144}
RATE=${3:-1.0}

mkdir -p "$OUT_DIR"

echo "=== Offline SLAM Replay ==="
echo "  Bags: bag_${START}.mcap → bag_${END}.mcap"
echo "  Rate: ${RATE}x"
echo "  Output: ${OUT_DIR}"
echo ""

# Build list of bag files (using container-internal /bags path)
BAG_INPUTS=""
for i in $(seq $START $END); do
    f="$BAG_DIR/bag_${i}.mcap"
    if [ -f "$f" ] && [ -s "$f" ]; then
        BAG_INPUTS="$BAG_INPUTS -i /bags/bag_${i}.mcap mcap"
    fi
done

echo "[1/4] Starting FAST-LIO2 (sim time)..."
docker run -d --rm \
    --name fastlio-offline \
    --network host \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///config/cyclonedds.xml \
    -e ROS_DOMAIN_ID=42 \
    -e ENABLE_SCANCONTEXT=0 \
    -v /home/thor/bess/containers/fast-lio/fast_lio_single.yaml:/ros2_ws/config/fast_lio_single.yaml:ro \
    -v /home/thor/bess/containers/fast-lio/launch_fast_lio_single.py:/ros2_ws/launch_fast_lio_single.py:ro \
    -v /home/thor/bess/containers/fast-lio/imu_guard.py:/ros2_ws/imu_guard.py:ro \
    -v /home/thor/bess/containers/fast-lio/scancontext_node.py:/ros2_ws/scancontext_node.py:ro \
    -v /home/thor/bess/containers/fast-lio/camera_tf_broadcaster.py:/ros2_ws/camera_tf_broadcaster.py:ro \
    -v /home/thor/bess/containers/fast-lio/tf_republisher.py:/ros2_ws/tf_republisher.py:ro \
    -v /home/thor/bess/config/cyclonedds.xml:/config/cyclonedds.xml:ro \
    localhost/bess-fast-lio:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && ros2 launch /ros2_ws/launch_fast_lio_single.py use_sim_time:=true'

echo "  Waiting for FAST-LIO2 to initialize..."
sleep 10

# Verify fastlio_mapping is running
if ! docker exec fastlio-offline pgrep -x fastlio_mapping >/dev/null 2>&1; then
    echo "ERROR: fastlio_mapping not running"
    docker logs fastlio-offline --tail 20
    docker stop fastlio-offline 2>/dev/null
    exit 1
fi
echo "  FAST-LIO2 ready"

echo ""
echo "[2/4] Starting SLAM output recorder..."
# Two critical changes vs earlier versions (see 2026-04-17 diagnosis):
#   (a) /ouster/points and /ouster/imu are NOT recorded — they are already in
#       the input bags, replaying them meant the recorder fought the bag-play
#       reader for the single HighPoint RM110 USB channel and dropped 11% of
#       point-cloud frames with 0.5-2.4 s gaps. FAST-LIO2's iEKF could not
#       survive a scan stream that irregular, which looked like SLAM
#       divergence but was actually I/O back-pressure.
#   (b) `--qos-profile-overrides-path` bind-mounts the shared QoS file so
#       BEST_EFFORT publishers (none in this topic list now, but harmless)
#       and RELIABLE /tf topics match — prevents silent subscription QoS
#       mismatches. Note: `ros2 bag record` in Jazzy does NOT accept
#       `--ros-args -p use_sim_time:=true` (its own `-p` is for
#       --polling-interval). The `--use-sim-time` CLI flag is the only
#       supported path and is sufficient — the fullstack run verified it
#       produced normal ~100-150 ms recv-vs-header skew.
docker run -d --rm \
    --name slam-recorder \
    --network host \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///config/cyclonedds.xml \
    -e ROS_DOMAIN_ID=42 \
    -v /home/thor/bess/config/cyclonedds.xml:/config/cyclonedds.xml:ro \
    -v /home/thor/rosbag_qos.yaml:/config/rosbag_qos.yaml:ro \
    -v "$OUT_DIR":/recordings:rw \
    localhost/bess-ouster:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && ros2 bag record \
        -o /recordings/slam_output \
        -s mcap \
        --use-sim-time \
        --qos-profile-overrides-path /config/rosbag_qos.yaml \
        /fast_lio/odometry \
        /fast_lio/cloud_registered \
        /fast_lio/cloud_registered_body \
        /fast_lio/path \
        /tf \
        /tf_static'

sleep 5
echo "  Recorder ready"

echo ""
echo "[3/4] Playing bags (rate=${RATE}x)..."
echo "  This will take ~$(($(echo "($END - $START + 1) * 20 / 1" | bc) / 1))s of data at ${RATE}x..."

docker run --rm \
    --name bag-player \
    --network host \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e CYCLONEDDS_URI=file:///config/cyclonedds.xml \
    -e ROS_DOMAIN_ID=42 \
    -v /home/thor/bess/config/cyclonedds.xml:/config/cyclonedds.xml:ro \
    -v "$BAG_DIR":/bags:ro \
    localhost/bess-ouster:jazzy \
    bash -c "source /opt/ros/jazzy/setup.bash && ros2 bag play \
        $BAG_INPUTS \
        --topics /ouster/points /ouster/imu \
        --clock 200 \
        -r $RATE \
        --read-ahead-queue-size 100 \
        --disable-keyboard-controls"

echo ""
echo "  Playback complete. Waiting for SLAM to flush..."
sleep 15

echo ""
echo "[4/4] Stopping containers..."
docker stop slam-recorder 2>/dev/null || true
docker stop fastlio-offline 2>/dev/null || true

echo ""
echo "=== Offline SLAM complete ==="
echo "Output bag: $OUT_DIR/slam_output/"
ls -lh "$OUT_DIR/slam_output/" 2>/dev/null
echo ""
echo "Next: run LAS export with the new clean poses:"
echo "  python3 scripts/offline_slam_to_las.py $OUT_DIR/slam_output --voxel 0.05"
