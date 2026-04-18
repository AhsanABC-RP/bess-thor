#!/bin/bash
# offline_slam_fastlio.sh — FAST-LIO2 solo offline SLAM replay.
#
# Strip of offline_slam_fullstack.sh: FAST-LIO2 + ouster_imu_guard + recorder
# + player. No DLIO, no GLIM, no map-accumulator. Goal is clean 10 Hz
# /fast_lio/odometry output.
#
# The fast-lio image has a -0.01s imu_cbk tolerance patch (2026-04-12) but
# raw /ouster/imu has backward-jitter up to ~11 ms (>10 ms), so the worst
# packet still clears the imu_buffer and sync_packages() starves. 2026-04-18
# Round 2 adds ouster_imu_guard.py to the offline script, mirroring
# offline_slam_dlio.sh. The guard forces monotonic stamps so the buffer
# never clears and /fast_lio/odometry stays at the 10 Hz input cadence.
#
# Usage:
#   bash scripts/offline_slam_fastlio.sh <start_bag> <end_bag> [rate] [tag]
#
# Examples:
#   bash scripts/offline_slam_fastlio.sh 16 22 1.0 drive1
#
# Output:
#   /mnt/bess-usb/bags/rolling/slam_offline/fastlio_<tag>/
#     slam_output/   MCAP with /fast_lio/* + /tf + /tf_static + /clock

set -euo pipefail

START=${1:?missing start_bag}
END=${2:?missing end_bag}
RATE=${3:-1.0}
TAG=${4:-run}

BAG_DIR="/mnt/bess-usb/bags/rolling/bag"
OUT_BASE="/mnt/bess-usb/bags/rolling/slam_offline/fastlio_${TAG}"
LOG_DIR="${OUT_BASE}/logs"
BAG_OUT="${OUT_BASE}/slam_output"

BESS_ROOT="/home/thor/bess"
CYCLONE_CFG="${BESS_ROOT}/config/cyclonedds.xml"
# Offline yaml: point_filter_num=3, cube=800, imu_topic=/ouster/imu_guarded.
# The guarded IMU comes from ouster_imu_guard.py (started below) and is REQUIRED:
# raw /ouster/imu has ≤11 ms backward-jitter that trips FAST-LIO2's imu_cbk
# even with our -0.01 s tolerance patch (see laserMapping.cpp:754). Clears drop
# the IMU buffer and sync_packages() starves → 5 Hz cadence observed on drive1.
FASTLIO_CFG="${BESS_ROOT}/containers/fast-lio/fast_lio_single_offline.yaml"

DOMAIN_ID=42

mkdir -p "$LOG_DIR"
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

echo "=== FAST-LIO2 Solo Offline Replay ==="
echo "  Tag:    $TAG"
echo "  Bags:   $BAG_COUNT (bag_${START}..bag_${END})"
echo "  Rate:   ${RATE}x"
echo "  Domain: $DOMAIN_ID"
echo "  Output: $OUT_BASE"
echo ""

for c in imuguard-off-fl fastlio-off recorder-off player-off; do
    docker rm -f "$c" >/dev/null 2>&1 || true
done

COMMON_ENV=(
    -e "ROS_DOMAIN_ID=$DOMAIN_ID"
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    -e "CYCLONEDDS_URI=file:///config/cyclonedds.xml"
)

echo "[1/4] Starting Ouster IMU guard (/ouster/imu -> /ouster/imu_guarded)..."
# Container name suffixed -fl to avoid colliding with DLIO script's imuguard-off
# if both ever run concurrently. Ouster-specific guard has 0.5 ms min_increment
# matched to the BMI085 ~640 Hz rate.
docker run -d --rm \
    --name imuguard-off-fl \
    --network host --ipc host \
    "${COMMON_ENV[@]}" \
    -v "$BESS_ROOT/containers/fast-lio/ouster_imu_guard.py":/ros2_ws/ouster_imu_guard.py:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-ouster:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
        exec python3 /ros2_ws/ouster_imu_guard.py --ros-args \
            -p input_topic:=/ouster/imu \
            -p output_topic:=/ouster/imu_guarded \
            -p use_sim_time:=true' \
    >/dev/null

sleep 3

echo "[2/4] Starting FAST-LIO2 (sim-time, scancontext off, /ouster/imu_guarded)..."
docker run -d --rm \
    --name fastlio-off \
    --network host --ipc host --privileged \
    "${COMMON_ENV[@]}" \
    -e ENABLE_SCANCONTEXT=0 \
    -v "$FASTLIO_CFG":/ros2_ws/config/fast_lio_single.yaml:ro \
    -v "$FASTLIO_CFG":/ros2_ws/config/fast_lio_single_offline.yaml:ro \
    -v "$BESS_ROOT/containers/fast-lio/launch_fast_lio_single.py":/ros2_ws/launch_fast_lio_single.py:ro \
    -v "$BESS_ROOT/containers/fast-lio/imu_guard.py":/ros2_ws/imu_guard.py:ro \
    -v "$BESS_ROOT/containers/fast-lio/scancontext_node.py":/ros2_ws/scancontext_node.py:ro \
    -v "$BESS_ROOT/containers/fast-lio/camera_tf_broadcaster.py":/ros2_ws/camera_tf_broadcaster.py:ro \
    -v "$BESS_ROOT/containers/fast-lio/tf_republisher.py":/ros2_ws/tf_republisher.py:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-fast-lio:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
             ros2 launch /ros2_ws/launch_fast_lio_single.py use_sim_time:=true' \
    >/dev/null

echo "  Giving FAST-LIO2 12s to initialize..."
sleep 12

if ! docker ps --format '{{.Names}}' | grep -q "^fastlio-off$"; then
    echo "ERROR: fastlio-off exited during startup"
    docker logs fastlio-off --tail 40 2>&1 || true
    exit 1
fi
if ! docker exec fastlio-off pgrep -x fastlio_mapping >/dev/null 2>&1; then
    echo "ERROR: fastlio_mapping not running inside container"
    docker logs fastlio-off --tail 40 2>&1 || true
    docker stop fastlio-off 2>/dev/null
    exit 1
fi
echo "  FAST-LIO2 ready"

echo ""
echo "[3/4] Starting recorder..."
mkdir -p "$BAG_OUT"
# Record ONLY SLAM outputs. /ouster/points + /ouster/imu stay in input bags
# (the LAS exporter reads them from there via --odom-bag-dir).
docker run -d --rm \
    --name recorder-off \
    --network host --ipc host \
    --user "$(id -u):$(id -g)" \
    "${COMMON_ENV[@]}" \
    -v "$BAG_OUT":/rec:rw \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-ouster:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
        exec ros2 bag record \
            -o /rec/slam_output \
            -s mcap \
            --use-sim-time \
            --max-cache-size 500000000 \
            --max-bag-size 5000000000 \
            /clock \
            /tf /tf_static \
            /fast_lio/odometry /fast_lio/path \
            /fast_lio/cloud_registered /fast_lio/cloud_registered_body' \
    >/dev/null
sleep 3

echo ""
echo "[4/4] Playing bags (${BAG_COUNT} @ ${RATE}x)..."
docker run --rm \
    --name player-off \
    --network host --ipc host \
    --init \
    "${COMMON_ENV[@]}" \
    -v "$BAG_DIR":/bags:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-ouster:jazzy \
    bash -c "
        source /opt/ros/jazzy/setup.bash
        PLAYED=0
        for IDX in $BAG_LIST; do
            PLAYED=\$((PLAYED + 1))
            echo \"[\${PLAYED}/$BAG_COUNT] Playing bag_\${IDX}.mcap...\"
            ros2 bag play \
                -i /bags/bag_\${IDX}.mcap mcap \
                --topics /ouster/points /ouster/imu /tf_static \
                --clock 50 \
                -r $RATE \
                --read-ahead-queue-size 4000 \
                --disable-keyboard-controls
        done
        echo 'Playback done. Flushing for 20s...'
        sleep 20
    "

echo ""
echo "Stopping containers..."
docker stop -t 10 recorder-off fastlio-off imuguard-off-fl 2>/dev/null || true

echo ""
echo "=== FAST-LIO2 solo replay complete ==="
echo "Output: $BAG_OUT/slam_output/"
ls -lh "$BAG_OUT/slam_output/" 2>/dev/null || true
echo ""
echo "Next: export LAS (reads /ouster/points from original input bags):"
echo "  python3 $BESS_ROOT/scripts/offline_slam_to_las.py \\"
echo "      /mnt/bess-usb/bags/rolling/slam_offline/raw_${TAG}_link \\"
echo "      --odom-bag-dir $BAG_OUT/slam_output \\"
echo "      --odom-topic /fast_lio/odometry \\"
echo "      --output $OUT_BASE/export"
