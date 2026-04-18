#!/bin/bash
# offline_slam_dlio.sh — DLIO solo offline SLAM replay.
#
# Mirror of offline_slam_fastlio.sh but for DLIO only.
# Components: imuguard-off (/ouster/imu -> /ouster/imu_guarded) + dlio-off +
# recorder-off + player-off. DLIO has no IMU-backward-jitter tolerance so
# the guarded topic is required (unlike FAST-LIO2 which has the patched
# imu_cbk tolerance).
#
# Usage:
#   bash scripts/offline_slam_dlio.sh <start_bag> <end_bag> [rate] [tag]
#
# Examples:
#   bash scripts/offline_slam_dlio.sh 16 22 1.0 drive1
#
# Output:
#   /mnt/bess-usb/bags/rolling/slam_offline/dlio_<tag>/
#     slam_output/   MCAP with /dlio/* + /tf + /tf_static + /clock

set -euo pipefail

START=${1:?missing start_bag}
END=${2:?missing end_bag}
RATE=${3:-1.0}
TAG=${4:-run}

BAG_DIR="/mnt/bess-usb/bags/rolling/bag"
OUT_BASE="/mnt/bess-usb/bags/rolling/slam_offline/dlio_${TAG}"
LOG_DIR="${OUT_BASE}/logs"
BAG_OUT="${OUT_BASE}/slam_output"

BESS_ROOT="/home/thor/bess"
CYCLONE_CFG="${BESS_ROOT}/config/cyclonedds.xml"
DLIO_CFG="${BESS_ROOT}/config/dlio/dlio_thor.yaml"

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

echo "=== DLIO Solo Offline Replay ==="
echo "  Tag:    $TAG"
echo "  Bags:   $BAG_COUNT (bag_${START}..bag_${END})"
echo "  Rate:   ${RATE}x"
echo "  Domain: $DOMAIN_ID"
echo "  Output: $OUT_BASE"
echo ""

for c in imuguard-off dlio-off recorder-off player-off; do
    docker rm -f "$c" >/dev/null 2>&1 || true
done

COMMON_ENV=(
    -e "ROS_DOMAIN_ID=$DOMAIN_ID"
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    -e "CYCLONEDDS_URI=file:///config/cyclonedds.xml"
)

echo "[1/4] Starting Ouster IMU guard (/ouster/imu -> /ouster/imu_guarded)..."
docker run -d --rm \
    --name imuguard-off \
    --network host --ipc host \
    "${COMMON_ENV[@]}" \
    -v "$BESS_ROOT/containers/fast-lio/imu_guard.py":/ros2_ws/imu_guard.py:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-ouster:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
        exec python3 /ros2_ws/imu_guard.py --ros-args \
            -p input_topic:=/ouster/imu \
            -p output_topic:=/ouster/imu_guarded \
            -p reliability:=best_effort \
            -p use_sim_time:=true' \
    >/dev/null

sleep 3

echo "[2/4] Starting DLIO (sim-time, /ouster/imu_guarded)..."
docker run -d --rm \
    --name dlio-off \
    --network host --ipc host --privileged \
    --memory=16g --memory-swap=16g \
    "${COMMON_ENV[@]}" \
    -v "$DLIO_CFG":/ros2_ws/dlio_thor.yaml:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-dlio:jazzy \
    bash -c '
        source /opt/ros/jazzy/setup.bash
        source /ros2_ws/install/setup.bash
        sed -i "s/use_sim_time: false/use_sim_time: true/" /ros2_ws/dlio_thor.yaml
        cp /ros2_ws/dlio_thor.yaml /ros2_ws/install/direct_lidar_inertial_odometry/share/direct_lidar_inertial_odometry/cfg/dlio.yaml
        cp /ros2_ws/dlio_thor.yaml /ros2_ws/install/direct_lidar_inertial_odometry/share/direct_lidar_inertial_odometry/cfg/params.yaml
        DLIO_LAUNCH=/ros2_ws/install/direct_lidar_inertial_odometry/share/direct_lidar_inertial_odometry/launch/dlio.launch.py
        if ! grep -q "dlio/map" "$DLIO_LAUNCH"; then
            sed -i "/'\''keyframes'\'', '\''dlio\/odom_node\/pointcloud\/keyframe'\''/a\\            ('\''map'\'', '\''dlio/map'\''),"  "$DLIO_LAUNCH"
        fi
        exec ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
            pointcloud_topic:=/ouster/points \
            imu_topic:=/ouster/imu_guarded \
            rviz:=false
    ' \
    >/dev/null

echo "  Giving DLIO 15s to initialize..."
sleep 15

if ! docker ps --format '{{.Names}}' | grep -q "^dlio-off$"; then
    echo "ERROR: dlio-off exited during startup"
    docker logs dlio-off --tail 40 2>&1 || true
    exit 1
fi
if ! docker exec dlio-off pgrep -f dlio_odom_node >/dev/null 2>&1; then
    echo "ERROR: dlio_odom_node not running inside container"
    docker logs dlio-off --tail 40 2>&1 || true
    docker stop dlio-off 2>/dev/null
    exit 1
fi
echo "  DLIO ready"

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
            /dlio/odom_node/odom /dlio/odom_node/pose /dlio/odom_node/path \
            /dlio/odom_node/pointcloud/deskewed \
            /dlio/odom_node/pointcloud/keyframe \
            /dlio/map' \
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
docker stop -t 10 recorder-off dlio-off imuguard-off 2>/dev/null || true

echo ""
echo "=== DLIO solo replay complete ==="
echo "Output: $BAG_OUT/slam_output/"
ls -lh "$BAG_OUT/slam_output/" 2>/dev/null || true
echo ""
echo "Next: export LAS (reads /ouster/points from original input bags):"
echo "  python3 $BESS_ROOT/scripts/offline_slam_to_las.py \\"
echo "      /mnt/bess-usb/bags/rolling/slam_offline/raw_${TAG}_link \\"
echo "      --odom-bag-dir $BAG_OUT/slam_output \\"
echo "      --odom-topic /dlio/odom_node/odom \\"
echo "      --output $OUT_BASE/export"
