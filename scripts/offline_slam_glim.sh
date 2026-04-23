#!/bin/bash
# offline_slam_glim.sh — GLIM solo offline SLAM replay.
#
# Mirror of offline_slam_dlio.sh but for GLIM. GLIM runs on ROS 2 Humble
# (image glim-ros2:humble-cyclone, CPU-only) and subscribes to /ouster/points
# + /ouster/imu_guarded. Extrinsic fixed in config_sensors_thor.json
# (previous default had -20 cm Z error, source of GLIM's historical
# too-sparse behaviour on this rig).
#
# Components:
#   imuguard-off-glim   — /ouster/imu -> /ouster/imu_guarded (BEST_EFFORT sub)
#   glim-tf-off         — static TF os_imu -> glim_base_link sidecar
#   glim-off            — glim_rosnode with config_offline_thor.json
#   glim-map-off        — voxel-hashing accumulator /glim_ros/aligned_points_corrected
#                         -> /glim_ros/map_voxel (global_mapping off, so native /glim_ros/map
#                          is empty — this is the Foxglove-visible world map)
#   recorder-off        — ros2 bag record /glim_ros/* + /tf + /clock
#   player-off          — ros2 bag play on input MCAPs at --clock
#
# Usage:
#   bash scripts/offline_slam_glim.sh <start_bag> <end_bag> [rate] [tag]
#
# Examples:
#   bash scripts/offline_slam_glim.sh 16 22 1.0 drive1
#   bash scripts/offline_slam_glim.sh 40 45 1.0 bags40to45
#
# Output:
#   /home/thor/nas/bess-bags/rolling/slam_offline/glim_<tag>/
#     slam_output/   MCAP with /glim_ros/* + /tf + /tf_static + /clock

set -euo pipefail

START=${1:?missing start_bag}
END=${2:?missing end_bag}
RATE=${3:-1.0}
TAG=${4:-run}

BAG_DIR="/home/thor/nas/bess-bags/rolling/bag"
OUT_BASE="/home/thor/nas/bess-bags/rolling/slam_offline/glim_${TAG}"
LOG_DIR="${OUT_BASE}/logs"
BAG_OUT="${OUT_BASE}/slam_output"

BESS_ROOT="/home/thor/bess"
CYCLONE_CFG="${BESS_ROOT}/config/cyclonedds.xml"
GLIM_CFG_DIR="${BESS_ROOT}/config/glim"

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

echo "=== GLIM Solo Offline Replay ==="
echo "  Tag:    $TAG"
echo "  Bags:   $BAG_COUNT (bag_${START}..bag_${END})"
echo "  Rate:   ${RATE}x"
echo "  Domain: $DOMAIN_ID"
echo "  Output: $OUT_BASE"
echo ""

for c in imuguard-off-glim glim-tf-off glim-off glim-map-off recorder-off player-off; do
    docker rm -f "$c" >/dev/null 2>&1 || true
done

COMMON_ENV_JAZZY=(
    -e "ROS_DOMAIN_ID=$DOMAIN_ID"
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    -e "CYCLONEDDS_URI=file:///config/cyclonedds.xml"
)
COMMON_ENV_HUMBLE=(
    -e "ROS_DOMAIN_ID=$DOMAIN_ID"
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    -e "CYCLONEDDS_URI=file:///dds/cyclonedds.xml"
)

# GLIM needs monotonic IMU stamps: its sub_mapping emits "imu_rate stamp does
# not cover the scan duration range" whenever a BMI085 rewind causes GLIM to
# skip samples, and sustained skipping (~30 % of frames on drive1) makes
# odometry_estimation_cpu never fire on_new_frame -> no /glim_ros/odom at all.
# The ouster_imu_guard now uses non-drifting semantics (raw stamp unless the
# stamp is non-monotonic, in which case push by min_increment only) so the
# IMU clock stays aligned to the LiDAR clock over arbitrarily long bags.

echo "[1/6] Starting Ouster IMU guard (/ouster/imu -> /ouster/imu_guarded)..."
docker run -d --rm \
    --name imuguard-off-glim \
    --network host --ipc host \
    "${COMMON_ENV_JAZZY[@]}" \
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

echo "[2/6] Starting static TF (os_imu -> glim_base_link)..."
docker run -d --rm \
    --name glim-tf-off \
    --network host --ipc host \
    "${COMMON_ENV_HUMBLE[@]}" \
    -v "$CYCLONE_CFG":/dds/cyclonedds.xml:ro \
    glim-ros2:humble-cyclone \
    bash -c 'source /opt/ros/humble/setup.bash && \
        exec ros2 run tf2_ros static_transform_publisher \
            --x 0 --y 0 --z 0 \
            --roll 0 --pitch 0 --yaw 0 \
            --frame-id os_imu --child-frame-id glim_base_link \
            --ros-args -p use_sim_time:=true' \
    >/dev/null

sleep 2

echo "[3/6] Starting GLIM (sim-time, config_offline_thor.json)..."
docker run -d --rm \
    --name glim-off \
    --network host --ipc host --privileged \
    --memory=16g --memory-swap=16g \
    "${COMMON_ENV_HUMBLE[@]}" \
    -e GLIM_DATA_PATH=/usr/local/share/glim \
    -v "$GLIM_CFG_DIR":/config:ro \
    -v "$CYCLONE_CFG":/dds/cyclonedds.xml:ro \
    glim-ros2:humble-cyclone \
    bash -c '
        source /opt/ros/humble/setup.bash
        # GLIM picks up the dispatcher file named config.json by default;
        # we override the per-run dispatcher via env GLIM_DATA_PATH + the
        # -p config_path arg (points at a directory). The directory must
        # contain a config.json. Mount-time we swap in the thor-offline
        # dispatcher by symlinking it as config.json inside a rw staging dir.
        mkdir -p /tmp/glim_run
        cp /config/*.json /tmp/glim_run/
        cp /tmp/glim_run/config_offline_thor.json /tmp/glim_run/config.json
        exec ros2 run glim_ros glim_rosnode \
            --ros-args \
            -p config_path:=/tmp/glim_run \
            -p use_sim_time:=true
    ' \
    >/dev/null

echo "  Giving GLIM 15s to initialize..."
sleep 15

if ! docker ps --format '{{.Names}}' | grep -q "^glim-off$"; then
    echo "ERROR: glim-off exited during startup"
    docker logs glim-off --tail 60 2>&1 || true
    exit 1
fi
if ! docker exec glim-off pgrep -f glim_rosnode >/dev/null 2>&1; then
    echo "ERROR: glim_rosnode not running inside container"
    docker logs glim-off --tail 60 2>&1 || true
    docker stop glim-off 2>/dev/null
    exit 1
fi
echo "  GLIM ready"

echo ""
echo "[4/6] Starting GLIM map accumulator (/glim_ros/aligned_points_corrected -> /glim_ros/map_voxel)..."
# GLIM's native /glim_ros/map is produced only by the global_mapping backend,
# which we keep OFF for single-drive solo replays (iSAM2 x1 indeterminant crashes
# with no loop closures). Run the same voxel-hashing accumulator used for
# FAST-LIO2 offline, but point it at /glim_ros/aligned_points_corrected (already
# in glim_map frame per rviz_viewer.cpp:466) so Foxglove still gets a unified
# world map. Corrected stream is the odometry-updated pose, so the accumulator
# benefits from every IMU smoothing step without needing global mapping.
docker run -d --rm \
    --name glim-map-off \
    --network host --ipc host \
    "${COMMON_ENV_JAZZY[@]}" \
    -v "$BESS_ROOT/scripts/slam_map_accumulator.py":/ros2_ws/slam_map_accumulator.py:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-ouster:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
        exec python3 /ros2_ws/slam_map_accumulator.py --ros-args \
            -p input_topic:=/glim_ros/aligned_points_corrected \
            -p output_topic:=/glim_ros/map_voxel \
            -p frame_id:=glim_map \
            -p voxel_size:=0.25 \
            -p max_voxels:=4000000 \
            -p publish_period_sec:=2.0 \
            -p use_sim_time:=true' \
    >/dev/null
sleep 3

echo ""
echo "[5/6] Starting recorder..."
mkdir -p "$BAG_OUT"
# Record ONLY SLAM outputs. /ouster/points + /ouster/imu stay in input bags
# (the LAS exporter reads them from there via --odom-bag-dir).
docker run -d --rm \
    --name recorder-off \
    --network host --ipc host \
    --user "$(id -u):$(id -g)" \
    "${COMMON_ENV_JAZZY[@]}" \
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
            /glim_ros/odom \
            /glim_ros/odom_corrected \
            /glim_ros/pose \
            /glim_ros/pose_corrected \
            /glim_ros/points \
            /glim_ros/points_corrected \
            /glim_ros/aligned_points \
            /glim_ros/aligned_points_corrected \
            /glim_ros/map \
            /glim_ros/map_voxel' \
    >/dev/null
sleep 3

echo ""
echo "[6/6] Playing bags (${BAG_COUNT} @ ${RATE}x)..."
docker run --rm \
    --name player-off \
    --network host --ipc host \
    --init \
    "${COMMON_ENV_JAZZY[@]}" \
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
        echo 'Playback done. Flushing for 30s (GLIM global mapping finalises)...'
        sleep 30
    "

echo ""
echo "Stopping containers..."
docker stop -t 10 recorder-off glim-map-off glim-off glim-tf-off imuguard-off-glim 2>/dev/null || true

echo ""
echo "=== GLIM solo replay complete ==="
echo "Output: $BAG_OUT/slam_output/"
ls -lh "$BAG_OUT/slam_output/" 2>/dev/null || true
echo ""
echo "Next: export LAS (uses GLIM's already-world-frame aligned_points_corrected,"
echo "       bypassing exporter SLERP-deskew — matches DLIO-deskewed pattern):"
echo "  python3 $BESS_ROOT/scripts/offline_slam_to_las.py \\"
echo "      $BAG_OUT/slam_output \\"
echo "      --odom-bag-dir $BAG_OUT/slam_output \\"
echo "      --odom-topic /glim_ros/odom \\"
echo "      --cloud-topic /glim_ros/aligned_points_corrected \\"
echo "      --output $OUT_BASE/export"
