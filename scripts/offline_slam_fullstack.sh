#!/bin/bash
# offline_slam_fullstack.sh — like-for-live offline SLAM replay.
#
# Mirrors the live `--profile slam` stack (FAST-LIO2 + DLIO + GLIM +
# slam-map-accumulator + recorder) driven by a sequential bag player
# on an isolated ROS domain so it doesn't collide with the live stack.
#
# Live stack runs on ROS_DOMAIN_ID=0; this replay uses ROS_DOMAIN_ID=42.
# All containers get `--name *-off` to avoid name collisions with live.
#
# Usage:
#   bash scripts/offline_slam_fullstack.sh <start_bag> <end_bag> [rate] [tag]
#
# Examples:
#   bash scripts/offline_slam_fullstack.sh 16 22 1.0 drive1   # moving range 1
#   bash scripts/offline_slam_fullstack.sh 64 71 1.0 drive2   # moving range 2
#
# Output:
#   /mnt/bess-usb/bags/rolling/slam_offline/fullstack_<tag>/
#     slam_output/          # MCAP bag w/ all three SLAM outputs
#     logs/{fastlio,dlio,glim,accum,player}.log

set -euo pipefail

START=${1:?missing start_bag}
END=${2:?missing end_bag}
RATE=${3:-1.0}
TAG=${4:-run}

BAG_DIR="/mnt/bess-usb/bags/rolling/bag"
OUT_BASE="/mnt/bess-usb/bags/rolling/slam_offline/fullstack_${TAG}"
LOG_DIR="${OUT_BASE}/logs"
BAG_OUT="${OUT_BASE}/slam_output"

BESS_ROOT="/home/thor/bess"
CYCLONE_CFG="${BESS_ROOT}/config/cyclonedds.xml"
# Use the LIVE yaml — pfilt:1, cube:1000, raw /ouster/imu. The "_offline"
# highway tuning (pfilt:3, cube:800, guarded IMU) diverged catastrophically
# at motion start (FAST-LIO Z→-1500m, DLIO → 10^9 m). The live params are the
# known-working config per 2026-04-12 diagnosis in CLAUDE.md.
FASTLIO_CFG="${BESS_ROOT}/containers/fast-lio/fast_lio_single.yaml"
DLIO_CFG="${BESS_ROOT}/config/dlio/dlio_thor.yaml"
GLIM_CFG_DIR="${BESS_ROOT}/config/glim"
MAP_ACCUM_PY="${BESS_ROOT}/scripts/slam_map_accumulator.py"

DOMAIN_ID=42

mkdir -p "$LOG_DIR"
sudo rm -rf "$BAG_OUT" 2>/dev/null || true

# Build bag list
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

echo "=== Offline SLAM Fullstack Replay ==="
echo "  Tag:       $TAG"
echo "  Bags:      $BAG_COUNT (bag_${START}..bag_${END})"
echo "  Rate:      ${RATE}x"
echo "  Domain:    $DOMAIN_ID (isolated from live=0)"
echo "  Output:    $OUT_BASE"
echo "  SLAMs:     FAST-LIO2 + DLIO + GLIM + map-accumulator"
echo ""

# Cleanup any prior offline containers
for c in imuguard-off fastlio-off dlio-off glim-off accum-off recorder-off player-off; do
    docker rm -f "$c" >/dev/null 2>&1 || true
done

# Common env block
COMMON_ENV=(
    -e "ROS_DOMAIN_ID=$DOMAIN_ID"
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
    -e "CYCLONEDDS_URI=file:///config/cyclonedds.xml"
)

echo "[0/5] Starting Ouster IMU guard (/ouster/imu -> /ouster/imu_guarded)..."
# Raw /ouster/imu has backwards-timestamp jumps up to ~11ms (BMI085
# interleaving). FAST-LIO2's iEKF has a ±10ms tolerance and DLIO has
# none — both diverge without a monotonic stream. The offline
# FAST-LIO2 yaml expects /ouster/imu_guarded; DLIO is also remapped
# below. Guard runs with sim-time so clock aligns with bag replay.
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

echo "[1/5] Starting FAST-LIO2 (sim-time, scancontext off)..."
docker run -d --rm \
    --name fastlio-off \
    --network host --ipc host --privileged \
    "${COMMON_ENV[@]}" \
    -e ENABLE_SCANCONTEXT=0 \
    -v "$FASTLIO_CFG":/ros2_ws/config/fast_lio_single.yaml:ro \
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

echo "[2/5] Starting DLIO (sim-time)..."
# Patched DLIO launch to remap /map -> /dlio/map (same as live container command)
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
        # Inject use_sim_time into the dlio yaml (original has use_sim_time: false)
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

echo "[3/5] Starting GLIM (sim-time, CPU-only)..."
docker run -d --rm \
    --name glim-off \
    --network host --ipc host \
    -e "ROS_DOMAIN_ID=$DOMAIN_ID" \
    -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
    -e "CYCLONEDDS_URI=file:///dds/cyclonedds.xml" \
    -e "CUDA_VISIBLE_DEVICES=" \
    -e "NVIDIA_VISIBLE_DEVICES=" \
    -e "GLIM_DATA_PATH=/usr/local/share/glim" \
    -v "$GLIM_CFG_DIR":/config:ro \
    -v "$CYCLONE_CFG":/dds/cyclonedds.xml:ro \
    glim-ros2:humble-cyclone \
    bash -c '
        set -e
        source /opt/ros/humble/setup.bash
        # GLIM needs os_imu -> glim_base_link static TF (same as live container)
        ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 os_imu glim_base_link \
            --ros-args -p use_sim_time:=true &
        exec ros2 run glim_ros glim_rosnode --ros-args \
            -p config_path:=/config \
            -p use_sim_time:=true
    ' \
    >/dev/null

echo "[4/5] Starting map accumulator (FAST-LIO2 cloud)..."
docker run -d --rm \
    --name accum-off \
    --network host --ipc host \
    "${COMMON_ENV[@]}" \
    -v "$MAP_ACCUM_PY":/ros2_ws/slam_map_accumulator.py:ro \
    -v "$CYCLONE_CFG":/config/cyclonedds.xml:ro \
    localhost/bess-ouster:jazzy \
    bash -c 'source /opt/ros/jazzy/setup.bash && \
             exec python3 /ros2_ws/slam_map_accumulator.py --ros-args \
                 -p input_topic:=/fast_lio/cloud_registered \
                 -p output_topic:=/fast_lio/map_voxel \
                 -p frame_id:=camera_init \
                 -p voxel_size:=0.12 \
                 -p max_voxels:=4000000 \
                 -p publish_period_sec:=2.0 \
                 -p use_sim_time:=true' \
    >/dev/null

echo "  Giving SLAM stacks 15s to initialize..."
sleep 15

# Sanity check
for c in imuguard-off fastlio-off dlio-off glim-off accum-off; do
    if ! docker ps --format '{{.Names}}' | grep -q "^${c}$"; then
        echo "ERROR: $c exited during startup"
        docker logs "$c" --tail 30 2>&1 || true
        exit 1
    fi
done
echo "  All 4 SLAM containers up"

echo ""
echo "[5/5] Starting recorder + bag player..."
mkdir -p "$BAG_OUT"

# Start recorder first.
# /ouster/points and /ouster/imu are deliberately NOT recorded — they are
# already in the input bags. Re-recording them made the recorder fight the
# bag-play reader for the single HighPoint RM110 USB channel and dropped
# ~11% of point-cloud frames with 0.5-2.4 s gaps (2026-04-17 forensics).
# LAS export reads raw /ouster/points from the original input bags via
# offline_slam_to_las.py --odom-bag-dir.
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
            /ouster/imu_guarded \
            /tf /tf_static \
            /fast_lio/odometry /fast_lio/path \
            /fast_lio/cloud_registered /fast_lio/cloud_registered_body \
            /fast_lio/map_voxel \
            /dlio/odom_node/odom /dlio/odom_node/pose /dlio/odom_node/path \
            /dlio/odom_node/pointcloud/deskewed \
            /dlio/odom_node/pointcloud/keyframe \
            /dlio/map \
            /glim_ros/odom /glim_ros/pose /glim_ros/path \
            /glim_ros/points /glim_ros/map \
            /glim_ros/submap' \
    >/dev/null
sleep 3

# Player (runs in foreground, sequential)
echo "  Recorder up; starting player (${BAG_COUNT} bags @ ${RATE}x)..."
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
                --clock 200 \
                -r $RATE \
                --read-ahead-queue-size 1000 \
                --disable-keyboard-controls
        done
        echo 'Playback done. Flushing for 20s...'
        sleep 20
    "

echo ""
echo "Stopping containers..."
docker stop -t 10 recorder-off fastlio-off dlio-off glim-off accum-off imuguard-off 2>/dev/null || true

echo ""
echo "=== Fullstack offline SLAM complete ==="
echo "Output bag: $BAG_OUT/slam_output/"
ls -lh "$BAG_OUT/slam_output/" 2>/dev/null || true
echo ""
echo "Next: export LAS with the best-performing SLAM's odometry, e.g.:"
echo "  python3 $BESS_ROOT/scripts/offline_slam_to_las.py \\"
echo "      $BAG_OUT/slam_output \\"
echo "      --odom-topic /fast_lio/odometry \\"
echo "      --out-dir $OUT_BASE/export_fastlio"
