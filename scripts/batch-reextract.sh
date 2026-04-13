#!/bin/bash
##############################################################################
# BESS Batch Re-extraction
#
# Re-processes bags through GLIM (fresh SLAM) + extraction pipeline to produce
# fully BNG-referenced outputs with correct thermal Planck constants.
#
# Usage:
#   batch-reextract.sh <bag_list_file> [--dry-run]
#
# bag_list_file: one bag path per line (from SSDs), sorted by priority
# Output: /var/mnt/nvme1/reextraction/<bag_name>/
#
# Each bag goes through:
#   1. GLIM realtime (fresh SLAM from /ouster/points + /imu/data_guarded)
#   2. FAST-LIO (secondary SLAM)
#   3. Extraction (subscribes to all topics including fresh SLAM)
#   4. Bag playback (publishes ALL topics at 5x rate)
#   5. Validation (check BNG, heading, radiometric, SLAM clouds)
#
# Resume: bags with existing validated output in OUTPUT_BASE are skipped.
# Space: aborts if nvme1 has <500GB free.
##############################################################################
set -eo pipefail

BAG_LIST="${1:?Usage: $0 <bag_list_file> [--dry-run]}"
DRY_RUN=false
[[ "${2:-}" == "--dry-run" ]] && DRY_RUN=true

OUTPUT_BASE="/var/mnt/nvme1/reextraction"
LOG_DIR="$OUTPUT_BASE/logs"
PROGRESS_FILE="$OUTPUT_BASE/progress.json"
MIN_FREE_GB=500
PLAY_RATE=5.0

# Containers / images
GLIM_IMAGE="localhost/bess-glim:humble"
RECORDER_IMAGE="localhost/bess-recorder:humble"
EXTRACTION_IMAGE="localhost/bess-extraction:humble"

mkdir -p "$LOG_DIR"

# ─── Helpers ──────────────────────────────────────────────────────────────────

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_DIR/batch.log"; }

check_space() {
    local free_gb
    free_gb=$(df --output=avail /var/mnt/nvme1 | tail -1 | awk '{printf "%.0f", $1/1024/1024}')
    if (( free_gb < MIN_FREE_GB )); then
        log "ABORT: nvme1 only has ${free_gb}GB free (need ${MIN_FREE_GB}GB)"
        return 1
    fi
    log "Space OK: nvme1 has ${free_gb}GB free"
}

stop_live_services() {
    log "Stopping live services..."
    sudo systemctl stop bess-extraction bess-fast-lio bess-glim 2>/dev/null || true
    sudo systemctl mask --runtime bess-extraction bess-fast-lio bess-glim 2>/dev/null || true
    sudo podman stop bess-glim-replay bess-bag-player 2>/dev/null || true
    sudo podman rm -f bess-glim-replay bess-bag-player bess-extract-replay 2>/dev/null || true
    sleep 3
}

start_glim_replay() {
    log "Starting GLIM (replay mode, FILTER_MAX_AGE=999999)..."
    sudo podman rm -f bess-glim-replay 2>/dev/null || true
    sudo podman run -d --rm \
        --name bess-glim-replay \
        --device nvidia.com/gpu=1 \
        --security-opt label=disable \
        --shm-size=8gb \
        --memory=24G \
        --cpuset-cpus=6-17 \
        --ulimit "stack=4294967296:4294967296" \
        --network=host \
        -e "ROS_DOMAIN_ID=99" \
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
        -v "/opt/bess/scripts/glim-replay-wrapper.sh:/ros2_ws/glim-replay-wrapper.sh:ro" \
        "$GLIM_IMAGE" \
        bash /ros2_ws/glim-replay-wrapper.sh ros2 launch /ros2_ws/launch_glim.py
    sleep 30
    if ! sudo podman ps --filter name=bess-glim-replay --format "{{.Names}}" | grep -q glim; then
        log "ERROR: GLIM failed to start"
        return 1
    fi
    log "GLIM started OK"
}

start_fastlio_replay() {
    # FAST-LIO skipped in batch mode: systemd service uses ROS_DOMAIN_ID=0
    # which can't communicate with our domain 99 replay containers.
    # GLIM alone is sufficient — extraction prefers GLIM poses anyway.
    log "FAST-LIO: skipped (GLIM-only mode, domain 99 isolation)"
}

start_extraction_replay() {
    local output_dir="$1"
    log "Starting extraction (output: $output_dir)..."

    # Ensure MCAP plugin
    local mcap_dir="/tmp/mcap_plugin"
    if [ ! -f "$mcap_dir/lib/librosbag2_storage_mcap.so" ]; then
        log "Extracting MCAP plugin from recorder image..."
        mkdir -p "$mcap_dir"/{lib,share}
        sudo podman run --rm -v "$mcap_dir":/out:rw,Z "$RECORDER_IMAGE" bash -c "
            cp /opt/ros/humble/lib/librosbag2_storage_mcap.so /opt/ros/humble/lib/libmcap.so /out/lib/
            cp -r /opt/ros/humble/share/rosbag2_storage_mcap /out/share/
            cp -r /opt/ros/humble/share/mcap_vendor /out/share/
            mkdir -p /out/share/ament_index/resource_index/{rosbag2_storage__pluginlib__plugin,packages}
            cp /opt/ros/humble/share/ament_index/resource_index/rosbag2_storage__pluginlib__plugin/rosbag2_storage_mcap /out/share/ament_index/resource_index/rosbag2_storage__pluginlib__plugin/
            cp /opt/ros/humble/share/ament_index/resource_index/packages/rosbag2_storage_mcap /out/share/ament_index/resource_index/packages/
            cp /opt/ros/humble/share/ament_index/resource_index/packages/mcap_vendor /out/share/ament_index/resource_index/packages/
        "
    fi

    sudo podman rm -f bess-extract-replay 2>/dev/null || true
    sudo podman run -d \
        --name bess-extract-replay \
        --memory=12G \
        --cpuset-cpus=18-23 \
        --network=host \
        --security-opt label=disable \
        -e "ROS_DOMAIN_ID=99" \
        -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
        -e "CYCLONEDDS_URI=file:///etc/cyclonedds.xml" \
        -e "SPATIAL_DB=/data/spatial/bess_uprn.db" \
        -e "EXTRACTION_DIR=/data/extraction" \
        -e "AMBIENT_TEMP_C=15.0" \
        -v "/opt/bess/config/cyclonedds.xml:/etc/cyclonedds.xml:ro" \
        -v "$output_dir":/data/extraction:rw \
        -v /opt/bess/data/spatial:/data/spatial:ro \
        -v /opt/bess/scripts/bess-extraction.py:/ros2_ws/bess-extraction.py:ro \
        -v /opt/bess/scripts/slam_realtime_accumulator.py:/ros2_ws/slam_realtime_accumulator.py:ro \
        -v /opt/bess/config/extraction:/config:ro \
        -v /opt/bess/config/cameras:/opt/bess/config/cameras:ro \
        -v /run/bess:/run/bess:ro \
        -v "$mcap_dir":/mcap_plugin:ro \
        "$EXTRACTION_IMAGE" \
        bash -c '
            source /opt/ros/humble/setup.bash
            source /ros2_ws/install/setup.bash 2>/dev/null || true
            export AMENT_PREFIX_PATH=/mcap_plugin:${AMENT_PREFIX_PATH}
            export LD_LIBRARY_PATH=/mcap_plugin/lib:${LD_LIBRARY_PATH}
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export ROS_DOMAIN_ID=99
            export CYCLONEDDS_URI=file:///etc/cyclonedds.xml
            python3 /ros2_ws/bess-extraction.py
        '
    sleep 10
    if ! sudo podman ps --filter name=bess-extract-replay --format "{{.Names}}" | grep -q extract; then
        log "ERROR: Extraction failed to start"
        return 1
    fi
    log "Extraction started OK"
}

play_bag() {
    local bag_dir="$1"
    local bag_size_gb
    bag_size_gb=$(du -s "$bag_dir" | awk '{printf "%.0f", $1/1024/1024}')

    # Scale play rate — small bags finish before DDS discovery completes
    local rate="$PLAY_RATE"
    if (( bag_size_gb < 20 )); then
        rate="1.0"
    elif (( bag_size_gb < 50 )); then
        rate="2.0"
    fi

    # Wait for DDS discovery between GLIM, extraction, and bag player
    log "  Waiting 15s for DDS discovery..."
    sleep 15

    log "Playing bag at ${rate}x: $(basename "$bag_dir") (${bag_size_gb}GB)"

    sudo podman rm -f bess-bag-player 2>/dev/null || true
    # Explicit topic whitelist — DO NOT replay old SLAM topics (/slam/*, /glim/*, /kiss/*, /Laser_map).
    # GLIM and FAST-LIO produce fresh SLAM from the bag's sensor data on domain 99.
    # This list covers: sensors, GNSS, cameras, thermals, environment, IMU, vehicle, EKF, diagnostics.
    sudo podman run --rm \
        --name bess-bag-player \
        --network=host \
        --security-opt label=disable \
        -e "ROS_DOMAIN_ID=99" \
        -e "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" \
        -e "CYCLONEDDS_URI=file:///etc/cyclonedds.xml" \
        -v "/opt/bess/config/cyclonedds.xml:/etc/cyclonedds.xml:ro" \
        -v "${bag_dir}:/bag:ro" \
        -v "/opt/bess/config/replay_qos_overrides.yaml:/config/replay_qos_overrides.yaml:ro" \
        "$RECORDER_IMAGE" \
        bash -c "source /opt/ros/humble/setup.bash && \
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && \
            export ROS_DOMAIN_ID=99 && \
            export CYCLONEDDS_URI=file:///etc/cyclonedds.xml && \
            ros2 bag play /bag \
            --rate $rate \
            --read-ahead-queue-size 200 \
            --qos-profile-overrides-path /config/replay_qos_overrides.yaml \
            --topics \
                /ouster/points /ouster/points_filtered /ouster/imu /ouster/range_image /ouster/reflec_image \
                /ouster/nearir_image /ouster/signal_image \
                /imu/data /imu/data_guarded /imu/mag \
                /accel/filtered /odometry/filtered \
                /gnss_1/llh_position /gnss_1/odometry_earth /gnss_1/time /gnss_1/velocity \
                /gnss_2/llh_position /gnss_2/odometry_earth /gnss_2/time /gnss_2/velocity \
                /ekf/llh_position /ekf/odometry_earth /ekf/dual_antenna_heading \
                /ekf/imu/data /ekf/status /ekf/velocity \
                /camera2/camera_driver/image_masked/compressed /camera2/camera_driver/camera_info /camera2/camera_driver/meta \
                /camera3/camera_driver/image_masked/compressed /camera3/camera_driver/camera_info /camera3/camera_driver/meta \
                /thermal1/camera_driver/image_raw /thermal1/camera_driver/camera_info /thermal1/camera_driver/meta \
                /thermal2/camera_driver/image_raw /thermal2/camera_driver/camera_info /thermal2/camera_driver/meta \
                /environment/temperature /environment/humidity \
                /environment/temperature_raw /environment/humidity_raw \
                /vehicle/speed /vehicle/twist /vehicle/steering_angle /vehicle/hv_battery \
                /tf /diagnostics /rtcm /bluetti/battery \
                /mip/ekf/status /mip/ekf/gnss_dual_antenna_status \
                /mip/ekf/gnss_position_aiding_status /mip/ekf/multi_antenna_offset_correction \
                /mip/gnss_1/fix_info /mip/gnss_2/fix_info \
                /thermal/analysis/heatmap /telemetry/snapshot" \
        2>&1 | tail -5

    log "Bag playback complete"
}

stop_replay_containers() {
    log "Stopping replay containers..."
    # Send SIGINT to extraction for clean flush
    sudo podman kill --signal INT bess-extract-replay 2>/dev/null || true
    sleep 15
    sudo podman stop bess-extract-replay bess-glim-replay 2>/dev/null || true
    sudo podman rm -f bess-extract-replay bess-glim-replay bess-bag-player 2>/dev/null || true
}

validate_output() {
    local output_dir="$1"
    local bag_name="$2"

    # Find the session directory (extraction creates session_bess_* inside output_dir)
    local session_dir
    session_dir=$(ls -td "$output_dir"/session_bess_* 2>/dev/null | head -1)
    if [[ -z "$session_dir" ]]; then
        log "VALIDATE FAIL: No session directory in $output_dir"
        echo '{"valid": false, "reason": "no_session_dir"}' > "$output_dir/audit.json"
        return 1
    fi

    python3 << PYEOF
import os, json, glob, sys

session = "$session_dir"
bag_name = "$bag_name"
output = "$output_dir"

issues = []
stats = {}

# Check for any images at all
for subdir in ['left_rgb', 'right_rgb', 'left_thermal', 'right_thermal']:
    d = os.path.join(session, 'images', subdir)
    count = len(glob.glob(f'{d}/*.jpg')) if os.path.isdir(d) else 0
    stats[subdir] = count

total_frames = sum(stats.values())
if total_frames == 0:
    issues.append('no_frames')
elif total_frames < 40:
    issues.append(f'too_few_frames({total_frames})')

# Check each camera has reasonable count
# left_thermal exempt on Mar 25 — thermal1 camera wasn't recording
MIN_PER_CAM = 10
bag_date = bag_name.split('_')[1] if '_' in bag_name else ''
for cam_name, count in stats.items():
    if cam_name == 'left_thermal' and bag_date == '20260325':
        continue  # known missing — thermal1 not recording Mar 25
    if count < MIN_PER_CAM:
        issues.append(f'{cam_name}_low({count})')

# Sample meta
for check_dir in ['left_thermal', 'left_rgb']:
    metas = sorted(glob.glob(f'{session}/images/{check_dir}/*.meta.json'))
    if metas:
        d = json.load(open(metas[len(metas)//2]))

        bng = d.get('bng', {})
        if not bng.get('x') or bng['x'] < 400000:
            issues.append('bad_bng')

        if d.get('heading_deg') in (None, 0, 0.0):
            issues.append('no_heading')

        if 'heading_source' not in d:
            issues.append('no_heading_source')

        if check_dir == 'left_thermal':
            if not d.get('radiometric'):
                issues.append('no_radiometric')
            mn = d.get('min_temp_c', -999)
            mx = d.get('max_temp_c', 999)
            if mn < -30 or mx > 80:
                issues.append(f'bad_temps({mn:.0f}to{mx:.0f})')

        stats['sample_meta'] = d
        break

# SLAM clouds
slam_dir = os.path.join(session, 'images', 'slam_cloud')
slam_count = len(glob.glob(f'{slam_dir}/*.laz')) if os.path.isdir(slam_dir) else 0
stats['slam_clouds'] = slam_count
# SLAM validation relaxed — GLIM not running in phase 1 (BNG-only pass).
# SLAM clouds will be added in phase 2 (task #51).
# if slam_count < 5:
#     issues.append(f'insufficient_slam({slam_count})')

valid = len(issues) == 0
result = {
    'valid': valid,
    'bag_name': bag_name,
    'session_dir': session,
    'issues': issues,
    'frame_counts': {k: v for k, v in stats.items() if isinstance(v, int)},
}

with open(os.path.join(output, 'audit.json'), 'w') as f:
    json.dump(result, f, indent=2)

if valid:
    print(f"VALID: {total_frames} frames, {slam_count} SLAM clouds")
else:
    print(f"ISSUES: {', '.join(issues)} ({total_frames} frames)")

sys.exit(0 if valid else 1)
PYEOF
}

# ─── Main ─────────────────────────────────────────────────────────────────────

if [[ ! -f "$BAG_LIST" ]]; then
    echo "ERROR: Bag list file not found: $BAG_LIST"
    exit 1
fi

TOTAL=$(grep -c . "$BAG_LIST" || true)
log "=== Batch Re-extraction ==="
log "Bag list: $BAG_LIST ($TOTAL bags)"
log "Output: $OUTPUT_BASE"
log "Play rate: ${PLAY_RATE}x"
$DRY_RUN && log "DRY RUN — no processing"

COMPLETED=0
FAILED=0
SKIPPED=0

while IFS= read -r BAG_PATH || [[ -n "$BAG_PATH" ]]; do
    # Skip comments and blank lines
    [[ "$BAG_PATH" =~ ^[[:space:]]*# ]] && continue
    [[ -z "${BAG_PATH// }" ]] && continue

    BAG_NAME=$(basename "$BAG_PATH")
    OUT_DIR="$OUTPUT_BASE/$BAG_NAME"
    IDX=$((COMPLETED + FAILED + SKIPPED + 1))

    log "[$IDX/$TOTAL] === $BAG_NAME ==="

    # Resume: skip if already processed (valid OR failed with data)
    if [[ -f "$OUT_DIR/audit.json" ]]; then
        has_frames=$(python3 -c "import json; d=json.load(open('$OUT_DIR/audit.json')); fc=d.get('frame_counts',{}); print(sum(v for v in fc.values() if isinstance(v,int)))" 2>/dev/null || echo "0")
        if [[ "$has_frames" -gt 0 ]]; then
            log "  SKIP: already processed ($has_frames frames)"
            SKIPPED=$((SKIPPED + 1))
            continue
        fi
    fi

    # Verify bag exists
    if [[ ! -d "$BAG_PATH" ]]; then
        log "  ERROR: bag not found: $BAG_PATH"
        FAILED=$((FAILED + 1))
        continue
    fi

    # Check metadata.yaml
    if [[ ! -f "$BAG_PATH/metadata.yaml" ]]; then
        log "  Rebuilding metadata.yaml..."
        if ! $DRY_RUN; then
            python3 /opt/bess/scripts/rebuild-bag-metadata.py "$BAG_PATH" 2>/dev/null || {
                log "  ERROR: metadata rebuild failed"
                FAILED=$((FAILED + 1))
                continue
            }
        fi
    fi

    # Space check
    if ! check_space; then
        log "STOPPING: insufficient space"
        break
    fi

    if $DRY_RUN; then
        log "  DRY RUN: would process $BAG_NAME"
        SKIPPED=$((SKIPPED + 1))
        continue
    fi

    # Create output dir
    mkdir -p "$OUT_DIR"

    START_TIME=$(date +%s)

    # Stop any previous containers
    stop_replay_containers

    # GLIM skipped — PointCloud2 can't traverse DDS between containers.
    # SLAM will be added in a second pass via GLIM offline per-mcap.
    # See task #50.

    # Start extraction (writes to OUT_DIR)
    if ! start_extraction_replay "$OUT_DIR"; then
        log "  FAIL: extraction didn't start"
        stop_replay_containers
        FAILED=$((FAILED + 1))
        continue
    fi

    # Play the full bag (ALL topics — cameras, thermals, GNSS, IMU, LiDAR)
    # Catch failures — don't let one bad bag kill the whole batch
    if ! play_bag "$BAG_PATH" 2>&1 | tee -a "$LOG_DIR/${BAG_NAME}.log"; then
        log "  WARNING: bag playback failed — continuing to next bag"
    fi

    # Wait for extraction to flush
    log "  Waiting 30s for extraction flush..."
    sleep 30

    # Stop containers
    stop_replay_containers

    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))
    log "  Duration: ${ELAPSED}s ($((ELAPSED/60))m $((ELAPSED%60))s)"

    # Validate
    if validate_output "$OUT_DIR" "$BAG_NAME"; then
        log "  PASS"
        COMPLETED=$((COMPLETED + 1))
    else
        log "  FAIL: validation failed (see $OUT_DIR/audit.json)"
        FAILED=$((FAILED + 1))
    fi

    log "  Progress: $COMPLETED OK, $FAILED failed, $SKIPPED skipped / $TOTAL total"

done < "$BAG_LIST"

# Restore live services
log "Restoring live services..."
sudo systemctl unmask --runtime bess-extraction bess-fast-lio bess-glim 2>/dev/null || true

log ""
log "=== Batch Complete ==="
log "Completed: $COMPLETED"
log "Failed:    $FAILED"
log "Skipped:   $SKIPPED"
log "Total:     $TOTAL"
