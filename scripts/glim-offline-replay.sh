#!/bin/bash
##############################################################################
# BESS GLIM Offline Bag Replay
#
# Replays a rosbag2 through GLIM's glim_rosbag tool for offline SLAM
# processing. Outputs map data to a specified directory.
#
# Usage:
#   glim-offline-replay.sh <bag_dir> [output_dir] [--dense] [--duration <sec>]
#
# Examples:
#   # Standard density, full bag
#   glim-offline-replay.sh /var/mnt/nvme1/bag-cache/bess_20260312_143905
#
#   # Dense mode (2.5x more points), first 10 minutes
#   glim-offline-replay.sh /var/mnt/nvme1/bag-cache/bess_20260312_143905 \
#       /tmp/glim_offline_test --dense --duration 600
#
#   # Custom output directory
#   glim-offline-replay.sh /path/to/bag /var/mnt/nvme2/glim_offline/run1
#
# Dense mode changes:
#   - random_downsample_rate: 0.1 -> 0.25 (keep 25% vs 10% of raw points)
#   - downsample_resolution: 1.0m -> 0.5m voxel grid
#   - submap_voxel_resolution: 0.5m -> 0.25m (denser final map)
#   - random_downsample_target: 10K -> 25K points
#
# Output:
#   <output_dir>/dump/         — GLIM internal state dump
#   <output_dir>/dump/map.pcd  — final accumulated map (PCD format)
#
# Notes:
#   - Requires GPU (RTX 2000 Ada / A2000)
#   - Uses same sensor config as live GLIM (T_lidar_imu, IMU noise, etc.)
#   - Stop live GLIM first: sudo systemctl stop bess-glim
#   - Bag must contain /ouster/points and /imu/data_guarded topics
#   - For bags recorded without /imu/data_guarded, remap is needed
##############################################################################
set -eo pipefail

BAG_DIR=""
OUTPUT_DIR=""
DENSE=false
DURATION=""
GLIM_IMAGE="localhost/bess-glim:humble"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        --dense)
            DENSE=true
            shift
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --help|-h)
            head -35 "$0" | tail -33
            exit 0
            ;;
        *)
            if [[ -z "$BAG_DIR" ]]; then
                BAG_DIR="$1"
            elif [[ -z "$OUTPUT_DIR" ]]; then
                OUTPUT_DIR="$1"
            fi
            shift
            ;;
    esac
done

if [[ -z "$BAG_DIR" ]]; then
    echo "Usage: glim-offline-replay.sh <bag_dir> [output_dir] [--dense] [--duration <sec>]"
    exit 1
fi

# Resolve absolute paths
BAG_DIR=$(realpath "$BAG_DIR")
if [[ -z "$OUTPUT_DIR" ]]; then
    OUTPUT_DIR="/var/mnt/nvme2/glim_offline/$(basename "$BAG_DIR")_$(date +%Y%m%d_%H%M%S)"
fi
OUTPUT_DIR=$(realpath -m "$OUTPUT_DIR")

# Validate
if [[ ! -d "$BAG_DIR" ]]; then
    echo "ERROR: Bag directory does not exist: $BAG_DIR"
    exit 1
fi

# Check for mcap files
MCAP_COUNT=$(ls "$BAG_DIR"/*.mcap 2>/dev/null | wc -l)
if [[ $MCAP_COUNT -eq 0 ]]; then
    echo "ERROR: No .mcap files found in $BAG_DIR"
    exit 1
fi

# Check metadata
if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
    echo "ERROR: No metadata.yaml in $BAG_DIR"
    echo "  Run: rebuild-bag-metadata.py $BAG_DIR"
    exit 1
fi

# Warn if live GLIM is running
if systemctl is-active --quiet bess-glim 2>/dev/null; then
    echo "WARNING: Live bess-glim service is running!"
    echo "  GPU contention will slow both. Stop it first:"
    echo "    sudo systemctl stop bess-glim"
    read -p "  Continue anyway? [y/N] " -n 1 -r
    echo
    [[ $REPLY =~ ^[Yy]$ ]] || exit 1
fi

# Select config
if $DENSE; then
    CONFIG_FILE="config_offline_dense.json"
    echo "Mode: DENSE (25% points, 0.5m preprocess voxel, 0.25m map voxel)"
else
    CONFIG_FILE="config_offline.json"
    echo "Mode: STANDARD (10% points, 1.0m preprocess voxel, 0.5m map voxel)"
fi

echo "Bag:      $BAG_DIR ($MCAP_COUNT mcap files)"
echo "Output:   $OUTPUT_DIR"
echo "Config:   $CONFIG_FILE"
[[ -n "$DURATION" ]] && echo "Duration: ${DURATION}s"
echo ""

# Create output directory
mkdir -p "$OUTPUT_DIR"

# Build container command
# glim_rosbag reads from the bag files directly (no ROS2 playback needed)
# It uses config_ros topics to know which bag topics to read
CONTAINER_CMD=(
    sudo podman run --rm
    --name bess-glim-offline
    --network=none
    # GPU
    --device nvidia.com/gpu=1
    --security-opt label=disable
    --shm-size=8gb
    --memory=24G
    --cpuset-cpus=6-17
    --ulimit "stack=4294967296:4294967296"
    # Environment
    -e "GLIM_CONFIG=/ros2_ws/config/$CONFIG_FILE"
    -e "GLIM_SENSOR_CONFIG=/ros2_ws/config/config_ouster.json"
    -e "CUDA_VISIBLE_DEVICES=0"
    -e "NVIDIA_VISIBLE_DEVICES=1"
    -e "NVIDIA_DRIVER_CAPABILITIES=compute,utility"
    -e "SPDLOG_LEVEL=info"
    # Mount configs
    -v "/opt/bess/config/glim:/ros2_ws/config:ro"
    -v "/opt/bess/config/glim/config_ros_offline.json:/ros2_ws/install/glim/share/glim/config/config_ros.json:ro"
    -v "/opt/bess/config/glim/config_sensors.json:/ros2_ws/install/glim/share/glim/config/config_sensors.json:ro"
    -v "/opt/bess/config/glim/config_odometry_gpu.json:/ros2_ws/install/glim/share/glim/config/config_odometry_gpu.json:ro"
    -v "/opt/bess/config/glim/config_sub_mapping_gpu.json:/ros2_ws/install/glim/share/glim/config/config_sub_mapping_gpu.json:ro"
    # Mount dense or standard configs depending on mode
    -v "/opt/bess/config/glim/${CONFIG_FILE}:/ros2_ws/install/glim/share/glim/config/config_offline.json:ro"
)

if $DENSE; then
    CONTAINER_CMD+=(
        -v "/opt/bess/config/glim/config_preprocess_dense.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro"
        -v "/opt/bess/config/glim/config_global_mapping_dense.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro"
    )
else
    CONTAINER_CMD+=(
        -v "/opt/bess/config/glim/config_preprocess.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro"
        -v "/opt/bess/config/glim/config_global_mapping_gpu.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro"
    )
fi

CONTAINER_CMD+=(
    # Mount bag and output
    -v "${BAG_DIR}:/bag:ro"
    -v "${OUTPUT_DIR}:/output:rw"
    # Image
    "$GLIM_IMAGE"
    # Command
    bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
        cd /output && \
        /ros2_ws/install/glim_ros/lib/glim_ros/glim_rosbag /bag \
        --ros-args -p config_file:=/ros2_ws/config/$CONFIG_FILE \
        -p sensor_config_file:=/ros2_ws/config/config_ouster.json \
        -p dump_path:=/output/dump \
        -p auto_quit:=true"
)

echo "Starting GLIM offline replay..."
echo "  This may take a while (bag is ${MCAP_COUNT} mcap files)"
echo "  Watch GPU: nvidia-smi -l 5"
echo ""

START_TIME=$(date +%s)
"${CONTAINER_CMD[@]}" 2>&1 | tee "$OUTPUT_DIR/glim_offline.log"
EXIT_CODE=${PIPESTATUS[0]}
END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

echo ""
echo "=== GLIM Offline Replay Complete ==="
echo "Duration: ${ELAPSED}s ($(( ELAPSED / 60 ))m $(( ELAPSED % 60 ))s)"
echo "Exit code: $EXIT_CODE"
echo "Output: $OUTPUT_DIR"

if [[ -d "$OUTPUT_DIR/dump" ]]; then
    echo ""
    echo "Output files:"
    ls -lh "$OUTPUT_DIR/dump/" 2>/dev/null
    # Convert PCD to LAZ if pdal is available
    # Merge submaps to LAZ
    MERGE_SCRIPT="/opt/bess/scripts/glim-merge-submaps.py"
    if [[ -x "$MERGE_SCRIPT" ]] || command -v python3 &>/dev/null; then
        echo ""
        echo "Merging submaps to LAZ..."
        python3 "$MERGE_SCRIPT" "$OUTPUT_DIR/dump" "$OUTPUT_DIR/glim_offline_map.laz" && \
            echo "  Ready for download: $OUTPUT_DIR/glim_offline_map.laz" || \
            echo "  Merge failed — submaps still available in $OUTPUT_DIR/dump/"
    fi
fi
