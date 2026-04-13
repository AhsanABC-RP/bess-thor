#!/bin/bash
# Extract observations from a bag file using the extraction pipeline.
# Runs ros2 bag play + extraction node in a dedicated container.
#
# Usage: extract-from-bag.sh /path/to/bag_dir [output_dir]

set -euo pipefail

BAG_DIR="${1:?Usage: $0 /path/to/bag_dir [output_dir]}"
BAG_NAME="$(basename "$BAG_DIR")"
OUTPUT_DIR="${2:-/var/mnt/nvme2/extraction/bag_extractions/$BAG_NAME}"
CONTAINER_NAME="bess-extract-bag-$(echo "$BAG_NAME" | tail -c 20)"

echo "=== Bag Extraction ==="
echo "Bag:    $BAG_DIR"
echo "Output: $OUTPUT_DIR"
echo "Container: $CONTAINER_NAME"

# Verify bag exists
if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: Bag directory not found: $BAG_DIR"
    exit 1
fi

# Create output dir
mkdir -p "$OUTPUT_DIR"

# Stop live extraction to free resources and avoid topic conflicts
echo "Stopping live extraction..."
sudo systemctl stop bess-extraction 2>/dev/null || true
sleep 2

# Clean up any previous container
sudo podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Ensure MCAP storage plugin is available (extraction image lacks it)
# Need libs + share dirs for pluginlib class_loader discovery
MCAP_LIB_DIR="/tmp/mcap_plugin"
if [ ! -f "$MCAP_LIB_DIR/lib/librosbag2_storage_mcap.so" ] || [ ! -d "$MCAP_LIB_DIR/share/rosbag2_storage_mcap" ]; then
    echo "Extracting MCAP plugin from recorder image..."
    mkdir -p "$MCAP_LIB_DIR"/{lib,share}
    sudo podman run --rm -v "$MCAP_LIB_DIR":/out:rw,Z localhost/bess-recorder:humble bash -c "
        cp /opt/ros/humble/lib/librosbag2_storage_mcap.so /opt/ros/humble/lib/libmcap.so /out/lib/
        cp -r /opt/ros/humble/share/rosbag2_storage_mcap /out/share/
        cp -r /opt/ros/humble/share/mcap_vendor /out/share/
        mkdir -p /out/share/ament_index/resource_index/{rosbag2_storage__pluginlib__plugin,packages}
        cp /opt/ros/humble/share/ament_index/resource_index/rosbag2_storage__pluginlib__plugin/rosbag2_storage_mcap /out/share/ament_index/resource_index/rosbag2_storage__pluginlib__plugin/
        cp /opt/ros/humble/share/ament_index/resource_index/packages/rosbag2_storage_mcap /out/share/ament_index/resource_index/packages/
        cp /opt/ros/humble/share/ament_index/resource_index/packages/mcap_vendor /out/share/ament_index/resource_index/packages/
    "
fi

echo "Starting bag extraction..."
sudo podman run \
    --name="$CONTAINER_NAME" \
    --memory=12G \
    --cpuset-cpus=6-23 \
    --ipc=host \
    --network=none \
    --security-opt label=disable \
    -e ROS_DOMAIN_ID=99 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e SPATIAL_DB=/data/spatial/bess_uprn.db \
    -e EXTRACTION_DIR=/data/extraction \
    -e AMBIENT_TEMP_C=15.0 \
    -v "$BAG_DIR":/data/bag:ro \
    -v "$OUTPUT_DIR":/data/extraction:rw \
    -v /opt/bess/data/spatial:/data/spatial:ro \
    -v /opt/bess/scripts/bess-extraction.py:/ros2_ws/bess-extraction.py:ro \
    -v /opt/bess/config/extraction:/config:ro \
    -v "$MCAP_LIB_DIR":/mcap_plugin:ro \
    -e AMENT_PREFIX_PATH=/mcap_plugin:/opt/ros/humble \
    -e LD_LIBRARY_PATH=/mcap_plugin/lib:/opt/ros/humble/lib \
    localhost/bess-extraction:humble \
    bash -c '
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash 2>/dev/null || true
        # Prepend MCAP plugin to paths (setup.bash overwrites env vars)
        export AMENT_PREFIX_PATH=/mcap_plugin:${AMENT_PREFIX_PATH}
        export LD_LIBRARY_PATH=/mcap_plugin/lib:${LD_LIBRARY_PATH}

        echo "Starting extraction node in background..."
        python3 /ros2_ws/bess-extraction.py &
        EXTRACT_PID=$!
        sleep 5

        echo "Playing bag..."
        ros2 bag play /data/bag --rate 10.0 --clock 2>&1 | tail -5

        echo "Bag play complete. Waiting for extraction to flush..."
        sleep 10

        # Send SIGINT to trigger clean shutdown + final flush
        kill -INT $EXTRACT_PID 2>/dev/null
        wait $EXTRACT_PID 2>/dev/null || true

        echo "=== Extraction complete ==="
        ls -la /data/extraction/ 2>/dev/null
        find /data/extraction -name "*.jpg" -o -name "*.laz" -o -name "*.json" 2>/dev/null | wc -l
    ' 2>&1

EXIT_CODE=$?

# Clean up container
sudo podman rm -f "$CONTAINER_NAME" 2>/dev/null || true

# Restart live extraction
echo "Restarting live extraction..."
sudo systemctl start bess-extraction

echo ""
echo "=== Results ==="
echo "Output: $OUTPUT_DIR"
ls -la "$OUTPUT_DIR" 2>/dev/null | head -20
TOTAL_FILES=$(find "$OUTPUT_DIR" -type f 2>/dev/null | wc -l)
echo "Total files: $TOTAL_FILES"

exit $EXIT_CODE
