#!/bin/bash
# BESS Full Recording Script
# Records all sensor topics to MCAP format with compression

set -e

source /opt/ros/humble/setup.bash

# Configuration
BAG_DIR="${BAG_DIR:-/data/bags}"
COMPRESSION="${COMPRESSION:-zstd}"
MAX_BAG_SIZE="${MAX_BAG_SIZE:-0}"  # 0 = unlimited
MAX_BAG_DURATION="${MAX_BAG_DURATION:-0}"  # 0 = unlimited

# Generate timestamp for bag name
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="bess_${TIMESTAMP}"

echo "Starting BESS rosbag2 recording..."
echo "Output: ${BAG_DIR}/${BAG_NAME}"

# Topics to record
TOPICS=(
    # LiDAR
    "/ouster1/points"
    "/ouster1/imu"
    "/ouster2/points"
    "/ouster2/imu"
    "/merged_points"

    # Cameras
    "/camera1/image_raw"
    "/camera1/camera_info"
    "/camera2/image_raw"
    "/camera2/camera_info"

    # IMU
    "/imu/data"
    "/imu/filter_status"

    # SLAM output
    "/slam/odometry"
    "/slam/cloud_registered"
    "/slam/path"

    # Odometry/EKF
    "/odometry/filtered"
    "/odom"

    # TF
    "/tf"
    "/tf_static"

    # Diagnostics
    "/diagnostics"
)

# Build topic string
TOPIC_ARGS=""
for topic in "${TOPICS[@]}"; do
    TOPIC_ARGS="${TOPIC_ARGS} ${topic}"
done

# Record
ros2 bag record \
    --storage mcap \
    --compression-mode file \
    --compression-format ${COMPRESSION} \
    --output ${BAG_DIR}/${BAG_NAME} \
    --max-bag-size ${MAX_BAG_SIZE} \
    --max-bag-duration ${MAX_BAG_DURATION} \
    ${TOPIC_ARGS}
