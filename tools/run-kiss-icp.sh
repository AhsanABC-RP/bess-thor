#!/bin/bash
# BESS KISS-ICP / KISS-SLAM (ROS2 Humble)
# Usage: ./run-kiss-icp.sh [command]
# Examples:
#   ./run-kiss-icp.sh ros2 launch kiss_icp odometry.launch.py topic:=/ouster/points
#   ./run-kiss-icp.sh kiss_icp_pipeline --help
#   ./run-kiss-icp.sh  (interactive shell)

DATA_DIR="${DATA_DIR:-/mnt/nvme2/bag-cache}"
MAPS_DIR="${MAPS_DIR:-/opt/bess/data/maps}"

exec podman run --rm -it \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/config/fastdds.xml \
    -v "$DATA_DIR":/data/bags:ro,z \
    -v "$MAPS_DIR":/data/maps:rw,z \
    -v /opt/bess/config:/config:ro,z \
    -v "$HOME":/data/home:rw,z \
    --workdir /data \
    localhost/bess-kiss-icp:latest \
    "$@"
