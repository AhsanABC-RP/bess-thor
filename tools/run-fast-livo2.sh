#!/bin/bash
# BESS FAST-LIVO2 (ROS2 Humble)
# LiDAR-Inertial-Visual Odometry
# Usage: ./run-fast-livo2.sh [command]
# Example:
#   ./run-fast-livo2.sh ros2 launch fast_livo2 mapping.launch.py

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
    localhost/bess-fast-livo2:latest \
    "$@"
