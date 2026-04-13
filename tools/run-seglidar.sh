#!/bin/bash
# BESS Segment-LiDAR (GPU)
# Usage: ./run-seglidar.sh [command]
# Example: ./run-seglidar.sh segment-lidar --input cloud.las --output segmented.las

DATA_DIR="${DATA_DIR:-/mnt/nvme2/bag-cache}"
MAPS_DIR="${MAPS_DIR:-/opt/bess/data/maps}"

exec podman run --rm -it \
    --device nvidia.com/gpu=0 \
    --shm-size=8gb \
    -v "$DATA_DIR":/data/bags:ro,z \
    -v "$MAPS_DIR":/data/maps:rw,z \
    -v "$HOME":/data/home:rw,z \
    --workdir /data \
    localhost/bess-seglidar:latest \
    "$@"
