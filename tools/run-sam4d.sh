#!/bin/bash
# BESS SAM4D (GPU)
# Joint camera + LiDAR segmentation
# Usage: ./run-sam4d.sh [command]
# Example: ./run-sam4d.sh python3 -m sam4d.inference --config configs/default.yaml

DATA_DIR="${DATA_DIR:-/mnt/nvme2/bag-cache}"
MAPS_DIR="${MAPS_DIR:-/opt/bess/data/maps}"

exec podman run --rm -it \
    --device nvidia.com/gpu=0 \
    --shm-size=8gb \
    -v "$DATA_DIR":/data/bags:ro,z \
    -v "$MAPS_DIR":/data/maps:rw,z \
    -v "$HOME":/data/home:rw,z \
    --workdir /data \
    localhost/bess-sam4d:latest \
    "$@"
