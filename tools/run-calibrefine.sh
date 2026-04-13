#!/bin/bash
# BESS CalibRefine (GPU)
# Targetless camera-LiDAR calibration refinement
# Usage: ./run-calibrefine.sh [command]
# Example: ./run-calibrefine.sh python3 Test.py

DATA_DIR="${DATA_DIR:-/mnt/nvme2/bag-cache}"

exec podman run --rm -it \
    --device nvidia.com/gpu=0 \
    --shm-size=8gb \
    -v "$DATA_DIR":/data/bags:ro,z \
    -v "$HOME":/data/home:rw,z \
    --workdir /opt/calibrefine \
    localhost/bess-calibrefine:latest \
    "$@"
