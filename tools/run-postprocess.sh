#!/bin/bash
# BESS Post-Processing Tools
# Usage: ./run-postprocess.sh [command]
# Examples:
#   ./run-postprocess.sh evo_traj kitti trajectory.txt --plot
#   ./run-postprocess.sh evo_ape kitti ref.txt est.txt --plot
#   ./run-postprocess.sh kiss_icp_pipeline --help
#   ./run-postprocess.sh python3 my_script.py
#   ./run-postprocess.sh  (interactive shell)

DATA_DIR="${DATA_DIR:-/mnt/nvme2/bag-cache}"
MAPS_DIR="${MAPS_DIR:-/opt/bess/data/maps}"

exec podman run --rm -it \
    -v "$DATA_DIR":/data/bags:ro,z \
    -v "$MAPS_DIR":/data/maps:rw,z \
    -v "$HOME":/data/home:rw,z \
    -v /opt/bess/config:/config:ro,z \
    --workdir /data \
    localhost/bess-postprocess:latest \
    "$@"
