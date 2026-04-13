#!/bin/bash
# BESS Camera-LiDAR Calibration (koide3 direct_visual_lidar_calibration)
# Usage: ./run-calibration.sh [command]
# Steps:
#   1. Collect bags: ros2 bag record /ouster/points /camera1/camera_driver/image_raw
#   2. Preprocess: ./run-calibration.sh ros2 run direct_visual_lidar_calibration preprocess
#   3. Initial guess: ./run-calibration.sh ros2 run direct_visual_lidar_calibration initial_guess_manual
#   4. Calibrate: ./run-calibration.sh ros2 run direct_visual_lidar_calibration calibrate

DATA_DIR="${DATA_DIR:-/mnt/nvme2/bag-cache}"

exec podman run --rm -it \
    --network host \
    -e ROS_DOMAIN_ID=0 \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$DATA_DIR":/data/bags:rw,z \
    -v "$HOME":/data/home:rw,z \
    --workdir /data \
    docker.io/koide3/direct_visual_lidar_calibration:humble \
    "$@"
