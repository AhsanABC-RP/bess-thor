#!/bin/bash
# Launch dvlc interactive GUI tools for camera-LiDAR calibration.
#
# REQUIRES: Physical display access (JetKVM or headed mode).
# The GUI window appears on the KDE Plasma desktop (DISPLAY=:0).
#
# Usage:
#   sudo ./dvlc-manual-gui.sh <cam_dir> [tool]
#
# Examples:
#   sudo ./dvlc-manual-gui.sh /var/mnt/nvme2/slam-output/dvlc_cam2 initial_guess_manual
#   sudo ./dvlc-manual-gui.sh /var/mnt/nvme2/slam-output/dvlc_cam3 viewer
#   sudo ./dvlc-manual-gui.sh /var/mnt/nvme2/slam-output/dvlc_cam2 calibrate
#
# Tools:
#   initial_guess_manual  - Pick point correspondences interactively (START HERE)
#   viewer                - View current calibration overlay
#   calibrate             - NID refinement (runs with display for live preview)

set -e

DATA_DIR="${1:?Usage: $0 <data_dir> [tool]}"
TOOL="${2:-initial_guess_manual}"

# Find Xauthority for the Plasma session
XAUTH=$(ls /run/user/1000/xauth_* 2>/dev/null | head -1)
if [ -z "$XAUTH" ]; then
    echo "ERROR: No Xauthority file found. Is a desktop session running?"
    echo "Check: ls /run/user/1000/xauth_*"
    exit 1
fi

# Resolve data dir to absolute path
DATA_DIR=$(realpath "$DATA_DIR")
PARENT_DIR=$(dirname "$DATA_DIR")
DIR_NAME=$(basename "$DATA_DIR")

echo "============================================"
echo "dvlc Manual GUI — $TOOL"
echo "============================================"
echo "Data:    $DATA_DIR"
echo "Display: :0 (via Xauth: $XAUTH)"
echo ""
echo "The GUI window will appear on the physical display."
echo "Use JetKVM (192.168.1.212) or headed mode to interact."
echo ""

if [ "$TOOL" = "initial_guess_manual" ]; then
    echo "INSTRUCTIONS:"
    echo "  1. Click on matching points in LiDAR intensity image and camera image"
    echo "  2. Pick 4-6 well-distributed correspondences (building corners, signs, etc.)"
    echo "  3. Press Enter/OK when done to compute initial T_lidar_camera"
    echo "  4. Result saved to: $DATA_DIR/calib.json (init_T_lidar_camera)"
    echo ""
fi

if [ "$TOOL" = "viewer" ]; then
    echo "INSTRUCTIONS:"
    echo "  1. Verify LiDAR-camera overlay alignment"
    echo "  2. Check building edges, road markings, signs match"
    echo "  3. Close when done"
    echo ""
fi

sudo podman run --rm -it \
    --device nvidia.com/gpu=all \
    --security-opt label=disable \
    -e DISPLAY=:0 \
    -e XAUTHORITY=/tmp/xauth \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$XAUTH":/tmp/xauth:ro \
    -v "$PARENT_DIR":/data \
    localhost/bess-calibration:humble \
    bash -c "source /root/ros2_ws/install/setup.bash && \
             echo 'Starting $TOOL...' && \
             ros2 run direct_visual_lidar_calibration $TOOL \
               --data_path /data/$DIR_NAME"
