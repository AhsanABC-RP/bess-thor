#!/bin/bash
# GLIM launch wrapper — runs points_filter relay then starts GLIM
# Prevents IndexedSlidingWindow crash on mid-drive restarts by dropping
# stale LiDAR frames before they reach GLIM.
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "[drain_and_launch] Starting points_filter relay..."

# Start the timestamp filter as a background process.
# It subscribes to /ouster/points, drops stale frames (>2s old),
# and republishes to /ouster/points_filtered.
python3 /ros2_ws/config/points_filter.py &
FILTER_PID=$!

# Give the filter node time to create its subscription and publisher
# so that by the time GLIM starts, the filtered topic is available.
sleep 2

# Verify the filter is still running
if ! kill -0 $FILTER_PID 2>/dev/null; then
    echo "[drain_and_launch] ERROR: points_filter died, launching GLIM without filter"
    # Fall through — GLIM will subscribe to whatever points_topic is in config_ros.json
fi

# Clean up filter when this script exits (GLIM container shutdown)
trap "kill $FILTER_PID 2>/dev/null; wait $FILTER_PID 2>/dev/null" EXIT

echo "[drain_and_launch] Starting GLIM..."
exec "$@"
