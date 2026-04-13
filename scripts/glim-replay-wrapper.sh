#!/bin/bash
# GLIM replay wrapper — same as drain_and_launch.sh but forces CycloneDDS.
# Required because setup.bash resets RMW_IMPLEMENTATION to fastrtps.
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# MUST set AFTER setup.bash (it clears env vars)
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=99
export CYCLONEDDS_URI=file:///etc/cyclonedds.xml

echo "[glim-replay] RMW=$RMW_IMPLEMENTATION DOMAIN=$ROS_DOMAIN_ID"
echo "[glim-replay] Starting points_filter relay..."

python3 /ros2_ws/config/points_filter.py &
FILTER_PID=$!
sleep 2

if ! kill -0 $FILTER_PID 2>/dev/null; then
    echo "[glim-replay] WARNING: points_filter died"
fi

trap "kill $FILTER_PID 2>/dev/null; wait $FILTER_PID 2>/dev/null" EXIT

echo "[glim-replay] Starting GLIM..."
exec "$@"
