#!/bin/bash
# BESS ROS2 Stack - Manual Start Script
# Use this if docker-compose is not installed

DOCKER_CMD="docker"
[ "$(groups)" != *docker* ] && DOCKER_CMD="sg docker -c docker"

echo "=== Starting BESS ROS2 Stack ==="

# Stop any existing containers
echo "Stopping existing containers..."
for name in bess-ouster bess-cameras bess-microstrain bess-recorder; do
    docker stop $name 2>/dev/null
    docker rm $name 2>/dev/null
done

# Common options
COMMON_OPTS="--network host --privileged --runtime nvidia \
  -e ROS_DOMAIN_ID=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all"

# Start Ouster LiDAR
echo ""
echo "Starting Ouster LiDAR container..."
docker run -d --name bess-ouster $COMMON_OPTS \
  -v $HOME/bess/config:/ros2_ws/config:ro \
  localhost/bess-ouster:jazzy

# Start Cameras (FLIR + Sony)
echo "Starting Cameras container..."
docker run -d --name bess-cameras $COMMON_OPTS \
  -v $HOME/bess/config:/ros2_ws/config:ro \
  -v /dev:/dev \
  localhost/bess-cameras:jazzy \
  bash -c 'source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 run spinnaker_camera_driver camera_driver_node'

# Start MicroStrain IMU
echo "Starting MicroStrain IMU container..."
docker run -d --name bess-microstrain $COMMON_OPTS \
  -v $HOME/bess/config:/ros2_ws/config:ro \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  localhost/bess-microstrain:jazzy 2>/dev/null || echo "  (No IMU at /dev/ttyACM0)"

# Start Recorder
echo "Starting Recorder container..."
mkdir -p $HOME/bess/data/bags
docker run -d --name bess-recorder $COMMON_OPTS \
  -v $HOME/bess/data/bags:/data/bags \
  -v $HOME/bess/config:/ros2_ws/config:ro \
  localhost/bess-recorder:jazzy

echo ""
echo "=== Stack Started ==="
echo ""
echo "Check status:  docker ps | grep bess"
echo "View logs:     docker logs -f bess-cameras"
echo "Stop all:      docker stop bess-ouster bess-cameras bess-microstrain bess-recorder"
