#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
[[ -f /workspace/install/setup.bash ]] && source /workspace/install/setup.bash
exec "$@"
