#!/bin/bash
# Bind-mounted over /entrypoint.sh in lucid1/lucid2 services to override the
# baked entrypoint inside localhost/bess-lucid:jazzy. The image's original
# entrypoint contained:
#
#     if [ -w /proc/sys/net/core/rmem_max ]; then
#         echo 134217728 > /proc/sys/net/core/rmem_max  || true
#         echo 134217728 > /proc/sys/net/core/rmem_default || true
#     fi
#
# With privileged: true in x-common, /proc/sys IS writable, so every lucid
# restart silently clobbered the host kernel sysctls — pulling rmem_max
# DOWN from 2147483647 (set by 60-gige-sensors.conf) to 134217728, and
# rmem_default UP from 67108864 to 134217728. Hosts buffer settings must
# come from /etc/sysctl.d, never from a container at startup.
#
# Source-of-truth for the bess-lucid image entrypoint lives at
# /home/thor/bess_platform/containers/lucid/entrypoint.sh. When that image
# is next rebuilt, drop the sysctl writes there and remove this bind mount.
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash

export LD_LIBRARY_PATH="${ARENA_ROOT}/lib:${ARENA_ROOT}/GenICam/library/lib/Linux64_ARM:${ARENA_ROOT}/ffmpeg:${LD_LIBRARY_PATH}"
export GENICAM_GENTL64_PATH="${ARENA_ROOT}/lib:${ARENA_ROOT}/GenICam/library/lib/Linux64_ARM"

echo "Arena SDK: ${ARENA_ROOT}"
echo "ROS Distro: ${ROS_DISTRO}"
echo "Starting Lucid camera driver..."

exec "$@"
