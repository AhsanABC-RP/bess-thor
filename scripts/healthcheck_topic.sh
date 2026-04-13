#!/usr/bin/env bash
# Data-flow healthcheck for ROS 2 sensor containers.
#
# Default healthchecks like `pgrep -x python3` only verify a process exists,
# not that it is publishing. Several drivers (Lucid Arena, MIP SDK on the
# MicroStrain GQ7, Spinnaker on disconnected GigE cameras) catch hardware
# errors internally and stay alive forever in an idle spin loop, which makes
# the container look "healthy" while no data is flowing. This script
# subscribes to the actual topic for a short window and exits non-zero if
# nothing was received, so Docker's restart policy can recover the container.
#
# Usage (from inside a ROS 2 container):
#   healthcheck_topic.sh <topic> [min_rate_hz] [window_sec]
#
# Defaults: min_rate_hz=0.5, window_sec=4
#
# Behavior:
#   - Sources whichever ROS distro is installed at /opt/ros/<distro>
#   - Runs `ros2 topic hz <topic> --window 3` for window_sec seconds
#   - Parses "average rate: N" and exits 0 if N >= min_rate_hz
#   - Exits 1 on any failure (no data, parse error, ros2 not found)
#
# NOTE: ROS 2's setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES,
# COLCON_TRACE etc.) so we MUST source it BEFORE enabling set -u, otherwise
# the source aborts mid-way and ros2 topic hz silently fails.
set -eo pipefail

TOPIC=${1:?topic name required}
MIN_RATE=${2:-0.5}
WINDOW=${3:-4}

# Find a ROS 2 distro (humble or jazzy). Containers in this stack mix both.
DISTRO=""
for d in jazzy humble; do
  if [[ -f /opt/ros/$d/setup.bash ]]; then
    DISTRO=$d
    break
  fi
done
if [[ -z $DISTRO ]]; then
  echo "healthcheck: no /opt/ros/{jazzy,humble} found" >&2
  exit 1
fi
# shellcheck disable=SC1090
source /opt/ros/$DISTRO/setup.bash
# Now safe to enable nounset — ROS env vars are populated.
set -u

# Run hz with the configured window. Capture both streams.
out=$(timeout "$((WINDOW + 3))" ros2 topic hz "$TOPIC" --window 3 2>&1 < /dev/null & \
      pid=$!; sleep "$WINDOW"; kill -INT $pid 2>/dev/null; wait $pid 2>/dev/null) || true

# "average rate: 9.987" — take the last one in case of multiple lines.
rate=$(echo "$out" | awk '/average rate:/ { r=$3 } END { print (r==""?0:r) }')

awk -v r="$rate" -v m="$MIN_RATE" 'BEGIN { exit !(r+0 >= m+0) }'
ec=$?
if (( ec != 0 )); then
  echo "healthcheck: $TOPIC rate=$rate Hz < min=$MIN_RATE Hz" >&2
fi
exit $ec
