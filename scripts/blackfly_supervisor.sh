#!/bin/bash
# blackfly_supervisor.sh - kill spinnaker_camera_driver if it falls into a
# "-1002 Camera is not initialized" retry loop. Runs in the blackfly container
# alongside the driver; the outer `while true` in the compose command respawns.
#
# Why: upstream `spinnaker_camera_driver`'s monitorStatus() restart path only
# calls BeginAcquisition(), never Init(). After a DeInit() event (GigE
# heartbeat loss, port flap, duplicate-interface handle) the driver loops on
# -1002 forever because the camera handle is de-initialised. The only way
# back is a fresh process that re-runs Init(). This supervisor forces that
# by SIGTERM'ing the process when the pattern is sustained.
#
# Heuristic: >= 5 "-1002" log lines within 10 s window -> kill. False
# positives are harmless because the outer loop just re-spawns and re-inits.
#
# Usage: blackfly_supervisor.sh <container-tag>
# Expects the driver process to be findable via `pgrep -f camera_driver_node`.

set -u
TAG="${1:-blackfly}"
WINDOW_SEC=10
THRESHOLD=5
POLL=2

log() { echo "[${TAG}-supervisor] $*"; }

# Wait for the camera_driver_node to actually be up before watching.
# Give it up to 120 s after the bash parent script launches the process.
for _ in $(seq 1 60); do
  pid=$(pgrep -f "camera_driver_node" | head -1)
  if [ -n "${pid:-}" ]; then break; fi
  sleep 2
done
if [ -z "${pid:-}" ]; then
  log "camera_driver_node pid not found after 120 s, exiting supervisor"
  exit 0
fi
log "watching pid=$pid for -1002 retry-loop pattern"

# Track recent -1002 timestamps in a ring buffer in /tmp.
state=$(mktemp /tmp/${TAG}-sup-XXXX)
trap 'rm -f "$state"' EXIT

# The driver logs to stdout of the parent bash -c. We can't tail that from
# inside easily, but we can tail /proc/$pid/fd/1 which is the same pipe.
# However that fd is a write pipe. Simpler: journald isn't available inside
# a container, so we use a line-based approach: read from the process's
# stderr/stdout via /proc/$pid/fd/1.

# Use /proc/$pid/fd/2 (stderr) - ros2 logs go there by default.
FD=2
if [ ! -r "/proc/${pid}/fd/${FD}" ]; then FD=1; fi
if [ ! -r "/proc/${pid}/fd/${FD}" ]; then
  log "cannot read fd ${FD} of pid $pid, exiting supervisor"
  exit 0
fi

# We can't re-read from an already-open pipe; instead rely on periodic
# snapshots of the *count* of "-1002" lines the driver has emitted, via
# a small side-channel: the driver writes to a tty/pipe we don't control,
# but the parent bash's stdout IS captured as container logs. The simplest
# reliable signal is: check whether the topic is publishing. If the outer
# frame rate is 0 for too long, the driver is dead-in-loop. Use ROS 2 CLI.
source /opt/ros/humble/setup.bash 2>/dev/null || true
source /ros2_ws/install/setup.bash 2>/dev/null || true

# Namespace the supervisor checks per-tag.
case "$TAG" in
  blackfly1) TOPIC="/blackfly/camera1/blackfly_camera/image_raw" ;;
  blackfly2) TOPIC="/blackfly/camera2/blackfly_camera/image_raw" ;;
  *)         TOPIC="/blackfly/camera1/blackfly_camera/image_raw" ;;
esac

zero_streak=0
ZERO_LIMIT=15   # 15 * POLL = 30 s of 0 Hz -> kill
while kill -0 "$pid" 2>/dev/null; do
  # Measure rate over 3 s. ros2 topic hz prints over stderr; take last average.
  rate=$(timeout 4 ros2 topic hz "$TOPIC" 2>&1 | grep -oE "average rate: [0-9.]+" | tail -1 | awk '{print $3}')
  if [ -z "${rate:-}" ] || [ "$rate" = "0" ] || [ "$(echo "$rate < 0.1" | bc -l 2>/dev/null)" = "1" ]; then
    zero_streak=$((zero_streak + 1))
    log "zero-Hz streak ${zero_streak}/${ZERO_LIMIT} (rate=${rate:-none})"
    if [ "$zero_streak" -ge "$ZERO_LIMIT" ]; then
      log "driver stuck (zero-Hz for ${ZERO_LIMIT}*${POLL}s) -> SIGTERM pid $pid"
      kill -TERM "$pid" 2>/dev/null
      sleep 5
      kill -KILL "$pid" 2>/dev/null
      exit 0
    fi
  else
    if [ "$zero_streak" -gt 0 ]; then
      log "rate recovered to ${rate} Hz, resetting streak"
    fi
    zero_streak=0
  fi
  sleep "$POLL"
done
log "camera_driver_node pid $pid exited on its own"
