#!/bin/bash
# topic_watchdog.sh — host-side topic-rate watchdog.
#
# Every POLL_INTERVAL seconds, sample the rate of every topic in the
# WATCHLIST. If a topic stays below MIN_RATE for TRIGGER_SECONDS, take
# the topic's registered recovery ACTION (targeted docker restart).
#
# Per-container rate limit: MAX_ACTIONS_PER_HOUR actions before giving
# up (log loudly + stop acting until the hour window rolls over). This
# prevents a wedged camera or crash-looping SLAM from thrashing Docker.
#
# JSON event log at LOG_PATH for forensics + soak correlation.
#
# Designed to run as systemd unit bess-topic-watchdog.service.
#
# Usage: topic_watchdog.sh [--once]
#   --once  run one pass then exit (for debugging)

set -u

LOG_PATH="${LOG_PATH:-/var/log/bess/topic_watchdog.log}"
POLL_INTERVAL="${POLL_INTERVAL:-30}"
HZ_SAMPLE_WINDOW="${HZ_SAMPLE_WINDOW:-5}"
TRIGGER_SECONDS="${TRIGGER_SECONDS:-60}"
MAX_ACTIONS_PER_HOUR="${MAX_ACTIONS_PER_HOUR:-3}"

# Topic → min_rate:container:action mapping. Action is one of:
#   restart           plain docker restart
#   recreate          docker compose up -d --force-recreate --no-deps
#   stop60_up         docker stop, sleep 60, docker compose up -d --no-deps
#   log_only          log + page, but take NO restart action
#
# Thermal cameras:
#   A6701 Pleora cameras (thermal1, thermal2) use log_only. Rapid
#   container flap pushes the Pleora iPORT into a PHY-dark state that
#   requires a 90s power-off + 2min untouched boot — software cannot
#   recover it. See memory feedback_a6701_pleora_flap_state.md. The
#   watchdog must NOT bounce these cameras; it only pages a human.
#   A70 cameras (thermal3, thermal4) use stop60_up: GigE Vision
#   heartbeat needs a full 60s to clear in the A70 firmware before a
#   new control session can open (-1010 stuck session pattern).
# Blackfly recreate is safe: compose has a 30s GigE clear sleep
#   baked into the outer while-true loop inside the container.
# SLAM plain restart: container supervisor terminates the launch on
#   odom-node death cleanly; restart:on-failure:10 already handles it,
#   but explicit `restart` from the watchdog ensures a wedged-process
#   case (not a crash) is also caught.
#
# Format: one line per topic, whitespace-separated.
#   <topic>  <min_rate_hz>  <container>  <action>
WATCHLIST=$(cat <<'EOF'
/ouster/points                                                3.0   ouster                restart
/ouster/imu                                                   50.0  ouster                restart
/imu/data                                                     50.0  microstrain           restart
/thermal/camera1/image_raw                                    3.0   thermal1              log_only
/thermal/camera2/image_raw                                    3.0   thermal2              log_only
/thermal/camera3/image_raw                                    3.0   thermal3              stop60_up
/thermal/camera4/image_raw                                    3.0   thermal4              stop60_up
/blackfly/camera1/blackfly_camera/image_raw/compressed        2.0   blackfly1             recreate
/blackfly/camera2/blackfly_camera/image_raw/compressed        2.0   blackfly2             recreate
/lucid1/camera_driver/image_raw/compressed                    1.0   lucid1                recreate
/lucid2/camera_driver/image_raw/compressed                    1.0   lucid2                recreate
/fast_lio/odometry                                            2.0   fast-lio              restart
/dlio/odom_node/odom                                          10.0  dlio                  restart
/fast_lio/map_voxel                                           0.2   slam-map-accumulator  restart
EOF
)

STATE_DIR="${STATE_DIR:-/tmp/bess-topic-watchdog}"
mkdir -p "$STATE_DIR"
mkdir -p "$(dirname "$LOG_PATH")"

# Pick a running container we can exec ros2 into. fast-lio is jazzy +
# rmw_cyclonedds and usually healthy — falls back to dlio, then
# slam-map-accumulator. If none are up, the watchdog can't sample and
# just logs an error per tick.
pick_ros_host() {
  for c in fast-lio slam-map-accumulator dlio ouster; do
    if docker exec "$c" true >/dev/null 2>&1; then
      echo "$c"
      return 0
    fi
  done
  return 1
}

# Sample rate for a topic. Returns empty string if silent.
sample_rate() {
  local host="$1" topic="$2"
  docker exec "$host" bash -c "
    source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash 2>/dev/null || true
    timeout ${HZ_SAMPLE_WINDOW} ros2 topic hz '$topic' 2>/dev/null | grep 'average rate' | tail -1 | awk '{print \$3}'
  " 2>/dev/null
}

# Append a JSON event to LOG_PATH.
log_event() {
  local topic="$1" state="$2" rate="$3" container="$4" action="$5" detail="${6:-}"
  local ts
  ts=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  printf '{"ts":"%s","topic":"%s","state":"%s","rate":"%s","container":"%s","action":"%s","detail":"%s"}\n' \
    "$ts" "$topic" "$state" "$rate" "$container" "$action" "$detail" >> "$LOG_PATH"
}

# Per-container action history. Rolling 60-minute window in STATE_DIR.
action_ok() {
  local container="$1"
  local hist="$STATE_DIR/actions_${container}.log"
  touch "$hist"
  # Drop entries older than 3600s
  local cutoff=$(( $(date +%s) - 3600 ))
  awk -v c="$cutoff" '$1>=c' "$hist" > "${hist}.tmp" && mv "${hist}.tmp" "$hist"
  local n
  n=$(wc -l < "$hist")
  if [ "$n" -ge "$MAX_ACTIONS_PER_HOUR" ]; then
    return 1
  fi
  echo "$(date +%s)" >> "$hist"
  return 0
}

# Execute the action for a container.
do_action() {
  local container="$1" action="$2"
  case "$action" in
    restart)
      docker restart "$container" >/dev/null 2>&1
      ;;
    recreate)
      docker compose -f /home/thor/bess/docker-compose.thor.yml up -d \
        --force-recreate --no-deps "$container" >/dev/null 2>&1
      ;;
    stop60_up)
      docker stop "$container" >/dev/null 2>&1
      sleep 60
      docker compose -f /home/thor/bess/docker-compose.thor.yml up -d \
        --no-deps "$container" >/dev/null 2>&1
      ;;
    log_only)
      # Intentionally takes no action. Used for A6701 Pleora cameras
      # where software bounce triggers the iPORT PHY-dark flap.
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

# Per-topic silence-since state file.
since_file() { echo "$STATE_DIR/since_$(echo "$1" | tr '/' '_').ts"; }

check_topic() {
  local host="$1" topic="$2" min_rate="$3" container="$4" action="$5"
  local since
  since=$(since_file "$topic")
  local rate
  rate=$(sample_rate "$host" "$topic")
  local now
  now=$(date +%s)

  local below
  below=$(awk -v r="${rate:-0}" -v m="$min_rate" 'BEGIN{print (r+0 < m+0) ? 1 : 0}')

  if [ "$below" = "1" ]; then
    if [ ! -f "$since" ]; then
      echo "$now" > "$since"
      log_event "$topic" "below" "${rate:-silent}" "$container" "$action" "min=$min_rate"
    else
      local started
      started=$(cat "$since" 2>/dev/null || echo "$now")
      local dur=$(( now - started ))
      if [ "$dur" -ge "$TRIGGER_SECONDS" ]; then
        if action_ok "$container"; then
          log_event "$topic" "act" "${rate:-silent}" "$container" "$action" "dur=${dur}s"
          do_action "$container" "$action"
          rm -f "$since"
        else
          log_event "$topic" "rate_limited" "${rate:-silent}" "$container" "$action" "max=$MAX_ACTIONS_PER_HOUR/h"
        fi
      fi
    fi
  else
    if [ -f "$since" ]; then
      log_event "$topic" "recovered" "$rate" "$container" "" ""
      rm -f "$since"
    fi
  fi
}

run_once() {
  local host
  host=$(pick_ros_host) || {
    log_event "" "no_ros_host" "" "" "" "all candidate containers down"
    return
  }
  while IFS= read -r line; do
    line=$(echo "$line" | xargs)
    [ -z "$line" ] && continue
    read -r topic min_rate container action <<<"$line"
    check_topic "$host" "$topic" "$min_rate" "$container" "$action"
  done <<<"$WATCHLIST"
}

if [ "${1:-}" = "--once" ]; then
  run_once
  exit 0
fi

log_event "" "startup" "" "" "" "poll=${POLL_INTERVAL}s trigger=${TRIGGER_SECONDS}s max/h=${MAX_ACTIONS_PER_HOUR}"
while true; do
  run_once
  sleep "$POLL_INTERVAL"
done
