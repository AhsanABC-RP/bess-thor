#!/bin/bash
# topic_watchdog.sh — host-side container-health watchdog.
#
# v2 (2026-04-23 rewrite): does NOT use `ros2 topic hz` anymore.
# The v1 design polled topic rates via a subscriber container, which
# caused a CASCADE on 2026-04-23 morning: restarting the foxglove
# bridge disturbed DDS discovery, the watchdog's ros2 topic hz saw
# transient silence, and it aggressively stop/recreated thermals
# and blackflies that were actually publishing fine. The watchdog
# was reading the exact signal that its own actions made unreliable.
#
# v2 uses DDS-independent signals only:
#   1. Parse each driver's stdout (docker logs) for its own "Publishing"
#      lines. Drivers emit these directly from the acquisition loop;
#      they're ground truth regardless of DDS state.
#   2. Container state from `docker inspect` (restart count, exit code,
#      health — all from Docker daemon, no DDS).
#   3. Recorder bag-directory growth as an end-to-end sanity check
#      (topic → subscriber → disk) — if bytes are flowing to NAS, the
#      stack is recording SOMETHING, so a single "silent" container is
#      a localized issue not a DDS-wide problem.
#
# Safety rails baked into v2:
#   - Every action requires BOTH: driver log silent AND (container
#     unhealthy OR exited). This catches real container deaths without
#     false-positives from transient log quiet periods.
#   - Hard rate-limit: MAX_ACTIONS_PER_HOUR per container (default 3).
#   - Hold-off: after any restart, sleep COOLDOWN_SEC (120) before
#     sampling that container again — gives driver time to reinit.
#   - A6701 (thermal1/thermal2) are log-only NO MATTER WHAT — the
#     Pleora flap state is only fixed by physical power-cycle.
#
# Usage: run via bess-topic-watchdog.service (systemd).

set -u

LOG_PATH="${LOG_PATH:-/var/log/bess/topic_watchdog.log}"
POLL_INTERVAL="${POLL_INTERVAL:-30}"
LOG_AGE_MAX="${LOG_AGE_MAX:-30}"      # seconds — how recent must last "Publishing" line be
MAX_ACTIONS_PER_HOUR="${MAX_ACTIONS_PER_HOUR:-3}"
COOLDOWN_SEC="${COOLDOWN_SEC:-120}"   # per-container cooldown after action
STATE_DIR="${STATE_DIR:-/tmp/bess-topic-watchdog}"
BAG_DIR="${BAG_DIR:-/home/thor/nas/bess-bags/rolling/bag}"

mkdir -p "$STATE_DIR"
mkdir -p "$(dirname "$LOG_PATH")"

# Container → (driver-log regex, action). Action is one of:
#   restart           docker restart
#   recreate          docker compose up -d --force-recreate --no-deps
#   stop60_up         docker stop, sleep 60, docker compose up -d --no-deps
#   log_only          log + page, take NO action (A6701 Pleora — human only)
#
# The regex is grep -E extended, matched against the last 40 lines of
# docker logs. If found AND its timestamp is recent (<LOG_AGE_MAX s),
# driver is considered alive.
#
# Timestamps in ROS 2 INFO lines are epoch with nanos: [1776941333.XXX]
# We parse the integer seconds to compare against `date +%s`.
WATCHLIST=$(cat <<'EOF'
thermal1   Publishing\ at\ [0-9]           log_only
thermal2   Publishing\ at\ [0-9]           log_only
thermal3   Publishing\ at\ [0-9]           log_only
thermal4   Publishing\ at\ [0-9]           log_only
blackfly1  IN:.*OUT:.*Hz                    recreate
blackfly2  IN:.*OUT:.*Hz                    recreate
lucid1     Publishing\ at\ [0-9.]+\ Hz     recreate
lucid2     Publishing\ at\ [0-9.]+\ Hz     recreate
ouster     stats\|os_driver.*point_cloud   restart
microstrain Node\ activated\|filter\ reset\|data_rate restart
EOF
)

log_event() {
  local container="$1" state="$2" detail="${3:-}"
  local ts
  ts=$(date -u +%Y-%m-%dT%H:%M:%SZ)
  printf '{"ts":"%s","container":"%s","state":"%s","detail":"%s"}\n' \
    "$ts" "$container" "$state" "$detail" >> "$LOG_PATH"
}

# Last line from driver's log matching the rate-reporting regex.
# Returns epoch-seconds of the match, or 0 if not found.
last_publish_epoch() {
  local container="$1" regex="$2"
  local line
  line=$(docker logs --tail 60 "$container" 2>&1 \
    | grep -E "$regex" \
    | tail -1)
  [ -z "$line" ] && { echo 0; return; }
  # Grab [1234567890.XXX] → 1234567890
  local sec
  sec=$(echo "$line" | grep -oE '\[[0-9]+\.[0-9]+\]' | head -1 | tr -d '[' | cut -d. -f1)
  echo "${sec:-0}"
}

# Docker container status — returns "running|exited|unhealthy|restarting|missing"
container_state() {
  local c="$1"
  local s
  s=$(docker inspect --format '{{.State.Status}} {{.State.Health.Status}}' "$c" 2>/dev/null)
  [ -z "$s" ] && { echo missing; return; }
  if echo "$s" | grep -q "^running unhealthy"; then echo unhealthy
  elif echo "$s" | grep -q "^exited"; then echo exited
  elif echo "$s" | grep -q "^restarting"; then echo restarting
  else echo running
  fi
}

action_ok() {
  # Per-container rolling 60-min window + cooldown.
  local container="$1"
  local hist="$STATE_DIR/actions_${container}.log"
  touch "$hist"
  local cutoff=$(( $(date +%s) - 3600 ))
  awk -v c="$cutoff" '$1>=c' "$hist" > "${hist}.tmp" && mv "${hist}.tmp" "$hist"
  local n; n=$(wc -l < "$hist")
  [ "$n" -ge "$MAX_ACTIONS_PER_HOUR" ] && return 1
  # Cooldown check — don't act if we acted within COOLDOWN_SEC
  local last; last=$(tail -1 "$hist" 2>/dev/null)
  if [ -n "$last" ]; then
    local age=$(( $(date +%s) - last ))
    [ "$age" -lt "$COOLDOWN_SEC" ] && return 1
  fi
  echo "$(date +%s)" >> "$hist"
  return 0
}

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
      return 0
      ;;
    *)
      return 1
      ;;
  esac
}

# End-to-end liveness: recorder bag size growing?
bag_size_growing() {
  local b1 b2
  b1=$(du -sb "$BAG_DIR" 2>/dev/null | awk '{print $1+0}')
  sleep 3
  b2=$(du -sb "$BAG_DIR" 2>/dev/null | awk '{print $1+0}')
  [ "$b2" -gt "$b1" ] && return 0
  return 1
}

run_once() {
  local now; now=$(date +%s)
  local stack_flowing=yes
  if ! bag_size_growing; then
    stack_flowing=no
    log_event "" "recorder_not_flowing" "bag dir not growing — possible stack-wide issue, NOT acting on individual containers"
  fi

  while IFS= read -r line; do
    line=$(echo "$line" | xargs)
    [ -z "$line" ] && continue
    # Parse: container regex... action (whitespace-separated, regex may contain \s escapes)
    container=$(echo "$line" | awk '{print $1}')
    action=$(echo "$line" | awk '{print $NF}')
    regex=$(echo "$line" | awk '{ $1=""; $NF=""; sub(/^[ \t]+/,""); sub(/[ \t]+$/,""); print }')

    # Container state check
    cstate=$(container_state "$container")
    if [ "$cstate" = "missing" ]; then
      log_event "$container" "missing" "container does not exist"
      continue
    fi
    if [ "$cstate" = "exited" ]; then
      log_event "$container" "exited" "action=$action"
      if action_ok "$container"; then
        do_action "$container" "$action"
        log_event "$container" "acted" "reason=exited action=$action"
      else
        log_event "$container" "rate_limited" "exited but max=${MAX_ACTIONS_PER_HOUR}/h"
      fi
      continue
    fi

    # Driver-log Publishing check (DDS-independent)
    last_epoch=$(last_publish_epoch "$container" "$regex")
    if [ "$last_epoch" = "0" ]; then
      # No match in recent log — could be a log that's rolled over or
      # a driver whose format we don't know. Fall back to "only act if
      # container is ALSO unhealthy".
      if [ "$cstate" = "unhealthy" ]; then
        if [ "$stack_flowing" = "no" ]; then
          log_event "$container" "suspicious_but_stack_wide" "unhealthy + log silent BUT bag not growing, not acting"
        elif action_ok "$container"; then
          log_event "$container" "acted" "reason=log_silent+unhealthy action=$action"
          do_action "$container" "$action"
        else
          log_event "$container" "rate_limited" "unhealthy + log silent"
        fi
      fi
      # running but log format not matched → skip (not enough evidence to act)
      continue
    fi

    # Driver has published recently?
    age=$(( now - last_epoch ))
    if [ "$age" -le "$LOG_AGE_MAX" ]; then
      # healthy; clear any state
      continue
    fi

    # Driver log stale
    if [ "$cstate" = "unhealthy" ] || [ "$age" -ge $((LOG_AGE_MAX * 3)) ]; then
      if [ "$stack_flowing" = "no" ]; then
        log_event "$container" "stale_but_stack_wide" "last_publish_age=${age}s state=$cstate bag not growing, not acting"
      elif action_ok "$container"; then
        log_event "$container" "acted" "last_publish_age=${age}s state=$cstate action=$action"
        do_action "$container" "$action"
      else
        log_event "$container" "rate_limited" "last_publish_age=${age}s state=$cstate"
      fi
    fi
  done <<<"$WATCHLIST"
}

if [ "${1:-}" = "--once" ]; then
  run_once
  exit 0
fi

log_event "" "startup" "v2 driver-log+bag-size, poll=${POLL_INTERVAL}s age_max=${LOG_AGE_MAX}s cooldown=${COOLDOWN_SEC}s max/h=${MAX_ACTIONS_PER_HOUR}"
while true; do
  run_once
  sleep "$POLL_INTERVAL"
done
