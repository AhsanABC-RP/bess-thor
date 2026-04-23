#!/usr/bin/env bash
# Soak runner: orchestrates recorder + offloader + monitor for a timed run.
#
# Architecture:
#   recorder  → writes mcap shards to NVMe cache (/home/thor/bess/data/cache)
#   offloader → moves sealed shards to USB (/home/thor/nas/bess-bags) in parallel
#   monitor   → samples rates, scrapes container logs, checks PTP, alerts
#
# Usage: run_soak.sh [duration_minutes] [bag_name_suffix]
set -u

DUR_MIN=${1:-25}
SUFFIX=${2:-}

CACHE=/home/thor/bess/data/cache
USB=/home/thor/nas/bess-bags
LOG_DIR=/home/thor/recordings
SCRIPT_DIR=/home/thor/bess/scripts
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
TAG="soak_${TIMESTAMP}${SUFFIX:+_$SUFFIX}"

mkdir -p "$CACHE" "$USB" "$LOG_DIR"

MONITOR_LOG="$LOG_DIR/${TAG}_monitor.log"
OFFLOAD_LOG="$LOG_DIR/${TAG}_offload.log"
RUNNER_LOG="$LOG_DIR/${TAG}_runner.log"

ts() { date -u '+%Y-%m-%dT%H:%M:%SZ'; }
say() { echo "[$(ts)] $*" | tee -a "$RUNNER_LOG"; }

cleanup() {
  say "cleanup: stopping recorder + offloader"
  docker compose -f /home/thor/bess/docker-compose.thor.yml stop recorder 2>&1 | tail -3 | tee -a "$RUNNER_LOG"
  if [[ -n "${OFFLOADER_PID:-}" ]]; then
    say "letting offloader drain remaining shards (up to 5 min)"
    local deadline=$(($(date +%s) + 300))
    while kill -0 "$OFFLOADER_PID" 2>/dev/null; do
      local remaining=$(find "$CACHE" -name '*.mcap' 2>/dev/null | wc -l)
      [[ $remaining -eq 0 ]] && { say "cache drained"; break; }
      [[ $(date +%s) -ge $deadline ]] && { say "drain timeout, killing offloader"; break; }
      sleep 5
    done
    kill -TERM "$OFFLOADER_PID" 2>/dev/null || true
    wait "$OFFLOADER_PID" 2>/dev/null || true
  fi
  if [[ -n "${MONITOR_PID:-}" ]]; then
    kill -TERM "$MONITOR_PID" 2>/dev/null || true
    wait "$MONITOR_PID" 2>/dev/null || true
  fi
  say "cleanup complete"
}
trap cleanup EXIT INT TERM

: > "$RUNNER_LOG"
say "soak runner start tag=$TAG duration=${DUR_MIN}m"
say "  cache=$CACHE"
say "  usb=$USB"
say "  monitor_log=$MONITOR_LOG"
say "  offload_log=$OFFLOAD_LOG"

# Pre-flight: check NVMe + USB headroom
nvme_free_gb=$(df -BG --output=avail "$CACHE" | tail -1 | tr -d 'G ')
usb_free_gb=$(df -BG --output=avail "$USB" | tail -1 | tr -d 'G ')
say "headroom: nvme=${nvme_free_gb}GB usb=${usb_free_gb}GB"
if (( nvme_free_gb < 60 )); then
  say "ABORT: NVMe free space < 60GB (need at least 1 max-bag-size + headroom)"
  exit 1
fi
if (( usb_free_gb < 350 )); then
  say "WARN: USB free space < 350GB; soak may not fit"
fi

# Pre-flight: cache must be empty (no orphan shards)
orphans=$(find "$CACHE" -maxdepth 2 -name '*.mcap' 2>/dev/null | wc -l)
if (( orphans > 0 )); then
  say "WARN: $orphans orphan mcap files in cache. Letting offloader drain first."
  bash "$SCRIPT_DIR/soak_offloader.sh" "$CACHE" "$USB" --once 2>&1 | tee -a "$OFFLOAD_LOG"
fi

# Start offloader (background)
say "starting offloader"
bash "$SCRIPT_DIR/soak_offloader.sh" "$CACHE" "$USB" >> "$OFFLOAD_LOG" 2>&1 &
OFFLOADER_PID=$!
say "  offloader pid=$OFFLOADER_PID"

# Start recorder
say "starting recorder"
docker compose -f /home/thor/bess/docker-compose.thor.yml up -d recorder 2>&1 | tee -a "$RUNNER_LOG"
sleep 5
if ! docker ps --format '{{.Names}}' | grep -q '^recorder$'; then
  say "ABORT: recorder did not start"
  exit 1
fi

# Find the bag dir the recorder just created
sleep 5
BAG_DIR=$(find "$CACHE" -maxdepth 1 -type d -name 'soak_*' -newer "$RUNNER_LOG" 2>/dev/null | sort | tail -1)
if [[ -z "$BAG_DIR" ]]; then
  say "WARN: cannot identify recorder bag dir, monitor will scan whole cache"
  BAG_DIR="$CACHE"
fi
say "  bag_dir=$BAG_DIR"

# Start monitor (background)
say "starting monitor"
bash "$SCRIPT_DIR/soak_monitor.sh" "$MONITOR_LOG" "$BAG_DIR" "$DUR_MIN" >> "$RUNNER_LOG" 2>&1 &
MONITOR_PID=$!
say "  monitor pid=$MONITOR_PID"

# Wait for the duration (monitor exits at deadline)
END=$(($(date +%s) + DUR_MIN*60))
say "soak running until $(date -u -d @$END '+%H:%M:%SZ')"
while (( $(date +%s) < END )); do
  if ! kill -0 "$OFFLOADER_PID" 2>/dev/null; then
    say "ABORT: offloader died"
    exit 2
  fi
  if ! docker ps --format '{{.Names}}' | grep -q '^recorder$'; then
    say "ABORT: recorder died"
    exit 3
  fi
  sleep 30
done

say "soak duration complete, triggering cleanup"
# trap will handle the rest
