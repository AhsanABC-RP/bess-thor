#!/usr/bin/env bash
# Soak monitor: disk growth, container health, PTP state, per-container log
# errors, and periodic per-topic rate sampling with drop alerts.
# Usage: soak_monitor.sh <monitor_log> <bag_dir> <duration_min>
set -u

LOG=${1:?monitor log path required}
BAG_DIR=${2:?bag dir required}
DUR_MIN=${3:-120}
ALERT=${LOG%.log}_alerts.log

declare -A TOPICS=(
  [/ouster/points]="ouster:10"
  [/ouster/imu]="ouster:640"
  [/imu/data]="microstrain:100"
  [/thermal/camera1/image_raw]="thermal1:18"
  [/thermal/camera2/image_raw]="thermal2:18"
  [/thermal/camera3/image_raw]="thermal3:18"
  [/thermal/camera4/image_raw]="thermal4:16"
  [/lucid1/camera_driver/image_raw]="lucid1:3"
  [/lucid2/camera_driver/image_raw]="lucid2:3"
  [/blackfly/camera1/blackfly_camera/image_raw]="blackfly1:5"
  [/blackfly/camera2/blackfly_camera/image_raw]="blackfly2:5"
  [/dlio/odom_node/odom]="dlio:100"
)
CONTAINERS=(ouster microstrain lucid1 lucid2 thermal1 thermal2 thermal3 thermal4 blackfly1 blackfly2 dlio fast-lio recorder foxglove rutx)

ts() { date -u '+%Y-%m-%dT%H:%M:%SZ'; }
say()   { echo "[$(ts)] $*" >> "$LOG"; }
alert() { echo "[$(ts)] ALERT $*" | tee -a "$LOG" "$ALERT" >/dev/null; }

ros_distro_of() {
  timeout 3 docker exec "$1" bash -lc 'ls /opt/ros/ 2>/dev/null | head -1' 2>/dev/null | tr -d '\r\n'
}

sample_rate() {
  local topic=$1 container=$2 distro out
  distro=$(ros_distro_of "$container")
  [[ -z $distro ]] && { echo 0; return; }
  out=$(timeout 7 docker exec "$container" bash -lc \
    "source /opt/ros/$distro/setup.bash && timeout 5 ros2 topic hz --window 20 $topic 2>&1" 2>/dev/null || true)
  echo "$out" | awk '/average rate:/ { r=$3 } END { print (r?r:0) }'
}

: > "$LOG"
: > "$ALERT"
say "soak monitor start bag_dir=$BAG_DIR duration=${DUR_MIN}m pid=$$"

# Baseline
say "baseline: sampling topic rates"
declare -A BASE
for t in "${!TOPICS[@]}"; do
  IFS=: read -r c exp <<< "${TOPICS[$t]}"
  r=$(sample_rate "$t" "$c")
  BASE[$t]=$r
  say "  baseline $t = ${r} Hz (expected ~${exp}Hz)"
done

START=$(date +%s)
DEADLINE=$((START + DUR_MIN*60))
PREV_BYTES=0
PREV_T=$START
tick=0

while (( $(date +%s) < DEADLINE )); do
  tick=$((tick+1))
  now=$(date +%s)
  elapsed=$((now-START))

  # disk
  bytes=$(du -sb "$BAG_DIR" 2>/dev/null | awk '{print $1}')
  bytes=${bytes:-0}
  gb=$(awk -v b="$bytes" 'BEGIN { printf "%.2f", b/(1024^3) }')
  rate="0.0"
  if (( PREV_BYTES > 0 )) && (( now > PREV_T )); then
    rate=$(awk -v n="$bytes" -v p="$PREV_BYTES" -v dt=$((now-PREV_T)) \
      'BEGIN { printf "%.1f", (n-p)/(dt*1024*1024) }')
  fi
  PREV_BYTES=$bytes; PREV_T=$now

  # containers
  ps_out=$(docker ps --format '{{.Names}}:{{.Status}}')
  bad=$(echo "$ps_out" | awk -F: '$2 ~ /unhealthy|Restarting|Exited/ { printf "%s ", $1 }')
  n_ctrs=$(echo "$ps_out" | wc -l)
  [[ -n ${bad// /} ]] && alert "containers bad: ${bad% }"

  # ptp state change detection
  ptp_changes=$(journalctl -u ptp4l-mgbe0.service --since "70 seconds ago" --no-pager 2>/dev/null \
    | grep -E "MASTER to |LISTENING to |FAULT|UNCALIBRATED|port 1 .* to " | tail -3)
  [[ -n $ptp_changes ]] && alert "ptp4l: $(echo "$ptp_changes" | tr '\n' '|')"
  phc_off=$(journalctl -u phc2sys-mgbe0.service --since "70 seconds ago" --no-pager 2>/dev/null \
    | grep -oE "offset[[:space:]]+-?[0-9]+" | tail -1 | awk '{print $NF}')

  # container log error scrape
  # Excludes: known DDS noise, rcutils overwrite churn, routine blackfly status
  # warns (which include literal "drop   0%"), foxglove rtcm schema, rutx http
  # offline. Alerts on: real errors, non-zero drops, fatal/abort/segfault.
  for c in "${CONTAINERS[@]}"; do
    errs=$(docker logs --since 65s "$c" 2>&1 \
      | grep -Eiv 'type hash|USER_DATA|sequence size|Failed to parse|Could not set gain|enumentr|error_handling\.c|rcutils_reset_error|This error state is being overwritten|with this new error message|drop[[:space:]]+0%|Failed to load schemaDefinition for topic "/rtcm"|HTTPSConnectionPool\(host=.192\.168\.1\.1' \
      | grep -Ei 'error|fatal|\bfail|drop[[:space:]]+[1-9]|incomplete|disconnect|\btimeout|segfault|abort' \
      | head -3)
    [[ -n $errs ]] && alert "$c: $(echo "$errs" | tr '\n' '|' | cut -c1-280)"
  done

  status="t=${elapsed}s disk=${gb}GB (+${rate}MB/s) ctrs=${n_ctrs} phc_off=${phc_off:-?}ns"

  # every 5 ticks sample rates in parallel
  if (( (tick % 5) == 1 )); then
    tmpdir=$(mktemp -d)
    for t in "${!TOPICS[@]}"; do
      IFS=: read -r c exp <<< "${TOPICS[$t]}"
      safe=$(echo "$t" | tr '/' '_')
      ( sample_rate "$t" "$c" > "$tmpdir/$safe" ) &
    done
    wait
    rates=""
    for t in "${!TOPICS[@]}"; do
      IFS=: read -r c exp <<< "${TOPICS[$t]}"
      safe=$(echo "$t" | tr '/' '_')
      r=$(cat "$tmpdir/$safe" 2>/dev/null || echo 0)
      tag=$(echo "$t" | awk -F/ '{print $2}')
      case "$t" in
        /ouster/points) tag=os_pts ;;
        /ouster/imu) tag=os_imu ;;
        /imu/data) tag=gq7_imu ;;
        /thermal/*) tag=th${t:15:1} ;;
        /lucid1/*) tag=lu1 ;;
        /lucid2/*) tag=lu2 ;;
        /blackfly/camera1/*) tag=bf1 ;;
        /blackfly/camera2/*) tag=bf2 ;;
        /dlio/*) tag=dlio ;;
      esac
      rates="$rates ${tag}=${r}"
      if awk -v r="$r" -v e="$exp" 'BEGIN { exit !(r+0 < e*0.8) }'; then
        alert "rate drop: $t=${r}Hz (expected ~${exp}Hz, <80%)"
      fi
    done
    rm -rf "$tmpdir"
    status="$status RATES:$rates"
  fi

  say "$status"
  sleep 60
done

say "soak monitor exit after ${DUR_MIN}m"
