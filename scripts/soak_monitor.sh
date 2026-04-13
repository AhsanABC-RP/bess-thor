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
  [/rtcm]="ntrip:1"
)
CONTAINERS=(ouster microstrain ntrip lucid1 lucid2 thermal1 thermal2 thermal3 thermal4 blackfly1 blackfly2 dlio fast-lio recorder foxglove rutx)
# Minimum acceptable GQ7 GNSS fix_type. MIP fix codes:
#   0=3D 1=2D 2=Time 3=None 4=Invalid 5=RTK Float 6=RTK Fixed
# We require RTK Float (5) or better while NTRIP corrections are flowing.
RTK_MIN_FIX=${RTK_MIN_FIX:-5}

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

# Read GQ7 GNSS fix_type. Returns the integer fix code (0..6) or -1 on error.
# Sourced from microstrain container because that's where mip msgs are built.
sample_rtk_fix_type() {
  local out
  out=$(timeout 8 docker exec microstrain bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    timeout 5 ros2 topic echo --once /mip/gnss_1/fix_info 2>/dev/null' 2>/dev/null || true)
  echo "$out" | awk '/^fix_type:/ { print $2; found=1; exit } END { if (!found) print -1 }'
}

# Read GQ7 RTK corrections epoch_status to confirm at least one constellation
# is currently being decoded by the receiver. Returns "1" if any of
# gps/glonass/galileo/beidou _received is true, else "0".
sample_rtk_corrections_active() {
  local out
  out=$(timeout 8 docker exec microstrain bash -lc '
    source /opt/ros/jazzy/setup.bash &&
    source /ros2_ws/install/setup.bash &&
    timeout 5 ros2 topic echo --once /mip/gnss_corrections/rtk_corrections_status 2>/dev/null' 2>/dev/null || true)
  echo "$out" | awk '
    /gps_received: true/      { found=1 }
    /glonass_received: true/  { found=1 }
    /galileo_received: true/  { found=1 }
    /beidou_received: true/   { found=1 }
    END { print (found?1:0) }'
}

# Ouster PTP slave offset_from_master in nanoseconds (signed). Empty on error.
# Polled directly from sensor HTTP API so it bypasses container/topic noise.
sample_ouster_ptp_offset_ns() {
  local out
  out=$(timeout 5 curl -sf http://169.254.70.119/api/v1/time/ptp 2>/dev/null || true)
  [[ -z $out ]] && { echo ""; return; }
  python3 -c 'import sys,json
try:
  d=json.load(sys.stdin)
  v=d.get("current_data_set",{}).get("offset_from_master","")
  print(int(v) if v != "" else "")
except Exception:
  print("")' <<< "$out" 2>/dev/null
}

# spinnaker_camera_driver prints periodic stats lines containing "off[s]: <secs>"
# from its underlying GenICam PTP slave servo. Returns the most recent value
# from this container's logs in the last 70s, signed seconds, or empty.
sample_spinnaker_ptp_offset_s() {
  local container=$1
  docker logs --since 70s "$container" 2>&1 \
    | grep -oE 'off\[s\]:[[:space:]]*-?[0-9eE.+-]+' \
    | tail -1 \
    | awk -F: '{gsub(/[[:space:]]/,"",$2); print $2}'
}

# FLIR thermal camera internal temperatures. The patched thermal_spinnaker_node
# reads SensorTemperature (FPA) + HousingTemperature from the A6701/A70
# GenICam nodemap every 15s and publishes sensor_msgs/Temperature on
# /thermal/cameraN/temperature and /thermal/cameraN/housing_temperature.
# This is the primary thermal-stress signal for the sealed sensor bay —
# cooling failures will show at FPA + housing before the rate drops.
#
# Returns "fpa_c housing_c" or empty on error.
sample_thermal_cam_temps() {
  local container=$1 cam_ns=$2 distro out_fpa out_hsg
  distro=$(ros_distro_of "$container")
  [[ -z $distro ]] && { echo ""; return; }
  out_fpa=$(timeout 5 docker exec "$container" bash -lc \
    "source /opt/ros/$distro/setup.bash && timeout 4 ros2 topic echo --once --field temperature /thermal/${cam_ns}/temperature 2>/dev/null" 2>/dev/null || true)
  out_hsg=$(timeout 5 docker exec "$container" bash -lc \
    "source /opt/ros/$distro/setup.bash && timeout 4 ros2 topic echo --once --field temperature /thermal/${cam_ns}/housing_temperature 2>/dev/null" 2>/dev/null || true)
  out_fpa=$(echo "$out_fpa" | tr -d '[:space:]')
  out_hsg=$(echo "$out_hsg" | tr -d '[:space:]')
  [[ -z $out_fpa && -z $out_hsg ]] && { echo ""; return; }
  echo "${out_fpa:-?} ${out_hsg:-?}"
}

# MikroTik CRS510 switch health. Returns "switch_c sfp_c board1_c board2_c"
# space-separated integers, or empty on error. The sensor bay is sealed so
# a cooling failure surfaces first at the SFP28 cages where all 10GigE
# camera links terminate — watch these during long soaks.
sample_mikrotik_temps() {
  local out
  out=$(timeout 5 sshpass -p '8RKUP2PUT9' ssh \
    -o StrictHostKeyChecking=no -o ConnectTimeout=3 \
    admin@169.254.100.254 '/system/health/print' 2>/dev/null || true)
  [[ -z $out ]] && { echo ""; return; }
  awk '
    /switch-temperature/ { s=$3 }
    /sfp-temperature/    { sp=$3 }
    /board-temperature1/ { b1=$3 }
    /board-temperature2/ { b2=$3 }
    END { if (s!="" && sp!="") print s" "sp" "b1" "b2 }' <<< "$out"
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
        /rtcm) tag=rtcm ;;
      esac
      rates="$rates ${tag}=${r}"
      if awk -v r="$r" -v e="$exp" 'BEGIN { exit !(r+0 < e*0.8) }'; then
        alert "rate drop: $t=${r}Hz (expected ~${exp}Hz, <80%)"
      fi
    done
    rm -rf "$tmpdir"
    status="$status RATES:$rates"

    # PTP slave offset watchdog. With phc2sys -S 0 the host PHC1 never steps,
    # so any sustained slave offset > PTP_MAX_OFFSET_MS means the slave servo
    # is broken (firmware lockup, BMCA flap, switch non-transparency). Catch
    # it during the soak rather than after extraction.
    PTP_MAX_OFFSET_MS=${PTP_MAX_OFFSET_MS:-100}
    ouster_ptp_ns=$(sample_ouster_ptp_offset_ns)
    if [[ -n $ouster_ptp_ns ]]; then
      status="$status ou_ptp=${ouster_ptp_ns}ns"
      if awk -v n="$ouster_ptp_ns" -v lim="$PTP_MAX_OFFSET_MS" \
          'BEGIN { exit !( (n<0?-n:n) > lim*1000000 ) }'; then
        alert "Ouster PTP slave offset out of range: ${ouster_ptp_ns}ns (limit ${PTP_MAX_OFFSET_MS}ms)"
      fi
    else
      status="$status ou_ptp=?"
    fi
    for c in blackfly1 blackfly2; do
      off_s=$(sample_spinnaker_ptp_offset_s "$c")
      [[ -z $off_s ]] && continue
      status="$status ${c}_ptp=${off_s}s"
      if awk -v s="$off_s" -v lim="$PTP_MAX_OFFSET_MS" \
          'BEGIN { exit !( (s<0?-s:s) > lim/1000 ) }'; then
        alert "$c PTP slave offset out of range: ${off_s}s (limit ${PTP_MAX_OFFSET_MS}ms)"
      fi
    done

    # MikroTik switch health watchdog. Sealed sensor bay means a cooling
    # failure manifests as SFP cage temps climbing before anything else
    # fails. Alert before thermal shutdown rather than after.
    MKTK_MAX_SFP_C=${MKTK_MAX_SFP_C:-80}
    MKTK_MAX_SWITCH_C=${MKTK_MAX_SWITCH_C:-60}
    mktk_temps=$(sample_mikrotik_temps)
    if [[ -n $mktk_temps ]]; then
      read sw_c sfp_c b1_c b2_c <<< "$mktk_temps"
      status="$status mktk_sw=${sw_c}c mktk_sfp=${sfp_c}c mktk_b=${b1_c}/${b2_c}c"
      if [[ $sfp_c =~ ^[0-9]+$ ]] && (( sfp_c > MKTK_MAX_SFP_C )); then
        alert "MikroTik SFP cage ${sfp_c}C exceeds limit ${MKTK_MAX_SFP_C}C — check sensor-bay cooling"
      fi
      if [[ $sw_c =~ ^[0-9]+$ ]] && (( sw_c > MKTK_MAX_SWITCH_C )); then
        alert "MikroTik switch ASIC ${sw_c}C exceeds limit ${MKTK_MAX_SWITCH_C}C"
      fi
    else
      status="$status mktk=?"
    fi

    # FLIR thermal camera internal DeviceTemperature, published by the patched
    # thermal_spinnaker_node on /thermal/cameraN/temperature.
    #
    # Two very different physical readings share the same topic:
    #
    #   - A6701 Xsc (thermal1/thermal2): cooled InSb MWIR. DeviceTemperature
    #     is the Stirling-cooled FPA in °C. Healthy is ~-200°C (~73K, LN2-
    #     adjacent). A *rising* reading means the Stirling cooler is failing;
    #     above about -150°C the detector is unusable. Alert high, not low.
    #
    #   - A70 (thermal3/thermal4): uncooled microbolometer LWIR. Device-
    #     Temperature is body temperature in °C (device reports Kelvin, node
    #     converts). Healthy 25-50°C under load; alert above 75°C.
    #
    # Samples 2 cameras per tick (thermal1=A6701, thermal3=A70); thermal2/4
    # share thermal mass and sampling all four doubles ros2 exec cost.
    A6701_FPA_ALERT_C=${A6701_FPA_ALERT_C:--150}   # above this = cooler fail
    A70_FPA_ALERT_C=${A70_FPA_ALERT_C:-75}         # above this = bay too hot
    for pair in "thermal1:camera1:A6701" "thermal3:camera3:A70"; do
      ct=${pair%%:*}
      rest=${pair#*:}
      cns=${rest%%:*}
      family=${pair##*:}
      tct=$(sample_thermal_cam_temps "$ct" "$cns")
      if [[ -n $tct ]]; then
        read fpa_c _hsg_unused <<< "$tct"
        status="$status ${ct}_fpa=${fpa_c}c"
        if [[ $family == A6701 ]]; then
          if awk -v v="$fpa_c" -v lim="$A6701_FPA_ALERT_C" \
              'BEGIN { exit !(v+0 > lim+0) }' 2>/dev/null; then
            alert "$ct (A6701) FPA ${fpa_c}C above ${A6701_FPA_ALERT_C}C — Stirling cryocooler failure"
          fi
        else
          if awk -v v="$fpa_c" -v lim="$A70_FPA_ALERT_C" \
              'BEGIN { exit !(v+0 > lim+0) }' 2>/dev/null; then
            alert "$ct (A70) body ${fpa_c}C exceeds ${A70_FPA_ALERT_C}C — sensor bay cooling check"
          fi
        fi
      else
        status="$status ${ct}_temp=?"
      fi
    done

    # GQ7 RTK fix-quality sampling. Only meaningful when ntrip container is up
    # and /rtcm is being delivered to the GQ7 aux port. Codes:
    #   0=3D 1=2D 2=Time 3=None 4=Invalid 5=RTKFloat 6=RTKFixed
    rtk_fix=$(sample_rtk_fix_type)
    rtk_corr=$(sample_rtk_corrections_active)
    status="$status rtk_fix=${rtk_fix} rtk_corr=${rtk_corr}"
    if [[ $rtk_fix -ge 0 ]] && (( rtk_fix < RTK_MIN_FIX )); then
      alert "GQ7 RTK fix degraded: fix_type=${rtk_fix} (<${RTK_MIN_FIX} required) corrections_active=${rtk_corr}"
    fi
    if [[ $rtk_corr == 0 ]]; then
      alert "GQ7 RTK corrections inactive: no constellation epoch_status received in last sample"
    fi
  fi

  say "$status"
  sleep 60
done

say "soak monitor exit after ${DUR_MIN}m"
