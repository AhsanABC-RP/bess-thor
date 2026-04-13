#!/bin/bash
# BESS System Audit — concise vehicle health report
# Usage: bess-audit
set -uo pipefail

VEHICLE=$(hostname)
TS=$(date -u '+%Y-%m-%dT%H:%M:%SZ')
TS_HUMAN=$(date -u '+%Y-%m-%d %H:%M UTC')
JSON=/tmp/bess-audit-latest.json
ISSUES=()
WARNINGS=()

add_issue()   { ISSUES+=("$1"); }
add_warning() { WARNINGS+=("$1"); }

# ─── HEADER ──────────────────────────────────────────────────────
printf "\n  BESS AUDIT — %s — %s\n" "$VEHICLE" "$TS_HUMAN"
printf "  ─────────────────────────────────────────────\n\n"

# ─── HEAD STATUS ─────────────────────────────────────────────────
SENSOR_NICS_UP=0
for iface in $(ls /sys/class/net/ | grep -E '^enp132|^enp193'); do
    state=$(cat /sys/class/net/$iface/operstate 2>/dev/null || echo "unknown")
    echo "$iface" | grep -qE 'f3np3$|s0f3$' && continue  # skip spare
    [ "$state" = "up" ] && SENSOR_NICS_UP=$((SENSOR_NICS_UP + 1))
done
IMU_MAIN=$(test -e /dev/microstrain_main && echo "yes" || echo "no")
HEAD_CONNECTED=false
if (( SENSOR_NICS_UP > 0 )) || [ "$IMU_MAIN" = "yes" ]; then
    HEAD_CONNECTED=true
    printf "  HEAD        CONNECTED (%d NICs, IMU=%s)\n" "$SENSOR_NICS_UP" "$IMU_MAIN"
else
    printf "  HEAD        DISCONNECTED\n"
fi

# ─── SYSTEM RESOURCES ────────────────────────────────────────────
LOAD=$(awk '{print $1}' /proc/loadavg)
NCPU=$(nproc)
MEM_USED=$(free -g | awk '/Mem/{print $3}')
MEM_TOTAL=$(free -g | awk '/Mem/{print $2}')
SWAP_USED=$(free -g | awk '/Swap/{print $3}')
UPTIME=$(uptime -p | sed 's/up //')
printf "  SYSTEM      load=%s/%d  mem=%sG/%sG  swap=%sG  up=%s\n" "$LOAD" "$NCPU" "$MEM_USED" "$MEM_TOTAL" "$SWAP_USED" "$UPTIME"
(( SWAP_USED > 6 )) && add_warning "High swap: ${SWAP_USED}G"

# ─── GPU ─────────────────────────────────────────────────────────
if command -v nvidia-smi &>/dev/null; then
    gpu_n=0
    while IFS=',' read -r name util mem_used mem_total temp; do
        name=$(echo "$name" | sed 's/^ *//;s/ *$//;s/NVIDIA //')
        util=$(echo "$util" | tr -d ' ')
        mem_used=$(echo "$mem_used" | tr -d ' ')
        mem_total=$(echo "$mem_total" | tr -d ' ')
        temp=$(echo "$temp" | tr -d ' ')
        printf "  GPU %d       %s  util=%s%%  mem=%s/%sMiB  temp=%s°C\n" "$gpu_n" "$name" "$util" "$mem_used" "$mem_total" "$temp"
        (( temp > 85 )) && add_warning "GPU $gpu_n temp ${temp}°C"
        gpu_n=$((gpu_n + 1))
    done < <(nvidia-smi --query-gpu=name,utilization.gpu,memory.used,memory.total,temperature.gpu \
        --format=csv,noheader,nounits 2>/dev/null)
fi

# ─── BLUETTI BATTERY ────────────────────────────────────────────
BLUETTI_SVC=$(systemctl is-active bluetti-mqtt.service 2>/dev/null || echo "inactive")
MQTT_OK=$(timeout 1 mosquitto_pub -h 127.0.0.1 -t 'bess/ping' -m 'ok' 2>/dev/null && echo "yes" || echo "no")
if [ "$BLUETTI_SVC" = "active" ] && [ "$MQTT_OK" = "yes" ]; then
    # Run all 3 subs in parallel (poll interval is 10s, need ≤12s wait)
    timeout 12 mosquitto_sub -h 127.0.0.1 -t 'bluetti/state/+/total_battery_percent' -C 1 > /tmp/.bat_pct 2>/dev/null &
    timeout 12 mosquitto_sub -h 127.0.0.1 -t 'bluetti/state/+/ac_input_power' -C 1 > /tmp/.ac_in 2>/dev/null &
    timeout 12 mosquitto_sub -h 127.0.0.1 -t 'bluetti/state/+/ac_output_power' -C 1 > /tmp/.ac_out 2>/dev/null &
    wait
    BAT_PCT=$(cat /tmp/.bat_pct 2>/dev/null || echo "?"); [ -z "$BAT_PCT" ] && BAT_PCT="?"
    AC_IN=$(cat /tmp/.ac_in 2>/dev/null || echo "?"); [ -z "$AC_IN" ] && AC_IN="?"
    AC_OUT=$(cat /tmp/.ac_out 2>/dev/null || echo "?"); [ -z "$AC_OUT" ] && AC_OUT="?"
    printf "  BATTERY     %s%%  AC_in=%sW  AC_out=%sW\n" "$BAT_PCT" "$AC_IN" "$AC_OUT"
    if [ "$BAT_PCT" != "?" ] && (( BAT_PCT < 20 )); then
        add_issue "Battery LOW: ${BAT_PCT}%"
    elif [ "$BAT_PCT" != "?" ] && (( BAT_PCT < 50 )); then
        add_warning "Battery ${BAT_PCT}%"
    fi
    if [ "$AC_IN" != "?" ] && [ "$AC_IN" = "0" ] && [ "$BAT_PCT" != "?" ] && (( BAT_PCT < 80 )); then
        add_warning "No AC input — running on battery (${BAT_PCT}%)"
    fi
elif [ "$BLUETTI_SVC" = "active" ]; then
    printf "  BATTERY     bluetti-mqtt active but MQTT broker unreachable\n"
    add_warning "MQTT broker unreachable — no battery data"
else
    printf "  BATTERY     bluetti-mqtt %s\n" "$BLUETTI_SVC"
    add_warning "Bluetti MQTT bridge not active"
fi

# ─── PTP ─────────────────────────────────────────────────────────
PTP4L=$(systemctl is-active ptp4l.service 2>&1)
PHC2SYS=$(systemctl is-active phc2sys.service 2>&1)
if [ "$PTP4L" = "active" ] && [ "$PHC2SYS" = "active" ]; then
    printf "  PTP         OK (ptp4l + phc2sys)\n"
else
    printf "  PTP         ptp4l=%s phc2sys=%s\n" "$PTP4L" "$PHC2SYS"
    $HEAD_CONNECTED && add_issue "PTP chain broken: ptp4l=$PTP4L phc2sys=$PHC2SYS"
fi

echo

# ─── SERVICES (compact) ─────────────────────────────────────────
printf "  SERVICES\n"
ALL_SERVICES="bess-cameras bess-ouster bess-recorder bess-extraction bess-pii-mask
bess-slam bess-fast-lio bess-glim bess-odometry bess-watchdog bess-telemetry
bess-dashboard bess-vehicle-bridge bess-phidgets bess-mosquitto bess-triton
bess-segformer bess-thermal-analysis bess-ml-client bess-monitor-poller
bess-slam-accumulator bess-glim-session bess-dispatch bess-executor
bess-microstrain bess-ptp-setup bess-sensor-discovery bess-ntrip"

ok_list=""
fail_list=""
inactive_list=""
masked_list=""
FAILED_PROD=0
for svc in $ALL_SERVICES; do
    state=$(systemctl is-active "$svc" 2>&1)
    load_state=$(systemctl show "$svc" -p LoadState --value 2>/dev/null)
    short=$(echo "$svc" | sed 's/^bess-//')
    if [ "$state" = "active" ]; then
        ok_list="${ok_list}${short} "
    elif [ "$load_state" = "masked" ]; then
        masked_list="${masked_list}${short} "
    elif [ "$state" = "inactive" ]; then
        inactive_list="${inactive_list}${short} "
    else
        fail_list="${fail_list}${short} "
        FAILED_PROD=$((FAILED_PROD + 1))
        add_issue "Service ${svc} is ${state}"
    fi
done
# Wrap OK list at ~80 chars
if [ -n "$ok_list" ]; then
    printf "    OK:       "
    col=14
    for s in $ok_list; do
        slen=${#s}
        if (( col + slen + 1 > 78 )); then
            printf "\n              "
            col=14
        fi
        printf "%s " "$s"
        col=$((col + slen + 1))
    done
    echo
fi
[ -n "$fail_list" ] && printf "    FAILED:   %s\n" "$fail_list"
[ -n "$masked_list" ] && printf "    MASKED:   %s\n" "$masked_list"
[ -n "$inactive_list" ] && printf "    OFF:      %s\n" "$inactive_list"

# Container health — only show problems
UNHEALTHY=$(sudo podman ps --format '{{.Names}} {{.Status}}' 2>/dev/null | grep -i unhealthy | awk '{print $1}' || true)
if [ -n "$UNHEALTHY" ]; then
    printf "    UNHEALTHY:"
    for c in $UNHEALTHY; do
        short=$(echo "$c" | sed 's/^bess-//')
        printf " %s" "$short"
    done
    echo
fi

echo

# ─── SENSORS (if head connected) ────────────────────────────────
if $HEAD_CONNECTED; then
    printf "  SENSORS\n"

    # Per-camera Hz via ros2 topic hz (camera1=right RGB, camera2=left RGB, thermal1=left, thermal2=right)
    _topic_hz() {
        local topic="$1" window="${2:-3}"
        sudo podman exec bess-cameras bash -c "source /opt/ros/humble/setup.bash && timeout 6 ros2 topic hz $topic --window $window 2>/dev/null" 2>/dev/null \
            | grep -oP 'average rate: \K[\d.]+' | tail -1
    }
    RGB_R_HZ=$(_topic_hz /camera2/camera_driver/image_raw)
    RGB_L_HZ=$(_topic_hz /camera3/camera_driver/image_raw)
    THERM_L_HZ=$(_topic_hz /thermal1/camera_driver/image_raw)
    THERM_R_HZ=$(_topic_hz /thermal2/camera_driver/image_raw)
    printf "    Right RGB:     %s Hz (target 5)\n" "${RGB_R_HZ:-??}"
    printf "    Left RGB:      %s Hz (target 5)\n" "${RGB_L_HZ:-??}"
    printf "    Left Thermal:  %s Hz (target 15)\n" "${THERM_L_HZ:-??}"
    printf "    Right Thermal: %s Hz (target 15)\n" "${THERM_R_HZ:-??}"
    [ -n "$RGB_R_HZ" ] && (( $(echo "$RGB_R_HZ < 4.0" | bc -l 2>/dev/null || echo 0) )) && add_issue "Right RGB rate low: ${RGB_R_HZ}Hz"
    [ -n "$RGB_L_HZ" ] && (( $(echo "$RGB_L_HZ < 4.0" | bc -l 2>/dev/null || echo 0) )) && add_issue "Left RGB rate low: ${RGB_L_HZ}Hz"
    [ -n "$THERM_L_HZ" ] && (( $(echo "$THERM_L_HZ < 12.0" | bc -l 2>/dev/null || echo 0) )) && add_issue "Left Thermal rate low: ${THERM_L_HZ}Hz"
    [ -n "$THERM_R_HZ" ] && (( $(echo "$THERM_R_HZ < 12.0" | bc -l 2>/dev/null || echo 0) )) && add_issue "Right Thermal rate low: ${THERM_R_HZ}Hz"

    # LiDAR
    OUSTER_HEALTH=$(sudo podman inspect --format '{{.State.Health.Status}}' bess-ouster 2>/dev/null || echo "unknown")
    printf "    LiDAR:    %s (target 10 Hz)\n" "$OUSTER_HEALTH"

    # IMU
    printf "    IMU:      main=%s aux=%s\n" "$IMU_MAIN" "$(test -e /dev/microstrain_aux && echo yes || echo no)"

    # GNSS — check from glim-session (has RTK status) or ntrip
    GNSS_FIX=$(sudo journalctl -u bess-glim-session --since "-5min" --no-pager 2>&1 | \
        grep -oP 'fix=(RTK_FIXED|RTK_FLOAT|DGPS|SPS|NONE)' | tail -1 | cut -d= -f2)
    if [ -z "$GNSS_FIX" ]; then
        # Fallback: check if ntrip is sending GGA
        NTRIP_GGA=$(sudo journalctl -u bess-ntrip --since "-60s" --no-pager 2>&1 | grep -c 'Sent GGA' || true)
        if (( NTRIP_GGA > 0 )); then
            GNSS_FIX="active (NTRIP streaming)"
        else
            GNSS_FIX="no data"
        fi
    fi
    printf "    GNSS:     %s\n" "$GNSS_FIX"

    echo

    # ─── SLAM PIPELINE ───────────────────────────────────────────
    printf "  SLAM\n"
    FLIO_STATE=$(systemctl is-active bess-fast-lio 2>&1)
    GLIM_STATE=$(systemctl is-active bess-glim 2>&1)
    GLIM_GAPS=""
    GLIM_AGE=""
    GLIM_STALE=false
    if [ "$GLIM_STATE" = "active" ]; then
        GLIM_GAPS=$(sudo podman logs --since 60s bess-glim 2>&1 | grep -c '\[GAP\]' || true)
        GLIM_UP=$(sudo podman ps --format '{{.Names}} {{.RunningFor}}' 2>/dev/null | grep bess-glim | awk '{$1=""; print $0}' | sed 's/^ //')
        GLIM_AGE="up ${GLIM_UP}"
        # Check if GLIM is actually processing (not stuck on backlog/stale)
        GLIM_RECENT=$(sudo podman logs --since 30s bess-glim 2>&1)
        GLIM_BACKLOG=$(echo "$GLIM_RECENT" | grep -oP 'frame backlog: \K\d+' | tail -1)
        GLIM_DROPPED=$(echo "$GLIM_RECENT" | grep -c 'DROPPED stale' || true)
        GLIM_WAITING=$(echo "$GLIM_RECENT" | grep -c 'waiting for IMU' || true)
        if [ -n "$GLIM_BACKLOG" ] && (( GLIM_BACKLOG > 100 )); then
            GLIM_STALE=true
            add_issue "GLIM stuck: ${GLIM_BACKLOG} frame backlog"
        fi
        if (( GLIM_DROPPED > 10 )); then
            GLIM_STALE=true
            add_issue "GLIM dropping frames: ${GLIM_DROPPED} dropped in 30s"
        fi
    fi

    FLIO_STALE=false
    if [ "$FLIO_STATE" = "active" ]; then
        FLIO_RECENT=$(sudo podman logs --since 30s bess-fast-lio 2>&1 | tail -5)
        FLIO_LAST_TS=$(echo "$FLIO_RECENT" | grep -oP '\[INFO\].*fastlio' | tail -1)
        if [ -z "$FLIO_LAST_TS" ]; then
            # No INFO logs in 30s — check if it logged anything at all recently
            FLIO_ANY=$(sudo podman logs --since 60s bess-fast-lio 2>&1 | wc -l)
            if (( FLIO_ANY < 2 )); then
                # Check vehicle speed — if stationary, stale SLAM is expected
                SPEED=$(mosquitto_sub -h localhost -t 'bess/nav/position' -C 1 -W 3 2>/dev/null | python3 -c "import sys,json; d=json.load(sys.stdin); print(d.get('speed_ms',0))" 2>/dev/null || echo "0")
                if [ "${SPEED%%.*}" = "0" ] 2>/dev/null; then
                    FLIO_STALE=true
                    # Stationary — not an issue, just note it
                else
                    FLIO_STALE=true
                    add_warning "FAST-LIO silent for 60s while vehicle moving"
                fi
            fi
        fi
    fi

    printf "    FAST-LIO: %s" "$FLIO_STATE"
    $FLIO_STALE && printf " (STALE)"
    echo
    printf "    GLIM:     %s  gaps=%s  %s" "$GLIM_STATE" "${GLIM_GAPS:-?}" "${GLIM_AGE}"
    $GLIM_STALE && printf " (STALE)"
    echo

    # Heading offset from extraction — check freshness
    HEADING_LOG=$(sudo journalctl -u bess-extraction --since "-30s" --no-pager 2>&1)
    HEADING=$(echo "$HEADING_LOG" | grep -oP 'Heading offset CONVERGED.*?°' | tail -1)
    HEADING_RESETS=$(echo "$HEADING_LOG" | grep -c 'Heading offset RESET' || true)
    if [ -n "$HEADING" ]; then
        printf "    Heading:  %s (live)" "$HEADING"
        (( HEADING_RESETS > 0 )) && printf "  (%d resets in 30s)" "$HEADING_RESETS"
        echo
    else
        # Check if converged but stale (>30s ago)
        HEADING_OLD=$(sudo journalctl -u bess-extraction --since "-5min" --no-pager 2>&1 | \
            grep -oP 'Heading offset CONVERGED.*?°' | tail -1)
        if [ -n "$HEADING_OLD" ]; then
            printf "    Heading:  %s (STALE >30s)\n" "$HEADING_OLD"
            add_warning "Heading converged but stale (>30s since last update)"
        else
            printf "    Heading:  not converged\n"
        fi
    fi

    echo
fi

# ─── RECORDING ───────────────────────────────────────────────────
printf "  RECORDING\n"
BAG_CACHE="/var/mnt/nvme1/bag-cache"
REC_STATE=$(systemctl is-active bess-recorder 2>&1)
if [ "$REC_STATE" = "active" ] && [ -d "$BAG_CACHE" ]; then
    LATEST_BAG=$(ls -td "$BAG_CACHE"/bess_* 2>/dev/null | head -1)
    if [ -n "$LATEST_BAG" ]; then
        bag_name=$(basename "$LATEST_BAG")
        mcap_count=$(ls "$LATEST_BAG"/*.mcap 2>/dev/null | wc -l)
        bag_size=$(du -sh "$LATEST_BAG" 2>/dev/null | cut -f1)
        NEWEST_MCAP=$(ls -t "$LATEST_BAG"/*.mcap 2>/dev/null | head -1)
        if [ -n "$NEWEST_MCAP" ]; then
            MCAP_AGE=$(( $(date +%s) - $(stat -c %Y "$NEWEST_MCAP") ))
            printf "    Bag:      %s (%d mcaps, %s, last write %ds ago)\n" "$bag_name" "$mcap_count" "$bag_size" "$MCAP_AGE"
            (( MCAP_AGE > 120 )) && add_warning "Recorder stale: ${MCAP_AGE}s since last write"
        else
            printf "    Bag:      %s (%d mcaps, %s)\n" "$bag_name" "$mcap_count" "$bag_size"
        fi
    fi
    # Recorder memory (check if near limit)
    REC_MEM=$(sudo podman stats --no-stream --format '{{.MemUsage}}' bess-recorder 2>/dev/null | awk -F/ '{print $1}' | tr -d ' ')
    [ -n "$REC_MEM" ] && printf "    Memory:   %s (limit 16GiB)\n" "$REC_MEM"
else
    printf "    Recorder: %s\n" "$REC_STATE"
    $HEAD_CONNECTED && add_issue "Recorder not active with head connected"
fi

# ─── BAG HZ VALIDATION ─────────────────────────────────────────
# Sample a completed MCAP file and check actual recorded rates
LATEST_BAG=${LATEST_BAG:-$(ls -td /var/mnt/nvme2/bag-cache/bess_* 2>/dev/null | head -1)}
if [ -n "${LATEST_BAG:-}" ] && [ -d "${LATEST_BAG:-}" ]; then
        # Pick second-to-last MCAP (latest may still be writing)
        COMPLETED_MCAP=$(ls -t "$LATEST_BAG"/*.mcap 2>/dev/null | sed -n '2p')
        if [ -n "$COMPLETED_MCAP" ]; then
            BAG_HZ=$(python3 -c "
import sys
try:
    from mcap.reader import make_reader
    with open('$COMPLETED_MCAP', 'rb') as f:
        r = make_reader(f)
        s = r.get_summary()
        if not s or not s.statistics: sys.exit(0)
        dur = (s.statistics.message_end_time - s.statistics.message_start_time) / 1e9
        if dur < 5: sys.exit(0)
        targets = {
            '/camera2/camera_driver/image_masked/compressed': ('RGB_R', 5, 3.5),
            '/camera3/camera_driver/image_masked/compressed': ('RGB_L', 5, 3.5),
            '/thermal1/camera_driver/image_raw': ('Therm1', 15, 12),
            '/thermal2/camera_driver/image_raw': ('Therm2', 15, 12),
            '/ouster/points': ('Ouster', 10, 8),
            '/imu/data': ('IMU', 500, 100),
        }
        for ch_id, count in s.statistics.channel_message_counts.items():
            topic = s.channels[ch_id].topic
            if topic in targets:
                name, target, minimum = targets[topic]
                hz = count / dur
                flag = 'OK' if hz >= minimum else 'LOW'
                print(f'{name}={hz:.1f}/{target}={flag}')
                del targets[topic]
        for topic, (name, target, minimum) in targets.items():
            print(f'{name}=0/{target}=MISSING')
except Exception as e:
    print(f'ERR={e}')
" 2>&1)
            if [ -n "$BAG_HZ" ]; then
                printf "    Bag Hz:   "
                BAG_OK=true
                for entry in $BAG_HZ; do
                    name=$(echo "$entry" | cut -d= -f1)
                    rest=$(echo "$entry" | cut -d= -f2-)
                    hz=$(echo "$rest" | cut -d/ -f1)
                    status=$(echo "$rest" | rev | cut -d= -f1 | rev)
                    if [ "$status" = "LOW" ] || [ "$status" = "MISSING" ]; then
                        printf "%s:%s(%s) " "$name" "$hz" "$status"
                        BAG_OK=false
                        add_issue "Bag Hz ${name} ${status}: ${hz}Hz"
                    else
                        printf "%s:%s " "$name" "$hz"
                    fi
                done
                echo
                $BAG_OK || add_warning "Bag Hz anomalies detected — check recorder"
            fi
        fi
    fi

echo

# ─── EXTRACTION ──────────────────────────────────────────────────
printf "  EXTRACTION\n"
EXTRACT_DIR="/var/mnt/nvme2/extraction"
if [ -d "$EXTRACT_DIR" ]; then
    latest_link=$(readlink "$EXTRACT_DIR/latest" 2>/dev/null)
    if [ -n "$latest_link" ]; then
        latest_dir="$EXTRACT_DIR/$latest_link"
        # Count assets from latest flush log
        FLUSH=$(sudo journalctl -u bess-extraction --since "-60s" --no-pager 2>&1 | \
            grep 'Flush:' | tail -1)
        if [ -n "$FLUSH" ]; then
            frames=$(echo "$FLUSH" | grep -oP '\d+ frames' | head -1)
            uprns=$(echo "$FLUSH" | grep -oP '\d+ UPRNs' | head -1)
            printf "    Session:  %s\n" "$latest_link"
            printf "    Live:     %s, %s\n" "${frames:-?}" "${uprns:-0 UPRNs}"
        else
            printf "    Session:  %s (no flush in last 60s)\n" "$latest_link"
        fi
    fi

    # Spatial DB
    SPATIAL=$(sudo podman logs bess-extraction 2>&1 | grep -oP 'spatial_db=(yes|no)' | tail -1 | cut -d= -f2)
    printf "    UPRN DB:  %s\n" "${SPATIAL:-unknown}"
    [ "$SPATIAL" = "no" ] && add_warning "Spatial DB not loaded"

    # Session count (compact)
    total=0; empty=0
    for d in "$EXTRACT_DIR"/session_bess_*; do
        [ -d "$d" ] || continue
        total=$((total + 1))
        ic=$(find "$d/images" -maxdepth 2 -name '*.jpg' 2>/dev/null | head -1)
        [ -z "$ic" ] && [ ! -f "$d/extraction_summary.json" ] && empty=$((empty + 1))
    done
    printf "    Sessions: %d total" "$total"
    (( empty > 0 )) && printf " (%d empty)" "$empty"
    echo
fi

echo

# ─── DISK (compact) ──────────────────────────────────────────────
printf "  STORAGE\n"
NVME_FREE=""
for mp in /var/mnt/nvme1 /var/mnt/nvme2; do
    if mountpoint -q "$mp" 2>/dev/null; then
        avail=$(df -BG "$mp" --output=avail | tail -1 | tr -d ' G')
        used_pct=$(df "$mp" --output=pcent | tail -1 | tr -d ' %')
        label=$(basename "$mp")
        printf "    %-10s %3s%% used  %sG free\n" "$label" "$used_pct" "$avail"
        [ -z "$NVME_FREE" ] && NVME_FREE="$avail"
        (( avail < 500 )) && add_warning "NVMe ${mp} low: ${avail}G free"
        (( avail < 200 )) && add_issue "NVMe ${mp} CRITICAL: ${avail}G free"
    fi
done

# SSD fleet — compact summary
ssd_mounted=0; ssd_used=0; ssd_empty=0; ssd_unmounted=0; ssd_sealed=0
for letter in a b c d e f g h i j k l; do
    mp="/var/mnt/ssd-${letter}"
    if mountpoint -q "$mp" 2>/dev/null; then
        ssd_mounted=$((ssd_mounted + 1))
        used_pct=$(df "$mp" --output=pcent | tail -1 | tr -d ' %')
        if (( used_pct >= 95 )); then
            ssd_sealed=$((ssd_sealed + 1))
        elif (( used_pct > 1 )); then
            ssd_used=$((ssd_used + 1))
        else
            ssd_empty=$((ssd_empty + 1))
        fi
    else
        ssd_unmounted=$((ssd_unmounted + 1))
        add_issue "ssd-${letter} not mounted"
    fi
done
printf "    SSDs      %d/%d mounted" "$ssd_mounted" "12"
(( ssd_used > 0 )) && printf "  %d active" "$ssd_used"
(( ssd_empty > 0 )) && printf "  %d empty" "$ssd_empty"
(( ssd_sealed > 0 )) && printf "  %d sealed" "$ssd_sealed"
(( ssd_unmounted > 0 )) && printf "  %d MISSING" "$ssd_unmounted"
echo

# Show active SSDs with usage
for letter in a b c d e f g h i j k l; do
    mp="/var/mnt/ssd-${letter}"
    if mountpoint -q "$mp" 2>/dev/null; then
        used_pct=$(df "$mp" --output=pcent | tail -1 | tr -d ' %')
        if (( used_pct > 1 )); then
            avail=$(df -BG "$mp" --output=avail | tail -1 | tr -d ' G')
            printf "              ssd-%s: %3s%% used  %sG free\n" "$letter" "$used_pct" "$avail"
        fi
    fi
done

echo

# ─── ERRORS (filtered — skip known benign) ───────────────────────
printf "  ERRORS (1h)\n"
err_count=0
for ctr in bess-cameras bess-ouster bess-recorder bess-extraction bess-fast-lio bess-glim; do
    # Filter known benign: recorder unknown type warnings, GLIM GAP warnings
    logs=$(sudo podman logs --since 1h "$ctr" 2>&1)
    errs=$(echo "$logs" | grep -ci 'error\|fail\|crash\|panic\|segfault\|oom' || true)
    errs=${errs:-0}
    if [ "$ctr" = "bess-recorder" ]; then
        noise=$(echo "$logs" | grep -ci 'unknown type\|more than one type' || true)
        noise=${noise:-0}
        errs=$((errs - noise))
        (( errs < 0 )) && errs=0
    fi
    if [ "$ctr" = "bess-glim" ]; then
        noise=$(echo "$logs" | grep -ci '\[GAP\]\|Failed to lookup transform' || true)
        noise=${noise:-0}
        errs=$((errs - noise))
        (( errs < 0 )) && errs=0
    fi
    if (( errs > 0 )); then
        short=$(echo "$ctr" | sed 's/^bess-//')
        printf "    %-14s %d\n" "$short" "$errs"
        err_count=$((err_count + errs))
    fi
done
if (( err_count == 0 )); then
    printf "    None\n"
else
    (( err_count > 100 )) && add_warning "High error count: $err_count in last hour"
fi

echo

# ─── SUMMARY ─────────────────────────────────────────────────────
if (( ${#ISSUES[@]} > 0 )) || (( ${#WARNINGS[@]} > 0 )); then
    printf "  ─────────────────────────────────────────────\n"
    for i in "${ISSUES[@]+"${ISSUES[@]}"}"; do
        [ -n "$i" ] && printf "  ✗ %s\n" "$i"
    done
    for w in "${WARNINGS[@]+"${WARNINGS[@]}"}"; do
        [ -n "$w" ] && printf "  ⚠ %s\n" "$w"
    done
    printf "  ─────────────────────────────────────────────\n"
else
    printf "  ───────── ALL CLEAR ─────────\n"
fi
echo

# ─── JSON OUTPUT ─────────────────────────────────────────────────
STALE=$(sudo podman ps -a --filter status=exited --format '{{.Names}}' 2>/dev/null | wc -l)
cat > "$JSON" <<JSONEOF
{
  "vehicle": "$VEHICLE",
  "timestamp": "$TS",
  "head_connected": $HEAD_CONNECTED,
  "load_1m": $LOAD,
  "mem_used_gb": $MEM_USED,
  "mem_total_gb": $MEM_TOTAL,
  "nvme_free_gb": ${NVME_FREE:-0},
  "failed_prod_services": $FAILED_PROD,
  "stale_containers": $STALE,
  "issues": ${#ISSUES[@]},
  "warnings": ${#WARNINGS[@]}
}
JSONEOF
