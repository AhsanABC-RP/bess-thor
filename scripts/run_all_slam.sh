#!/bin/bash
# run_all_slam.sh — bulk SLAM + LAS export + viz across (engine × segment).
#
# Segments are declared on the CLI as start:end:tag tuples. For each engine
# in ENGINES (default: fastlio dlio glim) and each segment, the script:
#   1. Skips if <slam_offline>/<engine>_<tag>/slam_output/.../*.mcap exists
#      AND --refresh is not set.
#   2. Runs scripts/offline_slam_<engine>.sh <start> <end> <rate> <tag>.
#   3. Runs scripts/offline_slam_to_las.py on the output bag.
#   4. Runs scripts/slam_viz.py on the output dir.
#   5. Records wall time + exit code to runs.csv in the SLAM_OFFLINE dir.
#
# Usage:
#   bash scripts/run_all_slam.sh [--refresh] [--engines="fastlio dlio glim"] \
#        SEG [SEG ...]
#
#   SEG is start:end:tag, e.g. 16:22:drive1 or 40:45:bags40to45.
#
# Examples:
#   bash scripts/run_all_slam.sh 16:22:drive1
#   bash scripts/run_all_slam.sh --engines="fastlio dlio" 0:0:static_1bag 40:45:highway

set -euo pipefail

REFRESH=0
ENGINES="fastlio dlio glim"
VOXEL=0.05
RATE=1.0
MAX_SPEED=35.0
MAX_EXTENT=3000.0
BESS_ROOT="/home/thor/bess"
SLAM_OFFLINE="/mnt/bess-usb/bags/rolling/slam_offline"
RUNS_CSV="$SLAM_OFFLINE/runs.csv"

usage() {
    grep '^#' "$0" | head -30
    exit 1
}

SEGS=()
while [ $# -gt 0 ]; do
    case "$1" in
        --refresh) REFRESH=1; shift ;;
        --engines=*) ENGINES="${1#--engines=}"; shift ;;
        --engines) ENGINES="$2"; shift 2 ;;
        --voxel=*) VOXEL="${1#--voxel=}"; shift ;;
        --voxel) VOXEL="$2"; shift 2 ;;
        --max-speed=*) MAX_SPEED="${1#--max-speed=}"; shift ;;
        --max-speed) MAX_SPEED="$2"; shift 2 ;;
        --help|-h) usage ;;
        *:*:*) SEGS+=("$1"); shift ;;
        *) echo "bad arg: $1" >&2; usage ;;
    esac
done

[ ${#SEGS[@]} -eq 0 ] && { echo "ERROR: need at least one SEG start:end:tag" >&2; exit 1; }

mkdir -p "$SLAM_OFFLINE"
[ -f "$RUNS_CSV" ] || echo "timestamp,engine,tag,start,end,phase,wall_s,exit,verdict" > "$RUNS_CSV"

log_run() {
    local engine="$1" tag="$2" start="$3" end="$4" phase="$5" wall="$6" rc="$7" verdict="${8:-}"
    echo "$(date -u +%FT%TZ),$engine,$tag,$start,$end,$phase,$wall,$rc,$verdict" >> "$RUNS_CSV"
}

odom_topic_for() {
    case "$1" in
        fastlio) echo "/fast_lio/odometry" ;;
        dlio) echo "/dlio/odom_node/odom" ;;
        glim) echo "/glim_ros/odom_corrected" ;;
        *) echo ""; return 1 ;;
    esac
}

cloud_topic_for() {
    case "$1" in
        fastlio) echo "/ouster/points" ;;                    # raw points, exporter SLERPs
        dlio) echo "/dlio/odom_node/deskewed" ;;               # world-frame
        glim) echo "/glim_ros/aligned_points_corrected" ;;     # world-frame
        *) echo ""; return 1 ;;
    esac
}

extrinsic_for() {
    case "$1" in
        fastlio) echo "fast_lio" ;;
        dlio) echo "dlio" ;;                                  # not used for deskewed
        glim) echo "identity" ;;                              # not used for world-frame
        *) echo ""; return 1 ;;
    esac
}

run_segment() {
    local engine="$1" start="$2" end="$3" tag="$4"
    local run_dir="$SLAM_OFFLINE/${engine}_${tag}"
    local slam_out="$run_dir/slam_output/slam_output"

    echo ""
    echo "=========================================================="
    echo "  $engine  bags ${start}-${end}  tag=${tag}"
    echo "=========================================================="

    local script="$BESS_ROOT/scripts/offline_slam_${engine}.sh"
    [ -x "$script" ] || { echo "  ERROR: $script missing"; log_run "$engine" "$tag" "$start" "$end" "slam" 0 127; return; }

    local has_output=0
    if compgen -G "$slam_out/*.mcap" > /dev/null 2>&1; then
        has_output=1
    fi

    if [ $has_output -eq 1 ] && [ $REFRESH -eq 0 ]; then
        echo "  [slam] SKIP (output exists — use --refresh to force)"
    else
        local t0=$(date +%s)
        set +e
        timeout 1800 bash "$script" "$start" "$end" "$RATE" "$tag" > "$run_dir/logs/slam_stdout.log" 2>&1
        local rc=$?
        set -e
        local wall=$(( $(date +%s) - t0 ))
        if [ $rc -ne 0 ]; then
            echo "  [slam] FAIL rc=$rc (${wall}s) — see $run_dir/logs/slam_stdout.log"
            log_run "$engine" "$tag" "$start" "$end" "slam" "$wall" "$rc"
            return
        fi
        echo "  [slam] OK (${wall}s)"
        log_run "$engine" "$tag" "$start" "$end" "slam" "$wall" 0
    fi

    # LAS export
    local export_dir="$run_dir/export"
    if [ -f "$export_dir/site_trial.las" ] && [ $REFRESH -eq 0 ]; then
        echo "  [las]  SKIP (LAS exists)"
    else
        local odom_topic cloud_topic ext_preset
        odom_topic=$(odom_topic_for "$engine")
        cloud_topic=$(cloud_topic_for "$engine")
        ext_preset=$(extrinsic_for "$engine")
        local t0=$(date +%s)
        set +e
        timeout 1800 python3 "$BESS_ROOT/scripts/offline_slam_to_las.py" \
            "$SLAM_OFFLINE/raw_${tag}_link" \
            --odom-bag-dir "$slam_out" \
            --odom-topic "$odom_topic" \
            --cloud-topic "$cloud_topic" \
            --extrinsic-preset "$ext_preset" \
            --output "$export_dir" \
            --voxel "$VOXEL" \
            --max-speed "$MAX_SPEED" \
            --max-extent "$MAX_EXTENT" \
            --force \
            > "$run_dir/logs/las_export.log" 2>&1
        local rc=$?
        set -e
        local wall=$(( $(date +%s) - t0 ))
        local verdict="ERR"
        if [ -f "$export_dir/sanity.txt" ]; then
            verdict=$(awk -F': ' '/^verdict/{print $2; exit}' "$export_dir/sanity.txt" | tr -d ' ')
        fi
        if [ $rc -ne 0 ] && [ "$verdict" != "FAIL" ] && [ "$verdict" != "WARN" ] && [ "$verdict" != "PASS" ]; then
            echo "  [las]  FAIL rc=$rc (${wall}s) — see $run_dir/logs/las_export.log"
        else
            echo "  [las]  $verdict (${wall}s)"
        fi
        log_run "$engine" "$tag" "$start" "$end" "las" "$wall" "$rc" "$verdict"
    fi

    # Viz
    if [ -f "$run_dir/viz/overview.png" ] && [ $REFRESH -eq 0 ]; then
        echo "  [viz]  SKIP (overview.png exists)"
    else
        local t0=$(date +%s)
        set +e
        timeout 300 python3 "$BESS_ROOT/scripts/slam_viz.py" "$run_dir" \
            > "$run_dir/logs/viz.log" 2>&1
        local rc=$?
        set -e
        local wall=$(( $(date +%s) - t0 ))
        if [ $rc -ne 0 ]; then
            echo "  [viz]  FAIL rc=$rc (${wall}s)"
        else
            echo "  [viz]  OK (${wall}s)"
        fi
        log_run "$engine" "$tag" "$start" "$end" "viz" "$wall" "$rc"
    fi
}

# Also symlink raw bag range so offline_slam_to_las can read the raw /ouster/points.
# For world-frame cloud topics (DLIO deskewed, GLIM aligned_points) the raw link
# is unused and the exporter reads from --odom-bag-dir; we still point at a raw
# link so FAST-LIO can get raw /ouster/points.
ensure_raw_link() {
    local start="$1" end="$2" tag="$3"
    local link_dir="$SLAM_OFFLINE/raw_${tag}_link"
    if [ -d "$link_dir" ]; then return; fi
    mkdir -p "$link_dir"
    for i in $(seq "$start" "$end"); do
        local src="/mnt/bess-usb/bags/rolling/bag/bag_${i}.mcap"
        [ -f "$src" ] && ln -sf "$src" "$link_dir/bag_${i}.mcap"
    done
    # ros2 bag expects a metadata.yaml next to splits — copy from top-level
    if [ -f "/mnt/bess-usb/bags/rolling/bag/metadata.yaml" ]; then
        ln -sf "/mnt/bess-usb/bags/rolling/bag/metadata.yaml" "$link_dir/metadata.yaml"
    fi
}

for seg in "${SEGS[@]}"; do
    IFS=':' read -r START END TAG <<< "$seg"
    [ -z "$START" ] || [ -z "$END" ] || [ -z "$TAG" ] && { echo "bad seg: $seg" >&2; continue; }
    ensure_raw_link "$START" "$END" "$TAG"
    for engine in $ENGINES; do
        mkdir -p "$SLAM_OFFLINE/${engine}_${TAG}/logs"
        run_segment "$engine" "$START" "$END" "$TAG"
    done
done

echo ""
echo "ALL DONE. Summary:"
tail -n "$((${#SEGS[@]} * 9))" "$RUNS_CSV" | column -t -s,
