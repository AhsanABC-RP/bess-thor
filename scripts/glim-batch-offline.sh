#!/bin/bash
##############################################################################
# BESS GLIM Batch Offline — Process bags with proper mcap ordering and
# gap-aware session splitting.
#
# Sorts mcaps numerically (not string order), groups contiguous mcaps into
# sessions (split at gaps >2s), runs GLIM offline per session in dense mode.
#
# Usage:
#   glim-batch-offline.sh <bag_dir> <output_dir> [--standard]
#
# Output per session:
#   <output_dir>/session_NNN/dump/        — GLIM submaps
#   <output_dir>/session_NNN/glim_map.laz — merged point cloud
#   <output_dir>/session_NNN/traj_*.txt   — trajectories
#
# Default mode is DENSE (max point density). Use --standard for faster/lighter.
##############################################################################
set -eo pipefail

BAG_DIR="${1:?Usage: $0 <bag_dir> <output_dir> [--standard]}"
OUTPUT_DIR="${2:?Usage: $0 <bag_dir> <output_dir> [--standard]}"
DENSE="dense"
[[ "${3:-}" == "--standard" ]] && DENSE="standard"
[[ "${3:-}" == "--ultradense" ]] && DENSE="ultradense"
[[ "${3:-}" == "--highdensity" ]] && DENSE="highdensity"

BAG_DIR=$(realpath "$BAG_DIR")
OUTPUT_DIR=$(realpath -m "$OUTPUT_DIR")
BAG_NAME=$(basename "$BAG_DIR")
GLIM_IMAGE="localhost/bess-glim:humble"
GAP_THRESHOLD=2.0  # seconds — split sessions at gaps larger than this

if [[ ! -d "$BAG_DIR" ]]; then
    echo "ERROR: Bag directory not found: $BAG_DIR"
    exit 1
fi

mkdir -p "$OUTPUT_DIR"

echo "=== GLIM Batch Offline ==="
echo "Bag:    $BAG_DIR"
echo "Output: $OUTPUT_DIR"
echo "Mode:   $DENSE"
echo ""

# Step 1: Group contiguous mcaps into sessions using Python
echo "Analyzing mcap timestamps and grouping sessions..."
python3 << PYEOF
import glob, os, re, json, sys
from mcap.reader import make_reader

bag_dir = "$BAG_DIR"
output_dir = "$OUTPUT_DIR"
gap_threshold = $GAP_THRESHOLD

def get_idx(f):
    m = re.search(r'_(\d+)\.mcap$', f)
    return int(m.group(1)) if m else 0

mcaps = sorted(glob.glob(os.path.join(bag_dir, '*.mcap')), key=get_idx)
if not mcaps:
    print("ERROR: No mcap files found")
    sys.exit(1)

print(f"  Found {len(mcaps)} mcaps (numeric order)")

# Get timestamps for each mcap
entries = []
for mcap in mcaps:
    try:
        with open(mcap, 'rb') as f:
            r = make_reader(f)
            s = r.get_summary()
            entries.append({
                'path': mcap,
                'name': os.path.basename(mcap),
                'start_ns': s.statistics.message_start_time,
                'end_ns': s.statistics.message_end_time,
                'msgs': sum(s.statistics.channel_message_counts.values()),
            })
    except Exception as e:
        print(f"  SKIP {os.path.basename(mcap)}: {e}")

# Group into sessions by gap threshold
sessions = []
current_session = [entries[0]]
for i in range(1, len(entries)):
    gap_s = (entries[i]['start_ns'] - entries[i-1]['end_ns']) / 1e9
    if gap_s > gap_threshold:
        sessions.append(current_session)
        print(f"  GAP {gap_s:.1f}s after {entries[i-1]['name']} — new session")
        current_session = [entries[i]]
    else:
        current_session.append(entries[i])
sessions.append(current_session)

print(f"  Split into {len(sessions)} sessions")
for i, sess in enumerate(sessions):
    duration_s = (sess[-1]['end_ns'] - sess[0]['start_ns']) / 1e9
    total_msgs = sum(e['msgs'] for e in sess)
    print(f"    Session {i}: {len(sess)} mcaps, {duration_s:.0f}s, {total_msgs} msgs")

# Save session plan
plan = []
for i, sess in enumerate(sessions):
    plan.append({
        'session_idx': i,
        'mcaps': [e['path'] for e in sess],
        'mcap_names': [e['name'] for e in sess],
        'start_ns': sess[0]['start_ns'],
        'end_ns': sess[-1]['end_ns'],
        'duration_s': (sess[-1]['end_ns'] - sess[0]['start_ns']) / 1e9,
        'total_msgs': sum(e['msgs'] for e in sess),
    })

with open(os.path.join(output_dir, 'session_plan.json'), 'w') as f:
    json.dump(plan, f, indent=2)
PYEOF

if [[ $? -ne 0 ]]; then
    echo "ERROR: Session planning failed"
    exit 1
fi

# Step 2: Process each session
PLAN_FILE="$OUTPUT_DIR/session_plan.json"
N_SESSIONS=$(python3 -c "import json; print(len(json.load(open('$PLAN_FILE'))))")

echo ""
echo "Processing $N_SESSIONS sessions..."

# Select config
case "$DENSE" in
    highdensity) CONFIG_FILE="config_offline_highdensity.json" ;;
    ultradense)  CONFIG_FILE="config_offline_ultradense.json" ;;
    dense)       CONFIG_FILE="config_offline_dense.json" ;;
    standard)    CONFIG_FILE="config_offline.json" ;;
esac

TOTAL_START=$(date +%s)

for SESSION_IDX in $(seq 0 $((N_SESSIONS - 1))); do
    SESSION_DIR="$OUTPUT_DIR/session_$(printf '%03d' $SESSION_IDX)"

    # Check if already processed
    if [[ -f "$SESSION_DIR/glim_map.laz" ]]; then
        echo "[$((SESSION_IDX+1))/$N_SESSIONS] SKIP session_$(printf '%03d' $SESSION_IDX) — already processed"
        continue
    fi

    mkdir -p "$SESSION_DIR"

    # Get mcap list for this session
    MCAP_LIST=$(python3 -c "
import json
plan = json.load(open('$PLAN_FILE'))
sess = plan[$SESSION_IDX]
print(' '.join(sess['mcaps']))
")
    DURATION=$(python3 -c "
import json
plan = json.load(open('$PLAN_FILE'))
print(f\"{plan[$SESSION_IDX]['duration_s']:.0f}\")
")
    N_MCAPS=$(echo $MCAP_LIST | wc -w)

    echo ""
    echo "[$((SESSION_IDX+1))/$N_SESSIONS] Session $SESSION_IDX: $N_MCAPS mcaps, ${DURATION}s"

    # Create temp bag dir with symlinked mcaps.
    # We mount the source SSD into the container so symlinks resolve.
    TEMP_BAG="/tmp/glim_session_bag"
    sudo rm -rf "$TEMP_BAG"
    mkdir -p "$TEMP_BAG"

    # Determine the SSD mount(s) needed for this session's mcaps
    SSD_MOUNTS=""
    unset SSD_SEEN 2>/dev/null; declare -A SSD_SEEN
    for mcap in $MCAP_LIST; do
        ssd_mount=$(df --output=target "$mcap" | tail -1)
        if [[ -z "${SSD_SEEN[$ssd_mount]:-}" ]]; then
            SSD_MOUNTS="$SSD_MOUNTS -v $ssd_mount:$ssd_mount:ro"
            SSD_SEEN[$ssd_mount]=1
        fi
        ln -s "$mcap" "$TEMP_BAG/$(basename "$mcap")"
    done

    # Rebuild metadata for this subset
    python3 /opt/bess/scripts/rebuild-bag-metadata.py "$TEMP_BAG" 2>/dev/null

    if [[ ! -f "$TEMP_BAG/metadata.yaml" ]]; then
        echo "  ERROR: metadata rebuild failed"
        continue
    fi

    # Run GLIM offline
    START_TIME=$(date +%s)

    CONTAINER_CMD=(
        sudo podman run --rm
        --name bess-glim-offline
        --network=none
        --device nvidia.com/gpu=1
        --security-opt label=disable
        --shm-size=8gb
        --memory=24G
        --cpuset-cpus=6-17
        --ulimit "stack=4294967296:4294967296"
        -e "GLIM_CONFIG=/ros2_ws/config/$CONFIG_FILE"
        -e "GLIM_SENSOR_CONFIG=/ros2_ws/config/config_ouster.json"
        -e "CUDA_VISIBLE_DEVICES=0"
        -e "NVIDIA_VISIBLE_DEVICES=1"
        -e "NVIDIA_DRIVER_CAPABILITIES=compute,utility"
        -e "SPDLOG_LEVEL=info"
        -v "/opt/bess/config/glim:/ros2_ws/config:ro"
        -v "/opt/bess/config/glim/config_ros_offline.json:/ros2_ws/install/glim/share/glim/config/config_ros.json:ro"
        -v "/opt/bess/config/glim/config_sensors.json:/ros2_ws/install/glim/share/glim/config/config_sensors.json:ro"
        -v "/opt/bess/config/glim/config_odometry_gpu.json:/ros2_ws/install/glim/share/glim/config/config_odometry_gpu.json:ro"
    )

    # Highdensity overrides sub_mapping — don't mount the default one
    if [[ "$DENSE" != "highdensity" ]]; then
        CONTAINER_CMD+=(-v "/opt/bess/config/glim/config_sub_mapping_gpu.json:/ros2_ws/install/glim/share/glim/config/config_sub_mapping_gpu.json:ro")
    fi

    case "$DENSE" in
        highdensity)
            CONTAINER_CMD+=(
                -v "/opt/bess/config/glim/${CONFIG_FILE}:/ros2_ws/install/glim/share/glim/config/config_offline.json:ro"
                -v "/opt/bess/config/glim/config_preprocess_highdensity.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro"
                -v "/opt/bess/config/glim/config_global_mapping_highdensity.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro"
                -v "/opt/bess/config/glim/config_sub_mapping_highdensity.json:/ros2_ws/install/glim/share/glim/config/config_sub_mapping_gpu.json:ro"
            ) ;;
        ultradense)
            CONTAINER_CMD+=(
                -v "/opt/bess/config/glim/${CONFIG_FILE}:/ros2_ws/install/glim/share/glim/config/config_offline.json:ro"
                -v "/opt/bess/config/glim/config_preprocess_ultradense.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro"
                -v "/opt/bess/config/glim/config_global_mapping_ultradense.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro"
            ) ;;
        dense)
            CONTAINER_CMD+=(
                -v "/opt/bess/config/glim/${CONFIG_FILE}:/ros2_ws/install/glim/share/glim/config/config_offline.json:ro"
                -v "/opt/bess/config/glim/config_preprocess_dense.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro"
                -v "/opt/bess/config/glim/config_global_mapping_dense.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro"
            ) ;;
        standard)
            CONTAINER_CMD+=(
                -v "/opt/bess/config/glim/config_preprocess.json:/ros2_ws/install/glim/share/glim/config/config_preprocess.json:ro"
                -v "/opt/bess/config/glim/config_global_mapping_gpu.json:/ros2_ws/install/glim/share/glim/config/config_global_mapping_gpu.json:ro"
            ) ;;
    esac

    CONTAINER_CMD+=(
        -v "${TEMP_BAG}:/bag:ro"
        $SSD_MOUNTS
        -v "${SESSION_DIR}:/output:rw"
        "$GLIM_IMAGE"
        bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && \
            cd /output && \
            /ros2_ws/install/glim_ros/lib/glim_ros/glim_rosbag /bag \
            --ros-args -p config_file:=/ros2_ws/config/$CONFIG_FILE \
            -p sensor_config_file:=/ros2_ws/config/config_ouster.json \
            -p dump_path:=/output/dump \
            -p auto_quit:=true"
    )

    sudo podman rm -f bess-glim-offline 2>/dev/null || true

    "${CONTAINER_CMD[@]}" > "$SESSION_DIR/glim.log" 2>&1
    GLIM_EXIT=$?

    END_TIME=$(date +%s)
    ELAPSED=$((END_TIME - START_TIME))

    if [[ $GLIM_EXIT -ne 0 ]]; then
        echo "  GLIM FAILED (exit $GLIM_EXIT, ${ELAPSED}s) — see $SESSION_DIR/glim.log"
        tail -3 "$SESSION_DIR/glim.log"
        continue
    fi

    # Merge submaps to LAZ
    if [[ -d "$SESSION_DIR/dump" ]]; then
        SUBMAP_COUNT=$(ls -d "$SESSION_DIR/dump/"[0-9]* 2>/dev/null | wc -l)
        echo "  GLIM OK: $SUBMAP_COUNT submaps in ${ELAPSED}s"

        python3 /opt/bess/scripts/glim-merge-submaps.py \
            "$SESSION_DIR/dump" "$SESSION_DIR/glim_map.laz" 2>/dev/null && \
            echo "  Merged to glim_map.laz ($(du -h "$SESSION_DIR/glim_map.laz" | cut -f1))" || \
            echo "  Merge failed — submaps still in dump/"
    else
        echo "  WARNING: No dump directory produced"
    fi

    # Fix output permissions and clean up temp bag
    sudo chown -R "$(id -u):$(id -g)" "$SESSION_DIR" 2>/dev/null
    sudo rm -rf "$TEMP_BAG"
done

TOTAL_END=$(date +%s)
TOTAL_ELAPSED=$((TOTAL_END - TOTAL_START))

echo ""
echo "=== GLIM Batch Complete ==="
echo "Bag: $BAG_NAME"
echo "Sessions: $N_SESSIONS"
echo "Total time: ${TOTAL_ELAPSED}s ($((TOTAL_ELAPSED/60))m)"
echo "Output: $OUTPUT_DIR"

# Summary
for sd in "$OUTPUT_DIR"/session_*/; do
    idx=$(basename "$sd")
    if [[ -f "$sd/glim_map.laz" ]]; then
        sz=$(du -h "$sd/glim_map.laz" | cut -f1)
        echo "  $idx: $sz"
    elif [[ -f "$sd/glim.log" ]]; then
        echo "  $idx: FAILED"
    fi
done
