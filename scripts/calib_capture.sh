#!/usr/bin/env bash
# Calibration-data capture helper.
#
# Records a ROS 2 MCAP bag of the topics needed for one calibration session,
# then converts it to the ROS 1 .bag format that Kalibr expects.
#
# Usage:
#   bash scripts/calib_capture.sh <mode> <session_name> [--duration SECS]
#
# Modes:
#   cam-mono <ns>    — single camera intrinsics (e.g. "lucid2")
#   cam-stereo <A> <B> — LEFT IN AS CAPTURE-ONLY HELPER. Thor's opposing-side
#                      pairs have no FOV overlap, so kalibr_calibrate_cameras
#                      --multi-camera cannot solve an extrinsic between them.
#                      Only use this if you are pointing two genuinely
#                      co-bore-sighted cameras at one target (rare on Thor).
#   imu-static       — IMU static capture for allan_variance_ros (no motion, ≥ 3 h ideal)
#   imu-cam <ns>     — IMU + single camera for kalibr_calibrate_imu_camera
#   imu-multicam     — IMU + all cams in one bag. NOT for inter-camera extrinsics;
#                      handy when recording IMU calibration alongside multiple
#                      mono-intrinsic captures in a single drive so you don't
#                      have to stop/start between cameras.
#
# Output: ${CALIB_BASE:-/home/thor/calib}/<session_name>/
#   - raw.mcap      ROS 2 bag (rosbag2 MCAP storage)
#   - raw.bag       ROS 1 bag (for Kalibr, created by rosbags-convert)
#   - manifest.txt  topic list + durations + host info
#
# Defaults to /home/thor/calib on host NVMe (~250 GB free). RM110 USB drive
# (warm-reboot stuck-state, see project_thor_rm110_warm_reboot.md). Override
#
# Prereqs:
#   - Target container(s) publishing on the expected topic names (check with
#     `ros2 topic list` from inside any ROS 2 container).
#   - CALIB_BASE writable with ≥ 10 GB free per 120 s Lucid native-res session.

set -euo pipefail

MODE="${1:?mode required: cam-mono | cam-stereo | imu-static | imu-cam | imu-multicam}"
SESSION="${2:?session_name required}"
shift 2

DURATION=60  # default seconds; Kalibr cam caps typically 60–180 s, IMU static 10800+
while [[ $# -gt 0 ]]; do
    case "$1" in
        --duration) DURATION="$2"; shift 2 ;;
        *) POSARGS+=("$1"); shift ;;
    esac
done

CALIB_BASE="${CALIB_BASE:-/home/thor/calib}"
OUT_BASE="${CALIB_BASE}/${SESSION}"
if [[ "${CALIB_BASE}" == /home/thor/nas/* ]] && ! mountpoint -q /home/thor/nas/bess-bags; then
    echo "ERROR: CALIB_BASE=${CALIB_BASE} but F8 NAS not mounted. Aborting." >&2
    exit 2
fi
mkdir -p "${OUT_BASE}"
# Free-space guard: 10 GB/120s at Lucid native-res is the dominant budget.
FREE_GB="$(df -BG --output=avail "${CALIB_BASE}" | tail -1 | tr -dc '0-9')"
if [[ "${FREE_GB:-0}" -lt 15 ]]; then
    echo "ERROR: ${CALIB_BASE} has only ${FREE_GB}G free — need ≥ 15G. Aborting." >&2
    exit 2
fi

# Namespace → image topic + camera_info topic resolver.
# Different camera nodes use different namespacing conventions; this maps the
# short ns the user passes on the CLI to the actual topic the driver publishes.
ns_to_image_topic() {
    case "$1" in
        lucid1|lucid2)   echo "/${1}/camera_driver/image_raw" ;;
        blackfly1)       echo "/blackfly/camera1/blackfly_camera/image_raw" ;;
        blackfly2)       echo "/blackfly/camera2/blackfly_camera/image_raw" ;;
        thermal1|thermal2|thermal3|thermal4)
                         echo "/thermal/camera${1#thermal}/image_raw" ;;
        zed|zed2i)       echo "/zed/zed/rgb/color/rect/image" ;;
        *)               echo "/${1}/image_raw" ;;
    esac
}
ns_to_info_topic() {
    case "$1" in
        lucid1|lucid2)   echo "/${1}/camera_driver/camera_info" ;;
        blackfly1)       echo "/blackfly/camera1/blackfly_camera/camera_info" ;;
        blackfly2)       echo "/blackfly/camera2/blackfly_camera/camera_info" ;;
        thermal1|thermal2|thermal3|thermal4)
                         echo "/thermal/camera${1#thermal}/camera_info" ;;
        zed|zed2i)       echo "/zed/zed/rgb/camera_info" ;;
        *)               echo "/${1}/camera_info" ;;
    esac
}

# Calibration-mode override for Lucid Atlas (binning=1 @ native 9568x6376).
# See docker-compose.calib-lucid.yml for rationale. Auto-applied when any
# lucid* appears in the capture target; auto-restored via trap on exit.
BESS_ROOT="/home/thor/bess"
LUCID_OVERRIDE_ACTIVE=""
LUCID_TARGETS=()
apply_lucid_calib_override() {
    [[ ${#LUCID_TARGETS[@]} -eq 0 ]] && return 0
    echo "=== Applying Lucid calibration override (native resolution 9568x6376 @ 1 fps) ==="
    echo "  affected: ${LUCID_TARGETS[*]}"
    (cd "$BESS_ROOT" && \
        docker compose -f docker-compose.thor.yml -f docker-compose.calib-lucid.yml \
        up -d --force-recreate --no-deps "${LUCID_TARGETS[@]}")
    LUCID_OVERRIDE_ACTIVE=1
    echo "  waiting 45s for Arena SDK re-init at full resolution..."
    sleep 45
}
restore_lucid_defaults() {
    [[ -z "$LUCID_OVERRIDE_ACTIVE" ]] && return 0
    echo "=== Restoring Lucid default operating mode (binning=4 @ 3 fps) ==="
    (cd "$BESS_ROOT" && \
        docker compose -f docker-compose.thor.yml \
        up -d --force-recreate --no-deps "${LUCID_TARGETS[@]}") || \
        echo "WARN: lucid restore command returned nonzero. Run manually:" >&2
    LUCID_OVERRIDE_ACTIVE=""
}
trap restore_lucid_defaults EXIT INT TERM

# Topic selection per mode.
case "$MODE" in
    cam-mono)
        NS="${POSARGS[0]}"
        TOPICS=("$(ns_to_image_topic "$NS")" "$(ns_to_info_topic "$NS")")
        [[ "$NS" == lucid* ]] && LUCID_TARGETS+=("$NS")
        ;;
    cam-stereo)
        NS_A="${POSARGS[0]}"
        NS_B="${POSARGS[1]}"
        TOPICS=(
            "$(ns_to_image_topic "$NS_A")" "$(ns_to_info_topic "$NS_A")"
            "$(ns_to_image_topic "$NS_B")" "$(ns_to_info_topic "$NS_B")"
        )
        [[ "$NS_A" == lucid* ]] && LUCID_TARGETS+=("$NS_A")
        [[ "$NS_B" == lucid* ]] && LUCID_TARGETS+=("$NS_B")
        ;;
    imu-static)
        TOPICS=("/ouster/imu" "/imu/data")
        ;;
    imu-cam)
        NS="${POSARGS[0]}"
        TOPICS=("$(ns_to_image_topic "$NS")" "$(ns_to_info_topic "$NS")" "/ouster/imu" "/imu/data")
        [[ "$NS" == lucid* ]] && LUCID_TARGETS+=("$NS")
        ;;
    imu-multicam)
        TOPICS=(
            /blackfly/camera1/blackfly_camera/image_raw /blackfly/camera1/blackfly_camera/camera_info
            /blackfly/camera2/blackfly_camera/image_raw /blackfly/camera2/blackfly_camera/camera_info
            /lucid1/camera_driver/image_raw /lucid1/camera_driver/camera_info
            /lucid2/camera_driver/image_raw /lucid2/camera_driver/camera_info
            /thermal/camera1/image_raw /thermal/camera2/image_raw
            /thermal/camera3/image_raw /thermal/camera4/image_raw
            /zed/zed/rgb/color/rect/image
            /ouster/imu /imu/data
        )
        LUCID_TARGETS+=(lucid1 lucid2)
        ;;
    *) echo "Unknown mode: $MODE" >&2; exit 1 ;;
esac

# Apply Lucid calib-mode override if any lucid is in the target set.
apply_lucid_calib_override

echo "=== BESS calibration capture ==="
echo "  mode:     $MODE"
echo "  session:  $SESSION"
echo "  duration: ${DURATION}s"
echo "  output:   $OUT_BASE"
echo "  topics:"
printf '    %s\n' "${TOPICS[@]}"
echo

# Manifest.
{
    echo "mode: $MODE"
    echo "session: $SESSION"
    echo "duration_s: $DURATION"
    echo "host: $(hostname)"
    echo "utc: $(date -u +%FT%TZ)"
    echo "topics:"
    printf '  - %s\n' "${TOPICS[@]}"
} > "${OUT_BASE}/manifest.txt"

# Record via the foxglove container (it has ros2 CLI + CycloneDDS + QoS overrides).
# Runs on host network and domain 0 to match the live stack.
docker exec -i foxglove bash -lc "
    source /opt/ros/jazzy/setup.bash && \
    cd /tmp && \
    timeout ${DURATION}s ros2 bag record \
        --storage mcap \
        --output /tmp/calib_${SESSION} \
        --qos-profile-overrides-path /config/rosbag_qos.yaml \
        ${TOPICS[*]}
"

# Pull the bag out and convert to ROS 1.
docker cp "foxglove:/tmp/calib_${SESSION}" "${OUT_BASE}/raw_mcap_dir"
mv "${OUT_BASE}/raw_mcap_dir"/*.mcap "${OUT_BASE}/raw.mcap" || true
rm -rf "${OUT_BASE}/raw_mcap_dir"
docker exec -i foxglove rm -rf "/tmp/calib_${SESSION}"

echo "=== Converting MCAP → ROS1 .bag ==="
docker run --rm -v "${OUT_BASE}:/data" localhost/bess-calib-util:jazzy \
    bash -lc "rosbags-convert --src /data/raw.mcap --dst /data/raw.bag"

echo
echo "Done. Feed /data/raw.bag to kalibr (bind-mount ${OUT_BASE} into bess-calib-kalibr)."
