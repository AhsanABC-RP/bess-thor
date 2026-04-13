#!/usr/bin/env bash
# GLIM Offline SLAM — process MCAP bags directly with glim_rosbag
#
# Usage:
#   ./glim_offline.sh <bag_dir> [output_dir] [duration_sec]
#
# Example:
#   ./glim_offline.sh /var/mnt/ssd-e/bags/2026/03/11/bess_20260311_163616
#   ./glim_offline.sh /var/mnt/ssd-e/bags/2026/03/11/bess_20260311_163616 /tmp/my_output 120
#
# Requires:
#   - bess-glim:humble container image (with rosbag2_storage_mcap)
#   - metadata.yaml in the bag directory (auto-generated if missing)
#   - NVIDIA GPU (uses CDI device nvidia.com/gpu=0)
#
# Output:
#   <output_dir>/
#     000000/ 000001/ ... (submaps with points_compact.bin, data.txt)
#     traj_lidar.txt, traj_imu.txt (TUM format trajectories)
#     odom_lidar.txt, odom_imu.txt (pre-optimization odometry)
#     graph.bin, graph.txt (factor graph)
#
# Config:
#   Uses BESS offline configs from ~/bess_platform/config/glim/
#   Key: config_offline.json -> config_ros_offline.json (no viewer, no extension modules)

set -euo pipefail

BAG_DIR="${1:?Usage: $0 <bag_dir> [output_dir] [duration_sec]}"
OUTPUT_DIR="${2:-/tmp/glim_offline_$(date +%Y%m%d_%H%M%S)}"
DURATION="${3:-0}"  # 0 = full bag

REPO_DIR="$HOME/bess_platform"
CONFIG_SRC="$REPO_DIR/config/glim"
CONFIG_DIR="/tmp/glim_offline_config_$$"

# Validate inputs
if [ ! -d "$BAG_DIR" ]; then
    echo "ERROR: Bag directory not found: $BAG_DIR" >&2
    exit 1
fi

if ! ls "$BAG_DIR"/*.mcap &>/dev/null; then
    echo "ERROR: No .mcap files in $BAG_DIR" >&2
    exit 1
fi

# Generate metadata.yaml if missing
if [ ! -f "$BAG_DIR/metadata.yaml" ]; then
    echo "Generating metadata.yaml for $(basename $BAG_DIR)..."
    python3 - "$BAG_DIR" << 'PYEOF'
import os, sys
from mcap.reader import make_reader

bag_dir = sys.argv[1]
mcap_files = sorted([f for f in os.listdir(bag_dir) if f.endswith('.mcap')])

# Read topic info from first file
path = os.path.join(bag_dir, mcap_files[0])
with open(path, 'rb') as f:
    reader = make_reader(f)
    summary = reader.get_summary()
    channels = {c.id: c for c in summary.channels.values()}
    schemas = {s.id: s for s in summary.schemas.values()}
    stats = summary.statistics
    start_ns = stats.message_start_time
    end_ns = stats.message_end_time

topics = {}
for cid, count in stats.channel_message_counts.items():
    if cid in channels:
        ch = channels[cid]
        schema = schemas.get(ch.schema_id)
        topics[ch.topic] = (schema.name if schema else 'unknown', count)

lines = ['rosbag2_bagfile_information:', '  version: 5', '  storage_identifier: mcap']
lines.append(f'  duration:\n    nanoseconds: {(end_ns - start_ns) * len(mcap_files)}')
lines.append(f'  starting_time:\n    nanoseconds_since_epoch: {start_ns}')
lines.append(f'  message_count: {sum(c for _, c in topics.values()) * len(mcap_files)}')
lines.append('  topics_with_message_count:')

qos = '- history: 1\n  depth: 10\n  reliability: 2\n  durability: 2\n  deadline:\n    sec: 9223372036\n    nsec: 854775807\n  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n    nsec: 854775807\n  avoid_ros_namespace_conventions: false'
for topic in sorted(topics.keys()):
    msg_type, count = topics[topic]
    lines.extend([
        f'    - topic_metadata:', f'        name: {topic}',
        f'        type: {msg_type}', f'        serialization_format: cdr',
        f'        offered_qos_profiles: "{qos}"',
        f'      message_count: {count}'
    ])

lines.extend(['  compression_format: ""', '  compression_mode: ""', '  relative_file_paths:'])
for mf in mcap_files:
    lines.append(f'    - {mf}')
lines.append('  files:')
for mf in mcap_files:
    lines.extend([f'    - path: {mf}', '      starting_time:', '        nanoseconds_since_epoch: 0',
                  '      duration:', '        nanoseconds: 0', '      message_count: 0'])

out = os.path.join(bag_dir, 'metadata.yaml')
with open(out, 'w') as f:
    f.write('\n'.join(lines) + '\n')
print(f"  Generated {out} ({len(topics)} topics, {len(mcap_files)} files)")
PYEOF
fi

# Prepare offline config directory
mkdir -p "$CONFIG_DIR"
cp "$CONFIG_SRC/config_offline.json" "$CONFIG_DIR/config.json"
cp "$CONFIG_SRC/config_ros_offline.json" "$CONFIG_DIR/config_ros_offline.json"
cp "$CONFIG_SRC/config_logging.json" "$CONFIG_DIR/"
cp "$CONFIG_SRC/config_sensors.json" "$CONFIG_DIR/"
cp "$CONFIG_SRC/config_preprocess.json" "$CONFIG_DIR/"
cp "$CONFIG_SRC/config_odometry_gpu.json" "$CONFIG_DIR/"
cp "$CONFIG_SRC/config_sub_mapping_gpu.json" "$CONFIG_DIR/"
cp "$CONFIG_SRC/config_global_mapping_gpu.json" "$CONFIG_DIR/"
[ -f "$CONFIG_SRC/config_viewer.json" ] && cp "$CONFIG_SRC/config_viewer.json" "$CONFIG_DIR/"

# Fix IMU topic for bag replay (data_guarded has timestamp ordering issues)
sed -i 's|/imu/data_guarded|/imu/data|g' "$CONFIG_DIR/config_ros_offline.json"

# Create output directory
mkdir -p "$OUTPUT_DIR"

echo "=== GLIM Offline SLAM ==="
echo "  Bag: $BAG_DIR ($(ls "$BAG_DIR"/*.mcap | wc -l) MCAP files)"
echo "  Output: $OUTPUT_DIR"
[ "$DURATION" != "0" ] && echo "  Duration limit: ${DURATION}s"
echo ""

# Build podman args
EXTRA_ARGS=""
if [ "$DURATION" != "0" ]; then
    EXTRA_ARGS="-p playback_duration:=${DURATION}.0"
fi

# Run GLIM offline
sudo podman run --rm --name "glim-offline-$(date +%s)" \
  --device nvidia.com/gpu=0 \
  --network none \
  --shm-size=8g \
  --memory=24g \
  -v "$CONFIG_DIR:/ros2_ws/install/glim/share/glim/config:ro,z" \
  -v "$BAG_DIR:/bag:ro,z" \
  -v "$OUTPUT_DIR:/output:rw,z" \
  -e DISPLAY= \
  localhost/bess-glim:humble \
  bash -c "
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash 2>/dev/null
/ros2_ws/install/glim_ros/lib/glim_ros/glim_rosbag \
  --ros-args -p dump_path:=/output $EXTRA_ARGS -- \
  /bag
"

# Cleanup temp config
rm -rf "$CONFIG_DIR"

echo ""
echo "=== Output ==="
if [ -f "$OUTPUT_DIR/traj_lidar.txt" ]; then
    N_SUBMAPS=$(ls -d "$OUTPUT_DIR"/0*/ 2>/dev/null | wc -l)
    N_FRAMES=$(wc -l < "$OUTPUT_DIR/traj_lidar.txt")
    echo "  Submaps: $N_SUBMAPS"
    echo "  Frames: $N_FRAMES"
    echo "  Trajectory: $OUTPUT_DIR/traj_lidar.txt"
    du -sh "$OUTPUT_DIR"
else
    echo "  ERROR: No trajectory output found" >&2
    exit 1
fi
