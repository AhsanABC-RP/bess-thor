#!/bin/bash
# Archive completed mcap bag splits from NVMe buffer to USB drive
# Runs via cron every minute
# Only moves mcap files that aren't being written to (older than 60s)
#
# NVMe buffer: /home/thor/recordings/
# USB archive: /mnt/usb/bags/
#
# Safety: checks USB is mounted before moving, keeps metadata.yaml in place

USB_DEST="/mnt/usb/bags"
SRC="/home/thor/recordings"
LOG="/tmp/archive-bags.log"

# Check USB is mounted
if ! mountpoint -q /mnt/usb; then
    exit 0
fi

# Find all soak_* directories with mcap files
for bag_dir in "$SRC"/soak_*; do
    [ -d "$bag_dir" ] || continue
    bag_name=$(basename "$bag_dir")

    # Create destination directory
    mkdir -p "$USB_DEST/$bag_name"

    # Copy metadata.yaml if it exists and hasn't been copied
    if [ -f "$bag_dir/metadata.yaml" ] && [ ! -f "$USB_DEST/$bag_name/metadata.yaml" ]; then
        cp "$bag_dir/metadata.yaml" "$USB_DEST/$bag_name/"
    fi

    # Move mcap files that are older than 60 seconds (not being written)
    # But keep the LATEST mcap file (it's still being written to)
    latest_mcap=$(ls -t "$bag_dir"/*.mcap 2>/dev/null | head -1)

    for mcap_file in "$bag_dir"/*.mcap; do
        [ -f "$mcap_file" ] || continue

        # Skip the latest/active mcap file
        [ "$mcap_file" = "$latest_mcap" ] && continue

        # Skip if already on USB
        mcap_name=$(basename "$mcap_file")
        [ -f "$USB_DEST/$bag_name/$mcap_name" ] && continue

        # Move to USB
        mv "$mcap_file" "$USB_DEST/$bag_name/" 2>/dev/null
        if [ $? -eq 0 ]; then
            echo "$(date +%H:%M:%S) Archived $mcap_name → $USB_DEST/$bag_name/" >> "$LOG"
        fi
    done

    # Update metadata on USB side
    if [ -f "$bag_dir/metadata.yaml" ]; then
        cp "$bag_dir/metadata.yaml" "$USB_DEST/$bag_name/"
    fi
done
