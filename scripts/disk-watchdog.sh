#!/bin/bash
# Disk watchdog — keeps recordings under 1TB by deleting oldest
# Run via cron every 5 minutes
MAX_GB=1000
DIR="/home/thor/recordings"

USED_KB=$(du -sk "$DIR" 2>/dev/null | awk '{print $1}')
USED_GB=$((USED_KB / 1048576))

if [ "$USED_GB" -gt "$MAX_GB" ]; then
    OLDEST=$(ls -dt "$DIR"/soak_* 2>/dev/null | tail -1)
    if [ -n "$OLDEST" ]; then
        echo "[$(date)] Disk watchdog: ${USED_GB}GB > ${MAX_GB}GB — removing $OLDEST"
        rm -rf "$OLDEST"
    fi
fi
