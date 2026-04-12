#!/usr/bin/env bash
# Soak offloader: watches NVMe cache for sealed mcap shards, copies them to USB
# crash-safely, then unlinks the NVMe original. Designed to run in parallel with
# an active rosbag2 recorder.
#
# Sealing rule: shard N is sealed once shard N+1 exists. The highest-numbered
# shard is the active one and is NEVER touched until the recorder is gone AND
# the shard has been idle for >IDLE_S seconds.
#
# Copy protocol: write to <dst>.partial, fsync, rename → <dst>, then unlink src.
# A killed offloader leaves at most one .partial file (reaped on next pass).
#
# Usage: soak_offloader.sh <cache_dir> <usb_dir> [--dry-run] [--once]
set -u

CACHE=${1:?cache dir required}
USB=${2:?usb dir required}
shift 2 || true
DRY=0
ONCE=0
for a in "$@"; do
  case "$a" in
    --dry-run) DRY=1 ;;
    --once)    ONCE=1 ;;
  esac
done

IDLE_S=60                # how long the highest shard must be idle before final flush
POLL_S=5                 # how often to scan
LOCK="$CACHE/.offloader.lock"

mkdir -p "$USB"
ts() { date -u '+%Y-%m-%dT%H:%M:%SZ'; }
log() { echo "[$(ts)] $*"; }

# Single-instance lock (unset on exit so we don't leave stale locks behind).
if [[ -e "$LOCK" ]]; then
  oldpid=$(cat "$LOCK" 2>/dev/null || echo)
  if [[ -n "$oldpid" ]] && kill -0 "$oldpid" 2>/dev/null; then
    log "another offloader running pid=$oldpid — exiting"
    exit 0
  fi
  log "stale lock found (pid=$oldpid), removing"
  rm -f "$LOCK"
fi
echo $$ > "$LOCK"
trap 'rm -f "$LOCK"' EXIT

log "offloader start cache=$CACHE usb=$USB dry=$DRY once=$ONCE idle=${IDLE_S}s"

shard_idx() {
  local f=$1
  [[ $f =~ _([0-9]+)\.mcap$ ]] && echo "${BASH_REMATCH[1]}" || echo -1
}

# Returns 0 if a recorder process appears to still be writing to this bag dir,
# meaning the active (highest-numbered) shard must not be touched.
recorder_alive() {
  local bagdir=$1
  pgrep -af 'ros2 bag record' 2>/dev/null | grep -q "$(basename "$bagdir")"
}

offload_shard() {
  local src=$1 dst_dir=$2
  local base=$(basename "$src")
  local dst="$dst_dir/$base"
  local tmp="$dst.partial"

  if [[ -e "$dst" ]]; then
    log "skip already-on-usb: $base"
    if (( DRY == 0 )); then rm -f "$src"; fi
    return 0
  fi

  local sz=$(stat -c %s "$src" 2>/dev/null || echo 0)
  log "offload $base (${sz} bytes) → $dst_dir"
  if (( DRY == 1 )); then
    log "  [DRY] would copy + sync + unlink"
    return 0
  fi

  # Cross-fs copy with sync. dd conv=fsync forces final fdatasync.
  if ! dd if="$src" of="$tmp" bs=8M conv=fsync status=none 2>/dev/null; then
    log "  ERROR copy failed for $base"
    rm -f "$tmp"
    return 1
  fi

  local src_sz=$(stat -c %s "$src" 2>/dev/null || echo 0)
  local tmp_sz=$(stat -c %s "$tmp" 2>/dev/null || echo 0)
  if [[ "$src_sz" != "$tmp_sz" ]]; then
    log "  ERROR size mismatch src=$src_sz tmp=$tmp_sz, aborting"
    rm -f "$tmp"
    return 1
  fi

  if ! mv "$tmp" "$dst"; then
    log "  ERROR rename failed"
    rm -f "$tmp"
    return 1
  fi
  sync -f "$dst" 2>/dev/null || true

  rm -f "$src"
  log "  done $base"
  return 0
}

scan_pass() {
  local bagdir
  for bagdir in "$CACHE"/*/; do
    [[ -d "$bagdir" ]] || continue
    bagdir=${bagdir%/}
    local bagname=$(basename "$bagdir")
    local dst_bag="$USB/$bagname"
    mkdir -p "$dst_bag"

    # Reap any stale .partial files from a previous crashed run.
    find "$dst_bag" -maxdepth 1 -name '*.partial' -mmin +5 -delete 2>/dev/null || true

    mapfile -t shards < <(find "$bagdir" -maxdepth 1 -name '*.mcap' -printf '%f\n' 2>/dev/null | sort -V)
    [[ ${#shards[@]} -eq 0 ]] && continue

    local max_idx=-1 max_file=""
    for s in "${shards[@]}"; do
      local i=$(shard_idx "$s")
      (( i > max_idx )) && { max_idx=$i; max_file=$s; }
    done

    local active_alive=0
    recorder_alive "$bagdir" && active_alive=1

    for s in "${shards[@]}"; do
      local src="$bagdir/$s"
      [[ -e "$src" ]] || continue
      local i=$(shard_idx "$s")

      if [[ "$s" == "$max_file" ]]; then
        # Active shard. Skip if recorder still alive.
        if (( active_alive == 1 )); then
          continue
        fi
        # Recorder gone — only flush if shard has been idle for IDLE_S seconds.
        local mtime=$(stat -c %Y "$src" 2>/dev/null || echo 0)
        local now=$(date +%s)
        if (( now - mtime < IDLE_S )); then
          log "active shard $s not yet idle (age=$((now-mtime))s), waiting"
          continue
        fi
        log "recorder gone + idle ${IDLE_S}s — flushing final shard $s"
      fi

      offload_shard "$src" "$dst_bag" || continue
    done

    # If recorder is gone AND every mcap has been moved, copy/rebuild metadata.
    if (( active_alive == 0 )); then
      local remaining=$(find "$bagdir" -maxdepth 1 -name '*.mcap' 2>/dev/null | wc -l)
      if (( remaining == 0 )); then
        if [[ -f "$bagdir/metadata.yaml" && ! -f "$dst_bag/metadata.yaml" ]]; then
          log "copying metadata.yaml for $bagname"
          (( DRY == 0 )) && cp "$bagdir/metadata.yaml" "$dst_bag/metadata.yaml"
        fi
        if [[ ! -f "$dst_bag/metadata.yaml" ]]; then
          log "rebuilding metadata.yaml on USB for $bagname"
          (( DRY == 0 )) && python3 /home/thor/bess/scripts/rebuild-bag-metadata.py "$dst_bag" >/dev/null 2>&1 || true
        fi
        if (( DRY == 0 )); then
          rm -f "$bagdir/metadata.yaml"
          rmdir "$bagdir" 2>/dev/null && log "removed empty cache dir $bagname"
        fi
      fi
    fi
  done
}

while true; do
  scan_pass
  (( ONCE == 1 )) && break
  sleep "$POLL_S"
done

log "offloader exit"
