#!/bin/bash
# Re-assert /etc/sysctl.d/60-gige-sensors.conf every fire. If any key in the
# file disagrees with what's currently live in /proc/sys, restore it and log
# a DRIFT event to the journal. Self-healing — bounded drift duration ≤60 s.
#
# Why this exists: at boot, the eBUS Pleora SysV init script clobbered
# net.core.rmem_max from 2 GiB → 10 MiB, GigE Vision UDP fragments dropped,
# thermals cascaded into -1011 / -1010. eBUS init.d is now disabled but any
# future SDK installer (Spinnaker patch, Arena upgrade, ZED redux) could
# reintroduce a /proc/sys clobber from a privileged container or post-install
# hook. This watchdog limits the blast radius to a single 60 s window.

set -u
CONF=/etc/sysctl.d/60-gige-sensors.conf

[ -r "$CONF" ] || { logger -t bess-sysctl-assert "ABORT: $CONF unreadable"; exit 0; }

drift=0
while IFS= read -r line; do
  # strip comments and blank lines
  line="${line%%#*}"
  line="${line## }"
  line="${line%% }"
  [ -z "$line" ] && continue

  # split key=value, strip whitespace around =
  key="${line%%=*}"
  val="${line#*=}"
  key="${key// /}"
  val="${val// /}"
  [ -z "$key" ] || [ -z "$val" ] && continue

  # /proc/sys path
  proc_path="/proc/sys/${key//./\/}"
  [ -r "$proc_path" ] || continue

  current=$(< "$proc_path" tr -s ' \t' ' ')
  expected=$(echo "$val" | tr -s ' \t' ' ')

  if [ "$current" != "$expected" ]; then
    logger -t bess-sysctl-assert "DRIFT: $key was=$current expected=$expected — restoring"
    sysctl -w "$key=$val" >/dev/null 2>&1 && drift=$((drift + 1)) || \
      logger -t bess-sysctl-assert "RESTORE_FAILED: $key"
  fi
done < "$CONF"

[ "$drift" -gt 0 ] && logger -t bess-sysctl-assert "restored $drift key(s) from $CONF"
exit 0
