#!/bin/bash
# thermal_pleora_recover.sh — software-only Pleora iPORT recovery via MikroTik
# port-bounce.
#
# Used when a thermal camera is on the wire (link-ok, MAC in MikroTik bridge
# host table) but firmware GVCP is unresponsive (-1005 / -1010 every connect,
# 100% ICMP loss). The MikroTik CRS518 has no PoE-out so we can't software-
# trigger a physical power-cycle, but admin-disabling the SFP port for 90 s
# forces the iPORT firmware to lose link long enough that its GigE Vision
# heartbeat times out and the dead control session drains. Often clears the
# stuck-firmware state without yanking power.
#
# If GVCP still doesn't respond after the bounce, exit 1 and tell the user
# to power-cycle the camera physically (CLAUDE.md R4).
#
# Usage:  sudo ./thermal_pleora_recover.sh thermal2
#         sudo ./thermal_pleora_recover.sh thermal4

set -u

CONTAINER="${1:-}"
case "$CONTAINER" in
  thermal1) SFP_PORT=sfp28-15 ; CAM_IP=169.254.20.43  ;;
  thermal2) SFP_PORT=sfp28-2  ; CAM_IP=169.254.249.149;;
  thermal3) SFP_PORT=sfp28-3  ; CAM_IP=169.254.20.1   ;;
  thermal4) SFP_PORT=sfp28-4  ; CAM_IP=169.254.20.2   ;;
  *)
    echo "Usage: $0 <thermal1|thermal2|thermal3|thermal4>" >&2
    exit 2
    ;;
esac

# Source MikroTik creds from the existing sweep script (single source of truth)
SWEEP=/home/thor/bess/scripts/mikrotik_sfp_sweep.sh
if [ ! -r "$SWEEP" ]; then
  echo "ERROR: $SWEEP not readable, can't get MT creds" >&2
  exit 2
fi
MT_HOST=$(grep -E '^MT_HOST=' "$SWEEP" | head -1 | sed 's/.*:-\([^}]*\)}.*/\1/')
MT_USER=$(grep -E '^MT_USER=' "$SWEEP" | head -1 | sed 's/.*:-\([^}]*\)}.*/\1/')
MT_PASS=$(grep -E '^MT_PASS=' "$SWEEP" | head -1 | sed 's/.*:-\([^}]*\)}.*/\1/')
[ -z "$MT_HOST" ] && MT_HOST=169.254.100.254
[ -z "$MT_USER" ] && MT_USER=admin

mt() {
  sshpass -p "$MT_PASS" ssh -o StrictHostKeyChecking=no -o LogLevel=ERROR \
    "$MT_USER@$MT_HOST" "$@"
}

LOG() { echo "[$(date -u +%H:%M:%SZ)] thermal_pleora_recover: $*"; }

# CLAUDE.md R3: stop the container so its retry-loop doesn't broadcast GVCP
# discovery to the camera while we're trying to recover its firmware.
LOG "stopping $CONTAINER (R3 — end retry broadcasts)"
docker stop "$CONTAINER" >/dev/null 2>&1 || LOG "container already stopped"

LOG "disabling MikroTik $SFP_PORT"
mt "/interface/ethernet/set $SFP_PORT disabled=yes" || {
  LOG "FATAL: MikroTik unreachable, cannot disable port"
  exit 2
}

# 90 s gives the Pleora iPORT GigE Vision heartbeat (~45 s) two full cycles
# of link-loss to tear down its session. Less than 90 s sometimes leaves a
# zombie session.
LOG "waiting 90s for Pleora session heartbeat to drain..."
sleep 90

LOG "re-enabling MikroTik $SFP_PORT"
mt "/interface/ethernet/set $SFP_PORT disabled=no" || {
  LOG "ERROR: failed to re-enable $SFP_PORT — check MikroTik manually!"
  exit 2
}

# A6701 / A70 firmware boot is ~30 s after link returns
LOG "waiting 30s for camera firmware to re-init"
sleep 30

# Passive GVCP probe — does the camera respond now?
LOG "probing $CAM_IP via gvcp_probe.py discover"
PROBE_OUT=$(timeout 10 /home/thor/bess/scripts/gvcp_probe.py discover 2>&1 || true)
if echo "$PROBE_OUT" | grep -qE "$CAM_IP|$(echo "$CONTAINER" | sed 's/thermal/A6701\|A70/')"; then
  LOG "GVCP discovery sees $CAM_IP — Pleora firmware revived"
  LOG "starting $CONTAINER container"
  cd /home/thor/bess
  docker compose -f docker-compose.thor.yml up -d --no-deps "$CONTAINER" || {
    LOG "ERROR: container start failed"
    exit 1
  }
  LOG "tail logs in 30s with: docker logs --tail 20 $CONTAINER"
  exit 0
else
  LOG "GVCP discovery did NOT find $CAM_IP — firmware still locked"
  LOG ""
  LOG "  >>> ESCALATE TO PHYSICAL POWER-CYCLE PER CLAUDE.md R4 <<<"
  LOG ""
  LOG "  1. unplug DC power from the camera"
  LOG "  2. wait >= 5 minutes (Pleora supercaps + deep firmware reset)"
  LOG "  3. plug back in, do NOT touch / probe for >= 2 min while it boots"
  LOG "  4. then run: docker compose -f docker-compose.thor.yml up -d --no-deps $CONTAINER"
  LOG ""
  exit 1
fi
