#!/bin/bash
# bess-cold-start-check.sh — post-power-off health check
#
# Run after any hard reboot / power cycle / unexpected crash to verify the
# system recovered into a sane state. Reports pass/warn/fail per subsystem.
# Does NOT auto-fix anything — prints the exact command to run if a check
# flags. The user decides.
#
# Usage:
#   ./scripts/bess-cold-start-check.sh          # full check, human-readable
#   ./scripts/bess-cold-start-check.sh --quiet   # errors/warnings only
#
# Exit codes:
#   0 = all checks passed
#   1 = one or more WARN (non-blocking but worth looking at)
#   2 = one or more FAIL (stack is not healthy, action required)

set -u
QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

PASS=0; WARN=0; FAIL=0
RED=$'\033[31m'; GRN=$'\033[32m'; YLW=$'\033[33m'; BLU=$'\033[34m'; RST=$'\033[0m'
[ ! -t 1 ] && { RED=""; GRN=""; YLW=""; BLU=""; RST=""; }

pass() { PASS=$((PASS+1)); [ $QUIET -eq 0 ] && echo "  ${GRN}PASS${RST}  $*"; }
warn() { WARN=$((WARN+1));                echo "  ${YLW}WARN${RST}  $*"; }
fail() { FAIL=$((FAIL+1));                echo "  ${RED}FAIL${RST}  $*"; }
hint() {                                  echo "        ${BLU}→${RST} $*"; }
header() { [ $QUIET -eq 0 ] && echo && echo "${BLU}== $* ==${RST}"; }

# ============================================================
# 1. Filesystem integrity (ext4 dirty bit after unclean shutdown)
# ============================================================
header "Filesystem integrity"
ROOT_DEV=$(findmnt -nt ext4 / -o SOURCE 2>/dev/null)
if [ -n "$ROOT_DEV" ]; then
    STATE=$(sudo tune2fs -l "$ROOT_DEV" 2>/dev/null | awk -F: '/Filesystem state/ {gsub(/^[ \t]+/,"",$2); print $2}')
    if [ "$STATE" = "clean" ]; then
        pass "root ($ROOT_DEV) ext4 state: clean"
    elif [ -z "$STATE" ]; then
        warn "root ($ROOT_DEV) ext4 state: could not read (tune2fs needs sudo)"
    else
        fail "root ($ROOT_DEV) ext4 state: $STATE"
        hint "sudo fsck -yvf $ROOT_DEV (unmount first — single-user boot)"
    fi
fi
# F8 TerraMaster NAS (primary bag store post-2026-04-23). NFSv4.2 over
# sfp28-16, mounted at /home/thor/nas/bess-bags via systemd automount.
if mountpoint -q /home/thor/nas/bess-bags; then
    NAS_FREE_GB=$(df -BG /home/thor/nas/bess-bags | awk 'NR==2 {gsub(/G/,"",$4); print $4}')
    if [ "$NAS_FREE_GB" -ge 500 ]; then
        pass "F8 NAS /home/thor/nas/bess-bags free: ${NAS_FREE_GB}G (recorder prereq ≥500G)"
    elif [ "$NAS_FREE_GB" -ge 100 ]; then
        warn "F8 NAS /home/thor/nas/bess-bags free: ${NAS_FREE_GB}G (below recorder prereq 500G)"
    else
        fail "F8 NAS /home/thor/nas/bess-bags free: ${NAS_FREE_GB}G (critically low)"
    fi
else
    fail "F8 NAS /home/thor/nas/bess-bags NOT mounted"
    hint "systemctl status home-thor-nas-bess\\\\x2dbags.mount; ping 169.254.100.30"
fi

ROOT_FREE_GB=$(df -BG / | awk 'NR==2 {gsub(/G/,"",$4); print $4}')
if [ "$ROOT_FREE_GB" -ge 100 ]; then
    pass "root free space: ${ROOT_FREE_GB}G"
elif [ "$ROOT_FREE_GB" -ge 20 ]; then
    warn "root free space: ${ROOT_FREE_GB}G (recorder fallback path can still fill this)"
else
    fail "root free space: ${ROOT_FREE_GB}G — BashTool subprocess writes will fail on ENOSPC"
    hint "docker system prune -af ; check /home/thor/recordings/tmp/"
fi

# ============================================================
# 2. Wall clock sanity (BMC RTC drift, timesyncd state)
# ============================================================
header "Wall clock sanity"
TSCTL=$(timedatectl 2>/dev/null)
NTP_ACTIVE=$(echo "$TSCTL" | awk -F: '/NTP service/ {gsub(/^[ \t]+/,"",$2); print $2}')
if [ "$NTP_ACTIVE" = "inactive" ] || [ "$NTP_ACTIVE" = "n/a" ]; then
    pass "systemd-timesyncd: $NTP_ACTIVE (masked by design — NTP steps break Ouster PTP)"
else
    fail "systemd-timesyncd: $NTP_ACTIVE (MUST be masked)"
    hint "sudo systemctl mask systemd-timesyncd.service"
fi

SYS_EPOCH=$(date -u +%s)
HTTP_EPOCH=$(curl -fsSI --max-time 4 https://www.google.com 2>/dev/null | awk -F': ' '/^[Dd]ate:/ {print $2}' | xargs -I{} date -u -d "{}" +%s 2>/dev/null)
if [ -n "$HTTP_EPOCH" ]; then
    SKEW=$((SYS_EPOCH - HTTP_EPOCH))
    ABS_SKEW=${SKEW#-}
    if [ "$ABS_SKEW" -le 30 ]; then
        pass "wall clock skew vs google.com HTTP Date: ${SKEW}s"
    elif [ "$ABS_SKEW" -le 300 ]; then
        warn "wall clock skew vs google.com HTTP Date: ${SKEW}s"
    else
        fail "wall clock skew vs google.com HTTP Date: ${SKEW}s (BMC RTC drifted)"
        hint "sudo timedatectl set-time \"\$(date -u -d @\$HTTP_EPOCH '+%Y-%m-%d %H:%M:%S')\""
        hint "Then re-anchor PHC1 ONLY IF sensor stack is offline:"
        hint "  sudo systemctl stop phc2sys-mgbe0 && sudo phc_ctl mgbe0_0 set && sudo systemctl start phc2sys-mgbe0"
    fi
else
    warn "could not fetch HTTP Date from google.com (no internet? RUT50 down?)"
fi

# ============================================================
# 3. PTP chain (ptp4l GM + phc2sys servo)
# ============================================================
header "PTP chain"
for unit in ptp4l-mgbe0 phc2sys-mgbe0; do
    if systemctl is-active --quiet "$unit"; then
        pass "$unit: active"
    else
        fail "$unit: $(systemctl is-active $unit)"
        hint "sudo systemctl restart $unit ; journalctl -u $unit -n 30"
    fi
done

# PHC1 offset to CLOCK_REALTIME — phc2sys -S 0 refuses to step, so huge offset
# means the servo is blocked and needs a manual phc_ctl anchor.
PHC_OFFSET=$(journalctl -u phc2sys-mgbe0 -n 3 --no-pager 2>/dev/null | grep -oP 'offset\s+-?\K[0-9]+' | tail -1)
if [ -n "$PHC_OFFSET" ]; then
    if [ "$PHC_OFFSET" -le 1000000 ]; then
        pass "phc2sys recent offset: ${PHC_OFFSET} ns (< 1 ms, servo locked)"
    elif [ "$PHC_OFFSET" -le 1000000000 ]; then
        warn "phc2sys recent offset: ${PHC_OFFSET} ns (> 1 ms, still slewing)"
    else
        fail "phc2sys recent offset: ${PHC_OFFSET} ns (> 1 s — servo blocked by -S 0)"
        hint "Sensor stack MUST be offline before re-anchoring PHC1:"
        hint "  docker compose -f /home/thor/bess/docker-compose.thor.yml --profile full down"
        hint "  sudo systemctl stop phc2sys-mgbe0 && sudo phc_ctl mgbe0_0 set && sudo systemctl start phc2sys-mgbe0"
    fi
fi

# ============================================================
# 4. Network: sensor NIC, MikroTik, RUT50 uplink
# ============================================================
header "Network"
IFACE=mgbe0_0
if ip link show $IFACE 2>/dev/null | grep -q 'state UP'; then
    pass "$IFACE: link UP"
else
    fail "$IFACE: link DOWN"
    hint "sudo nmcli connection up $IFACE"
fi

IPS=$(ip -4 addr show $IFACE 2>/dev/null | awk '/inet / {print $2}')
if echo "$IPS" | grep -q '^169\.254\.100\.1/16$'; then
    EXTRA=$(echo "$IPS" | grep -v '^169\.254\.100\.1/16$' | tr '\n' ' ')
    if [ -z "$EXTRA" ]; then
        pass "$IFACE holds only 169.254.100.1/16 (Spinnaker wrong-subnet-safe)"
    else
        fail "$IFACE has stray IPs: $EXTRA (causes A6701 wrong-subnet [-1015])"
        for ex in $EXTRA; do hint "sudo ip addr del $ex dev $IFACE"; done
    fi
else
    fail "$IFACE missing 169.254.100.1/16"
    hint "sudo ip addr add 169.254.100.1/16 dev $IFACE"
fi

MTU=$(ip link show $IFACE 2>/dev/null | grep -oP 'mtu \K[0-9]+')
if [ "${MTU:-0}" -ge 8000 ]; then
    pass "$IFACE MTU: $MTU (jumbo OK)"
else
    fail "$IFACE MTU: $MTU (need ≥ 8000 for Ouster + Spinnaker)"
    hint "nmcli connection modify $IFACE 802-3-ethernet.mtu 9000 ; nmcli connection up $IFACE"
fi

if ping -c1 -W2 169.254.100.254 >/dev/null 2>&1; then
    pass "MikroTik (169.254.100.254) reachable"
else
    fail "MikroTik (169.254.100.254) unreachable"
    hint "Check sfp28-9/11/13/15 breakout cabling; power-cycle MikroTik if needed"
fi

# Use HTTPS HEAD, not ICMP, for the internet check. EE UK (the SIM in
# the RUT50) drops outbound ICMP to 1.1.1.1 entirely, so a ping-based
# probe gives a false negative even when the 5G data path is healthy.
# google.com over HTTPS is the same path used by bess-time-bootstrap.
if curl -fsS --max-time 5 -o /dev/null https://www.google.com 2>/dev/null; then
    pass "Internet reachable (HTTPS HEAD https://www.google.com)"
else
    warn "Internet unreachable via HTTPS — RUT50 5G uplink may be down"
    hint "Check /tool ping on MikroTik bridgeWAN; see boot-sensors.sh STEP 3.5"
    hint "ICMP to 1.1.1.1 is blocked by carrier — do NOT use ping for this check"
fi

# ============================================================
# 5. Sensor reachability
# ============================================================
header "Sensors"
# A6701 thermals are NOT in this list — they use self-assigned link-local and
# Spinnaker MAC-matching; they don't reliably answer ICMP. Their liveness is
# the thermal1/2 container healthcheck (verified in the Docker section below).
for pair in \
    "Ouster:169.254.70.119" \
    "A70-1:169.254.20.1" \
    "A70-2:169.254.20.2" \
    "Blackfly1:169.254.1.20" \
    "Blackfly2:169.254.1.21" \
    "Lucid1:169.254.20.23" \
    "Lucid2:169.254.20.14"; do
    name="${pair%%:*}"; ip="${pair##*:}"
    if ping -c1 -W1 "$ip" >/dev/null 2>&1; then
        pass "$name ($ip) UP"
    else
        warn "$name ($ip) DOWN"
    fi
done

# ============================================================
# 6. USB storage (HighPoint RM110)
# ============================================================
header "USB storage (RM110)"
if lsusb -d 1103:0110 >/dev/null 2>&1; then
    pass "RM110 (VID:PID 1103:0110) enumerated"
else
    fail "RM110 not enumerated on USB bus"
    hint "Warm-reboot stuck state: physical re-seat is the ONLY reliable recovery"
    hint "bess-usb-recovery.service retries every 5 min but cannot power-cycle the PHY"
fi
if ls /dev/disk/by-id/*RM110* >/dev/null 2>&1; then
    pass "RM110 by-id symlinks present"
else
    fail "RM110 by-id symlinks missing — udev did not see the device"
fi

# ============================================================
# 7. Kernel sysctl tuning
# ============================================================
header "Kernel sysctl"
check_sysctl() {
    local key=$1 want=$2
    local got=$(sysctl -n "$key" 2>/dev/null)
    if [ "$got" = "$want" ]; then
        pass "$key = $got"
    else
        warn "$key = $got (want $want)"
        hint "sudo sysctl -w $key=$want ; check /etc/sysctl.d/60-gige-sensors.conf"
    fi
}
check_sysctl net.core.rmem_max             2147483647
check_sysctl net.core.rmem_default         67108864
check_sysctl net.core.wmem_max             67108864
check_sysctl net.core.wmem_default         67108864
check_sysctl net.core.netdev_max_backlog   100000
check_sysctl net.ipv4.ipfrag_high_thresh   26000000
check_sysctl net.ipv4.ipfrag_low_thresh    13000000
check_sysctl net.ipv4.conf.mgbe0_0.rp_filter 0

# ============================================================
# 8. Docker daemon + container state
# ============================================================
header "Docker daemon"
if systemctl is-active --quiet docker; then
    pass "docker.service: active"
else
    fail "docker.service: $(systemctl is-active docker)"
fi

if docker info >/dev/null 2>&1; then
    pass "docker CLI responsive"
    RUNNING=$(docker ps -q 2>/dev/null | wc -l)
    UNHEALTHY=$(docker ps --filter health=unhealthy -q 2>/dev/null | wc -l)
    RESTARTING=$(docker ps --filter status=restarting -q 2>/dev/null | wc -l)
    if [ "$RUNNING" -eq 0 ]; then
        warn "0 containers running — stack not started (run: sudo systemctl start bess-stack OR scripts/bess-stack-up.sh)"
    else
        pass "$RUNNING containers running"
    fi
    if [ "$UNHEALTHY" -eq 0 ]; then
        pass "0 containers unhealthy"
    else
        warn "$UNHEALTHY containers unhealthy:"
        docker ps --filter health=unhealthy --format '        - {{.Names}}'
    fi
    if [ "$RESTARTING" -eq 0 ]; then
        pass "0 containers restarting (no restart loop)"
    else
        fail "$RESTARTING containers in restart loop:"
        docker ps --filter status=restarting --format '        - {{.Names}}'
    fi
fi

# ============================================================
# Summary
# ============================================================
header "Summary"
echo "  ${GRN}PASS${RST}: $PASS   ${YLW}WARN${RST}: $WARN   ${RED}FAIL${RST}: $FAIL"
if [ $FAIL -gt 0 ]; then
    exit 2
elif [ $WARN -gt 0 ]; then
    exit 1
else
    exit 0
fi
