#!/usr/bin/env bash
# bess-sensor-firewall.sh — constrain GigE Vision (GVCP/GVSP) traffic to mgbe0_0
#
# Problem this fixes
# ==================
# Thor runs sensor containers with `network_mode: host` so Spinnaker sees ALL
# host interfaces: mgbe0_0 (sensor 169.254.0.0/16), wlP1p1s0 (WiFi/EERO),
# tailscale0, l4tbr0, docker0. The Spinnaker SDK's camera discovery walks
# every interface, enumerating and attempting control-channel handshakes.
#
# For a camera on 169.254.x.x, the kernel happens to route via mgbe0_0 by
# longest-prefix match on 169.254.0.0/16, BUT Spinnaker's per-interface
# enumeration binds sockets to each interface's IP explicitly. On A6701
# cameras specifically, when the camera sees GVCP control requests arriving
# from TWO source IPs (e.g., 169.254.100.1 via mgbe0_0 AND 192.168.4.104
# via wlP1p1s0 routed through Thor's main route) in quick succession, the
# Pleora iPORT firmware records the first source as the "session holder"
# and returns -1005 "Unable to set DeviceAccessStatus — may be open by
# another application" to the second.
#
# The `_find_sensor_interface()` patch in thermal_spinnaker_node_patched.py
# filters enumeration to the sensor interface, but does NOT stop the
# low-level Spinnaker control-socket handshake from emitting from other
# interfaces during the initial discovery.
#
# How this script fixes it
# ========================
# OUTPUT-chain DROP for any UDP:3956 (GVCP) or UDP:3957 (GVSP stream)
# destined to 169.254.0.0/16 that tries to egress from any interface
# other than mgbe0_0. Cameras never see any packet not bearing Thor's
# 169.254.100.1 as source IP → no multi-IP ownership confusion → -1005
# cannot occur.
#
# Rules are idempotent (flush our own chain each time this runs).
# Safe: does NOT touch inbound, does NOT affect any non-sensor-subnet
# traffic, does NOT interfere with Docker bridges. Other interfaces can
# still talk to internet, Tailscale, etc — only sensor-subnet GVCP/GVSP
# is constrained.

set -euo pipefail

CHAIN=BESS_SENSOR_FW
SENSOR_IF=mgbe0_0
SENSOR_CIDR=169.254.0.0/16

log() { echo "[bess-sensor-firewall] $*"; }

# Create our chain if it doesn't exist, and flush it.
iptables -N "$CHAIN" 2>/dev/null || true
iptables -F "$CHAIN"

# Hook our chain into OUTPUT exactly once.
if ! iptables -C OUTPUT -j "$CHAIN" 2>/dev/null; then
    iptables -I OUTPUT -j "$CHAIN"
    log "hooked $CHAIN into OUTPUT"
fi

# Rules: if the destination is the sensor subnet AND the egress interface
# is NOT mgbe0_0, drop the packet. Applied to GVCP (3956) and GVSP stream
# range (3958 and up — GVSP negotiates dynamic ports). We use a broader
# UDP drop to catch all GigE Vision protocol traffic.
for port in 3956 3957; do
    iptables -A "$CHAIN" -o lo -j RETURN   # allow loopback (localhost enumeration)
    iptables -A "$CHAIN" -o "$SENSOR_IF" -p udp --dport "$port" -d "$SENSOR_CIDR" -j RETURN
    iptables -A "$CHAIN" -p udp --dport "$port" -d "$SENSOR_CIDR" -j DROP
done

# Also drop any UDP to the sensor subnet on dynamic stream ports (8800-9000)
# NOT via mgbe0_0. The GVSP stream destination port is negotiated by
# Spinnaker and can be any ephemeral port — but it's always bound to the
# IP Spinnaker used for control. Constraining control to sensor iface
# implicitly constrains streams too, but this is belt-and-braces for
# multi-camera / retry edge cases where stale stream sockets linger.
iptables -A "$CHAIN" -p udp --dport 8800:9000 -d "$SENSOR_CIDR" -o "$SENSOR_IF" -j RETURN
iptables -A "$CHAIN" -p udp --dport 8800:9000 -d "$SENSOR_CIDR" -j DROP

log "rules installed:"
iptables -L "$CHAIN" -n --line-numbers | sed 's/^/[bess-sensor-firewall]   /'

# Report sensor-subnet packets that hit the DROP rule since last run
# (useful for confirming the fix is firing).
DROPS=$(iptables -L "$CHAIN" -n -v | awk '/DROP/ {sum+=$1} END{print sum+0}')
log "total drops hit so far: $DROPS (will grow if Spinnaker on non-sensor iface tries to reach cameras)"
