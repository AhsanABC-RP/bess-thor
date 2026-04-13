#!/bin/bash
# BESS Thor Sensor Network Bootstrap
# Run after every boot or network reset
# Usage: sudo bash boot-sensors.sh
#
# TOPOLOGY (2026-04-04):
#   Thor QSFP breakout (mgbe0_0-3) → sfp28-9/11/13/15 on MikroTik (4x 10G)
#   sfp28-1: A70 #1 (89900590)     sfp28-5: A6701 #1
#   sfp28-2: Blackfly #1 (25073269) sfp28-6: A6701 #2
#   sfp28-3: Blackfly #2 (25073270) sfp28-7: Lucid #1 (231800447) 10G
#   sfp28-4: A70 #2 (89901856)     sfp28-8: Lucid #2 (231800449) 10G
#
set +e  # Don't exit on errors — sensors may be offline

LOG() { echo "[$(date +%H:%M:%S)] $*"; }

# ============================================================
# STEP 1: Thor Host Network
# ============================================================
LOG "=== STEP 1: Thor Host Network ==="

IFACE="mgbe0_0"

# IPs
ip addr show $IFACE | grep -q "169.254.100.1" || ip addr add 169.254.100.1/16 dev $IFACE
ip addr show $IFACE | grep -q "192.168.2.50"  || ip addr add 192.168.2.50/24 dev $IFACE
ip addr show $IFACE | grep -q "192.168.127.1" || ip addr add 192.168.127.1/24 dev $IFACE
# 192.168.1.50/24 was removed 2026-04-13: RUT50 is on MikroTik sfp28-12 in
# bridgeWAN (not bridgeLocal), so Thor has no direct L2 path to 192.168.1.0/24.
# Thor reaches RUT50 by routing through MikroTik (169.254.100.254) which
# masquerade-NATs out bridgeWAN. See STEP 3.5 below.
ip addr del 192.168.1.50/24 dev $IFACE 2>/dev/null

# MTU (needs link bounce)
CUR_MTU=$(ip link show $IFACE | grep -oP 'mtu \K[0-9]+')
if [ "$CUR_MTU" -lt 8900 ]; then
    LOG "Setting MTU 9000 on $IFACE (was $CUR_MTU)..."
    ip link set $IFACE down
    ip link set $IFACE mtu 9000
    ip link set $IFACE up
    sleep 3
fi

# Enable IPv6 on mgbe0_0 (needed for camera discovery/management)
sysctl -w net.ipv6.conf.$IFACE.disable_ipv6=0 >/dev/null 2>&1
ip -6 addr show dev $IFACE | grep -q "fe80::100" || ip -6 addr add fe80::100/64 dev $IFACE 2>/dev/null

# Kernel tuning
sysctl -w net.core.rmem_max=2147483647 >/dev/null
sysctl -w net.core.rmem_default=2147483647 >/dev/null
sysctl -w net.core.wmem_max=67108864 >/dev/null
sysctl -w net.core.wmem_default=67108864 >/dev/null
sysctl -w net.core.netdev_max_backlog=100000 >/dev/null
sysctl -w net.ipv4.conf.all.rp_filter=0 >/dev/null
sysctl -w net.ipv4.conf.$IFACE.rp_filter=0 >/dev/null
sysctl -w net.ipv4.ipfrag_high_thresh=26000000 >/dev/null
sysctl -w net.ipv4.ipfrag_low_thresh=13000000 >/dev/null
sysctl -w net.ipv4.udp_rmem_min=262144 >/dev/null

# Disable enP2p1s0 — NetworkManager gives it duplicate 169.254.x.x which breaks GigE Vision discovery
nmcli device set enP2p1s0 managed no 2>/dev/null || true
ip addr flush dev enP2p1s0 2>/dev/null
ip link set enP2p1s0 down 2>/dev/null

# Disable mgbe1_0/2_0/3_0 — all 4 breakout lanes are bridged on MikroTik,
# so cameras appear 4x if all interfaces are up. Only use mgbe0_0.
for extra in mgbe1_0 mgbe2_0 mgbe3_0; do
    ip link set $extra down 2>/dev/null
done

LOG "Host network: OK (MTU=$(ip link show $IFACE | grep -oP 'mtu \K[0-9]+'))"

# ============================================================
# STEP 2: DHCP Server for Cameras
# ============================================================
LOG "=== STEP 2: DHCP Server ==="

# A70 #1, A70 #2, Lucid #1, and Lucid #2 are in DHCP mode.
# They need a DHCP server on mgbe0_0 to get stable IPs.
# Kill any existing dnsmasq on this interface
if [ -f /tmp/dnsmasq-sensors.pid ]; then
    kill "$(cat /tmp/dnsmasq-sensors.pid)" 2>/dev/null
    rm -f /tmp/dnsmasq-sensors.pid
fi

dnsmasq \
    --interface=$IFACE \
    --bind-interfaces \
    --dhcp-range=169.254.20.10,169.254.20.50,255.255.0.0,12h \
    --dhcp-host=00:40:7f:04:fa:7b,169.254.20.1,A70-1 \
    --dhcp-host=00:40:7f:11:04:ea,169.254.20.2,A70-2 \
    --dhcp-host=1c:0f:af:07:a9:cd,169.254.20.23,Lucid-1 \
    --dhcp-host=1c:0f:af:07:a9:e7,169.254.20.14,Lucid-2 \
    --no-resolv --no-hosts \
    --log-facility=/tmp/dnsmasq-sensors.log \
    --pid-file=/tmp/dnsmasq-sensors.pid \
    2>/dev/null

LOG "DHCP server started (PID $(cat /tmp/dnsmasq-sensors.pid 2>/dev/null))"

# ============================================================
# STEP 3: Wait for MikroTik
# ============================================================
LOG "=== STEP 3: MikroTik Switch ==="

MKTK="sshpass -p '8RKUP2PUT9' ssh -o StrictHostKeyChecking=no -o ConnectTimeout=5 admin@169.254.100.254"

# Wait for MikroTik (boots slower than Thor — up to 90s)
for i in $(seq 1 30); do
    if ping -c1 -W1 169.254.100.254 >/dev/null 2>&1; then
        LOG "MikroTik reachable after ${i}x3s"
        break
    fi
    [ $((i % 5)) -eq 0 ] && LOG "Waiting for MikroTik... (${i}/30)"
    sleep 3
done

if ! ping -c1 -W1 169.254.100.254 >/dev/null 2>&1; then
    LOG "WARN: MikroTik not reachable — skipping switch config"
else
    # MikroTik config persists across reboots. This is just verification.
    # Breakout ports sfp28-9/11/13/15 = 10G-baseCR (saved in RouterOS)
    # All sensor ports sfp28-1 to sfp28-8 in bridgeLocal (saved in RouterOS)
    eval $MKTK '"/interface bridge set bridgeLocal protocol-mode=none"' 2>/dev/null
    LOG "MikroTik OK"
fi

# ============================================================
# STEP 3.5: WAN uplink via RUT50 (Teltonika 5G router) through MikroTik
# ============================================================
# Topology (corrected 2026-04-13 18:00):
#   RUT50 LAN (1G copper) → MikroTik sfp28-12 (S-RJ01 module) → bridgeWAN
#   MikroTik has DHCP client on bridgeWAN (lease 192.168.1.191/24 from RUT50)
#   MikroTik has srcnat masquerade rule out-interface=bridgeWAN (pre-configured)
#   Thor reaches internet by sending via 169.254.100.254 → MikroTik NATs to RUT50
#
# bridgeWAN is deliberately NOT bridged into bridgeLocal (would create a second
# DHCP server racing with our dnsmasq on the sensor L2). Instead MikroTik acts
# as a router between bridgeLocal (sensors + Thor) and bridgeWAN (RUT50 uplink).
#
# Guarded on "MikroTik reachable AND MikroTik itself can reach 1.1.1.1" so this
# is a no-op when the RUT50 5G carrier is not delivering packets — we don't
# want to replace a working eero route with a dead cellular path.
LOG "=== STEP 3.5: RUT50 WAN uplink via MikroTik ==="
GW=169.254.100.254
if ping -c1 -W1 $GW >/dev/null 2>&1; then
    # Probe internet reachability from the MikroTik itself. This tests the
    # entire cellular path without depending on Thor's default route selection.
    if eval $MKTK '"/tool ping 1.1.1.1 count=2 interface=bridgeWAN"' 2>/dev/null \
           | grep -q 'received=[12]'; then
        LOG "RUT50 5G uplink healthy (MikroTik can reach 1.1.1.1 via bridgeWAN)"
        ip route del default via $GW dev $IFACE metric 50 2>/dev/null
        if ip route add default via $GW dev $IFACE metric 50 2>/dev/null; then
            LOG "Default route installed: default via $GW dev $IFACE metric 50"
            if ping -c1 -W3 1.1.1.1 >/dev/null 2>&1; then
                LOG "Internet reachable from Thor via RUT50 (1.1.1.1 OK)"
            else
                LOG "WARN: route installed but Thor cannot ping 1.1.1.1 — check rp_filter / NAT"
            fi
        else
            LOG "WARN: failed to install default route via $GW"
        fi
    else
        LOG "WARN: MikroTik cannot reach 1.1.1.1 via bridgeWAN — RUT50 5G not carrying traffic"
        LOG "  Check RUT50: SIM inserted, APN configured, 5G registered, data plan active"
        LOG "  (boot continues; default route stays as whatever else is present)"
    fi
else
    LOG "WARN: MikroTik unreachable — cannot install RUT50 default route"
fi

# ============================================================
# STEP 4: Tailscale Firewall
# ============================================================
LOG "=== STEP 4: Tailscale Firewall ==="
if ! iptables -L ts-input -n 2>/dev/null | grep -q "100.115.28.97"; then
    iptables -I ts-input 2 -s 100.115.28.97 -j ACCEPT 2>/dev/null || true
    LOG "iPad IP added to Tailscale firewall"
else
    LOG "iPad IP already in firewall"
fi

# ============================================================
# STEP 5: Discover Sensors
# ============================================================
LOG "=== STEP 5: Sensor Discovery ==="
LOG "Waiting 60s for cameras to boot and get DHCP leases..."
sleep 60

# Aravis scan
LOG "Aravis scan:"
arv-tool-0.8 2>/dev/null | while read line; do LOG "  $line"; done

# Ping all sensors
LOG "Ping check:"
for pair in \
    "MikroTik:169.254.100.254" \
    "A70-1:169.254.20.1" \
    "A70-2:169.254.20.2" \
    "Blackfly1:169.254.1.20" \
    "Blackfly2:169.254.1.21" \
    "A6701-1:169.254.128.202" \
    "A6701-2:169.254.249.149" \
    "Lucid1:169.254.20.23" \
    "Lucid2:169.254.20.14"; do
    name="${pair%%:*}"; ip="${pair##*:}"
    if ping -c1 -W2 $ip >/dev/null 2>&1; then
        LOG "  UP   $name ($ip)"
    else
        LOG "  DOWN $name ($ip)"
    fi
done

# DHCP leases
LOG "DHCP leases:"
cat /var/lib/misc/dnsmasq.leases 2>/dev/null | while read line; do LOG "  $line"; done

# Bridge MACs
LOG "MikroTik bridge MACs:"
eval $MKTK '"/interface bridge host print where !local"' 2>/dev/null | grep "D E" | while read line; do
    LOG "  $line"
done

LOG "=== BOOT COMPLETE ==="
