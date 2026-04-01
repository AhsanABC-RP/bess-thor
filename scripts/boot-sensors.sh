#!/bin/bash
# BESS Thor Sensor Network Bootstrap
# Run after every boot or network reset
# Usage: sudo bash boot-sensors.sh
set +e  # Don't exit on errors — sensors may be offline

LOG() { echo "[$(date +%H:%M:%S)] $*"; }

# ============================================================
# STEP 1: Thor Host Network
# ============================================================
LOG "=== STEP 1: Thor Host Network ==="

IFACE="mgbe0_0"

# IPs
ip addr show $IFACE | grep -q "169.254.100.1" || ip addr add 169.254.100.1/16 dev $IFACE
ip addr show $IFACE | grep -q "192.168.1.50"  || ip addr add 192.168.1.50/24 dev $IFACE
ip addr show $IFACE | grep -q "192.168.2.50"  || ip addr add 192.168.2.50/24 dev $IFACE
ip addr show $IFACE | grep -q "192.168.127.1" || ip addr add 192.168.127.1/24 dev $IFACE

# MTU (needs link bounce)
CUR_MTU=$(ip link show $IFACE | grep -oP 'mtu \K[0-9]+')
if [ "$CUR_MTU" -lt 8900 ]; then
    LOG "Setting MTU 9000 on $IFACE (was $CUR_MTU)..."
    ip link set $IFACE down
    ip link set $IFACE mtu 9000
    ip link set $IFACE up
    sleep 3
fi

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

LOG "Host network: OK (MTU=$(ip link show $IFACE | grep -oP 'mtu \K[0-9]+'))"

# ============================================================
# STEP 2: MikroTik Switch Config
# ============================================================
LOG "=== STEP 2: MikroTik Switch ==="

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
fi

# Configure MikroTik ports (only if reachable)
if ping -c1 -W1 169.254.100.254 >/dev/null 2>&1; then
    eval $MKTK '"
    /interface bridge set bridgeLocal protocol-mode=none mtu=9000
    /interface ethernet set sfp28-16 auto-negotiation=no speed=10G-baseCR fec-mode=off
    /interface ethernet set sfp28-2 auto-negotiation=no speed=1G-baseT-full
    /interface ethernet set sfp28-4 auto-negotiation=yes speed=10G-baseT
    "' 2>/dev/null
    LOG "MikroTik configured: bridge mtu=9000, sfp28-16=10G, sfp28-2=1G, sfp28-4=10G"
fi

# ============================================================
# STEP 3: FS Switch A70 Port Fix
# ============================================================
LOG "=== STEP 3: FS Switch ==="

# FS switch config persists across reboots (saved with `write`).
# A70 ports 2/4 forced to speed 100/duplex full — only reconfigure if A70s don't link.
# To reconfigure manually: telnet 192.168.1.1 → admin/admin → enable → conf
if ping -c1 -W2 192.168.1.1 >/dev/null 2>&1; then
    LOG "FS switch reachable at 192.168.1.1 (config persisted from write)"
else
    LOG "WARN: FS switch not reachable at 192.168.1.1"
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
LOG "Waiting 60s for cameras to boot..."
sleep 60

# Aravis scan
LOG "Aravis scan:"
arv-tool-0.8 2>/dev/null | while read line; do LOG "  $line"; done

# Ping known sensors
LOG "Ping check:"
for pair in \
    "MikroTik:169.254.100.254" \
    "Blackfly1:169.254.1.20" \
    "Blackfly2:169.254.1.21" \
    "Ouster:169.254.70.119"; do
    name="${pair%%:*}"; ip="${pair##*:}"
    if ping -c1 -W1 $ip >/dev/null 2>&1; then
        LOG "  UP   $name ($ip)"
    else
        LOG "  DOWN $name ($ip)"
    fi
done

# Dynamic discovery for cameras with changing IPs
LOG "MikroTik bridge MACs:"
eval $MKTK '"/interface bridge host print"' 2>/dev/null | grep "D E" | while read line; do
    LOG "  $line"
done

LOG "=== BOOT COMPLETE ==="
