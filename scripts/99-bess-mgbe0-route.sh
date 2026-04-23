#!/bin/bash
# Force mgbe0_0 default route to metric 50 so wired wins over WiFi (700).
# NM auto-bumps link-local route-metric by 20000 (mgbe0_0 IP is 169.254.x.x),
# so ipv4.route-metric=50 lands in kernel as 20050 — WiFi metric 700 wins
# even when wired path is healthy. This dispatcher overrides the metric on
# every mgbe0_0 up event so Thor → MikroTik → RUTX50 5G is preferred.
IFACE="$1"
ACTION="$2"
[ "$IFACE" = "mgbe0_0" ] || exit 0
[ "$ACTION" = "up" ] || exit 0
sleep 2
ip route del default via 169.254.100.254 dev mgbe0_0 2>/dev/null
ip route add default via 169.254.100.254 dev mgbe0_0 metric 50 2>/dev/null
logger -t bess-mgbe0-route "applied default-route metric=50 on mgbe0_0 up"
