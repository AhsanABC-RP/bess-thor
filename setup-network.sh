#!/bin/bash
# BESS Network Setup Script - Run with sudo
# Configures network interfaces for FLIR cameras and installs docker-compose

set -e

echo "=== BESS Network Setup ==="

# Install docker-compose plugin
echo "Installing docker-compose plugin..."
apt-get update
apt-get install -y docker-compose-plugin

# Configure ethernet interface for FLIR cameras (link-local)
echo ""
echo "Available network interfaces:"
ip link show | grep -E "^[0-9]+:" | awk '{print $2}' | tr -d ':'

echo ""
echo "Current interface status:"
for iface in enP2p1s0 mgbe0_0 mgbe1_0 mgbe2_0 mgbe3_0; do
    state=$(ip link show $iface 2>/dev/null | grep -oP 'state \K\w+' || echo "N/A")
    ip=$(ip addr show $iface 2>/dev/null | grep -oP 'inet \K[\d.]+' || echo "none")
    echo "  $iface: state=$state, ip=$ip"
done

echo ""
echo "Setting up link-local IP on enP2p1s0 for FLIR camera discovery..."
ip addr add 169.254.1.1/16 dev enP2p1s0 2>/dev/null || echo "  (IP may already be assigned)"

# Bring up 10GbE interfaces if cameras are connected there
echo ""
echo "Bringing up 10GbE interfaces (mgbe0_0 - mgbe3_0)..."
for iface in mgbe0_0 mgbe1_0 mgbe2_0 mgbe3_0; do
    ip link set $iface up 2>/dev/null || echo "  Could not bring up $iface"
    ip addr add 169.254.2.$(($(echo $iface | grep -oP '\d' | head -1) + 1))/16 dev $iface 2>/dev/null || true
done

# Configure receive buffer sizes for GigE Vision
echo ""
echo "Configuring network buffers for GigE Vision cameras..."
sysctl -w net.core.rmem_max=26214400
sysctl -w net.core.rmem_default=26214400

echo ""
echo "=== Setup Complete ==="
echo ""
echo "Next steps:"
echo "  1. Connect your FLIR cameras to the network"
echo "  2. Start the stack: cd ~/bess && docker compose up -d"
echo "  3. Check logs: docker compose logs -f"
echo "  4. View ROS topics: docker exec bess-cameras ros2 topic list"
