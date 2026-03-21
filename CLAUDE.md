# BESS Thor Dev Kit - ROS2 Sensing Stack

## Quick Reference

| Item | Value |
|------|-------|
| **Foxglove URL** | `wss://thor.tail902411.ts.net:8765` |
| **GitHub Repo** | https://github.com/AhsanABC-RP/bess-thor |
| **Platform** | NVIDIA Thor Dev Kit (Grace Blackwell, ARM64) |
| **ROS2** | Jazzy (containers) / Humble (some legacy) |
| **DDS** | CycloneDDS (Ouster) / FastRTPS (cameras) |

## Working Sensors

| Sensor | IP | Status | Topic |
|--------|-----|--------|-------|
| Ouster OS1-128 | 169.254.70.119 | **WORKING** ~5Hz | `/ouster/points` |
| A6701 #2 | 169.254.252.205 | Connecting | `/thermal/camera2/image_raw` |
| Blackfly #1 | 169.254.1.20 | Connecting | `/blackfly/camera1/image_raw` |
| Blackfly #2 | 169.254.1.21 | Connecting | `/blackfly/camera2/image_raw` |

## Network Configuration

### Active Interface
```
mgbe0_0: 169.254.100.1/16  MTU 1466
```

### CRITICAL: Disable Duplicate Interfaces
Thor has 4x 10GbE to MikroTik. Only use mgbe0_0:
```bash
sudo ip link set mgbe1_0 down
sudo ip link set mgbe2_0 down
sudo ip link set mgbe3_0 down
```
Without this, cameras appear 4x (multicast on all interfaces).

## Start Commands

```bash
# Start Ouster only (working)
docker compose -f docker-compose.thor.yml up -d ouster

# Start all sensors
docker compose -f docker-compose.thor.yml --profile sensors up -d

# Start with Foxglove
docker compose -f docker-compose.thor.yml --profile sensors --profile viz up -d

# Check status
docker ps --format "table {{.Names}}\t{{.Status}}"

# View logs
docker logs ouster --tail 20
docker logs thermal2 --tail 20 2>&1 | grep -v "sequence size"
```

## Foxglove Connection

### TLS Certificate (Tailscale-issued, browser-trusted)
```bash
# Regenerate if expired
sudo tailscale cert \
  --cert-file /home/thor/bess/config/certs/cert.pem \
  --key-file /home/thor/bess/config/certs/key.pem \
  thor.tail902411.ts.net
```

### iPad Firewall Fix
If iPad can't connect via Tailscale:
```bash
# Get iPad Tailscale IP
tailscale status | grep ipad

# Add firewall rule
sudo iptables -I ts-input 2 -s <ipad-tailscale-ip> -j ACCEPT
```

## Known Issues

### 1. "sequence size exceeds remaining buffer"
FastRTPS CDR deserialization error. Large images overwhelm buffers.
- Occurs on all FastRTPS containers
- Does NOT affect CycloneDDS containers (Ouster works)
- Camera images may fail to publish

### 2. Camera GenTL -1011 Errors
"Failed waiting for EventData on NEW_BUFFER_DATA"
- Network issue - camera not streaming image data
- Check: `ping -c3 <camera_ip>`
- Check: Only mgbe0_0 is up (disable others)

### 3. Multiple Camera Instances
Spinnaker sees 20+ cameras instead of 4.
- Cause: All 4 mgbe interfaces receiving multicast
- Fix: `sudo ip link set mgbe{1,2,3}_0 down`

## System Resources

```bash
# Check resources
top -bn1 | head -5
free -h
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
df -h /
```

Thor typical idle: 5GB RAM / 75% CPU idle / 743GB disk free

## Recording MCAP

```bash
# Start recorder (after sensors are streaming)
docker compose -f docker-compose.thor.yml up -d recorder

# Or manual recording
ros2 bag record -o /data/recording \
  /ouster/points \
  /thermal/camera2/image_raw/compressed \
  --storage mcap
```

## File Structure

```
/home/thor/bess/
├── CLAUDE.md                    # This file
├── docker-compose.thor.yml      # Main compose file
├── config/
│   ├── ouster/params.yaml       # Ouster config (has sensor_hostname)
│   ├── fastrtps.xml             # FastRTPS large buffer config
│   ├── cyclonedds.xml           # CycloneDDS config
│   └── certs/                   # TLS certs (gitignored)
├── dockerfiles/                 # Container build files
└── data/                        # Recordings (gitignored)
```

## Commit Protocol

Always push to bess-thor repo:
```bash
cd /home/thor/bess
git add -A
git commit -m "Description"
git push origin main
```

DO NOT push Thor changes to bess_platform (production vehicles).
