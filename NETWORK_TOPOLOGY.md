# Thor Dev Kit - Network Topology & Camera Configuration

## Network Interfaces

| Interface | IP Address | Subnet | MTU | Purpose |
|-----------|------------|--------|-----|---------|
| mgbe0_0 | 169.254.100.1 | /16 | 1466 | GigE cameras (link-local) |
| mgbe1_0 | 192.168.10.1 | /24 | 1466 | A70-89900590 (static IP) |
| mgbe2_0 | - | - | 1466 | Unused |
| mgbe3_0 | - | - | 1466 | Unused |
| enP2p1s0 | - | - | 1500 | Unused GigE |
| wlP1p1s0 | 192.168.4.104 | /22 | 1500 | WiFi |
| tailscale0 | 100.96.161.119 | /32 | 1280 | VPN |

## PROBLEM: MTU is 1466, NOT 9088
mgbe interfaces have MTU 1466 - jumbo frames will NOT work.
Code setting packet size 8972 will FAIL.

## Cameras Visible (arv-tool)

| Camera | Model | Serial | IP | MAC | Interface |
|--------|-------|--------|-----|-----|-----------|
| A6701 #2 | Xsc Series | - | 169.254.252.205 | 00:11:1c:05:8f:27 | mgbe0_0 |
| A70 #1 | FLIR A70 | 89900590 | 192.168.10.2 | 00:40:7f:04:fa:7b | mgbe1_0 |
| Blackfly #1 | BFS-PGE-120S6C | 25073269 | 169.254.1.20 | 2c:dd:a3:7e:96:75 | mgbe0_0 |
| Blackfly #2 | BFS-PGE-120S6C | 25073270 | 169.254.1.21 | 2c:dd:a3:7e:96:76 | mgbe0_0 |
| Lucid Atlas | ATX650G-C | 231800449 | 169.254.232.169 | 1c:0f:af:07:a9:e7 | mgbe0_0 |

## Cameras NOT Visible

| Camera | Expected MAC | Expected IP | Status |
|--------|--------------|-------------|--------|
| A6701 #1 | 00:11:1c:05:8f:20 | 169.254.x.x | NOT ON NETWORK |

## Docker Compose Camera Config

| Service | Camera | MAC Address | Expected IP | SDK |
|---------|--------|-------------|-------------|-----|
| thermal1 | A6701 #1 | 00:11:1c:05:8f:20 | link-local | Spinnaker |
| thermal2 | A6701 #2 | 00:11:1c:05:8f:27 | 169.254.252.205 | Spinnaker |
| thermal3 | A70 #1 | 00:40:7f:04:fa:7b | 192.168.10.2 | Spinnaker |
| thermal4 | A70 #2 | 00:40:7f:10:a2:bc | 169.254.101.156 | Spinnaker |
| lucid1 | Atlas | 1c:0f:af:07:a9:e7 | 169.254.232.169 | Arena SDK |
| blackfly1 | BFS #1 | 2c:dd:a3:7e:96:75 | 169.254.1.20 | Spinnaker |
| blackfly2 | BFS #2 | 2c:dd:a3:7e:96:76 | 169.254.1.21 | Spinnaker |

## Routing

| Destination | Via | Interface |
|-------------|-----|-----------|
| 169.254.0.0/16 | direct | mgbe0_0 |
| 192.168.10.0/24 | direct | mgbe1_0 |
| default | 192.168.4.1 | wlP1p1s0 (WiFi) |

## SDK Requirements

| Camera Type | Required SDK | Container Image |
|-------------|--------------|-----------------|
| Lucid Atlas | Arena SDK | bess-lucid:jazzy |
| FLIR A6701 | Spinnaker (PySpin) | bess-thermal-spinnaker:jazzy |
| FLIR A70 | Spinnaker (PySpin) | bess-thermal-spinnaker:jazzy |
| Blackfly S | Spinnaker (PySpin) | bess-thermal-spinnaker:jazzy |

## Current Issues

1. **MTU 1466**: mgbe interfaces have MTU 1466, not 9088 - jumbo frames won't work
2. **A6701 #1 missing**: MAC 00:11:1c:05:8f:20 not responding on any interface

## Working Cameras (as of 2026-03-08)

| Camera | Service | Rate | Compressed Topic |
|--------|---------|------|------------------|
| A6701 #1 | thermal1 | ~15 Hz | /thermal/camera1/image_raw/compressed |
| A6701 #2 | thermal2 | ~15 Hz | /thermal/camera2/image_raw/compressed |
| A70 #1 | thermal3 | 15 Hz | /thermal/camera3/image_raw/compressed |
| A70 #2 | thermal4 | 15 Hz | /thermal/camera4/image_raw/compressed |
| Lucid Atlas | lucid1 | 5 Hz | /lucid1/camera_driver/image_raw/compressed |
| Blackfly #1 | blackfly1 | 5 Hz | /blackfly/camera1/image_raw/compressed |
| Blackfly #2 | blackfly2 | 5 Hz | /blackfly/camera2/image_raw/compressed |

## Compression Services

Each camera has a compression container (compress-*) that subscribes to raw images
and publishes JPEG compressed versions for Foxglove streaming.

Full resolution raw images are too large for WiFi - use compressed topics in Foxglove.
