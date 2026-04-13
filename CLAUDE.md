# BESS Thor — Working Repo

This is the Thor dev-kit working repository. For full project rules, behaviour
guardrails, SDK matrix, network topology, hardware spec, and the mandatory
changelog, read `/home/thor/CLAUDE.md` first — it is the single source of
truth for Thor. This file is a short repo-level orientation.

## Quick Reference

| Item | Value |
|------|-------|
| Host working dir | `/home/thor/bess` |
| Remote | `git@github.com:AhsanABC-RP/bess-thor.git` (branch `main`) |
| Main compose | `docker-compose.thor.yml` |
| Project rules | `/home/thor/CLAUDE.md` (READ FIRST) |
| Vehicle doc | `/home/thor/bess_platform/vehicles/thor/CLAUDE.md` |
| Foxglove URL | `wss://thor.tail902411.ts.net:8765` |
| Platform | NVIDIA Thor Dev Kit (Grace Blackwell, ARM64, Ubuntu 24.04, kernel 6.8.12-tegra) |
| ROS 2 | Jazzy (primary) / Humble (thermal+cameras legacy) |
| DDS | CycloneDDS (see `config/cyclonedds.xml`) |

## Stack Operation

```bash
cd /home/thor/bess

# All sensors
docker compose -f docker-compose.thor.yml --profile sensors up -d

# Full stack (sensors + SLAM + viz + recording)
docker compose -f docker-compose.thor.yml --profile full up -d

# Status
docker compose -f docker-compose.thor.yml ps
```

Profiles available: `sensors`, `slam`, `viz`, `record`, `full`, `fast-lio`,
`glim-deprecated` (GLIM is deprecated for Ouster OS1-128 — too sparse).

## What's running (2026-04-13)

### Sensors
- **Ouster OS1-128** (`ouster`) — `/ouster/points` + `/ouster/imu` @ 10Hz, `TIME_FROM_ROS_TIME`
- **A6701 #1/#2** (`thermal1`, `thermal2`) — `/thermal/cameraN/image_raw` @ ~19Hz via Spinnaker SDK (MAC-matched)
- **A70 #1/#2** (`thermal3`, `thermal4`) — `/thermal/cameraN/image_raw` via Aravis, MAC-matched. A70 PTP node is non-writable (hardware limit), driver stamps in ROS time
- **Blackfly S #2** (`blackfly2`) — `/blackfly/camera2/image_raw` @ 5Hz via spinnaker_camera_driver. **Blackfly #1 is open** (see `/home/thor/CLAUDE.md` Current Issues #15)
- **Lucid Atlas #2** (`lucid2`) — `/lucid2/camera_driver/image_raw` @ 5Hz via Arena SDK. **Lucid #1 sfp28-7 rx-loss** (Current Issues #16)
- **MicroStrain GQ7** (`microstrain`) — `/imu/data` @ 100Hz, `/gnss_{1,2}/*` via USB direct. After USB re-enum the container needs `up --force-recreate`, not `restart`
- **Thermal colormap** (`thermal-colormap`) — inferno overlay on all four thermal cams
- **Thermal temp publishers** — `/thermal/cameraN/{temperature,housing_temperature}` at 15s period. A6701 reports Stirling-cooled FPA (healthy ~-200°C), A70 reports body temp (healthy 25-50°C)

### SLAM
- **FAST-LIO2** (`fast-lio`) — primary. `/slam/odometry` @ 10Hz, `base_link` TF. `map_en: false` — never re-enable, it chokes CycloneDDS
- **DLIO** (`dlio`) — alternative. `/dlio/odom_node/odom` @ ~90Hz, `dlio_base_link` TF. Uses `/ouster/imu` (not GQ7) for monotonic stamps + co-located extrinsic
- **Map accumulator** (`slam-map-accumulator`) — subscribes `/slam/cloud_registered`, publishes bounded `/slam/map_voxel` every 2s with TRANSIENT_LOCAL durability. 0.25m voxel, 2M voxel cap, FIFO evict

### Infrastructure
- **Foxglove bridge** (`foxglove`) — TLS via Tailscale cert, `wss://thor.tail902411.ts.net:8765`
- **Recorder** (`recorder`) — MCAP to `/mnt/bess-usb/bags` (7.3T ext4 on HighPoint RM110). Records Ouster, IMU, all four thermal cams (raw + color + temperature + housing_temperature), both Lucids, Blackflys, GNSS, SLAM topics
- **Soak monitor** (`scripts/soak_monitor.sh`) — ptp4l slave-offset + MikroTik SFP cage temp + thermal cam FPA watchdog (A6701 cryocooler fail, A70 bay overheat)
- **PTP grandmaster** — `ptp4l-mgbe0.service` + `phc2sys-mgbe0.service` on host. `phc2sys -S 0` (never step), `utc_offset 0` announced, PHC1 is GM at `4cbb47.fffe.0dbe94`

## SDK matrix (MANDATORY — wrong SDK locks hardware)

| Camera | SDK | Container |
|--------|-----|-----------|
| Lucid Atlas | Arena SDK | `bess-lucid:jazzy` |
| FLIR A6701 (cooled) | Spinnaker (PySpin) | `bess-thermal-spinnaker:jazzy` |
| FLIR A70 (uncooled) | Aravis | `bess-thermal:jazzy` |
| Blackfly S | spinnaker_camera_driver (C++) | `bess-cameras:jazzy` |

See `/home/thor/CLAUDE.md` HARD STOPS section for forbidden commands.

## Key config files

```
config/
├── ouster/params.yaml            TIME_FROM_ROS_TIME (PTP re-anchor needs phc_ctl + cold-boot)
├── dlio/dlio_thor.yaml           dlio_odom / dlio_base_link frames
├── cyclonedds.xml                DDS tuning
├── fastrtps.xml                  legacy containers (not primary)
├── blackfly_s_thor.yaml          PTP SlaveOnly + throughput limit
└── certs/                        Tailscale-issued TLS cert (gitignored)

containers/
├── fast-lio/fast_lio_single.yaml    map_en: false (never re-enable)
├── lucid/lucid_node.py              Arena SDK, PTP enable, MAC-matched
└── cameras/gpu_compress.py          JPEG compression for Foxglove

dockerfiles/thermal/
└── thermal_spinnaker_node_patched.py   SensorTemperature/HousingTemperature
                                         publishers + GevSCPD min(1000, camera_max)
                                         + GevIEEE1588 (A6701 only; A70 non-writable)

scripts/
├── boot-sensors.sh               bess-network.service, runs on boot
├── soak_monitor.sh               soak-time health watchdog
├── slam_map_accumulator.py       bounded unified map for Foxglove
└── bess-time-bootstrap           cold-boot HTTP-Date quorum fetch
```

## Commit protocol

```bash
cd /home/thor/bess
git add -A
git commit -m "Description"
git push origin main
```

DO NOT commit Thor changes to `bess_platform` — that repo's `bess-thor` branch
is the mirrored vehicle record, not the working tree. The Thor vehicle doc at
`bess_platform/vehicles/thor/CLAUDE.md` is maintained for cross-vehicle fleet
reference; keep it in sync when Thor hardware topology changes.

## Recovering from a bad session

If you've lost context mid-session:

1. `docker ps --format 'table {{.Names}}\t{{.Status}}'` — which containers are up
2. `git status --short` — uncommitted work
3. `git log --oneline -10` — what's landed
4. `cat /home/thor/CLAUDE.md` — full project rules + changelog
5. `cat /home/thor/.claude/projects/-home-thor/memory/MEMORY.md` — persisted feedback/learnings
6. `cat /home/thor/bess_platform/vehicles/thor/CLAUDE.md` — vehicle hardware details
7. Then, and only then, touch code. Do NOT patch blind — see GOLDEN RULE in `/home/thor/CLAUDE.md`.
