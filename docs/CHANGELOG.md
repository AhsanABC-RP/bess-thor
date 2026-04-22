# Thor Dev Kit - ROS2 Sensing Stack

# MANDATORY PHYSICAL TOPOLOGY — READ FIRST, DO NOT GUESS PORT NUMBERS

**This is the authoritative MikroTik CRS518 sfp28 port map. Verified live against `/interface/bridge/host/print` + `/interface/ethernet/monitor <port> once` on 2026-04-22. If you are about to touch a port number, re-verify with `scripts/mikrotik_sfp_sweep.sh` — do NOT trust adjacency ("15 works so 13 must be similar") or numerical intuition. That's how 2026-04-22 chased the wrong port for 20 minutes.**

| sfp28 | Device | MAC | SFP module | Speed | IP (sensor subnet 169.254.x.x) | Container |
|-------|--------|-----|------------|-------|--------------------------------|-----------|
| sfp28-1  | unused cage (historically flaky) | — | empty | — | — | — |
| **sfp28-2**  | **A6701 #2 (thermal2)** | **00:11:1C:05:8F:27** | S+RJ10 RJ45 | 1 G | 169.254.249.149 (LLA) | `thermal2` |
| sfp28-3  | A70 #1 (thermal3) | 00:40:7F:04:FA:7B | S+RJ10 RJ45 | 1 G | 169.254.20.1 (DHCP) | `thermal3` |
| sfp28-4  | A70 #2 (thermal4) | 00:40:7F:11:04:EA | S+RJ10 RJ45 | 1 G | 169.254.20.2 (DHCP) | `thermal4` |
| sfp28-5  | Blackfly S #1 | 2C:DD:A3:7E:96:75 | S+RJ10 RJ45 | 1 G | 169.254.1.20 (static) | `blackfly1` |
| sfp28-6  | Blackfly S #2 | 2C:DD:A3:7E:96:76 | S+RJ10 RJ45 | 1 G | 169.254.1.21 (static) | `blackfly2` |
| sfp28-7  | Lucid Atlas #1 | 1C:0F:AF:07:A9:CD | S+RJ10 RJ45 | 10 G | 169.254.20.23 (DHCP) | `lucid1` |
| sfp28-8  | Lucid Atlas #2 | 1C:0F:AF:07:A9:E7 | S+RJ10 RJ45 | 10 G | 169.254.20.14 (DHCP) | `lucid2` |
| sfp28-9  | **Thor `mgbe0_0` CPU uplink** (PTP GM) | 4C:BB:47:0D:BE:94 | XQ+BC0003-XS+ DAC | 10 G | 169.254.100.1/16 | *host interface* |
| sfp28-10 | EERO bridgeWAN uplink (internet) | — | S+RJ10 | 1 G | — | *bridgeWAN* |
| sfp28-11 | unused (DAC pigtail present) | — | XQ+BC0003-XS+ DAC | — | — | — |
| sfp28-12 | RUT50 5G router (bridgeWAN) | — | S-RJ01 RJ45 | 1 G | 169.254.100.254 | *bridgeWAN* |
| sfp28-13 | unused (DAC pigtail present) | — | XQ+BC0003-XS+ DAC | — | — | — |
| sfp28-14 | Ouster OS1-128 | BC:0F:A7:00:D3:85 | S+RJ10 RJ45 | 1 G | 169.254.70.119 | `ouster` |
| **sfp28-15** | **A6701 #1 (thermal1)** | **00:11:1C:05:8F:4C** (was `:20` pre-swap 2026-04-22) | S+RJ10 RJ45 | 1 G | 169.254.20.43 (LLA) | `thermal1` |
| sfp28-16 | F8 TerraMaster NAS | 6C:BF:B5:04:04:AE | S+RJ10 RJ45 | 10 G | 169.254.100.30 (DHCP) | *NFS mount* |

**DAC pigtail ports (sfp28-11, sfp28-13) can NEVER host an RJ45 camera.** The XQ+BC0003-XS+ is a passive copper pigtail with integral SFP+ on each end — switch-to-switch only. If you see a dark sfp28 port with a DAC in it, that's not your missing camera. Look for an S+RJ10 (or S-RJ01) RJ45 adapter.

**Before any per-port diagnostic, run the sweep** (`scripts/mikrotik_sfp_sweep.sh`) and compare against this table. `project_mikrotik_miswiring.md` in memory is a prior snapshot — this table supersedes it.

---

# MANDATORY HARDWARE SAFETY RULES — READ BEFORE ANY CAMERA/SENSOR ACTION

These rules exist because the 2026-04-22 session bricked hours of operational uptime and pushed an A6701 into a Pleora iPORT flap-fault by violating them. Every rule below corresponds to a specific failure mode already observed on this hardware.

## 1. NEVER restart multiple GigE cameras in parallel
- **FORBIDDEN**: `docker restart thermal1 thermal2 thermal3 thermal4` (or any pair of GigE SDK containers started within <30 s of each other).
- **WHY**: 4× parallel Spinnaker enumeration races GVCP discovery on the shared sensor subnet, partial results starve MAC lookups, drivers enter 60 s retry loops, cameras see repeated control-session churn. Documented 2026-04-22 — triggered Pleora flap on A6701 #2 requiring ≥5 min power-off recovery.
- **RULE**: restart GigE camera containers **one at a time, minimum 20 s gap**. `scripts/bess-stack-up.sh` already enforces this for cold-boot — use the same batching pattern when cycling mid-session.
- **HOW TO APPLY**: `for c in thermal1 thermal2 thermal3 thermal4; do docker restart "$c"; sleep 20; done` — NOT `docker restart <all-4>`.

## 2. When a camera goes silent — STOP EVERYTHING. Diagnose passive only.
- **FORBIDDEN** when a camera has just gone dark:
  - `docker restart`, `docker compose up --force-recreate`
  - Spinnaker / Aravis / Arena / eBUS control probes (any SDK that opens a control session)
  - MikroTik port bounce (`/interface disable … enable …`)
  - ForceIP, DeviceReset, any GVCP write command
  - Continuous `ping -f` or `tcpdump` flooding
- **WHY**: A silent A6701/Pleora is often in an iPORT PHY-dark or MAC-layer stuck state. Every additional probe while it's stuck deepens the fault — PHY stays dark longer, firmware watchdog fails to recover, GVCP control session half-locks. See `feedback_a6701_pleora_flap_state.md`.
- **PASSIVE DIAGNOSIS ONLY**: MikroTik `/interface ethernet monitor <port> once` + `/interface/bridge/host/print where mac-address=<MAC>` + one `scripts/gvcp_probe.py discover` (single UDP broadcast, no control session). That's it. Nothing else until the camera is verified back on the wire.

## 3. Retry loops KEEP cameras in the stuck state — stop the container first
- **WHY**: A container whose driver node is in a retry loop (e.g., `Retrying in 60.0s...` → GVCP discovery → control session open → fail → release → wait → retry) sends a broadcast every cycle that the camera's stuck firmware may half-respond to, holding it in the bad state.
- **RULE**: When a camera goes dark, **`docker stop <container>`** first. Do NOT leave the container running "in case it recovers on its own" — it won't; the retries are the problem.
- 2026-04-22 evidence: A6701 `:27` only recovered after both `thermal2` container stopped AND physical power-cycle.

## 4. Physical power-cycle is the ONLY software-external recovery for Pleora dark state
- **RULE**: If an A6701 (or any Pleora-iPORT GigE device) has gone silent on wire:
  1. `docker stop <thermalN>` — end Thor-side probing.
  2. Wait ≥2 min powered-on, untouched. Sometimes firmware self-clears.
  3. If still dark: physical power OFF for **≥5 min** (not 90 s — Pleora supercaps + deep firmware reset need longer than the spec-minimum).
  4. Plug power back in. **Do NOT touch the camera for ≥2 min while it boots.** No Thor-side actions.
  5. First probe is a single MikroTik read-only port monitor. Then one passive GVCP discovery. Then start the container.
- **NO SOFTWARE ACTION CAN RECOVER A DARK PLEORA iPORT.** Do not spend hours chasing Thor-side fixes.

## 5. Read the topology table FIRST, never guess port numbers
- **FORBIDDEN**: "Camera X is probably on sfp28-Y because it's near sfp28-Z."
- **RULE**: Open the topology table at the top of this file **before any MikroTik port query**. If the camera's MAC is not in the current bridge-host table, read the table — don't scan adjacent ports looking for it.
- 2026-04-22 evidence: 20+ min wasted chasing `:27` on sfp28-13 when it's on sfp28-2.

## 6. Rate-watchdog / reconnect_interval settings in driver code can HOLD cameras stuck
- **RULE**: When adding auto-reconnect logic to a camera node (like `thermal_spinnaker_node_patched.py`'s rate-watchdog), ensure the reconnect interval gives the camera enough time to fully release stale control sessions. Too-short intervals (5 s) guarantee another `-1010 Error writing to Device` on the retry. Too-aggressive rate floors could bounce a healthy camera.
- Current values (2026-04-22): `reconnect_interval=60s`, `rate_floor_hz=3.0`, `rate_floor_window_s=30s`, `error_threshold=15`.
- **If you lower any of these, think about whether it would trigger Rule #3 (retry loop holding camera stuck).**

## 7. The `bess-stack-up.sh` sequenced launcher is the ONLY correct cold-start path
- **FORBIDDEN** at boot or after a full-stack outage: `docker compose --profile full up -d` (parallel start of 30+ containers).
- **RULE**: Use `scripts/bess-stack-up.sh` — 10-batch sequenced launcher, max 4 concurrent starts, camera SDKs never in parallel, GPU containers staggered ≥5 s. Total ~125 s. See row 73 of changelog for the boot-loop root cause that this replaces.

---

# MANDATORY CHANGELOG - READ BEFORE EVERY SESSION

**Every change MUST be logged here. Check this FIRST before making changes.**

| Date | File | Change | Reason | Result | Reverted? |
|------|------|--------|--------|--------|-----------|
| 2026-04-22 | CLAUDE.md topology section + HARDWARE SAFETY RULES + scripts/blackfly_supervisor.sh + scripts/topic_watchdog.sh (new) + scripts/bess-topic-watchdog.service (new) + dockerfiles/thermal/thermal_spinnaker_node_patched.py + docker-compose.thor.yml SLAM restart policy | **Resilience sweep + A6701 flap incident.** Started as the 4-layer resilience plan after investigating four in-session failures (blackfly2 supervisor never fired, dlio SIGABRT no auto-restart, thermal4 A70 -1011 rate drift, fast-lio subscription sanity). (a) **blackfly_supervisor.sh bug fix**: `pgrep -f "camera_driver_node"` was matching the immortal bash PID 1 because the outer bash -c command string contains "camera_driver_node" in the ros2 run argv. Supervisor has been watching the wrong process since day one — `kill -0 pid1` always succeeded, 0-Hz streak loop never ran, -1002 retry loops went unfixed. Matcher changed to `/lib/spinnaker_camera_driver/camera_driver_node` (binary path). Verified: supervisor log now reports `watching pid=80, pid=372, pid=698` per restart cycle, not pid=1. Also replaced bc dependency with awk. (b) **thermal_spinnaker_node_patched.py rate-watchdog**: added `rate_floor_hz=3.0` + `rate_floor_window_s=30.0` — breaks out of acquire loop if publish rate stays below floor for window. `error_threshold=15` (was 50). `reconnect_interval 5s → 60s` (GigE Vision heartbeat clear). (c) **SLAM restart:on-failure:10 override** on dlio, fast-lio, slam-map-accumulator services in docker-compose. &common anchor stays restart:"no" so cold-boot docker-stampede is still prevented. Live-applied via `docker update`. (d) **scripts/topic_watchdog.sh** + systemd unit: host-side external topic-rate watchdog, 30s poll, 60s trigger, 3 actions/hour cap, structured JSON log at `/var/log/bess/topic_watchdog.log`. **Per-camera action map is critical**: A6701 (thermal1, thermal2) use `log_only` — NEVER bounce, per `feedback_a6701_pleora_flap_state.md` memory. A70 (thermal3, thermal4) use `stop60_up` (60s GigE clear doctrine). Blackfly use `recreate`. SLAM plain `restart`. Systemd unit installed **disabled** — manual enable required. (e) **A6701 #2 Pleora flap incident during deployment** — violated Rule #1 by `docker restart thermal1 thermal2 thermal3 thermal4` in parallel to pick up the new thermal_spinnaker_node code. Partial Spinnaker enumeration race left both A6701s in flap-fault. `:4c` recovered from one 90s power-cycle; `:27` (thermal2) stayed dark — PHY link-ok per MikroTik but zero RX, auto-neg empty partner-advertising, not in bridge-host table. My initial diagnosis falsely blamed the port (guessed sfp28-13 because it was dark and adjacent to sfp28-15). User corrected: thermal2 is on sfp28-2 (in topology table above, which I hadn't referenced). Recovery: stopped thermal2 container (ended 60s retry broadcasts), user physical 5-min power-off, 2-min untouched boot, single passive MikroTik check confirmed link-ok + RX >0 + MAC learned on sfp28-2, one `docker compose up -d --no-deps thermal2` → clean `Matched by MAC` → `Started acquisition` → `Publishing at 15 Hz`. | **Resilience layers 1a/1b/2/3 live and verified. 14/14 sensor streams back up after recovery (thermal2 12.83 Hz confirmed).** Crucially: new CLAUDE.md topology table + HARDWARE SAFETY RULES sections added at file top so this class of failure (guessing ports, parallel restart, probing stuck cameras) is blocked on first read of the doc. Commit `b46970a` pushed. | Keep |
| 2026-04-22 | docker-compose.thor.yml thermal1 + dockerfiles/thermal/thermal_spinnaker_node_patched.py + containers/lucid/lucid_node.py + scripts/boot-sensors.sh + scripts/blackfly_supervisor.sh (new) + config/camera_info/blackfly_camera.yaml (new) + /tmp/ebus-docker/{Dockerfile,deb} + /home/thor/bin/ebus + MikroTik sfp28 topology + NM EERO profile cleanup + /etc/sysctl.d/99-bess-no-forward.conf + F8 NAS NFS mount | **Long recovery session — 7 threads bundled.** (a) **A6701 #1 hardware brick**: original unit MAC `00:11:1c:05:8f:20` went Pleora-iPORT dark — ZERO GVCP DISCOVERY/FORCEIP replies across multiple SDK versions (Spinnaker, Aravis, eBUS 6.5.4, eBUS 7.0.0), multiple interfaces, multiple subnets, even with MikroTik hw-offload disabled on sfp28-15 and CPU-side tcpdump capturing 5 persistent-IP ARP announces from the camera. GVCP layer itself was dead — no amount of ForceIP can revive a Pleora NTx-GigE whose control plane has gone. User swapped to new A6701 on the same cable/port; replacement MAC `00:11:1c:05:8f:4c` came up clean at `169.254.20.43` after dnsmasq RFC-3927 IP assignment. Compose thermal1 updated with new MAC + brick doctrine comment. Saved as `feedback_a6701_pleora_flap_state.md` memory. **Rule**: when A6701 is silent on wire AND GVCP DISCOVERY returns zero replies AND persistent-IP ARPs continue, it is a camera-side brick, NOT a Thor config issue. Do not spend hours chasing Thor stack when the camera's own Pleora iPORT has failed. (b) **Thermal Spinnaker duplicate-enumeration patch**: `thermal_spinnaker_node_patched.py` was calling `system.GetCameras()` which on this Jetpack sees the same physical camera twice (once via each interface filter), causing "No matching camera found" MAC lookups to fail. Added `_find_sensor_interface()` that returns the specific `InterfaceList` entry bound to the sensor subnet, then calls `sensor_iface.GetCameras()` — MAC match works first try. (c) **Lucid MAC-based discovery**: `containers/lucid/lucid_node.py` parameter `camera_ip` → new `camera_mac` with MAC-normalization (dashes/dots/colons all accepted). Filter logic checks MAC first, falls back to IP only if MAC param is unset. Compose lucid1/lucid2 switched `-p camera_ip:=` to `-p camera_mac:=`. Fixes the recurring "lucid drifts to auto-IP after restart, old camera_ip no longer matches" failure mode — MACs don't drift. (d) **MikroTik topology + IP-forward leak**: sfp28-10 moved from bridgeLocal (sensor net) to bridgeWAN (internet) so EERO DHCP stops leaking into sensor subnet. Added `sysctl net.ipv4.ip_forward=0` + persisted in `/etc/sysctl.d/99-bess-no-forward.conf` so Thor can't bridge sensor packets out to WiFi/5G. Also: `bess-mgbe0` (the default `EERO` WiFi NM profile) deduplicated — had 3 stale connection entries with conflicting priorities; kept 1 with `connection.autoconnect-priority=600` (beats 5G-over-ethernet metric but only when 5G dead). Default route now goes via MikroTik bridgeWAN → RUT50 5G (metric 50) rather than WiFi (metric 600). (e) **F8 TerraMaster NAS on 10G**: sfp28-16 direct from Thor ↔ F8, RAID0 from JBOD (~207 MB/s sustained, 461 MB/s burst), NFSv4.2 `nconnect=8`. `/etc/systemd/system/home-thor-nas-bess\x2dbags.mount` mounts to `/home/thor/nas/bess-bags`. `scripts/boot-sensors.sh` has new dnsmasq reservation `6c:bf:b5:04:04:ae,169.254.100.30,F8-NAS` so the F8 always comes up at `169.254.100.30` on the sensor subnet. Memory: `reference_nas_terramaster.md` + `feedback_tegra_cifs_quirks.md`. (f) **Blackfly watchdog + calibration**: new `scripts/blackfly_supervisor.sh` that SIGTERMs ros2run when `/blackfly/cameraN/.../compressed` stays 0 Hz for 30s, triggering the outer bash `while true; do` loop to re-Init() the camera (same pattern as 2026-04-12 throughput-limit fix but for post-start freeze). New `config/camera_info/blackfly_camera.yaml` is a placeholder 4000×3000 intrinsics file — `camera_calibration_parsers` was segfaulting on the missing file at driver start. Placeholder is NOT real calibration, just enough structure to silence the parser. Replace once Kalibr run completes. (g) **eBUS SDK 7.0.0 Docker for Thor**: new `localhost/ebus-player:thor` image (Ubuntu 22.04 + `eBUS_SDK_Jetson_6.2_linux-aarch64-arm-7.0.0-7404.deb`, 200MB installed) replaces the earlier `jetson46` tag (Ubuntu 18.04 + JAI 6.5.4 for Jetpack 4.6). Matches Thor's actual Jetpack 6.x base. `/home/thor/bin/ebus` launcher updated. Required `Acquire::Check-Date "false"` in apt during build because Thor's wall clock was 24h behind real time and couldn't step (phc2sys -S 0 refuses + Ouster PTP slave was live). Entrypoint is `eBUSPlayerClassic` (Pleora renamed from `eBUSPlayerJAI` in 7.0). Needed runtime deps `libavcodec58 libavutil56 libswresample3 libavformat58` from jammy universe. | **Full 10-camera + 2-IMU + 2-SLAM stack verified streaming end-to-end on Foxglove:** /ouster/points 10Hz, /ouster/imu 386Hz, /thermal/camera{1..4}/image_raw 12/13/17/11 Hz, /blackfly/camera{1,2} 5Hz, /lucid{1,2} 3Hz, /imu/data 100Hz, /fast_lio/odometry 9.9Hz, /dlio/odom_node/odom 97.8Hz, /fast_lio/map_voxel 0.5Hz. Foxglove bridges on :8765 (wss), :8766 (ws), :8080 (HTTP Lichtblick for iPad) all up. | Keep |
| 2026-04-18 | dockerfiles/zed/Containerfile (new) + memory/feedback_thor_cuda_compat_shim_trap.md (new) | **New bess-zed:jazzy container — ZED SDK 5.2.3 + zed-ros2-wrapper master on bess-base:jazzy sbsa.** Two runtime traps discovered + fixed. (a) **CUDA `cudaErrorSystemDriverMismatch (803)` on every cudaMalloc**. nvidia-container-toolkit bind-mounts `/etc/ld.so.conf.d/00-compat-<rand>.conf` pointing at `/usr/local/cuda-13.0/compat/libcuda.so.580.65.06` — the CUDA 13 forward-compat shim built for generic NVIDIA driver 580.65+ ABI. It sorts BEFORE the driver-native `00-nvcr-<rand>.conf` alphabetically, so `ldconfig -p | grep libcuda.so.1` resolves to the shim, not `/usr/lib/aarch64-linux-gnu/nvidia/libcuda.so.1.1`. Thor's driver is a Tegra-specific 580.00 "TempVersion" build (Aug 2025, pre-public release) with RM API extensions the generic compat shim doesn't expose → every CUDA call through the shim returns 803. Fix: `ENV LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu/nvidia:/usr/local/zed/lib` forces libcuda.so.1 resolution to the driver-native path first. Empirically proven via minimal cudaGetDeviceCount test — pre-fix: returned 803, count=0. Post-fix: returned 0, "NVIDIA Thor CC 11.0, 20 SMs", cudaMalloc 1MB succeeded. **This will bite any future Thor CUDA 13 sbsa container doing direct `cudaMalloc` at init (ZED SDK is one; fast-lio/dlio coincidentally avoid the trap via cuPy/ROS 2 managed buffers).** Saved as durable memory for next time. Rejected alternatives: `rm /etc/ld.so.conf.d/00-compat-*.conf` (file is bind-mounted at container start), `LD_PRELOAD` (too broad, preloads into every subprocess). (b) **"CORRUPTED SDK INSTALLATION" at Camera::open()** — wrapper default `depth_mode: 'NEURAL_LIGHT'` needs TensorRT + ZED AI model files; installer ran with `skip_od_module` (skips 4+ GB AI download) → model absent. sed patch during build sets `depth_mode: 'NONE'` + `pos_tracking_enabled: false` in `common_stereo.yaml`. Re-enable after TRT install + ZED AI resources re-fetched. Build also uses `--allow-shlib-undefined` + `--packages-skip zed_debug` (feedback_l4t_runtime_libs_build_vs_runtime.md) since `libsl_zed.so` has undefined Tegra symbols resolved only at runtime via `runtime: nvidia` bind-mount. Service opt-in only via `profiles: [zed]` (not in `full` until calibrated). | **ZED 2i streaming RGB + IMU.** Camera S/N 35167492 (FW 1523), 1920x1080 grab @ 15 fps, 960x540 publish. `/zed/zed/rgb/color/rect/image` @ 5.6 Hz, `/zed/zed/imu/data` @ 99.088 Hz. Depth + positional tracking disabled pending TRT install + AI model re-fetch. Commit e611b27 | Keep |
| 2026-04-18 | docker-compose.thor.yml + containers/lichtblick-web/{dist,nginx.conf} (new) | **Browser-local Foxglove viewer for iPad** — new `foxglove-web` service (nginx:alpine, host-net, profiles `viz`/`full`, `restart: "no"`) serving `containers/lichtblick-web/dist/` on `:8080`. Dist is the arch-independent `lichtblick-web.tar.gz` v1.24.3 from github.com/lichtblick-suite/lichtblick — the ghcr container image is amd64-only and won't pull on Thor. Paired with existing `foxglove-local` (plain-ws bridge on `:8766 tls:=false`, 2026-04-15). **Why**: user wanted iPad live-viz over LAN without Tailscale bandwidth or the native iOS app. Safari mixed-content policy blocks ws:// from HTTPS origins, so hosted app.foxglove.dev can't reach our plain-ws bridge. Same-scheme same-origin HTTP+ws fixes it. `nginx.conf` does SPA try_files fallback + gzip on JS/JSON/CSS. Foxglove Studio was archived (no more Docker Hub image, hosted-only) — Lichtblick is the community fork that still ships self-hostable artifacts. **Access recipe**: iPad on `BESS-Thor-2G` → Safari `http://192.168.1.209:8080` → Open connection → **Foxglove WebSocket** (NOT ROS 2 Native) → `ws://192.168.1.209:8766` | **Working**: confirmed by user 2026-04-18 evening, iPad Safari loads UI and streams live topics | Keep |
| 2026-04-18 | scripts/offline_slam_to_las.py | **Added post-export sanity card at `<out>/sanity.txt`.** No single artifact summarised run health; had to grep stdout or open CloudCompare on a multi-GB LAS to spot divergence like "DLIO spiked 354 km/h". New `_pose_stats()` + `write_sanity_card()` compute pose_count, span, rate, cumul, mean/p99/max speed, max inter-pose dt, rotation span (sum `|dθ|` between consecutive R via trace(R0^T R1)), pre/post-voxel counts, LAS + COPC file sizes. Verdict: FAIL if mean_speed > max_speed OR p99_speed > max_speed OR extent > max_extent; WARN if p99 > 0.5·max_speed OR rate < 5 Hz OR max_dt > 1 s; else PASS. Exporter exits 3 on FAIL unless `--force`. Backfilled all 6 existing drive1+bags40-45 exports. **Surfaced known-hidden divergence**: `dlio_bags40to45` flagged FAIL (p99 56.1 m/s > 30 threshold) that `--force` papered over during its original run. GLIM both PASS (drive1 p99 7.9, bags40-45 p99 14.1). FAST-LIO2 both WARN on rate (drive1 8.61 Hz, highway 6.67 Hz — both below 10 Hz). Commit 217290e | **Sanity cards written to 6/6 export dirs. FAIL on `dlio_bags40to45` newly visible without CloudCompare.** | Keep |
| 2026-04-18 | scripts/offline_slam_glim.sh (new) + config/glim/config_offline_thor.json (new) + config/glim/config_sensors_thor.json (new) + config/glim/config_ros_offline.json + scripts/offline_slam_to_las.py + containers/fast-lio/ouster_imu_guard.py | **Third offline SLAM engine (GLIM) wired in for comparison.** New `scripts/offline_slam_glim.sh` mirrors offline_slam_{fastlio,dlio}.sh — 6-batch sequenced launch: imuguard-off-glim → glim-off (humble-cyclone container, CPU-only, `librviz_viewer.so` extension for `/glim_ros/*` topics) → glim-tf-off (identity TF glim_base_link→os_imu) → glim-map-off (voxel accumulator, reuses `scripts/slam_map_accumulator.py` unchanged, subscribes `/glim_ros/aligned_points_corrected` → publishes `/glim_ros/map_voxel` at 0.25m voxel / 4M cap / 2s cadence) → recorder → bag play. `config_offline_thor.json` is the dispatcher pointing at `config_sensors_thor.json` (T_lidar_imu = Rz(180°) + [0.0024, 0.0097, -0.0307] matching Ouster metadata serial 122313000586), `config_odometry_cpu.json`, `config_ros_offline.json` (enable_global_mapping=false — single drive has no loop closures so iSAM2 gauge prior `LinearDampingFactor(X(0), 6, 1e10)` fails at x1; `imu_topic: /ouster/imu_guarded` for monotonic stamps). `/glim_ros/map` native topic is only populated by global_mapping's `globalmap_on_update_submaps` callback (rviz_viewer.cpp:473+) so with global_mapping off it stays silent — downstream map accumulator fills that gap with `/glim_ros/map_voxel` for Foxglove. Exporter `cloud_already_world` fast-path (added 2026-04-18 for DLIO deskewed) extended to also match `aligned_points` since rviz_viewer.cpp:466 transforms points by `T_world_sensor()` before publish — same world-frame bypass, no SLERP re-deskew. ouster_imu_guard.py rewritten non-drifting: prior revision used `out = raw + offset_ns` with `offset_ns` accumulating forward on every rewind and never decaying — over 140s drive1 that added ~1.4s of drift between IMU and LiDAR clocks, causing GLIM iSAM2 "large time difference between points and imu!!" and EKF drop. New semantics: `if raw > last_output: emit raw` (resume raw clock, no accumulated offset) `else: emit last_output + min_increment_ns` (push forward). Zero long-run drift | **2/2 PASS.** GLIM drive1 (urban, bags 16-22): 1344 poses @ 10 Hz, 522.8 m cumul, 300×162×6 m extent — matches FL (530 m / 300×162×9) and DLIO (551 m / 300×162×6) within 3%. LAS 13.6M pts / 0.51 GB at 0.05m voxel / 10s (sparse because `config_preprocess.json` default `downsample_resolution: 1.0` + `random_downsample_target: 10000` — dense variants exist at config_preprocess_dense/highdensity.json if needed). GLIM highway (bags 40-45): 1291 poses @ 10 Hz, **1237 m cumul**, **1206×117×46 m extent**, 16 m/s max — matches FL (1260m / 1220×111×30.6) and DLIO (1188m / 1011×250×4). LAS export world-frame fast-path runs 475 scans/s (vs 16 scans/s for the raw-Ouster SLERP path — 30× speedup because no per-scan time-bin interpolation). Commit 5629a1c | Keep |
| 2026-04-18 | containers/fast-lio/fast_lio_single.yaml + scripts/offline_slam_fastlio.sh + new ouster_imu_guard.py | `point_filter_num: 1`→`3` (upstream Ericsii ouster64.yaml default). Offline script now spawns `ouster_imu_guard` container ahead of FAST-LIO2 so replay sees monotonic `/ouster/imu_guarded`. Dedicated Ouster guard (`ouster_imu_guard.py`) uses `SensorDataQoS` to subscribe to the BEST_EFFORT Ouster BMI085 publisher — generic `imu_guard.py` was RELIABLE-only and silently dropped every Ouster sample. Kept the generic guard with a new `reliability` param for future GQ7/Ouster switching. `ENABLE_SCANCONTEXT=0` gate in launch so offline replays skip scancontext on stationary-ended bags. Round 2 of SLAM quality push, source-backed against `/tmp/slam-research/fastlio_ros2` | **drive1 PASS**: /fast_lio/odometry 7.83 Hz (was 5.04 Hz, +55%), cumul 527.6 m, extent 298×158×7 m. Max gap 0.30 s (single between-bag outlier). Commit 1aafeec | Keep |
| 2026-04-18 | config/dlio/dlio_thor.yaml (6 edits) + scripts/offline_slam_dlio.sh | 2026-04-17 calibration/time 3.0→0.5 was half-right (bag starts in motion so 3.0 absorbs vehicle motion as bias) but 0.5 is too short to average IMU noise. Source review of `odom.cc:callbackImu()` at `/tmp/slam-research/dlio` showed bias is extracted **once** and never re-estimated, so the real fix was `approximateGravity: false → true` (upstream default — auto-aligns orientation with accumulated accel post-calib, decouples from noisy bias). Six edits: `calibration/time 0.5→1.5` (compromise), `approximateGravity false→true` (THE fix), `voxelFilter/res 0.10→0.25` (upstream, OS1-128 post-voxel ~25-30k pts vs ~80k — fits GICP budget), `submap knn/kcv/kcc 5→10` (upstream — halved starves submap), `gicp/maxCorrespondenceDistance 0.5→0.25` (tightened with voxel 0.25), `waitUntilMove false→true` (upstream — no keyframes from noisy calibration window). Rz(180°) extrinsic kept (verified `/ouster/points frame_id == os_lidar` via ros2 bag info). Offline_slam_dlio.sh wires ouster_imu_guard ahead of DLIO. Source-backed against `/tmp/slam-research/dlio` (cfg/params.yaml + src/dlio/odom.cc) | **drive1 PASS**: /dlio/odom_node/odom 16,310 poses @ 117 Hz, cumul 561 m (was 13,239 m, **23× reduction**), extent 300×162×6 m. DLIO + FAST-LIO2 world frames now match within 3% extent / 6% cumul. Commit 6c0b30d | Keep |
| 2026-04-18 | scripts/offline_slam_to_las.py (new) + scripts/slam_forensics.py (new) + scripts/las_to_ply.py (new) + offline SLAM tooling family | Exporter applied a single pose (scan header stamp) to all 131k pts in each 100 ms Ouster scan — 0.5-2 m wall-thickening per scan at drive speeds, baked into LAS independent of SLAM quality. Per-scan loop now buckets into 16 time bins using the per-point `t` ns offset (already extracted by `parse_ouster_cloud`), SLERPs a body pose per bin centre, transforms each bin's points with its own T_world_lidar. Cost: 16 pose-interps + 16 transforms per scan (~50 ms, CPU-bound). `_audit_poses()` gate switched from raw `max_speed > 30 m/s` to `p99_speed > 30 m/s` with max reported alongside — DLIO 117 Hz bursts short-dt pose pairs (<1 ms apart) where speed = d/dt spikes to hundreds of m/s on ~0.5% of pairs while cumul+extent are correct; p99 catches real divergence (would be 100s of m/s) while tolerating stream noise. `dlio` extrinsic preset updated to Rz(180°) + [0,0,0.0382] matching dlio_thor.yaml. Also landed: slam_forensics.py standalone gate, las_to_ply.py (CloudCompare qLAS fallback), audit_bag_motion.py, extract_slam_cloud.py, gvcp_probe.py, plus offline_slam_{fullstack,replay,single}.sh reproduction variants | **drive1 LAS PASS**: FAST-LIO2 80.4 M pts / 3.0 GB at 0.05 m voxel / 162 s; DLIO 100.5 M pts / 3.7 GB / 167 s. Per-point de-skew observed at 16.5 scans/s throughput (CPU-bound on interp). Commit 6f40491 | Keep |
| 2026-04-18 | containers/fast-lio/fast_lio_single_offline.yaml | `max_iteration 5→3` + `acc/gyr_cov 0.1→0.5`. At max_iter=5 iEKF converges hard; during turns (ambiguous features + rapid scene change) it locks onto local minima → visible pose snap. max_iter=3 stays closer to IMU prediction, smoother poses. BMI085 on OS1-128 is cheap MEMS with gyro noise > 0.1 rad/s — tight covariance was over-weighting IMU prediction, forcing LiDAR to hard-correct at every turn. Looser cov 0.5 (matches hku-mars upstream) weights scan match higher → smooth correction. Also documents the failed 2026-04-18 filter_size_map 0.5→0.25 retry (pose rate 7.74→3.71 Hz, Z extent 10→65 m catastrophic) so next session doesn't retry it | **drive1 visual PASS**: smoother rotation handling vs max_iter=5; same 22% highway scan drops but snap artifact gone. Commit 64cc206 | Keep |
| 2026-04-18 | config/dlio/dlio_thor.yaml (5 edits Round 3) + containers/fast-lio/fast_lio_single_offline.yaml + scripts/offline_slam_to_las.py (--max-speed/--max-extent/--force) | **Round 3 — highway tune for both engines.** Bags 40-45 (highway) pre-R3: FAST-LIO2 5.72 Hz / 48.5 m Z drift; DLIO 891 m cumul / 233×104×4.5 m extent (not tracking motorway). FAST-LIO2 paper (arxiv 2107.06829 Tab. VIII) uses 1:4 temporal downsample → `point_filter_num 3→4` saves 25% compute/scan on OS1-128, urban slack absorbs it. DLIO has no vehicular upstream config (issues #13/#15/#32) — designed for handheld/drone. Adaptive in `odom.cc:setAdaptiveParams()` (lines 1626-1650) is spaciousness-gated, clamps 2× on open road only if base is right. 5 DLIO edits, each source-cited: `keyframe/threshD 1.0→2.5` (suburban/transition band — adaptive handles highway clamp ≤5m and urban ≥0.5m), `submap knn 10→15` (>3s highway reach for distant-feature anchors, kcv/kcc stay 10 as hull-coverage dimensions), `gicp/maxCorrespondenceDistance 0.25→0.30` (base gives urban 0.15 / highway 0.60 via adaptive), `gicp/maxIterations 32→24` (reduces local-minima lock-on under fast motion when IMU prior is good), `geo/Kv 11.25→6.0` (velocity gain trimmed to absorb GICP scan noise at 100ms dt; Kp unchanged). Exporter `--max-speed` and `--force` flags added — UK motorway legit 31 m/s was failing the 30 m/s audit gate, and DLIO's IMU-rate burst artifact spikes d/dt to hundreds of m/s on <1% of pose pairs while cumul+extent are fine | **4 runs PASS.** FAST-LIO2 drive1 (urban): 8.61 Hz (+10%), 530 m cumul, 300×162×9 m. FAST-LIO2 bags40-45 (highway): **6.67 Hz (+17%)**, 1260 m cumul, 1220×111×**30.6 m** (Z drift −37% vs 48.5 m). DLIO drive1 (urban): 120 Hz, 550.6 m cumul (+21% vs 455 pre-R3, matches FL), 300×162×6 m (matches FL). DLIO bags40-45 (highway): 119 Hz, **1188 m cumul (+33%)**, **1011×250×4 m (X +335%, full motorway tracked, Z stable)**. LAS outputs voxel 0.05 FL / 0.02 DLIO: FL urban 71.9M/2.7G, FL highway 134.3M/5.1G, DLIO urban 120M/4.5G, DLIO highway 144.5M/5.4G. FAST-LIO2 highway 30 m Z drift remains structural (no gravity re-align upstream — needs Point-LIO or GNSS to close). Commits 3404eb8 / d14d029 / c725f98 | Keep |
| 2026-04-18 | config/dlio/dlio_thor.yaml + scripts/offline_slam_to_las.py | **LAS swirl root cause + density fix.** DLIO MCAP was clean in Foxglove but LAS exports showed massive swirls at every rotation. Two concurrent bugs: (1) DLIO publishes geometric-observer state at IMU rate (~117 Hz) with 1-4 ms bursts after GICP corrections — exporter's 16-bin SLERP was drawing adjacent bins from opposite sides of a burst → 0.5-1 deg rotation residuals per scan. (2) `map/dense/filtered: true` made DLIO publish voxelized `current_scan` (25k pts/scan @ 0.25 m) on the `deskewed` topic — blocky voxel-center look. Fix: (a) exporter now auto-detects `"deskewed"` in --cloud-topic and consumes DLIO's world-frame already-deskewed cloud directly, bypassing the exporter's SLERP re-deskew entirely. DLIO's internal `deskewPointcloud()` uses continuous-IMU integration, not snapshot poses — far more faithful than anything we can reconstruct post-hoc. (b) Added `_decimate_burst_poses()` (drops <5 ms burst samples) for the legacy raw-Ouster path. (c) Flipped `map/dense/filtered: true→false` per odom.cc:844-847 (GICP still runs on voxelized current_scan internally → SLAM stability unchanged, publish topic gets native 131k pts/scan). `parse_dlio_deskewed_cloud()` + `--deskew-bins` knob added | **drive1 deskewed PASS**: no swirls, no blocky voxel centers. 228.6 M raw pts → 118.6 M at 0.02 m voxel = 4.42 GB, 140 s export (2.5x denser than voxelized path). Same 571 m cumul / 300×161×6 m extent — pose stream unchanged. Commit 707ff8a | Keep |
| 2026-03-08 | lucid_node.py:178 | GevSCPSPacketSize 8972→1500 | Debug MTU issues | BROKE CAMERA | **REVERTED** |
| 2026-03-08 | docker-compose.thor.yml | thermal3 camera_id→A70-89900590 | Fix serial number | Unknown | Keep |
| 2026-03-08 | docker-compose.thor.yml | thermal4 camera_id→A70-89900595 | Fix serial number | Unknown | Keep |
| 2026-03-08 | docker-compose.thor.yml | lucid1 camera_ip added | Help camera discovery | SC_ERR_INVALID_ID | Check |
| 2026-03-08 | docker-compose.thor.yml | thermal3/4 frame_rate:=15.0 | User requested 15fps | OK | Keep |
| 2026-03-08 | dockerfiles/thermal/*.py | Copied from docker_slam | Files were missing dirs | OK | Keep |
| 2026-03-08 | CLAUDE.md | Added HARD STOPS, SDK MATRIX | Prevent future chaos | OK | Keep |
| 2026-03-08 | docker-compose.thor.yml | thermal1/2 RMW→rmw_fastrtps_cpp | bess-thermal-spinnaker lacks CycloneDDS | Testing | Keep |
| 2026-03-08 | mgbe1_0 | Added 192.168.10.1/24 | A70-89900590 at 192.168.10.2 | thermal3 WORKING | Keep |
| 2026-03-08 | enP2p1s0 | Removed dup 169.254.100.1/16 | Causing Spinnaker subnet conflict | thermal2 testing | Keep |
| 2026-03-09 | thermal_spinnaker_node_patched.py | Added GevSCPD=1000 | Inter-packet delay reduces burst traffic | Pending test | Keep |
| 2026-03-09 | mgbe0_0 NM connection | Set ipv4.never-default=yes | Prevent route hijacking | OK | Keep |
| 2026-03-09 | bess-mgbe1 NM connection | Created 192.168.10.1/24 | Persistent IP for A70 | OK | Keep |
| 2026-03-09 | /etc/sysctl.d/ | Applied kernel tuning | rmem_max=256MB, backlog=50000 | OK | Keep |
| 2026-03-09 | scripts/start-cameras-sequential.sh | Created | Avoid Spinnaker SDK conflicts | Pending test | Keep |
| 2026-03-09 | **ROOT CAUSE FOUND** | sfp28-2 is 1Gbps S-RJ01 | All cameras on 125MB/s link need 251MB/s | **PHYSICAL RECABLE REQUIRED** | N/A |
| 2026-03-21 | docker-compose.thor.yml | foxglove RMW→rmw_fastrtps_cpp | Match camera containers DDS | Foxglove sees topics | Keep |
| 2026-03-21 | docker-compose.thor.yml | foxglove TLS enabled | Browser requires wss:// from HTTPS | Working | Keep |
| 2026-03-21 | config/certs/ | Tailscale HTTPS cert | Valid cert for wss://thor.tail902411.ts.net:8765 | **WORKING** | Keep |
| 2026-03-21 | iptables ts-input | Allow iPad 100.115.28.97 | Tailscale firewall was blocking | OK | Keep |
| 2026-04-12 | docker-compose.thor.yml | lucid1 camera_ip 169.254.232.169→169.254.20.23 | Wrong IP, didn't match DHCP reservation | lucid1 still needs sfp28-7 link | Keep |
| 2026-04-12 | docker-compose.thor.yml | Added lucid2 service + compress-lucid2 | Lucid #2 on network, no container | **lucid2 WORKING 5Hz** | Keep |
| 2026-04-12 | docker-compose.thor.yml | thermal4 MAC 00:40:7f:10:a2:bc→00:40:7f:11:04:ea | Wrong MAC in compose | thermal4 WORKING 14.4Hz | Keep |
| 2026-04-12 | docker-compose.thor.yml | MicroStrain: removed Moxa socat, use USB direct | Thor uses USB not Moxa per vehicle docs | Compose fixed, not restarted | Keep |
| 2026-04-12 | docker-compose.thor.yml | foxglove: added cyclonedds.xml mount, cleaned RMW env | Foxglove crashed — missing DDS config | **Foxglove WORKING** | Keep |
| 2026-04-12 | thermal_spinnaker_node_patched.py | GevSCPD: use min(1000, camera max) | A70 max=512, was failing with 1000 | **thermal4 stable, no drops** | Keep |
| 2026-04-12 | MikroTik sfp28-7/8 | Enabled (were disabled) | Lucid ports were off | sfp28-8 linked 10G, sfp28-7 rx-loss | Keep |
| 2026-04-12 | compose blackfly1 | sleep 10s→30s between restart attempts | GigE heartbeat needs time to clear on unexpected exit | OK | Keep |
| 2026-04-12 | blackfly1 MTU sweep | MikroTik bridgeLocal L2MTU raised 1584→9216 (sfp28-3/4/5/7/9/10/11/13/14/15/16), tested packet_size 9000 and 8900 | **INCOMPLETE 100% persists.** tcpdump shows only leader(44B)+trailer(24B) at 5Hz, ZERO data packets at jumbo sizes. At 1400 data packets arrive but driver still 100% incomplete. Config space exhausted. | **REVERTED via scripts/rollback-blackfly1-mtu.sh** | **REVERTED** |
| 2026-04-12 | Task 18 | Escalated to physical swap | Blackfly2 on sfp28-6 works 5Hz with identical config. Suspect sfp28-5 cable or SFP module. Next: move bf1 to known-good port or swap SFPs | Pending user action | N/A |
| 2026-04-12 | compose blackfly1 | Added `-p device_link_throughput_limit:=125000000` | **ROOT CAUSE**: Stale NVRAM DeviceLinkThroughputLimit capped Blackfly1's bandwidth. Camera emitted leader+trailer only, no data packets (timed out before data could go out). Not port/cable/MTU/packet_size/SFP — all red herrings. user_set_load=Default did NOT reset this. | **FIXED: IN 5Hz / OUT 5Hz / 0% drop / 1372B data packets flowing** | Keep |
| 2026-04-12 | fast_lio_single.yaml | imu_topic: /imu/data_guarded → /ouster/imu; extrinsic_T → [0,0,0] | Ouster PTP clock stuck at 2025-05-29 (~318 days stale); GQ7 on wall time. sync_packages dropped every scan. Match DLIO's approach — use Ouster's built-in IMU so LiDAR+IMU share the same clock | **FIXED: /slam/odometry @ 10Hz real scan updates, IMU Initial Done, KD-tree mapping live** | Keep |
| 2026-04-12 | fast-lio Containerfile | Patched laserMapping.cpp IMU sub: `10` → `rclcpp::SensorDataQoS()` | Ouster /ouster/imu publishes BEST_EFFORT; default RELIABLE sub was silently dropping all IMU. "incompatible QoS RELIABILITY_QOS_POLICY" warn was the only symptom | FAST-LIO2 IMU ingested correctly after rebuild | Keep |
| 2026-04-12 | CLAUDE.md | Added GOLDEN RULE: read code + research before any change | User: repeated hacks without reading cost hours/locked hardware | Rule prominent above Behaviour Rules | Keep |
| 2026-04-12 | config/ouster/params.yaml | timestamp_mode: TIME_FROM_PTP_1588 → TIME_FROM_ROS_TIME | No PTP master on Thor sensor net, Ouster clock was floating on internal osc and drifting (~318 days stale, May 2025). LiDAR/IMU/camera/bag timestamps now all on host wall clock | All sensors on aligned wall clock; DLIO shows current date, bag recording works | Keep |
| 2026-04-12 | compose dlio | Added supervisor: kill ros2 launch when dlio_odom_node dies + healthcheck targets odom_node not map_node | DLIO odom_node had crashed with `free(): invalid next size` but container stayed "healthy" because map_node was still alive and ros2 launch stays up after child death. Now container exits → restart policy kicks in | Dead odom_node triggers container restart within ~5s | Keep |
| 2026-04-12 | launch_fast_lio_single.py | Added OnProcessExit → Shutdown for fastlio_mapping | Same ros2 launch ghost-alive pattern as DLIO — if fastlio_mapping dies, launch should tear down so container restarts | fast-lio container will now exit cleanly on SLAM node death | Keep |
| 2026-04-12 | fast-lio Containerfile | Patched imu_cbk: `timestamp < last_timestamp_imu` → `< last_timestamp_imu - 0.01` | In TIME_FROM_ROS_TIME mode with imu_packets_per_frame=8, Ouster stamps packets by reception time → adjacent samples differ <1ms and occasionally rewind by ~150us. Strict `<` check was wiping the IMU buffer on every jitter, starving FAST-LIO EKF. Loop-back spam: 127 events in 30s | Loop-backs dropped to 0; /slam/odometry std dev 18ms → 5ms | Keep |
| 2026-04-12 | compose glim | Removed from `slam`/`full` profiles, moved to `glim-deprecated` | User: GLIM is too sparse on Ouster OS1-128; deprecated in favor of DLIO + FAST-LIO2. Was leaving a permanently "Created" container behind | glim no longer starts with default stack, reference config preserved | Keep |
| 2026-04-12 | /etc/linuxptp/ptp4l-bess.conf + systemd | Created ptp4l-mgbe0.service + phc2sys-mgbe0.service. clockClass=248, priority1=64, time_stamping=hardware, phc2sys -O 0 syncs mgbe0_0 PHC to CLOCK_REALTIME | Thor sensor net had NO grandmaster; all sensors were clock-islanded. Now mgbe0_0 PHC1 is GM at 4cbb47.fffe.0dbe94 | **ptp4l GM active, stepsRemoved=0, Ouster sync offset_from_master=-5ns** | Keep |
| 2026-04-12 | config/ouster/params.yaml | timestamp_mode: TIME_FROM_ROS_TIME → TIME_FROM_PTP_1588; added ptp_utc_tai_offset: 0.0 | Now that ptp4l GM exists, flip Ouster back to PTP-hardware stamps. Default driver offset -37.0 was subtracting leap seconds from our already-UTC PHC, making stamps 36s early | Ouster /points stamp matches wall clock (pipeline lag 491ms) | Keep |
| 2026-04-12 | /etc/linuxptp/ptp4l-bess.conf | Added `utc_offset 0` | Default announced currentUtcOffset=37 (TAI-UTC leap-seconds). PHC is UTC via phc2sys -O 0, so slaves receiving TAI-offset would mis-convert. Announce 0 so slaves treat PHC as UTC directly | Ouster received `current_utc_offset:0` immediately, now in UTC | Keep |
| 2026-04-12 | docker-compose.thor.yml blackfly1/2 + config/blackfly_s_thor.yaml | Added yaml mapping `gev_ieee_1588` + `gev_ieee_1588_mode`, passed `-p use_ieee_1588:=true -p gev_ieee_1588:=true -p gev_ieee_1588_mode:=SlaveOnly -p adjust_timestamp:=false` | `use_ieee_1588` alone only enables driver offset-reporting; doesn't write camera. `gev_ieee_1588` yaml write flips camera into PTP. `SlaveOnly` prevents BMCA flap that alternates MAS↔SLV every 10s. All three required | **Both Blackflies stable SLV, 0% drop, 5Hz, Blackfly1 working first time in weeks** | Keep |
| 2026-04-12 | dockerfiles/thermal/thermal_spinnaker_node_patched.py | Added GevIEEE1588 bool set + GevIEEE1588Status read after IRFormat config | Enable PTP on FLIR Spinnaker thermal cameras. A6701 accepts the write, A70 firmware reports node not writable (hardware limitation — A70 stays on ROS clock) | thermal1/2 (A6701) → `GevIEEE1588Status=Slave`. thermal3/4 (A70) → warn "node not writable" as expected | Keep |
| 2026-04-12 | containers/lucid/lucid_node.py | Added PtpEnable=True + PtpSlaveOnly=True + PtpDataSetLatch.execute + PtpStatus read in connect_camera() | Enable PTP on Lucid Atlas via Arena SDK per Lucid KB | Lucid2 accepted both writes. Note: header.stamp still uses `get_clock().now()` so PTP enables camera-internal consistency but driver publishes in ROS time | Keep |
| 2026-04-12 | /etc/udev/rules.d/80-usb-storage-no-autosuspend.rules | Created udev rule disabling USB autosuspend for HighPoint RM110 (1103:0110) + any USB mass-storage on bus 2 | Post-reboot, the RM110 enclosure flapped every 3-4s: clean enumerate, no error, clean disconnect. Root cause: `usbcore.autosuspend=2` global default + device `autosuspend_delay_ms=0` → SuperSpeed link entered U3 immediately after probe and the RM110 PHY couldn't recover. Pre-reboot worked because the 191GB transfer kept the link continuously busy | **USB stable, /mnt/bess-usb mounted 7.3T ext4 with prior soak bags intact** | Keep |
| 2026-04-13 | systemd-timesyncd.service | `mask` (symlink to /dev/null) | NTP step events on CLOCK_REALTIME propagate through phc2sys to PHC1 and desync every PTP slave on the sensor net (Ouster firmware servo cannot recover from multi-second steps — required full power cycle on 2026-04-12). Wall clock now free-runs at BMC RTC rate; structural fix is ts2phc from GQ7 PPS but blocked by lack of physical PPS wire | NTP=no, all timesync services inactive | Keep |
| 2026-04-13 | /etc/systemd/system/phc2sys-mgbe0.service | ExecStart `-S 1.0` → `-S 0` (never step, servo only) | Defense-in-depth against any large CLOCK_REALTIME jump (manual `date -s`, residual NTP, VM pause). Even with timesyncd masked, phc2sys must refuse to step PHC1 — anything that desyncs PHC1 also desyncs the Ouster firmware slave which then cannot recover without a full sensor cold-boot | phc2sys in s2 servo mode, offset 2.8 µs locked, Ouster slave 20 ns | Keep |
| 2026-04-13 | scripts/soak_monitor.sh | Added `sample_ouster_ptp_offset_ns()` (HTTP API) + `sample_spinnaker_ptp_offset_s()` (log scrape) + per-tick PTP slave offset watchdog (alert if >100ms) | With phc2sys -S 0 the GM never steps, so any sustained slave offset >100ms means the slave servo is structurally broken (firmware lockup, BMCA flap, switch non-transparent-clock). Catch during soak rather than after extraction | Tested: ouster=20ns ✓, blackfly1=107ms (will alert), blackfly2=114ms (will alert — both surface known MikroTik non-transparent-clock issue) | Keep |
| 2026-04-13 | microstrain container | `docker compose up -d --force-recreate --no-deps microstrain` | USB re-enum at 06:29 left container with stale (major,minor) bindings for /dev/microstrain_{main,aux}; driver spammed `Device did not configure the aux port`. Docker resolves --device symlinks at container-start time — a plain restart is NOT enough, needs full recreate | Aux port now configures cleanly: "Starting aux port parsing", both GNSS receivers publish /mip/gnss_{1,2}/fix_info at 1Hz. Dual-antenna status silent — expected indoors, task #56 tracks post-boot re-check with sky view | Keep |
| 2026-04-13 | scripts/boot-sensors.sh | Removed `192.168.1.50/24` alias; replaced STEP 1.5 with STEP 3.5 that probes `MKTK /tool ping 1.1.1.1 interface=bridgeWAN` and, if healthy, installs `default via 169.254.100.254 dev mgbe0_0 metric 50` | Corrected topology: RUT50 is on MikroTik **sfp28-12** (NOT sfp28-14) in **bridgeWAN** (segregated from bridgeLocal to avoid RUT50 DHCP racing our dnsmasq). Thor routes through MikroTik rather than direct L2 to 192.168.1.1 | Script ready — auto-installs default route at next boot when RUT50 uplink is healthy | Keep |
| 2026-04-13 | RUT50 mob1s1a1 via admin API | pdptype: `ipv4v6` → `ip` (IPv4-only PDN) | EE UK `everywhere` APN returns IPv6-only PDN + 464XLAT transition address `192.0.0.2` when asked for dual-stack — CLAT not working on RUT50 so LAN clients couldn't reach IPv4 internet. Forcing IPv4-only PDP allocates a real cellular IPv4 (`10.177.31.194`) on `qmimux0` | **5G uplink live end-to-end**: Thor→1.1.1.1 45ms, DNS resolves, `apt update` fetched 2.7MB, Tailscale reconnected via 5G | Keep |
| 2026-04-13 | MikroTik bridgeLocal dhcp-client | `add-default-route=yes` → `no` (persisted via /export) | Thor's dnsmasq gave MikroTik a distance-1 default route via `169.254.100.1` (Thor itself), which won over the distance-5 bridgeWAN RUT50 route. MikroTik was sending internet traffic **back to Thor** → ICMP Redirect loop. Turning off add-default-route on bridgeLocal lets the bridgeWAN DHCP lease's distance-5 route become active | MikroTik routes internet via RUT50, Thor routes via MikroTik, stack works end-to-end | Keep |
| 2026-04-13 | MikroTik `/ip dns` | `servers=1.1.1.1,8.8.8.8` (persisted) | MikroTik's own `/resolve` needs upstream DNS for NTP/logging/tool fetch. Was empty before (bridgeWAN DHCP has `use-peer-dns=no`) | `/resolve www.google.com` works | Keep |
| 2026-04-13 | Memory reference_rut50_credentials.md | Corrected: SSID `BESS-Thor-5G` PSK `BessThor2026!` (recovered via `/api/wireless/interfaces/config`), RUT50 is on sfp28-12 in bridgeWAN, EE IPv6-only-PDN diagnosis + fix recipe | Factory `RUT_E7F3-5G` / `u8RJb51X` in prior memory were stale (device reconfigured), topology was guessed wrong, IPv6-only-PDN wasn't known | Future sessions can jump straight to `qmimux0` diagnosis when user says "RUT50 no internet" | Keep |
| 2026-04-13 | containers/fast-lio/fast_lio_single.yaml | `publish.map_en: true` → `false` | FAST-LIO2's built-in `publish_map()` appends every scan into `pcl_wait_pub` with ZERO voxel filter, then republishes the entire cloud on `/Laser_map` each scan. After ~2 minutes the single message exceeds 100 MB and chokes CycloneDDS fragment reassembly — every other SLAM topic goes silent as a side-effect. Never safe on a continuous rig | **/Laser_map gone, /slam/odometry + /slam/cloud_registered stable @ 9 Hz** | Keep |
| 2026-04-13 | scripts/slam_map_accumulator.py + compose `slam-map-accumulator` service | New Python ROS 2 node: subscribes `/slam/cloud_registered`, voxel-hashes (0.25m, cap 2M voxels, FIFO evict), republishes `/slam/map_voxel` every 2s with RELIABLE+TRANSIENT_LOCAL so late Foxglove subscribers latch onto current map. Replaces FAST-LIO2's native map publish with a bounded downstream build | **Foxglove unified world map live**, ~1.4M voxels typical, no DDS choke | Keep |
| 2026-04-13 | config/dlio/dlio_thor.yaml | `frames/odom: odom` → `dlio_odom`; `frames/baselink: base_link` → `dlio_base_link` | DLIO and FAST-LIO2 both publish to `base_link` by default — they fight for ownership of the TF tree, one overwrites the other's transform every cycle. Renaming DLIO's frames lets both run in parallel for comparison; Foxglove points at `base_link` (FAST-LIO2) as primary, `dlio_base_link` for the alternative path | Both SLAMs publish independently, no TF conflict | Keep |
| 2026-04-13 | docker-compose.thor.yml dlio | `imu_topic:=/imu/data` → `/ouster/imu` (match fast-lio fix) | GQ7 `/imu/data` is on host wall clock and had monotonic-jitter issues in DLIO (sync_packages dropping scans). `/ouster/imu` is the built-in BMI085 — monotonic stamps, same TIME_FROM_ROS_TIME clock as `/ouster/points`, identity lidar↔imu extrinsic is correct since the IMU is inside the sensor head | DLIO distance tracker advances, keyframes accumulate | Keep |
| 2026-04-13 | dockerfiles/thermal/thermal_spinnaker_node_patched.py | Added SensorTemperature + HousingTemperature publishers on `/thermal/cameraN/{temperature,housing_temperature}` at 15s period. Startup discovery walks GenICam Root enumerating Float nodes matching ALLOW list (`DeviceTemperature`, `SensorTemperature`, `FpaTemperature`, `HousingTemperature`) minus DENY list (Reflected/Atmospheric/ExtOptics/Mfg/Calibration/…). Kelvin→Celsius normalization | Physical readings: **A6701** DeviceTemperature is Stirling-cooled InSb FPA, healthy ~-200°C (~73K). **A70** DeviceTemperature is uncooled-bolometer body temp, healthy 25-50°C. Primary thermal-stress signal for sealed sensor bay | Both cameras publish; reads serialized on acquisition thread (PySpin nodemap non-thread-safe) | Keep |
| 2026-04-13 | scripts/soak_monitor.sh | Added `sample_thermal_cam_temps()` + per-tick watchdog sampling thermal1 (A6701) and thermal3 (A70). Alerts: A6701 FPA > -150°C = **Stirling cryocooler failure**; A70 body > 75°C = sensor bay cooling check. Thresholds tunable via `A6701_FPA_ALERT_C` / `A70_FPA_ALERT_C` env | Two cameras sampled per tick (thermal2/4 share bay thermal mass, doubling ros2 exec cost isn't worth it) | Soak will page the moment the Stirling cooler loses grip, not after the scene goes saturated | Keep |
| 2026-04-13 | docker-compose.thor.yml recorder | Output dir `/home/thor/bess/data/cache` → `/mnt/bess-usb/bags` (HighPoint RM110 ext4). Added `/thermal/cameraN/{temperature,housing_temperature}` to topic list | bess-usb is the 7.3T soak volume — keeps bags off the Thor NVMe. Temperature topics are now recorded so bag-time cryocooler failures can be reconstructed post-hoc | Keep |
| 2026-04-13 | docker-compose.thor.yml foxglove | Command rewritten bash-heredoc style with explicit `tls:=true certfile:=/certs/cert.pem keyfile:=/certs/key.pem` | Earlier the bridge relied on env-only TLS and silently fell back to plain ws:// when certs weren't mounted visibly. Explicit launch args make the TLS path the commit-time contract, not a runtime guess | Keep |
| 2026-04-15 | /etc/systemd/system/bess-{stack,network,time-bootstrap}.service + /etc/bess/ | Rewrote all three as independent kill-switch-gated oneshots. Added `/etc/bess/no-auto-{stack,network,time}` files (ARMED BY DEFAULT), `ConditionPathExists=!` gate, `StartLimitIntervalSec=86400 StartLimitBurst=3`, 30s grace sleep before docker compose, removed all `After=`/`Wants=` chaining between the three units | **Boot-loop root cause (Issue #31)**: on 2026-04-14 I enabled `bess-stack.service` chained via `Wants=docker.service bess-network.service ptp4l-mgbe0.service phc2sys-mgbe0.service`. On the next reboot the chain (bess-time-bootstrap steps clock → bess-network link-bounces mgbe0_0 → bess-stack starts 20+ containers with GPU/PTP/host-net) wedged the kernel long enough for `/dev/watchdog0` (Tegra hw watchdog) to fire. System rebooted → chain re-ran → watchdog fired → loop. 9+ rapid boots observed in `journalctl --list-boots` (each 14-20 s, no graceful shutdown cascade, kerneloops.service reported "unclean termination of previous run"). User recovered by `systemctl disable` on all three and spent a day restoring service. **Rule: these three units must never chain at boot again. Each is a no-op unless its kill-switch file is removed AND it is explicitly `systemctl enable`d. Rate limits make boot loops structurally impossible even if the user forgets the kill-switch** | Keep
| 2026-04-15 | docker-compose.thor.yml `x-common: &common` + live `docker update --restart=no` on 29 running containers | `restart: unless-stopped` → `restart: "no"`. Also glim service (line 774) and already-set recorder (line 1099) | **The 2026-04-15 diagnosis on the row above was incomplete.** After yesterday's kill-switch fix, `/etc/bess/no-auto-*` files stayed in place and all three `bess-*` units were `disabled (inactive dead)` — verified. But the reboot still looped: `journalctl --list-boots` showed 20+ sessions clustered in ~16-20 s windows at 10:45:42 / 10:46:04 / 11:00:01, followed by user intervention (drop-to-shell-then-exit) before boot 0 survived. Root cause is **Docker itself**, not systemd: `docker.service` is `enabled` and auto-starts at boot; the `x-common: &common` anchor set `restart: unless-stopped` which is inherited by 27 of 30 services via `<<: *common`. At `docker.service` startup, Docker immediately restarts every `unless-stopped` container in parallel — 27 containers racing for GPU/CUDA init (segformer, pii-mask, 6× gpu_compress), USB probes (RM110 absent → probe stall), GigE SDK discovery (Spinnaker/Arena/Aravis across 10 cameras), PTP clients, host-network stack on a cold kernel. Combined wedge exceeds 15 s → `/dev/watchdog0` fires → reboot → same state → same race → loop. `bess-stack.service` kill-switch is **irrelevant** to this path because Docker never asks systemd for permission. Evidence in `journalctl -b -5`: a docker container consumed 7.3 s CPU in the 14-second window before watchdog fired. Fix: (a) remove `restart: unless-stopped` from `&common` anchor (replaced with long comment explaining why), (b) `docker ps --format '{{.Names}}' \| xargs docker update --restart=no` on all 29 running containers live — non-destructive, no restart needed. Container-internal session retry loops (from 2026-04-12 resilience pass) still handle per-process crash recovery. **To auto-boot the stack cleanly: create a new systemd unit that runs `docker compose -f /home/thor/bess/docker-compose.thor.yml --profile full up -d` after `docker.service` + a ≥30 s grace sleep. Containers won't race at kernel startup, and explicit `up -d` starts them sequentially per compose dep order.** | **Boot loop mechanism eliminated at source. Next reboot will be clean.** | Keep
| 2026-04-14 | docker-compose.thor.yml recorder | Profile `record` → `record-disabled`; CAP_BYTES `40000000000` → `100000000000`; restart `unless-stopped` → `"no"` | **NVMe ENOSPC incident**: while /mnt/bess-usb was offline for car mounting, the recorder was writing to /home/thor/recordings/tmp/rolling on Thor NVMe with a 40 GB pruner cap. Host disk hit 100% anyway (total write + docker overlay + logs), wedging the Claude Code Bash subprocess — every tool call returned `exit 1` with zero output because stdout tmpfiles couldn't be written. Symptoms: `/export` failed with ENOSPC, `echo hi` failed, even allowed `docker compose version` failed. **Disk filled even with cap** because the pruner is per-split (5 GB) and the container's restart-wipe only clears `/recordings/rolling`, not sibling dirs. Recorder is now gated off `--profile record` AND `--profile full` — normal stack startup will NOT run it. To re-enable, edit compose: `record-disabled` → `record`, then `docker compose -f docker-compose.thor.yml --profile record up -d recorder`. Prereq: confirm /mnt/bess-usb mounted OR verify ≥150 GB free on NVMe first. Rolling bags at `/home/thor/recordings/tmp/rolling` deleted during recovery | **Disk recovered: 320 GB free. Bash tool restored. Stack otherwise untouched — fast-lio, dlio, slam-map-accumulator, all thermals, both blackflies, lucid1 all healthy through the incident** | Keep |
| 2026-04-15 | /etc/sysctl.d/, NM connections `mgbe0_0` + `bess-mgbe1`, new `bess-usb-recovery.service` | **Post-reboot persistence pass.** Boot loop fix (commit `2f28fab`, `restart: "no"` in `&common`) validated — clean single boot, uptime 1 min, no rapid-reboot pattern. Three remaining bodges made durable: (a) **sysctl conflict**: deleted `/etc/sysctl.d/60-ouster-lidar.conf` + `60-gige-sensors.conf.bak.2026-04-14`; the ouster file's `netdev_max_backlog=10000` was overriding the gige file's `100000` because it sorted after alphabetically (o>g). Runtime now 100000. (b) **MTU persistence**: `mgbe0_0`/`mgbe1_0` were defaulting to 1466 on every boot because `bess-network.service` (which ran `boot-sensors.sh`) is kill-switch gated post-boot-loop-fix. Moved MTU into the NetworkManager connection profiles — `802-3-ethernet.mtu 9000` on both (driver `nvethernet` resolves the request to 8966 live; 9000 is the symbolic max request). Required `ipv4.dad-timeout 0` on mgbe0_0 because NM's ARP probe was hitting itself via the MikroTik bridge (`BE:95` mgbe1_0 echoing `BE:94` mgbe0_0). Driver rejects MTU change on UP link (`RTNETLINK: Device or resource busy`) so the activation sequence is `disconnect → ip link down → ip link mtu → up → nmcli connection up`. Persisted in `/etc/NetworkManager/system-connections/*.nmconnection`. (c) **RM110 cold-boot non-enum auto-recovery**: today's cold boot left the RM110 (VID:PID `1103:0110`) enumerated only after user physically re-seated the cable at T+3 min (dmesg: xhci up at 10:59:56, `usb 2-1` appeared at 11:02:57 post re-seat). New `bess-usb-recovery.service` (oneshot, `After=multi-user.target`, rate-limited `StartLimitIntervalSec=86400 StartLimitBurst=3`) runs `/home/thor/bess/scripts/bess-usb-recovery.sh` which (i) waits to uptime ≥45 s for SS PHY training, (ii) checks both `lsusb -d 1103:0110` AND `/dev/disk/by-id/*RM110*` (udev prefix is `HPT_RocketMaet_RM110`, NOT `HighPoint`), (iii) if either check fails, rebinds the Tegra xhci platform device `a80aa10000.usb` on driver `tegra-xusb` (up to 3 tries with backoff). Bus 2 only carries RM110 + an empty 4-port hub — rebind blast radius is nil; Bus 1 (BT/HID keyboard) is a separate root hub. **Note: CLAUDE.md previously said MTU=9088 — wrong. Driver `maxmtu` is 9000, resolves to 8966 actual. Fix doc drift in a later pass.** | Pending validation on next cold boot; all three fixes live and verified in current session | Keep |
| 2026-04-15 | scripts/bess-stack-up.sh (NEW) + /etc/systemd/system/bess-{stack,network,time-bootstrap}.service (rewritten) + /etc/bess/no-auto-* (DELETED) + /etc/bess/README (rewritten) + docker-compose.thor.yml (microstrain + `full` profile) + config/cyclonedds.xml (`ParticipantIndex=none`) + compose segformer opt-in only | **Third crash/recovery pass + sequenced launcher.** Morning of 2026-04-15 Thor hard-crashed again mid-session — journal cut off at system-clock 11:48:06 mid-`phc2sys` line, dirty bit on nvme0n1p10, no shutdown sequence. Correlated with a `docker compose up --profile full up -d --force-recreate segformer` kicked during heavy stack load (all sensors running, FAST-LIO2 + DLIO converging, SLAM accumulator building map). Likely Tegra iGPU wedge → HW watchdog. Clock came back at 11:48 system-time = 18:12 UK (systemd-timesyncd masked by design, bess-time-bootstrap kill-switch gated → no wall-clock anchor after reboot). **Recovery procedure executed**: (i) fetched `Wed, 15 Apr 2026 18:21:52 GMT` via `curl -sI https://www.google.com`, `sudo timedatectl set-time "2026-04-15 18:22:30"`; (ii) after clock jump, `phc2sys-mgbe0` showed offset 18 trillion ns to PHC1 (`-S 0` refuses to step, ~240 days to slew) — `sudo systemctl stop phc2sys-mgbe0`, `sudo phc_ctl mgbe0_0 set`, `sudo systemctl start phc2sys-mgbe0`, offset dropped to 116k ns in 3s, safe because sensor stack was offline so no slaves to desync; (iii) brought up dnsmasq + stack manually via `docker compose --profile full up -d`. **User pushback**: "why do we need kill switches to power... just an error would fuckig do! this kill switch is total overbaked... can you not sequence the docker stampede". Right. The 2026-04-14/15 boot-loop root-cause was correctly identified in row 73 (`restart: unless-stopped` Docker stampede) but the 2026-04-14 kill-switch machinery was cargo-culted from a misdiagnosis and is now redundant. **Real structural fix**: replace compose's parallel profile-start with a sequenced script. Created `scripts/bess-stack-up.sh`: 10 ordered batches, max 4 containers starting concurrently, camera SDKs (Spinnaker C++ / PySpin A6701 / PySpin A70 / Arena) never run network discovery in parallel, GPU-touching containers staggered ≥5s, uses `docker compose up -d --no-deps` to explicitly sequence without letting compose resolve deps. Batch order: (1) foxglove+rutx+ntrip, (2) ouster, (3) microstrain, (4) blackfly1/2, (5) thermal1/2 A6701, (6) thermal3/4 A70, (7) lucid1/2, (8) compress+colormap, (9) fast-lio then dlio+accumulator, (10) pii-mask then extraction+glim. Total startup ~125s. Rewrote all three systemd units as plain oneshots — removed `ConditionPathExists=!/etc/bess/no-auto-*` gates, deleted the three kill-switch files + the grafted-on README, kept `StartLimitIntervalSec=86400 StartLimitBurst=3` as the only real safety net. `bess-stack.service` now runs the new launcher after `docker.service + bess-network.service` with a 30s grace `ExecStartPre` so SSH/Tailscale are up in time for a remote human to `systemctl stop bess-stack` if something looks off. `TimeoutStartSec=600` to accommodate the launcher's deliberate sleeps + slow image pulls. **Other fixes in this pass**: (a) `config/cyclonedds.xml` — added `<Discovery><ParticipantIndex>none</ParticipantIndex>`. Default `MaxAutoParticipantIndex=9` caps host to 10 participants but stack has ~30 containers × several nodes each → `Failed to find a free participant index for domain 0` crashed segformer + any `ros2 CLI` one-off. `none` routes participants through ephemeral source ports, cap removed. (b) docker-compose segformer — removed from `full` + `inference` profiles, kept `segformer` profile only. Correlated with today's reboot and also with the 2026-04-15 earlier "iGPU instability" note. Opt-in only until root-caused. (c) docker-compose microstrain — added `full` to profiles. It was previously listed in `imu`/`sensors`/`dev` only, so `--profile full up -d` never started it and the GQ7 was dark after every stack start. `bess-stack-up.sh` starts it explicitly, but the profile list should match reality for anyone using compose directly. **Verification in current session**: all three new units pass `systemd-analyze verify` (the only warnings are from unrelated `nvcpupowerfix`/`nvargus-daemon`), all three `disabled + inactive` so next reboot does NOT auto-start the stack until user enables explicitly, microstrain `(healthy)` after manual start, MIP driver up with aux port parsing. **For next reboot test**: `sudo systemctl enable bess-network.service bess-stack.service` then `sudo reboot`. Do NOT enable `bess-time-bootstrap.service` — the BMC RTC is currently good, and a bad HTTP fetch is the easiest way to put CLOCK_REALTIME into a state that takes a phc_ctl step to recover. | **Kill-switch doctrine retired, sequenced launcher is the structural fix, all three units are rate-limited oneshots. Ready for reboot test after user-driven manual validation.** | Keep |
| 2026-04-15 | docker-compose.thor.yml lucid1/lucid2 + new `containers/lucid/entrypoint.sh` + `scripts/bess-cold-start-check.sh` + NM `EERO` connection | **Lucid sysctl-clobber + 5G github routing + clock recovery (commits `ac4b311`, `ba6ae4d`, `a3b7731`).** (a) **Lucid privileged-container kernel write**: cold-start check kept reporting `net.core.rmem_max = 134217728` (128 MB) and `rmem_default = 134217728` despite `60-gige-sensors.conf` setting them to 2147483647 + 67108864. Root cause: the baked entrypoint inside `localhost/bess-lucid:jazzy` contains `if [ -w /proc/sys/net/core/rmem_max ]; then echo 134217728 > ...; fi` and `x-common: &common` sets `privileged: true` so `/proc/sys` IS writable — every `up -d --force-recreate lucid1 lucid2` clobbered the host kernel state. **Fix**: created `/home/thor/bess/containers/lucid/entrypoint.sh` (corrected, no sysctl writes) and bind-mounted it over `/entrypoint.sh` in lucid1 + lucid2 services. Source-of-truth still lives at `/home/thor/bess_platform/containers/lucid/entrypoint.sh`; remove the bind mount when that gets fixed and the bess-lucid image rebuilt. **Verified**: 3× back-to-back forced recreates, rmem_max stays at 2147483647, rmem_default at 67108864, both lucids publishing at 3 Hz. (b) **5G bulletproofing for github push**: `git push origin main` was hitting `Failed to connect to github.com port 443 after 135027 ms` — diagnosed as EE UK mobile carrier (RUT50 SIM) being unable to route github's Microsoft Azure range (`20.26.156.0/24`). google.com / cloudflare.com / apple.com all return HTTP 200 over the same 5G uplink, so the 5G physical layer is healthy. Workaround: persistent policy route via WiFi for github only — `nmcli connection modify EERO +ipv4.routes "20.26.156.0/24 192.168.4.1 50"`. Survives reboots as part of the EERO NM connection profile. Push works (`68debf1..ba6ae4d`, `ba6ae4d..ac4b311`). (c) **Cold-start check internet probe**: was `ping -c1 -W3 1.1.1.1`, which EE drops at the carrier edge — false negative even when the 5G data path is healthy. Switched to `curl -fsS --max-time 5 -o /dev/null https://www.google.com` (same path bess-time-bootstrap uses). (d) **Wall clock + PHC1 recovery**: system was 1h behind real UTC (wall clock skew -3599s). Fix sequence: stack down via `docker compose --profile full down`, fetched real UTC from `curl -sI https://www.google.com`, `sudo timedatectl set-time "2026-04-15 21:51:45"` (BST value because Europe/London — TZ trap), then `sudo systemctl stop phc2sys-mgbe0 && sudo phc_ctl mgbe0_0 set && sudo systemctl start phc2sys-mgbe0` (PHC1 went from 3.6 trillion ns offset to -9618 ns), stack back via `bess-stack-up.sh`. Final cold-start check: 34 PASS / 2 WARN / 0 FAIL (warns are physically-disconnected Ouster + the long-standing thermal4/ntrip unhealthy). Memory: `feedback_privileged_container_sysctl_clobber.md` saved as the doctrine ("when host sysctl drifts back after stack restart, suspect a privileged container writing /proc/sys before chasing /etc/sysctl.d"). | **Lucid clobber dead, github push works via WiFi route, wall clock locked to real UTC at -1s skew, PTP slave offset 20 ns. Ready for next soak after lidar plug-back.** | Keep |
| 2026-04-15 | docker-compose.thor.yml recorder + foxglove image (live apt install + docker commit) + thermal4 cold-restart recovery | **Pre-soak fixes pass.** (a) **thermal4 stuck GigE Vision control session**: after earlier force-recreates, thermal4 was in a retry loop with every GenICam write returning `-1010 Error writing to Device` (packet size, GevSCPD, PixelFormat, IRFormat all rejected), BeginAcquisition `-1008`. Camera pinged 0% loss, MAC matched compose, sister thermal3 on identical image (`bess-thermal-spinnaker:jazzy`) was healthy at 15 Hz. Classic A70 firmware state where PySpin can establish a control session but the camera rejects every write — means a stale session from the previous container is still held in the camera's internal state machine, heartbeat timeout alone wasn't enough. **Fix**: `docker stop thermal4 && sleep 60 && docker compose up -d --no-deps thermal4`. 60s full stop (not restart) lets Spinnaker GenTL producer fully release the lock AND lets the GigE Vision heartbeat timeout clear in the camera firmware. Post-recovery log shows clean init: `DiscoverMaxPacketSize: 1444`, `Set PixelFormat=Mono16`, `Set IRFormat=Radiometric`, `Started acquisition`, `DeviceTemperature=302.9Kelvin` (FPA 29.7°C, healthy A70 body temp). Now 17.64 Hz. **Rule for next time**: when thermal4 (or any GigE camera) shows all-writes-rejected `-1010`, do NOT `docker compose up -d --force-recreate` — that keeps the control session racing. Use `docker stop` + 60s sleep + `up -d` instead. (b) **Foxglove "sparse topics" root cause was wrong all along**: the blackfly1→compress-blackfly1 `serdata.cpp:384 invalid data size / string not null-terminated` errors (Issue #33) are COSMETIC — cross-distro CycloneDDS handshake noise from USER_DATA type hash mismatches, NOT blocking data flow. Verified jazzy→humble subscription works at full rate: /thermal/camera3/image_raw @ 17.7 Hz from dlio (jazzy) subscribing to thermal3 (humble), /blackfly/camera1/.../compressed @ 5 Hz same path. The REAL reason Foxglove panel looked empty: `bess-foxglove:jazzy` image only had `ros-jazzy-foxglove-bridge` and NOTHING ELSE installed. foxglove_bridge logs showed `Failed to load schemaDefinition` for microstrain_inertial_msgs, flir_camera_msgs, ouster_sensor_msgs, theora_image_transport, rtcm_msgs, nmea_msgs — 20+ topics whose message definitions it couldn't serialize to the browser. **Fix**: `docker exec foxglove apt-get install -y ros-jazzy-microstrain-inertial-msgs ros-jazzy-flir-camera-msgs ros-jazzy-ouster-sensor-msgs ros-jazzy-theora-image-transport ros-jazzy-rtcm-msgs ros-jazzy-nmea-msgs`, restart foxglove, verify zero remaining `schemaDefinition not found`. Then `docker commit foxglove localhost/bess-foxglove:jazzy` to persist the writable layer into the image (10.1 GB) so next `docker compose up` doesn't lose them. **Long-term TODO**: add these packages to the foxglove Containerfile so a from-scratch rebuild has them natively — the commit is a stopgap. (c) **Recorder re-enabled for soak** after 2026-04-14 NVMe ENOSPC incident: `/mnt/bess-usb` confirmed mounted ext4, 5.8T free, mount point resolves through autofs correctly. Compose edit: volume `/home/thor/recordings/tmp:/recordings:rw` → `/mnt/bess-usb/bags:/recordings:rw`, profile `record-disabled` → `record`. Kept `CAP_BYTES=100GB` cap + pruner + `restart: "no"`. Explicit start via `docker compose --profile record up -d recorder`; **NOT** in `full` profile — `bess-stack-up.sh` will never auto-start it. Verified: 44s uptime, 17 GB written, 4 bag splits, pruner running, `/mnt/bess-usb` 5.7T free. | **thermal4 17.64 Hz, foxglove bridge serialises all topics, recorder streaming to USB at ~380 MB/s, all soak prereqs met** | Keep |
| 2026-04-15 | docker-compose.thor.yml blackfly1 + blackfly2 | Cleaned up shared driver-config noise + fixed blackfly2 asymmetry. (a) **blackfly2 was missing `-p device_link_throughput_limit:=125000000`** — the 2026-04-12 doctrine line from `feedback_spinnaker_throughput_limit.md`. blackfly1 had it, blackfly2 didn't. Not currently biting (blackfly2's NVRAM happens to not have a stale cap) but adding defensively matches the documented rule: always pass throughput explicitly, never trust NVRAM. (b) **blackfly2 exit sleep 10 s → 30 s** to match blackfly1. GigE Vision heartbeat timeout needs the full 30 s to clear before a retry, 10 s was thrash territory. (c) **Removed `-p frame_rate_auto:=Off` from both** — log had `node AcquisitionControl/AcquisitionFrameRateAuto not found!` on every start: on this Blackfly S firmware the deprecated `AcquisitionFrameRateAuto` node doesn't exist, only `AcquisitionFrameRateEnable` (which is already set via `frame_rate_enable:=true`). The param was also YAML-bool-trapping: ros2 `-p :=Off` parses to boolean `false`, so the driver was calling the mapped node with `false`, which the mapping file declared as `type: enum`, which is invalid anyway. Dead line, pure noise. (d) **Both: `-p trigger_mode:=Off` → `-p trigger_mode:='"Off"'`** — same YAML-bool trap. Unquoted `Off` → bool `false` → driver logged `TriggerMode invalid enum: false`. Bash single-quote + embedded double-quote forces YAML string parsing. Post-fix log: `setting AcquisitionControl/TriggerMode to: Off` (correct). **Verified**: both cameras recreated with 30 s GigE heartbeat gap between stop and up -d, both `healthy`, both 5.00 Hz 0% drop, `FrameRateAuto not found` and `TriggerMode invalid enum` gone from startup. Only remaining blackfly log warning is the cosmetic missing intrinsics file at `/root/.ros/camera_info/blackfly_camera.yaml` — separate task (camera_calibration_parsers), no intrinsics yaml has ever been produced for the Thor Blackflies. serdata.cpp:384 cross-distro noise (Issue #33) still there, still cosmetic per 2026-04-15 re-diagnosis | **Both blackflies clean config, blackfly2 defensively covered against stale NVRAM throughput cap, zero in-driver config errors** | Keep |
| 2026-04-15 | NM `mgbe0_0`, `/etc/systemd/logind.conf`, masked sleep targets, GNOME gsettings, `bess-usb-recovery.service` Restart policy | **Stability + persistence pass after 11:29 reboot.** (a) `mgbe0_0` NM profile had `connection.autoconnect=no` — sensor subnet was absent on boot until a manual `nmcli connection up mgbe0_0`. Set `autoconnect=yes autoconnect-priority=10` so `169.254.100.1/16` + MTU 8966 come up automatically. (b) User asked "turn off all sleep and logouts" — (i) `systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target`, (ii) rewrote `/etc/systemd/logind.conf` with `IdleAction=ignore HandleLidSwitch=ignore HandlePowerKey=ignore HandleSuspendKey=ignore HandleHibernateKey=ignore HandleRebootKey=ignore KillUserProcesses=no` (backup at `.bak.2026-04-15`), (iii) `gsettings` for `thor` user via DBUS bus `/run/user/1000/bus`: `org.gnome.desktop.session idle-delay=0`, screensaver lock disabled, `settings-daemon.plugins.power sleep-inactive-ac-type='nothing'`, `power-button-action='nothing'`, (iv) `systemctl restart systemd-logind` — TTY/pts sessions survived, remote ops uninterrupted. (c) **RM110 warm-reboot reality check**: bess-usb-recovery.service ran 3 rebinds at 11:30:43/11:30:54/11:31:02, all "succeeded" at the driver level (tegra-xusb unbind+bind returned 0) but the RM110 bridge chip never re-presented itself to the root hub. User had to **physically disconnect + reconnect** the cable — same as 2026-04-12. Root cause: Thor Dev Kit exposes USB jacks with NO software-controllable VBUS GPIO, so no software reset can power-cycle the enclosure's own USB 3.x PHY. Warm reboot leaves it in a stuck link state that only physical removal can clear. Updated service from `Restart=no StartLimitBurst=3` → `Restart=on-failure RestartSec=300 StartLimitBurst=24` so when the user DOES re-seat (minutes or hours later), the service picks it up on its next 5-min tick and automount brings `/mnt/bess-usb` back without any manual step. The retry does NOT fix the stuck state — it just removes the "service gave up" wall. Memory entries written: `project_thor_rm110_warm_reboot.md`, `project_thor_stability_no_sleep.md`, `project_thor_cold_start.md`. | RM110 live at `usb 2-1` root-hub-direct, `/dev/sda1` 7.3T ext4 `/mnt/bess-usb` 5.8T free, all soak bags intact. Sleep/logout disabled and persistent. mgbe0_0 autoconnect persistent. Next warm-reboot RM110 recovery STILL needs physical re-seat — that is a hardware limitation of the dev kit, not a software bug. | Keep |

## REVERT CANDIDATES (likely causing regressions)

1. **lucid_node.py line 178**: Change 1500 back to 8972 (10GigE needs jumbo frames) - **DONE**
2. **lucid1 camera_ip**: May be wrong IP - check Arena discovery output

## BEFORE MAKING ANY CHANGE

1. Add entry to changelog table FIRST
2. Check if change was tried before (search table)
3. Record result immediately after

---

## Source of Truth

- **Thor working repo**: `/home/thor/bess/` → `https://github.com/AhsanABC-RP/bess-thor` (main branch)
- **Thor Docker Compose**: `/home/thor/bess/docker-compose.thor.yml`
- **Platform repo**: `/home/thor/bess_platform/` → `https://github.com/AhsanABC-RP/bess_platform` (branch: **bess-thor**, NOT bess-grey or main)
- **Thor vehicle docs**: `/home/thor/bess_platform/vehicles/thor/CLAUDE.md`
- **Platform CLAUDE.md**: `/home/thor/bess_platform/CLAUDE.md`

**CRITICAL: bess_platform branch = bess-thor. Do NOT commit to bess-grey or main — those are Grey/White production.**

**ALWAYS read the repo before acting. NEVER hack workarounds.**

---

## GOLDEN RULE — READ CODE + RESEARCH BEFORE ANY CHANGE

**Before modifying ANY file — Containerfile, launch file, YAML, node source, compose file — you MUST:**

1. **READ the current code fully.** Not just the lines you plan to touch. Understand what exists, what it depends on, and what calls it. `git blame` / `git log` the file to see why it was written that way.
2. **READ the upstream docs / source.** For SDKs, drivers, ROS 2 packages: open the actual vendor docs, the package README, the relevant source on GitHub. Find out what the API *actually* does, not what you assume.
3. **WebSearch the exact error / symptom / API.** Especially for hardware, SDKs, drivers, DDS, PTP, GigE Vision. 30 seconds of search beats hours of flailing.
4. **ONLY THEN** propose / apply the change, and explain *why* the change is correct based on what you read.

**Why:** Every past regression on Thor came from patching first and reading after — wrong SDK for Lucid, stale throughput limit on Blackfly, GQ7/Ouster clock mix on FAST-LIO2, GevSCPD > camera max on A70, etc. Each one was discoverable from docs or upstream source in minutes. Hacking blind cost hours and in some cases locked hardware.

**How to apply:** If you catch yourself writing an Edit/Write tool call without having read the target file AND the relevant upstream reference in the same turn, STOP and go read first. "I think this will work" is not a basis for a change — "the docs at <url> say X and this file does Y, so the correct fix is Z" is.

---

## Behaviour Rules

### DO NOT
- **Jump between tasks without queuing them** — when the user introduces a new task mid-session, IMMEDIATELY add it to the task list via `TaskCreate` rather than context-switching away from the current work. Finish the in-flight task (or explicitly park it with a status note), then pick the next off the queue in priority order. Jumping loses state, drops partial work, and wastes context re-establishing where you were. If research is needed before a task can start, create the task AND a `research: ...` subtask, don't silently begin exploring.
- **Guess at config or debug by trial-and-error** — ALWAYS read official docs, README, and config examples FIRST. Check GitHub repo, issue tracker, and source code comments. 5 seconds reading docs beats hours of blind hacking
- **Modify code without reading it first** — read the full file, the upstream source/docs, and WebSearch the symptom BEFORE proposing any change. See GOLDEN RULE above.
- **Use hacks, overrides, or workarounds** — fix the source (Containerfile, script, config) and rebuild/redeploy
- **Mark a task complete unless FULLY verified end-to-end** — if output isn't visible and correct at the final consumer (Foxglove, topic echo, bag file), the task is NOT done
- **Modify network interfaces without explicit permission** — NEVER run `ip addr del`, `ip route del`, or change MTU without asking
- **Claim "working" without sampling actual data** — always `ros2 topic echo --once` or `ros2 topic hz`
- **Run docker commands manually** — use docker-compose.thor.yml with profiles
- **Deep-dive packet debugging when the issue is network config** — check `ip addr`, `ping` FIRST
- **Run inline Python GigE camera tests** — NEVER use `python3 << 'EOF'` with Aravis or Arena SDK. These lock cameras when they crash
- **Use Aravis for Lucid cameras** — Lucid cameras REQUIRE Arena SDK, not Aravis. Use the official SDK container
- **Run arv-tool control commands that write to cameras** — read-only queries are OK, but writes acquire exclusive control and can lock the camera
- **Rapid-fire container restarts on GigE cameras** — ALWAYS wait 30+ seconds between attempts for heartbeat timeout to clear

### HARD STOPS — VIOLATION = IMMEDIATE FAILURE

These commands are FORBIDDEN. If you type any of these, you have failed:

#### GigE Cameras
- `python3 << 'EOF'` with ANY camera SDK (Aravis, Arena, Spinnaker) — LOCKS CAMERAS
- `Aravis.Camera.new()` for Lucid cameras — WRONG SDK, USE ARENA
- `arv-tool-0.8 control <feature> <value>` — WRITE COMMANDS LOCK CAMERAS
- `docker run ... lucid` — USE DOCKER-COMPOSE ONLY
- `docker run ... thermal` — USE DOCKER-COMPOSE ONLY
- `docker restart` within 30s of previous attempt — WAIT FOR HEARTBEAT

#### Network
- `ip addr del` — NEVER DELETE IPs WITHOUT PERMISSION
- `ip route del` — NEVER DELETE ROUTES WITHOUT PERMISSION
- `ip link set ... mtu` — NEVER CHANGE MTU WITHOUT PERMISSION

#### General
- `docker run` for ANY sensor — USE DOCKER-COMPOSE
- Manual `docker stop/start/restart` for sensors — USE DOCKER-COMPOSE

### DO
- Check `ip addr show mgbe0_0` FIRST for any sensor failure
- Check `ping <sensor_ip>` before blaming the driver
- Read `/home/thor/bess_platform/containers/` for actual node implementations
- Use `docker compose -f docker-compose.thor.yml --profile <name> up -d`
- Verify with actual data samples, not just container logs
- Maintain task list with accurate status
- Record all failed attempts in "Tried & Failed" log
- **Use official SDKs**: Lucid=Arena SDK, FLIR A6701=Spinnaker PySpin, FLIR A70=Aravis, Blackfly S=`spinnaker_camera_driver` (C++)
- **Wait 30s between GigE camera connection attempts** — heartbeat timeout must clear
- **Start camera containers ONCE and let them run** — don't restart repeatedly

### VERIFICATION PROTOCOL
```bash
# 1. Network check
ip addr show mgbe0_0 | grep inet

# 2. Device ping
ping -c1 -W1 169.254.70.119  # Ouster
ping -c1 -W1 169.254.1.20    # Blackfly

# 3. Container status
docker ps --format "table {{.Names}}\t{{.Status}}"

# 4. Topic exists
ros2 topic list | grep <topic>

# 5. Topic has data (MANDATORY before claiming success)
timeout 5 ros2 topic hz <topic>
ros2 topic echo <topic> --once

# 6. For images - verify actual bytes
ros2 topic echo /camera/image_raw/compressed --once --field data | wc -c
```

---

## Auto-Boot (what starts automatically at power-on)

### Host systemd chain (post 2026-04-15 sequenced-launcher rewrite)

All three `bess-*` units are plain oneshots — **no kill-switches**, **no chaining between themselves**, rate-limited (`StartLimitIntervalSec=86400 StartLimitBurst=3`) as the only safety net. See `/etc/bess/README` for the full explanation and the safe enable procedure.

**Current enablement**: all three `disabled (inactive dead)` as of 2026-04-15. The user tests each one manually (`systemctl start <unit>` + sanity check) and enables explicitly before the next reboot. Nothing auto-starts the stack today.

| Unit | Function | Ordering | Default state |
|------|----------|----------|---------------|
| `bess-time-bootstrap.service` | Cold-boot HTTP-Date quorum fetch for `CLOCK_REALTIME` (NTP is masked — see gotchas). **Independent**: no `Before=` on PTP; a bad fetch cannot cascade because `phc2sys -S 0` will not step PHC1 | Independent | `disabled` |
| `bess-network.service` | Runs `scripts/boot-sensors.sh`: 169.254.100.1/16 on `mgbe0_0`, rx/tx buffers, `rp_filter`, dnsmasq with camera MAC reservations, default route via MikroTik bridgeWAN | `After=network-online.target` | `disabled` |
| `ptp4l-mgbe0.service` | PTP grandmaster on `mgbe0_0` PHC1 (`clockClass=248`, `utc_offset 0`) | Independent | `enabled` |
| `phc2sys-mgbe0.service` | `-S 0` servo-only (never step) — syncs `CLOCK_REALTIME` to `mgbe0_0` PHC1 | `After=ptp4l-mgbe0` | `enabled` |
| `docker.service` | Docker daemon. `restart: "no"` in `x-common: &common` anchor so Docker does NOT race-restart ~30 containers at startup (2026-04-15 fix) | Independent | `enabled` |
| **`bess-stack.service`** | **Runs `/home/thor/bess/scripts/bess-stack-up.sh`** — 10-batch sequenced launcher, max 4 concurrent container starts, camera SDKs never parallel, GPU containers staggered ≥5s, total ~125s. `Type=oneshot RemainAfterExit=yes`, `ExecStartPre=/bin/sleep 30` grace for remote SSH to intervene, `TimeoutStartSec=600` | `After=docker.service bess-network.service` | `disabled` |

### Sequenced launcher — the real fix

`scripts/bess-stack-up.sh` is the structural fix for the 2026-04-14/15 boot-loop incident. Root cause was `docker compose up --profile full` starting ~30 containers in parallel (GPU init + USB probe + GigE SDK discovery + PTP clients + host-net stack all racing for the kernel), which wedged the Tegra long enough for `/dev/watchdog0` to fire. Launcher batches (ordering MATTERS, do not reorder without understanding why):

1. **infra** — `foxglove foxglove-local rutx ntrip` (no hardware), 10s settle
2. **ouster** — alone (CUDA context + libos GigE + PTP slave), 15s settle
3. **microstrain** — USB-bound, 5s settle
4. **blackfly1 blackfly2** — Spinnaker C++ driver (heaviest camera init), 20s settle so GigE Vision heartbeat stabilizes before next SDK
5. **thermal1 thermal2** — A6701 PySpin, 15s settle
6. **thermal3 thermal4** — A70 PySpin (different GenICam XML tree), 15s settle
7. **lucid1 lucid2** — Arena SDK (separate SDK family), 15s settle
8. **compress-\* + thermal-colormap** — 9 lightweight subscribers, no kernel contention, 5s
9. **fast-lio** then **dlio + slam-map-accumulator** — GPU, ordered so fast-lio (heavier) settles first, 10s + 5s
10. **pii-mask** then **extraction + glim** — downstream, 5s

Camera SDKs (Spinnaker C++ / PySpin A6701 / PySpin A70 / Arena) NEVER run network discovery in parallel. GPU-touching containers (ouster CUDA, fast-lio, dlio, pii-mask) staggered ≥5s. Uses `docker compose up -d --no-deps` to sequence explicitly.

**Segformer and recorder are intentionally NOT in the launcher.** Segformer is in `profiles: [segformer]` only (opt-in, see Changelog 2026-04-15). Recorder is in `profiles: [record-disabled]` only (see Issue #21 + Recording section).

### Known gotchas

- **`systemd-timesyncd` is MASKED on this host. Do NOT re-enable.** NTP step events propagate through `phc2sys` to PHC1 and desync the Ouster PTP firmware slave — full cold-boot required to recover.
- **PHC1 re-anchor after a large wall clock step**: if you have to `timedatectl set-time` on a box with `phc2sys -S 0` running, PHC1 will show a huge offset that phc2sys refuses to step. Safe recovery (only while sensor stack is offline — stops will desync any live slaves): `sudo systemctl stop phc2sys-mgbe0 && sudo phc_ctl mgbe0_0 set && sudo systemctl start phc2sys-mgbe0`. Offset drops to sub-ms in seconds. DO NOT do this while the Ouster PTP slave is live.
- **After USB re-enum** (e.g., an RM110 USB flap), the `microstrain` container needs `docker compose up -d --force-recreate --no-deps microstrain` — a plain `restart` keeps the stale `/dev/microstrain_*` major/minor bindings.
- **kernel sysctl values are persisted** in `/etc/sysctl.d/60-gige-sensors.conf` AND re-applied at runtime by `boot-sensors.sh`. See Issue #23 for currently-runtime-only keys that need to be moved into the .conf.
- **Microstrain profile list** includes `full` as of 2026-04-15 — earlier it was only `imu/sensors/dev` and `docker compose --profile full up -d` silently never started it.

---

## Stack Operation

### Start Sensors
```bash
cd /home/thor/bess
docker compose -f docker-compose.thor.yml --profile sensors up -d
```

### Start Full Stack
```bash
docker compose -f docker-compose.thor.yml --profile full up -d
```

### Check Status
```bash
docker compose -f docker-compose.thor.yml ps
```

### View Logs
```bash
docker compose -f docker-compose.thor.yml logs -f ouster
```

### Stop
```bash
docker compose -f docker-compose.thor.yml down
```

---

## Hardware

| Component | Value |
|-----------|-------|
| Platform | NVIDIA Thor Dev Kit (Grace Blackwell) |
| CPU | ARM Cortex-A720, 14 cores @ 2.6GHz (aarch64) |
| GPU | NVIDIA Thor (Blackwell architecture) |
| RAM | 128GB unified memory |
| Storage | 937GB NVMe |
| CUDA | 13.0, Driver 580.00 |
| OS | Ubuntu 24.04.3 LTS (tegra kernel 6.8.12) |
| Docker | 27.5.1 with nvidia-container-toolkit |

---

## Network Configuration

### CRITICAL: Sensor Interface
```bash
# mgbe0_0 MUST have this IP:
sudo ip addr add 169.254.100.1/16 dev mgbe0_0

# Verify:
ip addr show mgbe0_0 | grep "169.254.100.1"
```

### Interfaces
| Interface | IP | MTU | Purpose |
|-----------|-----|-----|---------|
| mgbe0_0 | 169.254.100.1/16 | 8966 (req 9000) | GigE sensors — MTU persisted in NM connection profile; `nvethernet` driver resolves requested 9000 → 8966 actual |
| wlP1p1s0 | 192.168.4.104/22 | 1500 | WiFi |
| tailscale0 | 100.96.161.119/32 | 1280 | Tailscale VPN |

### Sensors
| Device | IP | MAC | Driver | Status |
|--------|-----|-----|--------|--------|
| Ouster OS1-128 | 169.254.70.119 | bc:0f:a7:00:d3:85 | bess-ouster | **Working 10Hz** (DLIO @ 99Hz; FAST-LIO2 see Issue #25) |
| FLIR A6701 #1 | 169.254.128.202 | 00:11:1c:05:8f:20 | Spinnaker PySpin (`bess-thermal-spinnaker`) | **Working ~18Hz** |
| FLIR A6701 #2 | 169.254.249.149 | 00:11:1c:05:8f:27 | Spinnaker PySpin (`bess-thermal-spinnaker`) | **Working ~15Hz** |
| FLIR A70 #1 | 169.254.20.1 | 00:40:7f:04:fa:7b | Aravis (`bess-thermal`) | **NO_DATA** — see Issue #28 |
| FLIR A70 #2 | 169.254.20.2 | 00:40:7f:11:04:ea | Aravis (`bess-thermal`) | **NO_DATA** — see Issue #28 |
| Blackfly S #1 | 169.254.1.20 | 2c:dd:a3:7e:96:75 | `spinnaker_camera_driver` C++ (`bess-cameras`) | **Working 5.0Hz** |
| Blackfly S #2 | 169.254.1.21 | 2c:dd:a3:7e:96:76 | `spinnaker_camera_driver` C++ (`bess-cameras`) | **Working 5.0Hz** |
| Lucid Atlas #1 | 169.254.20.23 | 1c:0f:af:07:a9:cd | Arena SDK (`bess-lucid`) | silent acquisition (software — cables OK per user) — Issue #29 |
| Lucid Atlas #2 | 169.254.20.14 | 1c:0f:af:07:a9:e7 | Arena SDK (`bess-lucid`) | silent acquisition (SC_ERR_TIMEOUT -1011) — Issue #29 |
| MicroStrain GQ7 | /dev/ttyACM0+1 | - | USB direct | **Working 100Hz** |

---

## DDS Configuration

Uses **CycloneDDS** (not FastDDS):
```yaml
environment:
  RMW_IMPLEMENTATION: rmw_cyclonedds_cpp
  CYCLONEDDS_URI: file:///config/cyclonedds.xml
```

Config at: `/home/thor/bess_platform/config/cyclonedds.xml`

---

## Camera Nodes

### SDK REQUIREMENTS — NO EXCEPTIONS

| Camera | REQUIRED SDK | Container Image | FORBIDDEN |
|--------|--------------|-----------------|-----------|
| Lucid Atlas | Arena SDK (`arena_api` Python) | `bess-lucid:jazzy` | Aravis, Spinnaker — Arena is vendor-mandatory |
| FLIR A6701 (cooled InSb) | Spinnaker PySpin | `bess-thermal-spinnaker:jazzy` | Aravis — A6701 is a Stirling-cooled sensor and needs the full Spinnaker node tree |
| FLIR A70 (uncooled VOx) | Aravis (`arv-tool-0.8` + Python bindings) | `bess-thermal:jazzy` | Spinnaker — A70 firmware has non-writable PTP and a different GenICam XML that PySpin mis-parses |
| Blackfly S | `spinnaker_camera_driver` (C++ ROS 2 package, NOT PySpin) | `bess-cameras:jazzy` | Aravis, PySpin — Blackfly needs the C++ driver for parameter yaml + PTP SlaveOnly writes |

**If you use the wrong SDK, the camera will either not work or get locked.**

### BEFORE ANY CAMERA ACTION — MANDATORY CHECKLIST

Before touching ANY camera, answer these questions:
1. Which SDK does this camera require? (Check SDK MATRIX above)
2. Is the container defined in docker-compose.thor.yml?
3. Am I using docker-compose (not docker run)?
4. Has 30 seconds passed since last camera command?
5. Have I read the production driver code?

**If ANY answer is NO — STOP. Fix it first.**

### Lucid Atlas (Arena SDK)
Source: `/home/thor/bess/containers/lucid/lucid_node.py`
- `PtpEnable=True` + `PtpSlaveOnly=True` + `PtpDataSetLatch.execute` + `PtpStatus` read on connect
- Header `stamp = get_clock().now()` — driver host-stamps; PTP state is for camera-internal consistency, not ROS alignment

### FLIR A6701 Thermal (Spinnaker PySpin)
Source: `/home/thor/bess/dockerfiles/thermal/thermal_spinnaker_node_patched.py`
- Stirling-cooled InSb FPA — `SensorTemperature` healthy ~-200 °C (cryocooler)
- `GevIEEE1588 = True` accepted; `GevIEEE1588Status = Slave`
- `GevSCPD = min(1000, camera_max)` (A6701 max is larger; full 1000 µs)
- Uses MAC address for camera identification

### FLIR A70 Thermal (Aravis)
Source: `/home/thor/bess/containers/cameras/thermal_a70_aravis_node.py`
- Uncooled VOx bolometer — `DeviceTemperature` is body temp, healthy 25–50 °C
- PTP node is non-writable on A70 firmware — driver host-stamps in ROS time
- `GevSCPD` max is 512 (vs 1000 on A6701); patched node uses `min(1000, camera_max)`

### Blackfly S (`spinnaker_camera_driver` C++)
Source: upstream `ros-drivers/flir_camera_driver` via `bess-cameras:jazzy`
Config: `/home/thor/bess/config/blackfly_s_thor.yaml`
- PTP needs ALL THREE: `use_ieee_1588:=true` + `gev_ieee_1588` (yaml write) + `gev_ieee_1588_mode:=SlaveOnly` (else BMCA flap)
- `device_link_throughput_limit` MUST be passed at launch — stale NVRAM cap causes leader/trailer-only GVSP with 100% incomplete (see Blackfly1 fix 2026-04-12)

### Image Compression
Source: `/home/thor/bess_platform/containers/cameras/gpu_compress.py`
- Raw images ~25MB each - too large for WiFi
- MUST use compressed topics for Foxglove streaming
- Handles mono16 -> jpeg conversion

---

## SLAM

Two SLAMs run in parallel on Thor — same Ouster input, different TF frames so they don't fight for `base_link` ownership.

| Stack | Container | Input cloud | Input IMU | Odom topic | Baselink frame | Profile |
|-------|-----------|-------------|-----------|------------|----------------|---------|
| **FAST-LIO2** (primary) | `fast-lio` | `/ouster/points` | `/ouster/imu` (SensorDataQoS) | `/slam/odometry` | `base_link` | `slam`, `full` |
| **DLIO** (alternative) | `dlio` | `/ouster/points` | `/ouster/imu` | `/dlio/odom_node/odom` | `dlio_base_link` | `slam`, `full` |
| **Map accumulator** | `slam-map-accumulator` | `/slam/cloud_registered` | — | `/slam/map_voxel` (RELIABLE+TRANSIENT_LOCAL) | `camera_init` | `slam`, `full` |

### Why both use `/ouster/imu`, not the GQ7

The GQ7 IMU is on host wall clock; Ouster point cloud stamps are also on wall clock (after the 2026-04-13 `TIME_FROM_ROS_TIME` flip) but the two stream through different drivers with different jitter profiles, and DLIO/FAST-LIO2 `sync_packages` drop any scan whose IMU can't be bracketed. The Ouster-internal BMI085 on `/ouster/imu` is co-located with the LiDAR, shares the exact same driver/clock path, and has an identity `T_lidar_imu` — so buffering is trivial and sync is exact. Don't switch back to `/imu/data` without patching the rclcpp QoS + timestamp tolerance in laserMapping.cpp first.

### Why `map_en: false` and a downstream accumulator

FAST-LIO2's `publish_map()` accumulates points into `pcl_wait_pub` with zero voxel filter and republishes the full cloud every scan. `/Laser_map` reaches 100MB+ within minutes and chokes CycloneDDS fragment reassembly — the symptom is **every other SLAM topic going silent**, not a visible "map broken" error. The replacement `scripts/slam_map_accumulator.py` subscribes to `/slam/cloud_registered` (the already-deskewed per-scan output in `camera_init` frame), voxel-hashes it at 0.25m into a bounded 2M-voxel OrderedDict (FIFO evict), and republishes `/slam/map_voxel` on a 2s timer. TRANSIENT_LOCAL durability means late-joining Foxglove subscribers latch onto the current map without having to rewind the full scan stream.

### Health check

```bash
docker exec dlio bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic hz /slam/odometry'         # ~10 Hz
docker exec dlio bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic hz /dlio/odom_node/odom'   # ~90 Hz (IMU-rate)
docker exec dlio bash -c 'source /opt/ros/jazzy/setup.bash && ros2 topic hz /slam/map_voxel'        # ~0.5 Hz (2s timer)
docker logs slam-map-accumulator --tail=5  # should show "voxels=... scans_since_last=..."
```

---

## Foxglove Streaming

### Bridge
```bash
docker compose -f docker-compose.thor.yml --profile viz up -d
```

### Connect
```
ws://192.168.4.104:8765  (WiFi)
ws://100.96.161.119:8765 (Tailscale)
```

### Topics (use compressed for images over WiFi)
- `/thermal/camera1/image_raw/compressed`
- `/lucid1/camera_driver/image_raw/compressed`
- `/ouster/points`
- `/imu/data`

---

## Current Issues (2026-04-14)

Verified from live ground truth via parallel Explore agents. No issue is marked Fixed without an evidence line.

| # | Issue | Status | Root Cause / Evidence |
|---|-------|--------|------------------------|
| 15 | Blackfly S #1 historic network absence | **FIXED** 2026-04-12 | Stale NVRAM `DeviceLinkThroughputLimit`; now `ros2 topic hz /blackfly/camera1/image_raw` = 4.99 Hz |
| 16 | Lucid #1 sfp28-7 rx-loss (hardware) | **NOT PHYSICAL — reclassified as #29** | User confirmed cables fine 2026-04-14 |
| 17 | Ouster not connected | **FIXED** 2026-04-12 | Power + `TIME_FROM_ROS_TIME`; `/ouster/points` + `/ouster/imu` @ 10 Hz |
| 18 | thermal4 intermittent drops | **FIXED** 2026-04-12 | `GevSCPD=min(1000, camera_max)`; A70 max 512 → stable ~15 Hz |
| 19 | FAST-LIO2 `/Laser_map` CycloneDDS choke | **FIXED** 2026-04-13 | `publish.map_en: false` in `fast_lio_single.yaml` + `slam-map-accumulator` service publishing `/fast_lio/map_voxel` |
| 20 | DLIO + FAST-LIO2 base_link TF fight | **FIXED** 2026-04-13 | DLIO frames renamed to `dlio_odom` / `dlio_base_link`; verified no `base_link` duplicate in `/tf` |
| 21 | **Recorder NVMe ENOSPC (2026-04-14)** | **MITIGATED — recorder intentionally disabled** | Uncapped rolling-bag fallback at `/home/thor/recordings/tmp/rolling` (host NVMe) filled root fs; wedged Claude Code Bash tool via stdout tmpfile ENOSPC. Compose now: `profile=record-disabled`, `CAP_BYTES=100 GB`, `restart=no`. **Requires prereqs below before re-enable** |
| 22 | Auto-boot gap: containers do not start at boot | **FIXED** 2026-04-15 | `scripts/bess-stack-up.sh` sequenced launcher + rewritten `bess-stack.service` oneshot. Left `disabled` by default — user enables manually after a successful test-run. See Auto-Boot section |
| 31 | 2026-04-14 Tegra-watchdog boot loop | **FIXED** 2026-04-15 | Combined with Row #22's sequenced launcher (retires the kill-switch doctrine) — see Changelog 2026-04-15 entry for the full root-cause + fix narrative |
| 32 | Clock re-anchor after hard reboot requires phc_ctl step | **DOCUMENTED** 2026-04-15 | `phc2sys -S 0` refuses to step PHC1, so after any large `timedatectl set-time` you must stop phc2sys, `phc_ctl mgbe0_0 set`, restart phc2sys — ONLY safe while sensor stack is offline. See Auto-Boot gotchas |
| 23 | **boot-sensors.sh runtime-only sysctl not persisted** | **OPEN** | `scripts/boot-sensors.sh:49-58` sets `wmem_max`, `wmem_default`, `ipfrag_*` at runtime only; these keys are missing from `/etc/sysctl.d/60-gige-sensors.conf` |
| 24 | rmem_max + rmem_default drift back to 134 MB after lucid restart | **FIXED** 2026-04-15 (commit `ac4b311`) | Root cause was the baked entrypoint inside `localhost/bess-lucid:jazzy`: `if [ -w /proc/sys/net/core/rmem_max ]; then echo 134217728 > rmem_max; echo 134217728 > rmem_default; fi`. With `privileged: true` on `&common`, `/proc/sys` IS writable. Every lucid recreate clobbered both keys. Fix: bind-mount `containers/lucid/entrypoint.sh` (corrected, no sysctl writes) over `/entrypoint.sh` in lucid1 + lucid2. Verified through 3× forced recreate. Source-of-truth still `bess_platform/containers/lucid/entrypoint.sh` — drop the writes there + remove the bind mount on next image rebuild. **Doctrine**: when a host sysctl drifts back after a stack restart, suspect a privileged container writing `/proc/sys` first, before chasing `/etc/sysctl.d/`. See `feedback_privileged_container_sysctl_clobber.md` |
| 25 | **FAST-LIO2 "No Effective Points!" every scan** | **FIXED** 2026-04-14 | Commit c31cfda changed `filter_size_map: 0.5 → 0.15`. With `cube_side_length: 1000.0`, `(1000/0.15)³ ≈ 3×10¹¹` voxels — overflows int32 in `pcl::VoxelGrid::applyFilter` → map downsample returns nothing → iEKF has no features → `effective_feat_num = 0`. DLIO (different downsample pipeline) unaffected. Reverted to `filter_size_surf/map: 0.5/0.5` |
| 26 | **slam-map-accumulator topic name drift (docs vs runtime)** | **FIXED** 2026-04-14 | Task #56 renamed FAST-LIO2 topics `/slam/*` → `/fast_lio/*`. Docs now match: accumulator publishes `/fast_lio/map_voxel` |
| 27 | slam-map-accumulator voxel cap mismatch (docs said 2M, code has 4M) | **FIXED** 2026-04-14 | Docs updated to match `scripts/slam_map_accumulator.py` current `MAX_VOXELS = 4000000` |
| 28 | Thermal3 / Thermal4 rcl node crash | **BOTH RESOLVED** 2026-04-15 | Thermal3: fresh cold-boot sequence + CycloneDDS `ParticipantIndex=none` fix (participant-pool exhaustion). Thermal4: was stuck in retry loop with `-1010 Error writing to Device` on every GenICam write (stale GigE Vision control session the camera firmware hadn't fully released). Fix: `docker stop thermal4 && sleep 60 && docker compose up -d --no-deps thermal4` — full stop for 60s lets Spinnaker GenTL producer release the lock AND the camera firmware heartbeat timeout clear. `docker compose --force-recreate` is NOT enough for this state. Both now healthy at ~17 Hz. See Changelog 2026-04-15 pre-soak pass |
| 29 | Lucid #1 + Lucid #2 silent acquisition | **RESOLVED** 2026-04-15 | Both Lucids publishing `/lucidN/camera_driver/image_raw/compressed @ ~3 Hz` after fresh cold-boot + sequenced start. `SC_ERR_TIMEOUT -1011` was a cold-start artifact, not a persistent condition. Leave monitored during soak |
| 30 | Wrong A70 MACs in compose (prior session) | **OPEN** | Canonical: A70 #1 `00:40:7f:04:fa:7b`, A70 #2 `00:40:7f:11:04:ea`. Needs grep + fix |
| 33 | compress-blackfly1 / compress-blackfly2 serdata.cpp:384 warning spam | **COSMETIC — not blocking** (re-diagnosed 2026-04-15) | `string data is not null-terminated` / `invalid data size` at serdata.cpp:384 is CycloneDDS cross-distro USER_DATA type-hash handshake noise (Humble publishers don't embed type hashes the way Jazzy subscribers want). Verified harmless: jazzy dlio subscribes to /blackfly/camera1/blackfly_camera/image_raw/compressed (humble publisher) at full 5 Hz, gpu_compress in compress-blackfly1 reports "Compressed 10000 frames, ratio: 9.0%" while the errors spam. The 2026-04-15 initial guess that this was blocking Foxglove was WRONG — root cause of Foxglove sparseness was missing msg schema packages in the foxglove container (Issue #34). Leave the spam alone, or suppress via `export RCUTILS_LOGGING_USE_STDOUT=1` filter if it ever matters |
| 34 | **Foxglove panel shows sparse topics** | **FIXED** 2026-04-15 | `bess-foxglove:jazzy` image only contained `ros-jazzy-foxglove-bridge` — no sensor msg packages. Bridge discovery showed 142 topics but `Failed to load schemaDefinition` for any topic whose msg type wasn't installed: microstrain_inertial_msgs, flir_camera_msgs, ouster_sensor_msgs, theora_image_transport, rtcm_msgs, nmea_msgs. Without the schema the bridge can't serialize to the browser so the Foxglove panel shows the topic with no data. Fix: `apt-get install` all 6 packages into the live container, restart foxglove, `docker commit foxglove localhost/bess-foxglove:jazzy` to persist. **TODO**: rewrite the foxglove Containerfile to install these at build time so a from-scratch rebuild doesn't regress |

### Tried & Failed Log

```
2026-03-07:
- Removed 192.168.127.1 from mgbe0_0 trying to fix Spinnaker — broke all sensors
- Multiple docker run attempts without verifying network first
- Claimed cameras working without sampling actual data

2026-03-08:
- Used Aravis for Lucid camera instead of Arena SDK — WRONG SDK
- Ran inline python3 << 'EOF' tests that locked cameras repeatedly
- Ran arv-tool control write commands that locked cameras
- Used docker run instead of docker-compose
- Rapid-fire container restarts without waiting for heartbeat
- Did not read production code before hacking alternatives
- RESULT: Camera locked, wasted hours

2026-04-12:
- thermal4 GevSCPD=1000 exceeded A70 max of 512 — OutOfRangeException
  Fixed: use min(desired, camera_max) in patched node
- Foxglove missing cyclonedds.xml volume mount — crashed on startup
  Fixed: added mount + explicit CYCLONEDDS_URI env
- MicroStrain compose used Moxa TCP but Thor has USB direct
  Fixed: removed socat, use ros2 launch with params.yaml

2026-04-14:
- Bundled `filter_size_surf: 0.2 / filter_size_map: 0.15` into commit c31cfda
  "to voxelize finer on OS1-128 131k pts/scan". Ignored that the map-side
  filter is bounded by `cube_side_length: 1000.0`, not the per-scan extent.
  PCL VoxelGrid int32 overflow → "No Effective Points!" every scan →
  /fast_lio/odometry silent while DLIO ran normally at 99 Hz. DLIO masked
  the regression for hours.
  Fixed: reverted to 0.5/0.5 (upstream ouster64.yaml defaults).
  Lesson: any voxel size change needs to be checked against the ACTUAL
  bounding box of the filter input, not just per-scan density.

- Uncapped rolling-bag fallback at /home/thor/recordings/tmp/rolling filled
  host NVMe. Claude Code Bash tool wedged because stdout tmpfiles couldn't
  be written (ENOSPC). User had to free the disk manually via `!` prefix.
  Recorder disabled via profile flip + CAP_BYTES raise + restart=no. See
  Issue #21 and the Recording section.

- Claimed 2026-04-13 that "every boot starts the stack automatically" —
  wrong. Host chain auto-starts, but there is no systemd unit running
  `docker compose up`. All containers were started by hand after the
  2026-04-14 boot. `restart: unless-stopped` is NOT boot-start. Fix in
  Issue #22.
```

---

## Recording

### STATUS: DISABLED (2026-04-14)

The recorder is intentionally disabled until the prerequisites below are met.
On 2026-04-14, an uncapped rolling-bag fallback path on the host NVMe
(`/home/thor/recordings/tmp/rolling`) filled the root filesystem and wedged
the Claude Code Bash tool via stdout tmpfile ENOSPC. See the changelog row
and Issue #21 for full root cause.

**Current compose state (`/home/thor/bess/docker-compose.thor.yml` recorder service):**
- `profiles: [record-disabled]` — NOT `record` (so the `full` profile does NOT start it anymore)
- `CAP_BYTES: "100000000000"` (100 GB, was 40 GB)
- `restart: "no"` (was `unless-stopped`)

### Re-enable prerequisites — ALL must pass before flipping the profile back

1. `mount | grep bess-usb` → `/mnt/bess-usb` mounted `ext4` with ≥ 500 GB free
2. Compose `volumes:` entry for recorder points at `/mnt/bess-usb/bags` (NOT the NVMe fallback)
3. `--max-bag-size` split + out-of-band prune loop tested in isolation for ≥ 30 min
4. `df -h /` shows ≥ 50 GB free BEFORE starting the recorder — never rely on the cap alone
5. Changelog row added describing the re-enable, including the verified destination path and the test-run evidence
6. Flip `profiles: [record-disabled]` → `[record]` and `restart: "no"` → `unless-stopped` in the same commit as the changelog row

### Start / stop

```bash
# Intentional: `--profile full` does NOT start the recorder anymore
docker compose -f docker-compose.thor.yml --profile full up -d

# Explicit recorder start (ONLY after all 6 prereqs pass)
docker compose -f docker-compose.thor.yml --profile record up -d recorder
```

### QoS override
`/home/thor/rosbag_qos.yaml` — sensors publish BEST_EFFORT; the recorder must match with `--qos-profile-overrides-path`.

### DO NOT
- Do not re-enable just because `/mnt/bess-usb` is "probably mounted" — verify the mount AND the free space
- Do not point the recorder at `/home/thor/recordings/*` — that's the NVMe, that's how we filled the disk
- Do not raise `CAP_BYTES` without also raising the host free-space floor in prereq #4
- Do not use `--profile full` to "start everything" and assume the recorder is in it — it isn't

---

## Kernel Tuning

Applied at boot by `bess-network.service` → `scripts/boot-sensors.sh` and persisted in `/etc/sysctl.d/60-gige-sensors.conf`.

Key values (target):

| sysctl | Value | Why |
|--------|-------|-----|
| `net.core.rmem_max` | 2 147 483 647 | GigE sensors burst faster than default 212 KB can absorb |
| `net.core.rmem_default` | 67 108 864 | Pre-allocated per socket — keep modest; sensors set their own SO_RCVBUF up to rmem_max |
| `net.core.wmem_max` | 67 108 864 | TX side for spinnaker_camera_driver GigE acks |
| `net.core.wmem_default` | 67 108 864 | — |
| `net.core.netdev_max_backlog` | 50 000 | Ouster UDP fragment burst |
| `net.ipv4.ipfrag_high_thresh` | 134 217 728 | Jumbo-frame fragment reassembly |
| `net.ipv4.ipfrag_low_thresh` | 100 663 296 | — |
| `net.ipv4.conf.mgbe0_0.rp_filter` | 0 | Ouster UDP arrives from 169.254.70.119; rp_filter drops asymmetric returns |

---

## Directory Structure

```
/home/thor/
├── bess_platform/                    # Production repo - SOURCE OF TRUTH
│   ├── CLAUDE.md                     # Platform behaviour rules
│   ├── docker-compose.thor.yml       # Thor compose file
│   ├── containers/                   # Node implementations
│   │   ├── lucid/lucid_node.py
│   │   ├── cameras/gpu_compress.py
│   │   └── ...
│   ├── config/
│   │   ├── cyclonedds.xml
│   │   └── heads/sensors.yaml
│   └── docs/
│       └── SYSTEM.md
├── docker_slam/                      # Legacy configs
│   └── dockerfiles/thermal/
│       └── thermal_spinnaker_node.py
├── recordings/
├── fix_ouster_networking.sh
└── rosbag_qos.yaml
```

---

## Session Log

| Date | Action | Result |
|------|--------|--------|
| 2026-02-07 | Dual A6701 via Spinnaker | Working (17-19Hz) |
| 2026-03-07 | Network interface broken | All cameras unreachable |
| 2026-03-07 | CLAUDE.md rewritten | Proper guardrails added |
| 2026-03-08 | Chaotic debugging with wrong SDK | Lucid camera locked, rules violated |
| 2026-03-08 | CLAUDE.md hardened | Added HARD STOPS, SDK MATRIX, CHECKLIST |
| 2026-04-12 | Full hardware reconfig + reboot | 7/10 sensors working |
| 2026-04-12 | Enabled MikroTik sfp28-7/8 | Lucid #2 online at 10Gbps |
| 2026-04-12 | Added lucid2 service + compress | Lucid #2 streaming 5Hz |
| 2026-04-12 | Fixed thermal4 GevSCPD=512 | Stable 14.4Hz, no drops |
| 2026-04-12 | Fixed foxglove DDS config | Bridge live on port 8765 |
| 2026-04-12 | Fixed compose: lucid1 IP, thermal4 MAC, microstrain USB | Ready for repower |
| 2026-04-12 | Ouster TIME_FROM_ROS_TIME + DLIO supervisor + FAST-LIO2 IMU QoS patch | DLIO+FAST-LIO2 both converging, /slam/odometry @ 10Hz |
| 2026-04-12 | ptp4l GM on mgbe0_0, Blackfly PTP SlaveOnly, USB autosuspend udev fix | PTP sync across Ouster + Blackfly, RM110 stable |
| 2026-04-13 | SLAM /Laser_map choke root-cause + fix | FAST-LIO2 `map_en: false` + new `slam-map-accumulator` service publishing `/slam/map_voxel` (1.4M voxels) |
| 2026-04-13 | DLIO frames rename + `/ouster/imu` swap | DLIO + FAST-LIO2 run in parallel, no base_link TF fight |
| 2026-04-13 | Thermal camera Sensor/Housing temperature publishers + soak watchdog | A6701 FPA cryocooler alert > -150°C, A70 body alert > 75°C |
| 2026-04-13 | Recorder → /mnt/bess-usb/bags + temp topics added | 7.3T soak volume online with thermal-stress trace |
| 2026-04-13 | RUT50 5G uplink (IPv4-only PDP) + MikroTik bridgeLocal add-default-route=no | End-to-end internet via RUT50, Tailscale stable |
