# BESS Thor systemd units — sequenced-launcher doctrine (2026-04-15)

As of 2026-04-15 (post second boot-loop recovery pass), the three BESS Thor
boot-time units are plain oneshots — **no kill-switch files**, **no
`ConditionPathExists=!` gates**. The structural fix for the 2026-04-14/15
boot-loop incident was to replace `docker compose --profile full up -d`
(which race-started ~30 containers in parallel) with
`/home/thor/bess/scripts/bess-stack-up.sh`, a sequenced launcher that brings
containers up in 10 batches of ≤4 at a time. No more parallel stampede →
no more Tegra watchdog wedge → no more boot loop.

The old kill-switch machinery (`no-auto-stack` / `no-auto-network` /
`no-auto-time` files under `/etc/bess/`) is gone. If you grep for
`ConditionPathExists=!/etc/bess/` you should find nothing.

## The units

| Unit | Function | Ordering |
|------|----------|----------|
| `bess-network.service` | Runs `scripts/boot-sensors.sh`: 169.254.100.1/16 on `mgbe0_0`, kernel sysctls, dnsmasq with camera MAC reservations, MikroTik probe, default route via RUT50 bridgeWAN | `After=network-online.target NetworkManager.service` |
| `bess-time-bootstrap.service` | Cold-boot HTTP-Date quorum fetch to seed `CLOCK_REALTIME` when BMC RTC is stale. **Independent of PTP chain** — `phc2sys -S 0` will not step PHC1 even if this unit sets a weird wall clock, so a failure here cannot cascade into sensor downtime | `After=network-online.target` |
| `bess-stack.service` | Runs `scripts/bess-stack-up.sh`. `ExecStartPre=/bin/sleep 30` grace before touching docker so SSH/Tailscale come up in time for a human to intervene remotely | `After=docker.service bess-network.service network-online.target` |
| `bess-usb-recovery.service` | RM110 cold-boot non-enum auto-recovery. Rebinds `tegra-xusb` platform device if `1103:0110` or `/dev/disk/by-id/*RM110*` is missing at T+45 s. `Restart=on-failure RestartSec=300` so a later physical re-seat is picked up on the next 5 min tick | `After=multi-user.target` |

## Safety nets that remain

1. **Sequenced launcher** (primary defense): max 4 containers starting at any
   instant, camera SDKs never co-init, GPU-touching containers staggered
   ≥5 s. Total ~125 s for a cold-boot startup.

2. **Rate limit** (secondary defense): each unit has
   `StartLimitIntervalSec=86400` + `StartLimitBurst=3`. After 3 failures in
   24 h the unit stays in `failed` and systemd gives up. Even if the
   sequenced launcher itself develops a fatal regression, systemd will not
   put the box into a boot loop.

3. **30-second grace in `bess-stack.service`**: `ExecStartPre=/bin/sleep 30`.
   If you boot into a broken stack via SSH during this window, you can
   `systemctl stop bess-stack.service` + `systemctl disable bess-stack`
   before any container touches GPU/PTP/USB.

## Enabling auto-start (safe procedure)

Do ONE service at a time. Test without rebooting first, then reboot to
confirm the boot path works.

```bash
# Test network bootstrap by hand first
sudo systemctl start bess-network.service
journalctl -u bess-network.service -b

# Test the sequenced launcher by hand (docker must be up; live stack is OK, idempotent)
sudo systemctl start bess-stack.service
docker ps --format 'table {{.Names}}\t{{.Status}}'
journalctl -u bess-stack.service -b

# Only after both above pass: enable on boot
sudo systemctl enable bess-network.service bess-stack.service
# time-bootstrap is optional — only enable if BMC RTC is unreliable
# sudo systemctl enable bess-time-bootstrap.service

# Then reboot to confirm the boot path works end-to-end
sudo reboot
```

## If the box boots into a broken stack

SSH in during the 30 s grace window (before containers start):

```bash
sudo systemctl stop bess-stack.service
sudo systemctl disable bess-stack.service
```

If you missed the window and the stack is running but wedged:

```bash
cd /home/thor/bess
docker compose -f docker-compose.thor.yml --profile full down
sudo systemctl disable bess-stack.service
```

Then diagnose in peace.

## Install paths

The canonical source-of-truth copies of each unit file live in this repo
under `systemd/`:

| Repo file | Installed at |
|-----------|-------------|
| `systemd/bess-stack.service` | `/etc/systemd/system/bess-stack.service` |
| `systemd/bess-network.service` | `/etc/systemd/system/bess-network.service` |
| `systemd/bess-time-bootstrap.service` | `/etc/systemd/system/bess-time-bootstrap.service` |
| `systemd/bess-usb-recovery.service` | `/etc/systemd/system/bess-usb-recovery.service` |
| `systemd/etc-bess-README.md` | `/etc/bess/README` |

To redeploy after a repo update:

```bash
sudo cp systemd/bess-stack.service /etc/systemd/system/
sudo cp systemd/bess-network.service /etc/systemd/system/
sudo cp systemd/bess-time-bootstrap.service /etc/systemd/system/
sudo cp systemd/bess-usb-recovery.service /etc/systemd/system/
sudo cp systemd/etc-bess-README.md /etc/bess/README
sudo systemctl daemon-reload
```
