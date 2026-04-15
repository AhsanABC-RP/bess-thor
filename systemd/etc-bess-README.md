# /etc/bess/ — BESS Thor boot-time safety kill-switches

After the 2026-04-14 boot-loop incident (see `/home/thor/CLAUDE.md`
changelog) three systemd units that used to auto-start the sensor stack at
boot were rewired to be individually gated behind a kill-switch file. The
files in `/etc/bess/` control whether each service runs on next boot.

- **ARMED** (file present) = service is a NO-OP at boot. System comes up
  clean, no containers start, no network bootstrap runs, no wall clock step.
- **DISARMED** (file absent) = service runs on next boot.

**Default state: ALL THREE ARMED.** You must explicitly disarm each one.

## Files

### `/etc/bess/no-auto-stack`
Gates `bess-stack.service` (`docker compose --profile full up -d`).
When armed: docker compose does not auto-start. You still get `ptp4l`,
`phc2sys`, and `docker.service`. Run the stack by hand once you have
verified the box is healthy:
```bash
cd /home/thor/bess
docker compose -f docker-compose.thor.yml --profile full up -d
```

### `/etc/bess/no-auto-network`
Gates `bess-network.service` (`scripts/boot-sensors.sh`).
When armed: the sensor-net bootstrap (IP/MTU/sysctls/dnsmasq, MikroTik probe,
link-bounce on `mgbe0_0`) does not run at boot. Interfaces come up via
NetworkManager only. Run the script by hand after boot with:
```bash
sudo /home/thor/bess/scripts/boot-sensors.sh
```

### `/etc/bess/no-auto-time`
Gates `bess-time-bootstrap.service` (`/usr/local/sbin/bess-time-bootstrap`).
When armed: no boot-time HTTP-Date wall clock step. Wall clock is whatever
the BMC RTC holds, `ptp4l`/`phc2sys` anchor on that. Run the script by hand
BEFORE starting PTP if you know the RTC is stale:
```bash
sudo /usr/local/sbin/bess-time-bootstrap --check    # dry run first
sudo /usr/local/sbin/bess-time-bootstrap
```

## Enabling auto-start (safe procedure)

Do ONE service at a time. Reboot and verify between each one.

```bash
# 1. Disarm one kill-switch
sudo rm /etc/bess/no-auto-stack

# 2. Enable the unit (writes the WantedBy= symlink)
sudo systemctl enable bess-stack.service

# 3. Test without rebooting first
sudo systemctl start bess-stack.service
systemctl status bess-stack.service
docker ps

# 4. If healthy, reboot to confirm the boot path works
sudo reboot

# 5. After the reboot, verify
systemctl status bess-stack.service
docker ps
```

## Emergency re-disarm (from a live session)

```bash
sudo touch /etc/bess/no-auto-stack
sudo systemctl stop bess-stack.service
sudo systemctl disable bess-stack.service
```

The touch-file is the structural guarantee: even if the service is still
`enabled`, the next boot is a no-op because systemd evaluates
`ConditionPathExists=` before Execing anything inside the service.

## Why not `ExecCondition=`?

systemd `ExecCondition=` runs INSIDE the service transaction and can itself
wedge. `ConditionPathExists=!` is evaluated by pid 1 OUTSIDE the service and
cannot fail open. That matters during a boot loop — you want the exit path
to be as close to "systemd skips the unit" as possible.

## Rate limits

Each unit has `StartLimitIntervalSec=86400` + `StartLimitBurst=3`, meaning
after 3 failures in 24 hours systemd gives up and the unit stays in
`failed` state. Combined with the touch-file kill-switches, this makes
Tegra-watchdog boot loops structurally impossible.

## Install paths

The canonical source-of-truth copies of each unit file live in this repo
under `systemd/`:

| Repo file | Installed at |
|-----------|-------------|
| `systemd/bess-stack.service` | `/etc/systemd/system/bess-stack.service` |
| `systemd/bess-network.service` | `/etc/systemd/system/bess-network.service` |
| `systemd/bess-time-bootstrap.service` | `/etc/systemd/system/bess-time-bootstrap.service` |
| `systemd/etc-bess-README.md` | `/etc/bess/README` |

To redeploy after a repo update:
```bash
sudo cp systemd/bess-stack.service /etc/systemd/system/
sudo cp systemd/bess-network.service /etc/systemd/system/
sudo cp systemd/bess-time-bootstrap.service /etc/systemd/system/
sudo cp systemd/etc-bess-README.md /etc/bess/README
sudo systemctl daemon-reload
```
