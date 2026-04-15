# Session 2026-04-15 — Reboot-loop root cause + fix

Written right before the hard-power-kill reboot test, so a future
session (or a future context compaction) can pick up without losing
hours of re-diagnosis.

---

## TL;DR

The 2026-04-14 rapid-reboot loop was wrongly diagnosed as a
systemd chain. It isn't. It is Docker's `restart: unless-stopped`
racing 27 containers at `docker.service` startup on a cold kernel,
wedging it long enough for the Tegra hardware watchdog
(`/dev/watchdog0`) to fire at ~15 s. The kill-switch on
`bess-stack.service` was irrelevant because Docker never asks
systemd for permission.

**Fixed** in commit `2f28fab` on `bess-thor` main:
- `x-common: &common` anchor: `restart: unless-stopped` → `restart: "no"`
  with inline comment explaining the failure mode
- `glim` service (line 774): same
- Recorder (line 1099): already `restart: "no"` from the
  2026-04-14 NVMe ENOSPC recovery — untouched
- Live `docker update --restart=no` applied to all 29 running
  containers (non-destructive, no restart needed)

Plus fstab hardened so a missing RM110 USB contributes **zero**
boot delay (not 10 s like before).

---

## Evidence that led to the diagnosis

1. `journalctl --list-boots` showed **20+ boot sessions** clustered
   in 16–20 s windows around `10:45:42`, `10:46:04`, and `11:00:01`,
   followed by user recovery (drop-to-shell-then-exit) before boot 0
   survived. Textbook Tegra watchdog pattern.
2. `systemctl is-active` and `systemctl is-enabled` on
   `bess-stack.service`, `bess-network.service`,
   `bess-time-bootstrap.service` all showed **inactive / disabled**
   with kill-switch files present. **They could not have caused the
   new loop.**
3. `journalctl -b -5` (5 boots ago, deep inside the loop) caught a
   Docker container that had just deactivated after consuming
   `7.327 s CPU time` in a ~14 s wall-clock window. Docker was
   bringing containers up during the kernel-cold phase.
4. `grep -c 'restart:' docker-compose.thor.yml` returned only 3
   explicit matches — but 30 containers were running, because the
   `x-common: &common` anchor at line 32 carries
   `restart: unless-stopped` which is inherited by 27 of 30 services
   via `<<: *common`.

## Boot-loop mechanism (corrected)

1. Kernel boots, Tegra `/dev/watchdog0` starts ticking (~15 s budget)
2. `docker.service` (`enabled`) starts
3. Docker engine reads container state from last shutdown
4. Docker restarts every `unless-stopped` container **simultaneously**
5. 27 containers compete for:
   - GPU / CUDA init (segformer, pii-mask, 6× gpu_compress)
   - USB device probes (RM110 absent → xHCI probe stalls)
   - GigE Vision SDK discovery (Spinnaker + Arena + Aravis on 10 cameras)
   - PTP clients hitting the fresh grandmaster
   - Host-network stack under heavy container-startup pressure
6. Combined kernel wedge exceeds watchdog budget → hw reboot
7. Boot state at next iteration is identical → same race → loop

## The 2026-04-14 kill-switch fix had the right *shape* but wrong *target*

Yesterday's commit `15e6f64` made `bess-stack/network/time-bootstrap`
independently kill-switch-gated, with `ConditionPathExists=!` and
`StartLimitIntervalSec=86400 StartLimitBurst=3`. That fix is still
correct defensively — if those units ever get re-enabled with a
chain, the rate limit will now structurally prevent a loop. But
they were never running this time; the loop came from Docker.

## Files touched in this session

| File | Change |
|------|--------|
| `docker-compose.thor.yml` | `&common` anchor: `restart: "no"` + comment. glim: same. (commit `2f28fab`) |
| `/etc/fstab` | `/mnt/bess-usb` line: `noauto,x-systemd.automount,x-systemd.device-timeout=1s,nofail` + comment. Backed up to `/etc/fstab.bak.2026-04-15` |
| `/home/thor/CLAUDE.md` | Second 2026-04-15 row appended with the corrected diagnosis |
| Live containers | `docker update --restart=no` on all 29 running containers |
| Live systemd | `systemctl start mnt-bess\x2dusb.automount` + daemon-reload |

## State at reboot moment (pre-hard-kill)

- All 30 containers `restart=no` (verified with `sort -u` → single value "no")
- All 3 kill-switches present in `/etc/bess/no-auto-{stack,network,time}`
- All 3 `bess-*` units disabled
- `docker.service`, `containerd.service`, `ptp4l-mgbe0.service`,
  `phc2sys-mgbe0.service` are enabled — these are fine, not the cause
- `/dev/sdb1` (RM110 BESS-USB, 7.3 T, 1.2 T used) mounted at
  `/mnt/bess-usb` via autofs
- `sudo sync && sudo sync` run before power kill
- `pd-mapper.service` remains `failed` — known-benign Qualcomm PD
  mapper error, present on every boot including successful ones

## Known residual issues to deal with post-reboot

### 1. 52 GB phantom data on NVMe at `/mnt/bess-usb/extraction`

Docker's `extraction` service has bind mounts
`/mnt/bess-usb/extraction:/data/extraction:rw` and
`/mnt/bess-usb/spatial:/data/spatial:ro` (compose lines 1179–1180).
While `/mnt/bess-usb` was unmounted, the `extraction` container's
bind mount wrote **52 GB of rolling-bag buffer test data** onto the
underlying root NVMe. Now that the real USB is mounted, those
52 GB are hidden behind the mount but still consuming `/dev/nvme0n1p1`.

**User confirmed: disposable test/soak data on both USB and NVMe,
safe to delete.** Cleanup procedure:

```bash
sudo systemctl stop 'mnt-bess\x2dusb.automount'   # unmounts the USB
sudo rm -rf /mnt/bess-usb/extraction /mnt/bess-usb/spatial   # removes phantoms on NVMe
sudo systemctl start 'mnt-bess\x2dusb.automount'  # re-activates automount
ls /mnt/bess-usb   # triggers re-mount of real USB
```

### 2. `extraction` container bind-mount fragility

Same mechanism will recur if the USB unmounts while the extraction
container runs. Two options for a durable fix (not applied yet):

- **(a)** Add `:ro,bind-propagation=rprivate` + a preflight check in
  the container command that refuses to start unless
  `mountpoint -q /data/extraction`
- **(b)** Change the compose bind mount to a named docker volume
  backed by `/mnt/bess-usb/extraction`, with the volume driver
  refusing to create if the mount is absent

### 3. Next-session desire: cleanly auto-start the stack at boot

Currently, after reboot the user has to manually run
`docker compose -f /home/thor/bess/docker-compose.thor.yml --profile full up -d`.
The clean fix is a new systemd unit that:

- `After=docker.service`
- `ExecStartPre=/bin/sleep 30` (grace window for kernel to settle)
- `ExecStart=docker compose -f … --profile full up -d`
- `StartLimitIntervalSec=86400 StartLimitBurst=3` (same rate-limit
  pattern as the kill-switch units)
- NO `Wants=`/`After=` chain to `bess-network` / `bess-time-bootstrap`
- Does NOT re-introduce `restart: unless-stopped` — relies on
  explicit `up -d` plus per-container session retry loops for
  crash recovery

### 4. RM110 USB link history

- 2026-04-12: autosuspend U3 link-death flap after reboot — fixed
  with `/etc/udev/rules.d/80-usb-storage-no-autosuspend.rules`
- 2026-04-15 (this session): at one point the RM110 blue LED was on
  but kernel saw **zero xhci enumeration events** — not a flap,
  host controller never registered attach. User then re-seated /
  re-plugged and it came up cleanly on `/dev/sdb`. Cause of the
  earlier non-enum event not fully root-caused — could have been
  cable seating, cold port, or a kernel residual from the prior
  boot loop
