#!/bin/bash
# RM110 (HighPoint 1103:0110) cold-boot auto-recovery.
#
# Background: on ~2026-04-15 we observed that a cleanly connected RM110
# (cable untouched across reboot) failed to enumerate at all on cold boot —
# zero xhci/usb-storage events in dmesg, kernel host controller never saw
# an attach. Physical unplug+replug recovered it. Root cause is a Tegra
# xusb PHY training issue on cold boot; the kernel USB stack is fine.
#
# This script runs once after boot, checks whether the RM110 is present,
# and if not, rebinds the Tegra xhci platform device (a80aa10000.usb on
# driver tegra-xusb). The rebind forces a full controller re-probe which
# re-trains the SS PHY and picks up the device.
#
# Bus 2 (the Tegra SS root hub that carries the RM110) also carries one
# 4-port USB hub with no critical devices. Bus 1 (BT + HID keyboard) is
# a separate root hub and is NOT affected by the rebind.
#
# Rate limited by the calling systemd unit (bess-usb-recovery.service) —
# never more than 3 attempts per day.

set -u

VID_PID="1103:0110"
PLATDRV="/sys/bus/platform/drivers/tegra-xusb"
DEVNAME="a80aa10000.usb"
MAX_TRIES=3
LOG_PREFIX="[bess-usb-recovery]"
# Wait up to this many seconds after boot for the USB stack to settle before
# declaring the RM110 missing. Cold-boot SS PHY training can take 30-60 s on
# Tegra; we saw a successful recovery at T+3 min in one instance so a 45 s
# grace period is a reasonable floor before we intervene.
BOOT_GRACE_SECS=45

log() { echo "${LOG_PREFIX} $*"; }

# Two checks — BOTH must be true for "present":
#   1. USB device enumerated (visible to lsusb by VID:PID)
#   2. Block device exists (/dev/disk/by-id/*RM110*)
# The block-device check catches a secondary failure mode where the USB
# device enumerates but the uas/SCSI layer doesn't expose /dev/sdX.
# Note: udev symlinks for this device use prefix "HPT_" / "RM110" (not
# "HighPoint"), e.g. usb-HPT_RocketMaet_RM110_2219AFF000109-0:0.
rm110_present() {
    lsusb -d "${VID_PID}" 2>/dev/null | grep -q "${VID_PID}" || return 1
    # USB device visible — now confirm the block device too.
    local found=""
    for link in /dev/disk/by-id/usb-HPT_*RM110* /dev/disk/by-id/*RM110*; do
        [ -e "${link}" ] && found="${link}" && break
    done
    if [ -z "${found}" ]; then
        log "USB device enumerated but no /dev/disk/by-id/*RM110* block device"
        return 1
    fi
    return 0
}

rebind_xhci() {
    if [ ! -e "${PLATDRV}/${DEVNAME}" ]; then
        log "tegra-xusb not bound to ${DEVNAME} — nothing to rebind"
        return 1
    fi
    log "unbinding ${DEVNAME}"
    echo "${DEVNAME}" > "${PLATDRV}/unbind" 2>&1 || {
        log "unbind failed"
        return 1
    }
    sleep 2
    log "rebinding ${DEVNAME}"
    echo "${DEVNAME}" > "${PLATDRV}/bind" 2>&1 || {
        log "bind failed — bus 2 USB is now offline until next reboot"
        return 1
    }
    sleep 5
    return 0
}

main() {
    # Boot-time grace period: give the USB stack time to finish cold-boot
    # enumeration before we intervene. This also prevents the service from
    # firing and rebinding the controller WHILE the kernel is still probing
    # the device.
    local uptime_s
    uptime_s=$(awk '{print int($1)}' /proc/uptime 2>/dev/null || echo 0)
    if [ "${uptime_s}" -lt "${BOOT_GRACE_SECS}" ]; then
        local sleep_s=$((BOOT_GRACE_SECS - uptime_s))
        log "uptime ${uptime_s}s < ${BOOT_GRACE_SECS}s grace — waiting ${sleep_s}s"
        sleep "${sleep_s}"
    fi

    if rm110_present; then
        log "RM110 (${VID_PID}) present + block device OK — nothing to do"
        return 0
    fi

    log "RM110 (${VID_PID}) NOT fully present — will attempt tegra-xusb rebind"

    for try in $(seq 1 ${MAX_TRIES}); do
        log "attempt ${try}/${MAX_TRIES}"
        rebind_xhci || {
            log "rebind attempt ${try} failed"
            sleep $((try * 3))
            continue
        }
        if rm110_present; then
            log "RM110 recovered after ${try} rebind attempt(s)"
            return 0
        fi
        log "rebind completed but RM110 still missing"
        sleep $((try * 3))
    done

    log "FAILED: RM110 still missing after ${MAX_TRIES} attempts — manual re-seat required"
    return 1
}

main "$@"
