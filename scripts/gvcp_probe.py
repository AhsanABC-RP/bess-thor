#!/usr/bin/env python3
"""
GVCP (GigE Vision Control Protocol) probe — raw UDP, no camera SDK.

Modes:
  discover                — broadcast DISCOVERY_CMD, print replies
  forceip <mac> <ip> [mask] [gw]  — broadcast FORCEIP_CMD, wait for ACK

Source binding: 169.254.100.1 (Thor mgbe0_0 sensor subnet)
Dest:           255.255.255.255:3956 for cmds, replies come back direct.
"""
import argparse
import socket
import struct
import sys
import time

GVCP_PORT = 3956
SRC_IP = "169.254.100.1"

KEY_CODE = 0x42
FLAG_ACK_REQ = 0x01

CMD_DISCOVERY_CMD = 0x0002
CMD_DISCOVERY_ACK = 0x0003
CMD_FORCEIP_CMD = 0x0004
CMD_FORCEIP_ACK = 0x0005


def mk_header(command, payload_len, req_id):
    return struct.pack(">BBHHH", KEY_CODE, FLAG_ACK_REQ, command, payload_len, req_id)


def parse_mac(mac_str):
    parts = mac_str.split(":")
    if len(parts) != 6:
        raise ValueError(f"bad MAC: {mac_str}")
    return bytes(int(p, 16) for p in parts)


def ip_to_bytes(ip_str):
    return socket.inet_aton(ip_str)


def mac_to_str(mac_bytes):
    return ":".join(f"{b:02x}" for b in mac_bytes)


def open_socket():
    """Single UDP socket bound to SRC_IP:ephemeral. GVCP ACKs come back to
    the ephemeral source port we sent from (not to :3956 fixed)."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((SRC_IP, 0))
    s.settimeout(0.2)
    return s


def discover(timeout=5.0):
    s = open_socket()
    req_id = 0xAA11
    hdr = mk_header(CMD_DISCOVERY_CMD, 0, req_id)
    s.sendto(hdr, ("255.255.255.255", GVCP_PORT))
    print(f"[tx] DISCOVERY_CMD req_id=0x{req_id:04x}", flush=True)

    replies = {}
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            pkt, src = s.recvfrom(4096)
        except socket.timeout:
            continue
        if len(pkt) < 8:
            continue
        key, flag, cmd, plen, rid = struct.unpack(">BBHHH", pkt[:8])
        if cmd != CMD_DISCOVERY_ACK:
            continue
        # GVCP DISCOVERY_ACK payload: 248 bytes per spec
        # MAC: offset 10-15 (inside payload: 2 pad + 2 mac_high + 4 mac_low — but
        # the discovery_ack body layout in spin-tcpdump is:
        #   0..1  version
        #   2..3  device mode
        #   4..5  reserved
        #   6..7  mac high (big-endian u16)
        #   8..11 mac low (big-endian u32)
        #   12..15 reserved
        #   16..19 supported ip config
        #   20..23 current ip config
        #   24..35 reserved
        #   36..39 current ip
        #   40..51 reserved
        #   52..55 current subnet
        #   56..67 reserved
        #   68..71 current gateway
        #   72..103 manufacturer name (ASCII)
        #   104..135 model name
        #   ... etc
        if plen < 72:
            continue
        body = pkt[8:]
        # GVCP DISCOVERY_ACK body: MAC at offset 10..15 (6 bytes contiguous),
        # current IP at offset 36..39.
        mac_bytes = body[10:16]
        mac = mac_to_str(mac_bytes)
        cur_ip = socket.inet_ntoa(body[36:40])
        try:
            mfg = body[72:104].rstrip(b"\x00").decode("ascii", errors="replace")
            model = body[104:136].rstrip(b"\x00").decode("ascii", errors="replace")
        except Exception:
            mfg, model = "?", "?"
        replies[mac] = (src[0], cur_ip, mfg, model)

    s.close()
    print(f"[rx] {len(replies)} replies", flush=True)
    for mac, (src_ip, ip, mfg, model) in sorted(replies.items()):
        print(f"  mac={mac}  ip={ip:<15}  src={src_ip:<15}  {mfg} {model}", flush=True)
    return replies


def forceip(target_mac_str, new_ip_str, mask_str="255.255.0.0", gw_str="0.0.0.0",
            timeout=3.0):
    mac = parse_mac(target_mac_str)
    new_ip = ip_to_bytes(new_ip_str)
    mask = ip_to_bytes(mask_str)
    gw = ip_to_bytes(gw_str)

    # FORCEIP_CMD payload (68 bytes):
    #   reserved (2)  mac_high (2)  mac_low (4)  reserved (12)
    #   static_ip (16, last 4 = ip)  mask (16, last 4)  gw (16, last 4)
    payload = b""
    payload += b"\x00\x00"               # reserved
    payload += mac[0:2]                  # mac high
    payload += mac[2:6]                  # mac low
    payload += b"\x00" * 12              # reserved
    payload += b"\x00" * 12 + new_ip     # static ip
    payload += b"\x00" * 12 + mask       # mask
    payload += b"\x00" * 12 + gw         # gw
    assert len(payload) == 68, len(payload)

    req_id = 0xAA22
    hdr = mk_header(CMD_FORCEIP_CMD, len(payload), req_id)

    s = open_socket()
    local_ip, local_port = s.getsockname()
    pkt = hdr + payload
    print(f"[dbg] local_port={local_port}", flush=True)
    print(f"[dbg] hex: {pkt.hex()}", flush=True)
    s.sendto(pkt, ("255.255.255.255", GVCP_PORT))
    print(f"[tx] FORCEIP_CMD target={target_mac_str} -> {new_ip_str}/{mask_str} gw={gw_str} req_id=0x{req_id:04x}",
          flush=True)

    deadline = time.monotonic() + timeout
    ack = None
    while time.monotonic() < deadline:
        try:
            pkt, src = s.recvfrom(4096)
        except socket.timeout:
            continue
        if len(pkt) < 8:
            continue
        key, flag, cmd, plen, rid = struct.unpack(">BBHHH", pkt[:8])
        if cmd == CMD_FORCEIP_ACK and rid == req_id:
            ack = src
            print(f"[rx] FORCEIP_ACK from {src[0]}", flush=True)
            break

    s.close()
    if ack is None:
        print("[rx] NO FORCEIP_ACK within timeout — camera GVCP stack did not reply",
              flush=True)
        return 2
    return 0


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="mode", required=True)
    sub.add_parser("discover")
    fp = sub.add_parser("forceip")
    fp.add_argument("mac")
    fp.add_argument("ip")
    fp.add_argument("--mask", default="255.255.0.0")
    fp.add_argument("--gw", default="0.0.0.0")
    args = ap.parse_args()

    if args.mode == "discover":
        r = discover()
        return 0 if r else 3
    elif args.mode == "forceip":
        return forceip(args.mac, args.ip, args.mask, args.gw)


if __name__ == "__main__":
    sys.exit(main())
