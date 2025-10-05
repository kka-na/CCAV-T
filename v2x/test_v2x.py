#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse
import socket
import struct
import sys
import time
import os
import select

MAGIC = b"5GVX"
PORT = 47347

# eV2x_App_Payload_Id (subset)
PID_TX      = 0x10
PID_RX      = 0x11
PID_WSR     = 0x12   # WsmServiceReq
PID_WSC     = 0x13   # WsmServiceConfirm

# PSIDs
PSID_V2V    = 58200
PSID_V2I    = 58201

# TLVC types
EM_PT_OVERALL = 58220
EM_PT_RAW     = 58221
EM_PT_STATUS  = 58223


def crc16_ccitt_table_init():
    table = []
    poly = 0x1021
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if (crc & 0x8000) != 0:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


_CRC_TABLE = crc16_ccitt_table_init()


def crc16_ccitt(data: bytes, init=0x0000) -> int:
    """Matches CalcCRC16 in provided C code: poly=0x1021, init=0x0000, no reflect."""
    crc = init & 0xFFFF
    for b in data:
        idx = ((crc >> 8) ^ b) & 0xFF
        crc = ((_CRC_TABLE[idx] ^ ((crc << 8) & 0xFFFF)) & 0xFFFF)
    return crc & 0xFFFF


def build_frame(payload_id: int, payload: bytes, seq: int = 0) -> bytes:
    """
    Frame: | MAGIC(4) | LEN(2) | SEQ(2) | PID(2) | PAYLOAD(n) | CRC16(2) |
    LEN counts from SEQ to CRC inclusive.
    CRC is over bytes after MAGIC up to but not including CRC.
    """
    header_wo_magic = struct.pack("!H", 0) + struct.pack("!H", seq) + struct.pack("!H", payload_id)
    body = header_wo_magic + payload
    length = 2 + 2 + len(payload) + 2
    body = struct.pack("!H", length) + body[2:]
    crc = crc16_ccitt(body)
    frame = MAGIC + body + struct.pack("!H", crc)
    return frame


def build_wsr(action_add: bool, psid: int) -> bytes:
    inner = struct.pack("!B", 0 if action_add else 1) + struct.pack("!I", psid)
    inner_crc = crc16_ccitt(inner)
    payload = inner + struct.pack("!H", inner_crc)
    return build_frame(PID_WSR, payload)


def build_ext_v2v_with_seq(sequence: int) -> bytes:
    overall = struct.pack("!IH4sBBH", EM_PT_OVERALL, (8+1+1+2+2), b'EMOP', 1, 0, 0)
    raw_v = struct.pack("!I", sequence)
    raw = struct.pack("!IH", EM_PT_RAW, len(raw_v)+2) + raw_v
    raw_crc = crc16_ccitt(raw)
    raw += struct.pack("!H", raw_crc)

    status_wo_crc = struct.pack("!IHBBIHHQ", EM_PT_STATUS, (1+1+4+2+2+8+2), 11, 0, 1, 2, 3, 0)
    status_crc = crc16_ccitt(status_wo_crc)
    status = status_wo_crc + struct.pack("!H", status_crc)

    num_pkg = 2
    len_pkg = len(raw) + len(status)
    overall_len = (4+2+4+1+1+2+2)
    overall = struct.pack("!IH4sBBH", EM_PT_OVERALL, overall_len-6, b'EMOP', 1, num_pkg, len_pkg)
    overall_crc = crc16_ccitt(overall)
    overall += struct.pack("!H", overall_crc)

    payload_inner = overall + raw + status
    payload = struct.pack("!I", PSID_V2V) + payload_inner
    return build_frame(PID_TX, payload)


def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)


def recv_frames(sock, timeout=2.0):
    sock.setblocking(0)
    end = time.time() + timeout
    buf = b""
    out = []
    while time.time() < end:
        r, _, _ = select.select([sock], [], [], 0.1)
        if not r:
            continue
        chunk = sock.recv(65535)
        if not chunk:
            break
        buf += chunk
        while True:
            m = buf.find(MAGIC)
            if m < 0:
                buf = buf[-3:]
                break
            if len(buf) < m + 6:
                break
            start = m + 4
            length = struct.unpack("!H", buf[start:start+2])[0]
            total = 4 + 2 + (length)
            if len(buf) < m + total:
                break
            frame = buf[m:m+total]
            out.append(frame)
            buf = buf[m+total:]
    return out


def parse_frame(frame: bytes):
    if not frame.startswith(MAGIC):
        return {"ok": False, "reason": "magic"}
    if len(frame) < 4+2+2+2+2:
        return {"ok": False, "reason": "short"}
    body = frame[4:-2]
    crc_expect = struct.unpack("!H", frame[-2:])[0]
    crc_calc = crc16_ccitt(body)
    ok_crc = (crc_calc == crc_expect)
    length, seq, pid = struct.unpack("!HHH", body[:6])
    info = {"ok": ok_crc, "len": length, "seq": seq, "pid": pid, "crc": hex(crc_expect)}
    p = body[6:]
    if pid == PID_WSC:
        if len(p) >= 7:
            action_result = p[0]
            psid = struct.unpack("!I", p[1:5])[0]
            info.update({"type": "WSC", "action_result": action_result, "psid": psid})
    elif pid == PID_RX:
        if len(p) >= 5:
            psid = struct.unpack("!I", p[:4])[0]
            rcpi = p[4]
            info.update({"type": "RX", "psid": psid, "rcpi": rcpi, "data_len": len(p)-5})
    elif pid == PID_TX:
        if len(p) >= 4:
            psid = struct.unpack("!I", p[:4])[0]
            info.update({"type": "TX-echo?", "psid": psid, "payload_len": len(p)-4})
    elif pid == PID_WSR:
        if len(p) >= 7:
            action = p[0]
            psid = struct.unpack("!I", p[1:5])[0]
            info.update({"type": "WSR", "action": action, "psid": psid})
    else:
        info.update({"type": "UNKNOWN"})
    return info


def maybe_bind_iface(sock, iface: str):
    if not iface:
        return
    SO_BINDTODEVICE = 25
    try:
        sock.setsockopt(socket.SOL_SOCKET, SO_BINDTODEVICE, iface.encode('ascii') + b'\x00')
    except PermissionError:
        print("[warn] SO_BINDTODEVICE requires root. Continuing without binding.", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser(description="Minimal V2X tester for 5GVX framing")
    ap.add_argument("ip", help="OBU IP, e.g., 192.168.1.124")
    ap.add_argument("--iface", help="bind to interface (e.g., enp4s0), needs root", default=None)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p1 = sub.add_parser("wsr", help="Send WSM Service Request and wait for confirm")
    p1.add_argument("--add", action="store_true", help="ADD service")
    p1.add_argument("--delete", action="store_true", help="DEL service")
    p1.add_argument("--psid", type=int, default=PSID_V2V, help="PSID")

    p2 = sub.add_parser("v2vseq", help="Send Extensible V2V with sequence and wait RX")
    p2.add_argument("--count", type=int, default=1)
    p2.add_argument("--period_ms", type=int, default=100)

    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    maybe_bind_iface(sock, args.iface)
    sock.settimeout(5.0)
    sock.connect((args.ip, PORT))
    sock.settimeout(None)
    print(f"[ok] connected to {args.ip}:{PORT}")

    if args.cmd == "wsr":
        if args.add == args.delete:
            print("Specify exactly one of --add or --del", file=sys.stderr)
            sys.exit(2)
        frame = build_wsr(args.add, args.psid)
        print(f"[tx] WSR {'ADD' if args.add else 'DEL'} psid={args.psid} len={len(frame)}")
        print(hexdump(frame))
        sock.sendall(frame)
        frames = recv_frames(sock, timeout=2.0)
        if not frames:
            print("[rx] no frames within 2 s")
        for fr in frames:
            info = parse_frame(fr)
            print(f"[rx] {info}")
    elif args.cmd == "v2vseq":
        for i in range(args.count):
            frame = build_ext_v2v_with_seq(i+1)
            print(f"[tx] V2V SEQ={i+1} len={len(frame)}")
            sock.sendall(frame)
            time.sleep(args.period_ms/1000.0)
        frames = recv_frames(sock, timeout=2.0)
        if not frames:
            print("[rx] no frames within 2 s")
        for fr in frames:
            info = parse_frame(fr)
            print(f"[rx] {info}")

    sock.close()


if __name__ == "__main__":
    main()
