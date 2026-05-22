#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
Live-preview host bridge (legacy / fallback path).

The kernel now runs a WebSocket server itself (PR #48), so the editor
connects to ws://<kernel>:5001/ directly in the default config. This
bridge is no longer required for normal use.

Keep it for two cases:
  * The kernel is reachable only via a UDP path (e.g. a flat L2 segment
    where TCP is filtered).
  * You're debugging the chunked-UDP TSV upload protocol itself.

How it worked:
  Editor connects via WebSocket; bridge chunks the text frame into
  1400-byte UDP datagrams keyed by a session id and pushes them at the
  kernel's UDP listener on port 5002. The kernel reassembles + calls
  ui_builder::load_tsv.

Usage (only when you actually need the bridge):
  python3 tools/ui_editor/live_preview.py --kernel-ip 10.0.2.15
  # then in the editor:
  #   localStorage.setItem("livePreviewUrl", "ws://localhost:5001/")
  # so the editor talks to this bridge rather than the kernel directly.

Dependencies: stdlib only.
"""

import argparse
import base64
import hashlib
import os
import random
import socket
import struct
import sys
import threading
from contextlib import closing

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
KERNEL_UDP_PORT = 5002
TSV_UPLOAD_MAGIC = 0x55565354  # 'TSVU' as LE uint32

# Layout — must match hmi_service.cpp TsvUploadHeader. 20 bytes.
HEADER_FMT = "<I H H I H H I"
HEADER_SIZE = struct.calcsize(HEADER_FMT)
assert HEADER_SIZE == 20, f"header size {HEADER_SIZE} != 20"

# Max payload per UDP datagram on a typical Ethernet path: 1500 MTU - 20
# IPv4 - 8 UDP = 1472. Subtract our 20-byte header. Leave a few bytes of
# slack for any path-MTU surprises.
CHUNK_SIZE = 1400


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    p.add_argument("--kernel-ip", required=True,
                   help="IP of the QEMU / hardware kernel running hmi_service")
    p.add_argument("--kernel-port", type=int, default=KERNEL_UDP_PORT,
                   help=f"kernel UDP listener port (default: {KERNEL_UDP_PORT})")
    p.add_argument("--port", type=int, default=5001,
                   help="WebSocket listen port (default: 5001)")
    p.add_argument("--bind", default="127.0.0.1",
                   help="WebSocket bind address (default: 127.0.0.1)")
    p.add_argument("--verbose", "-v", action="store_true")
    return p.parse_args()


def chunk_tsv(tsv_bytes: bytes, session: int):
    """Yield (header + payload) datagrams covering the whole TSV."""
    total = len(tsv_bytes)
    offset = 0
    while offset < total:
        clen = min(CHUNK_SIZE, total - offset)
        header = struct.pack(HEADER_FMT,
                             TSV_UPLOAD_MAGIC, session, 0,
                             offset, clen, 0, total)
        yield header + tsv_bytes[offset:offset + clen]
        offset += clen


def push_to_kernel(tsv_bytes: bytes, dst_ip: str, dst_port: int, verbose: bool):
    """Send the TSV in order to the kernel. Returns (chunks_sent, total)."""
    session = random.randint(1, 0xFFFE)
    sent = 0
    with closing(socket.socket(socket.AF_INET, socket.SOCK_DGRAM)) as s:
        for datagram in chunk_tsv(tsv_bytes, session):
            s.sendto(datagram, (dst_ip, dst_port))
            sent += 1
    if verbose:
        print(f"  session={session:#06x} chunks={sent} bytes={len(tsv_bytes)}",
              file=sys.stderr)
    return sent, len(tsv_bytes)


# ----- minimal WebSocket server (RFC 6455 fragment-friendly subset) -----

def ws_accept_key(client_key: str) -> str:
    h = hashlib.sha1((client_key + WS_GUID).encode("ascii")).digest()
    return base64.b64encode(h).decode("ascii")


def ws_handshake(conn: socket.socket) -> bool:
    """Read HTTP upgrade request, respond with 101 if valid."""
    data = b""
    while b"\r\n\r\n" not in data:
        chunk = conn.recv(2048)
        if not chunk:
            return False
        data += chunk
        if len(data) > 8192:
            return False
    header_text = data.split(b"\r\n\r\n", 1)[0].decode("latin-1", errors="replace")
    headers = {}
    for line in header_text.splitlines()[1:]:
        if ":" in line:
            k, _, v = line.partition(":")
            headers[k.strip().lower()] = v.strip()
    if "websocket" not in headers.get("upgrade", "").lower():
        conn.sendall(b"HTTP/1.1 400 Bad Request\r\n\r\n")
        return False
    accept = ws_accept_key(headers["sec-websocket-key"])
    conn.sendall(
        b"HTTP/1.1 101 Switching Protocols\r\n"
        b"Upgrade: websocket\r\n"
        b"Connection: Upgrade\r\n"
        b"Sec-WebSocket-Accept: " + accept.encode("ascii") + b"\r\n\r\n"
    )
    return True


def ws_read_frame(conn: socket.socket) -> bytes | None:
    """Read one (possibly fragmented) text frame. Returns the assembled
    payload or None on close. Only handles text (opcode 1) and
    continuation (opcode 0); ping / pong / binary are not used by the
    editor and dropped."""
    payload = bytearray()
    while True:
        b1b2 = conn.recv(2)
        if len(b1b2) < 2:
            return None
        b1, b2 = b1b2[0], b1b2[1]
        fin = b1 & 0x80
        opcode = b1 & 0x0F
        masked = b2 & 0x80
        length = b2 & 0x7F
        if length == 126:
            length = struct.unpack(">H", conn.recv(2))[0]
        elif length == 127:
            length = struct.unpack(">Q", conn.recv(8))[0]
        if masked:
            mask = conn.recv(4)
        else:
            mask = None
        # Read the full payload
        remaining = length
        data = bytearray()
        while remaining > 0:
            chunk = conn.recv(min(65536, remaining))
            if not chunk:
                return None
            data += chunk
            remaining -= len(chunk)
        if mask:
            data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
        if opcode == 0x8:  # close
            return None
        if opcode in (0x1, 0x0):  # text / continuation
            payload += data
        if fin:
            return bytes(payload)


def ws_send_text(conn: socket.socket, text: str) -> None:
    """Send a short text frame (no masking on the server side per RFC)."""
    data = text.encode("utf-8")
    n = len(data)
    if n < 126:
        header = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        header = struct.pack("!BBH", 0x81, 126, n)
    else:
        header = struct.pack("!BBQ", 0x81, 127, n)
    conn.sendall(header + data)


def handle_client(conn: socket.socket, addr, dst_ip: str, dst_port: int,
                  verbose: bool) -> None:
    with closing(conn):
        if not ws_handshake(conn):
            return
        if verbose:
            print(f"+ client {addr}", file=sys.stderr)
        while True:
            frame = ws_read_frame(conn)
            if frame is None:
                break
            try:
                # Editor sends TSV as a UTF-8 text frame. Forward as bytes.
                sent, total = push_to_kernel(frame, dst_ip, dst_port, verbose)
                ws_send_text(conn, f"OK chunks={sent} bytes={total}")
            except OSError as e:
                ws_send_text(conn, f"FAIL {e}")
        if verbose:
            print(f"- client {addr}", file=sys.stderr)


def main() -> int:
    args = parse_args()
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    sock.listen(4)
    print(
        f"live-preview bridge: ws://{args.bind}:{args.port}/ -> "
        f"kernel udp {args.kernel_ip}:{args.kernel_port}",
        file=sys.stderr,
    )
    try:
        while True:
            conn, addr = sock.accept()
            t = threading.Thread(
                target=handle_client,
                args=(conn, addr, args.kernel_ip, args.kernel_port, args.verbose),
                daemon=True,
            )
            t.start()
    except KeyboardInterrupt:
        print("\nshutdown", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
