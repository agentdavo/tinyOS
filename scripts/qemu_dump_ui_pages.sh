#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELF="$ROOT/build/arm64/miniOS_kernel_arm64.elf"
OUT_DIR="${1:-$ROOT/ui_captures}"

mkdir -p "$OUT_DIR"

if [[ ! -f "$ELF" ]]; then
    echo "ELF not found: $ELF — building TARGET=arm64" >&2
    make -C "$ROOT" TARGET=arm64 >/dev/null
fi

python3 - "$ELF" "$OUT_DIR" <<'PY'
import os
import re
import select
import signal
import subprocess
import sys
import time

ELF = sys.argv[1]
OUT_DIR = sys.argv[2]
PAGES = ("dashboard", "jog", "mdi", "program", "offsets", "service", "macros", "probe")
SCALE = 6
PROMPT = b"miniOS> "
BEGIN_RE = re.compile(rb"UI_DUMP_BEGIN\s+(\d+)\s+(\d+)\s+(\d+)\n")

os.makedirs(OUT_DIR, exist_ok=True)

def read_until(fd, needle: bytes, timeout: float, poke=None) -> bytes:
    buf = bytearray()
    deadline = time.monotonic() + timeout
    next_poke = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if poke and now >= next_poke:
            poke()
            next_poke = now + 0.5
        remaining = deadline - time.monotonic()
        r, _, _ = select.select([fd], [], [], max(0.0, remaining))
        if not r:
            continue
        chunk = os.read(fd, 65536)
        if not chunk:
            break
        buf.extend(chunk)
        if needle in buf:
            return bytes(buf)
    raise RuntimeError(f"timeout waiting for {needle!r}")

def read_dump(fd, timeout: float):
    buf = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        r, _, _ = select.select([fd], [], [], max(0.0, remaining))
        if not r:
            continue
        chunk = os.read(fd, 65536)
        if not chunk:
            break
        buf.extend(chunk)
        match = BEGIN_RE.search(buf)
        if not match:
            continue
        payload_size = int(match.group(3))
        payload_start = match.end()
        needed = payload_start + payload_size
        while len(buf) < needed and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            r, _, _ = select.select([fd], [], [], max(0.0, remaining))
            if not r:
                continue
            more = os.read(fd, 65536)
            if not more:
                break
            buf.extend(more)
        if len(buf) < needed:
            raise RuntimeError("short UI dump payload")
        payload = bytes(buf[payload_start:needed])
        tail = bytes(buf[needed:])
        return payload, tail
    raise RuntimeError("timeout waiting for UI_DUMP_BEGIN")

proc = subprocess.Popen(
    [
        "qemu-system-aarch64",
        "-M", "virt",
        "-cpu", "max",
        "-smp", "4",
        "-m", "128M",
        "-global", "virtio-mmio.force-legacy=false",
        "-netdev", "socket,id=n0,listen=:20001",
        "-device", "virtio-net-device,netdev=n0",
        "-netdev", "socket,id=n1,connect=127.0.0.1:20001",
        "-device", "virtio-net-device,netdev=n1",
        "-netdev", "user,id=n2",
        "-device", "virtio-net-device,netdev=n2",
        "-device", "virtio-gpu-device",
        "-device", "virtio-keyboard-device",
        "-device", "virtio-tablet-device",
        "-nographic",
        "-kernel", ELF,
    ],
    stdin=subprocess.PIPE,
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    bufsize=0,
    start_new_session=True,
)

try:
    fd = proc.stdout.fileno()
    def poke_prompt():
        proc.stdin.write(b"\r")
        proc.stdin.flush()

    read_until(fd, PROMPT, 90.0, poke=poke_prompt)

    for page in PAGES:
        proc.stdin.write(f"ui_page {page}\r".encode("ascii"))
        proc.stdin.flush()
        read_until(fd, PROMPT, 10.0)

        proc.stdin.write(f"ui_dump {SCALE}\r".encode("ascii"))
        proc.stdin.flush()
        payload, tail = read_dump(fd, 20.0)

        ppm_path = os.path.join(OUT_DIR, f"{page}.ppm")
        png_path = os.path.join(OUT_DIR, f"{page}.png")
        with open(ppm_path, "wb") as f:
            f.write(payload)

        subprocess.run(
            [
                "ffmpeg", "-y", "-loglevel", "error",
                "-i", ppm_path,
                "-frames:v", "1",
                png_path,
            ],
            check=True,
        )

        if PROMPT not in tail:
            read_until(fd, PROMPT, 10.0)

    print(f"UI page dumps written to {OUT_DIR}")
finally:
    try:
        if proc.stdin:
            proc.stdin.close()
    except Exception:
        pass
    try:
        os.killpg(proc.pid, signal.SIGTERM)
    except Exception:
        pass
    try:
        proc.wait(timeout=5)
    except Exception:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            pass
PY
