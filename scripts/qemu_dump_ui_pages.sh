#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ELF="$ROOT/build/arm64/miniOS_kernel_arm64.elf"
OUT_DIR="${1:-$ROOT/ui_captures}"
UI_TSV="$ROOT/devices/embedded_ui.tsv"

mkdir -p "$OUT_DIR"

if [[ ! -f "$ELF" ]]; then
    echo "ELF not found: $ELF — building TARGET=arm64" >&2
    make -C "$ROOT" TARGET=arm64 >/dev/null
fi

python3 - "$ELF" "$OUT_DIR" "$UI_TSV" <<'PY'
import os
import re
import select
import signal
import subprocess
import sys
import time

ELF = sys.argv[1]
OUT_DIR = sys.argv[2]
UI_TSV = sys.argv[3]

# Pages to skip even if defined in the TSV. bottom_nav is the shared navbar
# template included on every real page; selecting it standalone shows the
# navbar against an empty canvas, which is not informative.
SKIP_PAGES = frozenset({"bottom_nav"})

# Render order. Pages not listed here fall through to TSV order. This keeps
# the operator-flow surfaces (dashboard, jog, mdi...) at the top of UI.md
# regardless of how the TSV is rearranged.
PAGE_ORDER = (
    "dashboard", "jog", "mdi", "machine_view", "program", "offsets",
    "service", "ethercat", "homing", "alarms", "macros", "probe",
    "pec", "geometry", "sphere", "network", "axis_status", "tool_change",
)

PAGE_RE = re.compile(r"^page\s+id=(\S+)\s+title=(.+?)\s*$")

def load_pages(tsv_path):
    found = {}
    order = []
    try:
        with open(tsv_path, "r", encoding="utf-8") as f:
            for raw in f:
                line = raw.rstrip("\n").rstrip("\r")
                if not line or line.startswith("#"):
                    continue
                m = PAGE_RE.match(line)
                if not m:
                    continue
                pid = m.group(1)
                if pid in SKIP_PAGES or pid in found:
                    continue
                found[pid] = m.group(2).strip()
                order.append(pid)
    except OSError as e:
        print(f"warning: could not read {tsv_path}: {e}", file=sys.stderr)
    ordered = [p for p in PAGE_ORDER if p in found]
    ordered.extend(p for p in order if p not in PAGE_ORDER)
    return [(p, found[p]) for p in ordered]

PAGES = load_pages(UI_TSV)
if not PAGES:
    PAGES = [("dashboard", "Dashboard")]

with open(os.path.join(OUT_DIR, "pages.tsv"), "w", encoding="utf-8") as f:
    for pid, title in PAGES:
        f.write(f"{pid}\t{title}\n")

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
    # The arm64 PL011 driver's UARTDriver::puts (hal_qemu_arm64.cpp) cooks
    # every '\n' (0x0A) on the wire to '\r\n' (0x0D 0x0A) so that human
    # terminals get proper line breaks. The cli's chunk-queue drain
    # (cli.cpp uart_io_entry::flush_line) routes every byte through that
    # same puts, so the cooking applies to ui_dump's binary payload too.
    #
    # The transform is deterministic (every 0x0A gains exactly one
    # leading 0x0D — see the unconditional `if (*str == '\n') putc('\r')`
    # branch) and therefore losslessly reversible: stripping one 0x0D
    # immediately before every 0x0A on the wire restores the original
    # byte stream. A pre-existing 0x0D 0x0A in the source became wire
    # 0x0D 0x0D 0x0A and decodes correctly because we only strip one CR
    # per LF.
    #
    # Decode the wire buffer in place each iteration so both the
    # UI_DUMP_BEGIN marker line and the binary RGB payload are framed
    # in their original form before BEGIN_RE / payload_size are applied.
    wire = bytearray()
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        r, _, _ = select.select([fd], [], [], max(0.0, remaining))
        if not r:
            continue
        chunk = os.read(fd, 65536)
        if not chunk:
            break
        wire.extend(chunk)
        decoded = wire.replace(b"\r\n", b"\n")
        match = BEGIN_RE.search(decoded)
        if not match:
            continue
        payload_size = int(match.group(3))
        payload_start = match.end()
        needed = payload_start + payload_size
        while len(decoded) < needed and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            r, _, _ = select.select([fd], [], [], max(0.0, remaining))
            if not r:
                continue
            more = os.read(fd, 65536)
            if not more:
                break
            wire.extend(more)
            decoded = wire.replace(b"\r\n", b"\n")
        if len(decoded) < needed:
            raise RuntimeError("short UI dump payload")
        payload = bytes(decoded[payload_start:needed])
        tail = bytes(decoded[needed:])
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

    for page, title in PAGES:
        print(f"capturing {page} ({title})", file=sys.stderr)
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
