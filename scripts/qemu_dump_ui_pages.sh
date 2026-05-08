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

# Transcript of every byte the guest emitted, decoded (CRLF -> LF) so it
# reads as plain text. Written incrementally inside the read helpers so
# that even a hard timeout leaves a usable artifact behind. The CI
# workflow uploads OUT_DIR on failure (if: always() on the artifact
# step), which means we can post-mortem which page hung and what the
# kernel was logging right before.
KERNEL_LOG = open(os.path.join(OUT_DIR, "kernel.log"), "wb", buffering=0)
ACTIVE_PAGE = "boot"  # updated by the per-page loop; surfaces in errors

def _record(chunk: bytes) -> None:
    if not chunk:
        return
    KERNEL_LOG.write(chunk.replace(b"\r\n", b"\n"))

def _tail(decoded: bytes, limit: int = 2048) -> str:
    if len(decoded) <= limit:
        snippet = decoded
    else:
        snippet = decoded[-limit:]
    try:
        return snippet.decode("utf-8", errors="replace")
    except Exception:
        return repr(snippet)

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
        _record(chunk)
        buf.extend(chunk)
        if needle in buf:
            return bytes(buf)
    decoded_buf = bytes(buf).replace(b"\r\n", b"\n")
    raise RuntimeError(
        f"timeout waiting for {needle!r} (page={ACTIVE_PAGE!r})\n"
        f"--- last {min(len(decoded_buf), 2048)} bytes of wire ---\n"
        f"{_tail(decoded_buf)}\n"
        f"--- end ---"
    )

def read_dump(fd, timeout: float):
    # cli.cpp:cmd_ui_dump emits the marker + PPM header + binary RGB +
    # trailer all via direct uart->putc with the platform UART's
    # write-lock held for the duration. That bypasses the cli's queued
    # output path (which would CRLF-cook every '\n' in the binary RGB
    # and slow the per-byte rate to ~17 B/s under chunk-pool turnover).
    #
    # So everything from UI_DUMP_BEGIN onward is RAW — no '\n' cooking.
    # The bytes that came BEFORE the marker (boot logs, prompts, the
    # echo of "ui_dump 6", the '\n' submit_line emits) DID go through
    # the cooked path, but we don't need to decode that prefix: the
    # marker token is unique and matches in raw wire just as well.
    #
    # Search the raw wire for `UI_DUMP_BEGIN \d+ \d+ \d+\n` directly,
    # then read exactly payload_size raw bytes after the '\n'. No
    # CRLF transform is applied to the payload, so 0x0D / 0x0A pixel
    # bytes round-trip losslessly.
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
        _record(chunk)
        wire.extend(chunk)
        match = BEGIN_RE.search(wire)
        if not match:
            continue
        payload_size = int(match.group(3))
        payload_start = match.end()
        needed = payload_start + payload_size
        while len(wire) < needed and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            r, _, _ = select.select([fd], [], [], max(0.0, remaining))
            if not r:
                continue
            more = os.read(fd, 65536)
            if not more:
                break
            _record(more)
            wire.extend(more)
        if len(wire) < needed:
            raise RuntimeError(
                f"short UI dump payload (page={ACTIVE_PAGE!r}, "
                f"got {len(wire) - payload_start}/{payload_size} payload bytes)"
            )
        payload = bytes(wire[payload_start:needed])
        tail = bytes(wire[needed:])
        return payload, tail
    decoded = bytes(wire).replace(b"\r\n", b"\n")
    raise RuntimeError(
        f"timeout waiting for UI_DUMP_BEGIN (page={ACTIVE_PAGE!r})\n"
        f"--- last {min(len(decoded), 2048)} bytes of wire ---\n"
        f"{_tail(decoded)}\n"
        f"--- end ---"
    )

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

def drain_quiet(fd, settle: float = 0.5, max_wait: float = 10.0) -> None:
    """Read everything pending on fd until it goes silent for `settle`
    seconds (or `max_wait` total elapses). All consumed bytes are
    recorded into kernel.log so the post-mortem still sees them; we
    don't return them because the caller doesn't care — the goal is
    just to flush the pipe before the next read_until so it can't
    accidentally match a stale token."""
    deadline = time.monotonic() + max_wait
    last_recv = time.monotonic()
    while time.monotonic() < deadline:
        timeout = min(settle, deadline - time.monotonic())
        r, _, _ = select.select([fd], [], [], max(0.0, timeout))
        if r:
            chunk = os.read(fd, 65536)
            if not chunk:
                return
            _record(chunk)
            last_recv = time.monotonic()
        else:
            if time.monotonic() - last_recv >= settle:
                return

try:
    fd = proc.stdout.fileno()

    # Boot order: arm64's kernel_main() spawns the cli thread early enough
    # that CLI::run() emits "miniOS CLI ready." + the first "miniOS> "
    # naturally — no \r pokes from the host needed. The previous script
    # *did* poke every 500 ms, which caused a real-world bug here:
    # accumulated \r bytes in the QEMU chardev buffer were processed by
    # the cli AFTER the banner, each one triggering a spurious empty
    # submit_line() that emitted a fresh prompt. The per-page read_until
    # then matched one of those STALE prompts after sending the first
    # ui_page command — before the command had actually been dispatched —
    # and the next ui_dump fired into a still-busy cli, never producing
    # UI_DUMP_BEGIN. Removing the poke removes that whole class of race.
    READY_BANNER = b"miniOS CLI ready."
    read_until(fd, READY_BANNER, 90.0)
    read_until(fd, PROMPT, 30.0)
    # Settle: let EC master / hmi / virtio-gpu emit their initial init
    # banners (DHCP discover, cycle banner, fault-clear, frame flushes)
    # so the first ui_page command isn't racing with first-time logging.
    time.sleep(3.0)
    # And drain whatever those secondary services emitted during the
    # settle so the per-page read_until below can't match a stale
    # prompt or boot log that arrived after our last read.
    drain_quiet(fd, settle=0.5, max_wait=10.0)

    for page, title in PAGES:
        ACTIVE_PAGE = page
        print(f"capturing {page} ({title})", file=sys.stderr, flush=True)
        proc.stdin.write(f"ui_page {page}\r".encode("ascii"))
        proc.stdin.flush()
        # 30 s — render under concurrent EC-fault / virtio-gpu flush traffic
        # is slower than steady-state, and we'd rather wait than flake.
        read_until(fd, PROMPT, 30.0)

        proc.stdin.write(f"ui_dump {SCALE}\r".encode("ascii"))
        proc.stdin.flush()
        payload, tail = read_dump(fd, 30.0)

        ppm_path = os.path.join(OUT_DIR, f"{page}.ppm")
        png_path = os.path.join(OUT_DIR, f"{page}.png")
        with open(ppm_path, "wb") as f:
            f.write(payload)

        # Cheap "is this image trivially broken" check before we
        # spend ffmpeg cycles on it. The previous CI gate only
        # verified that *some* PNG existed for each page — a real
        # regression like "all dashboards render solid black" or
        # "TSV NOT_LOADED fallback rendered for every page" would
        # have committed cleanly because the file is still the
        # right size and format.
        #
        # Parse the PPM header to find the binary RGB region, then
        # build a histogram of colors quantised to 4 bits per channel
        # (so dumps don't allocate a slot per 24-bit color).
        #
        # Failure requires BOTH conditions:
        #   - >99% of pixels share the dominant color (almost
        #     entirely one color), AND
        #   - <5 distinct quantised colors (essentially monochrome).
        #
        # Either alone has too many false positives. The CNC
        # operator surfaces routinely have a single solid background
        # color covering >97% of the panel at scale=6 (the dashboard,
        # the ec page in steady state, etc — they have widgets but
        # the widgets are sparse small numerical readouts). However,
        # a real surface always has at least 5–10 distinct colors
        # (background, panel borders, title text, value text, anti-
        # aliased edges) — so the AND of "almost all one color" and
        # "almost monochrome" reliably identifies the failure modes
        # we care about (uninitialised fb, TSV NOT_LOADED fallback)
        # without flagging legitimate sparse pages.
        try:
            hdr_end = 0
            line_count = 0
            i = 0
            while i < len(payload):
                if payload[i:i+1] == b"\n":
                    line_count += 1
                    if line_count == 3:
                        hdr_end = i + 1
                        break
                i += 1
            if hdr_end == 0:
                raise ValueError("PPM header parse failed")
            rgb = payload[hdr_end:]
            if len(rgb) % 3 != 0:
                raise ValueError(f"RGB length {len(rgb)} not divisible by 3")
            quant = {}
            for off in range(0, len(rgb), 3):
                key = (rgb[off] >> 4, rgb[off+1] >> 4, rgb[off+2] >> 4)
                quant[key] = quant.get(key, 0) + 1
            total = len(rgb) // 3
            top = max(quant.values()) if quant else 0
            top_frac = top / total if total else 1.0
            distinct = len(quant)
            print(
                f"  {page}: {total} px, {distinct} colors, "
                f"dominant={top_frac:.1%}",
                file=sys.stderr, flush=True,
            )
            if top_frac > 0.99 and distinct < 5:
                print(
                    f"  WARNING: {page}.ppm looks blank "
                    f"({top_frac:.1%} dominant, {distinct} distinct "
                    f"4-bit-quantised colors). Likely render regression — "
                    f"uninitialised fb, TSV NOT_LOADED fallback, or page "
                    f"failed to draw any widgets. Continuing so the PNG "
                    f"is still committed for inspection.",
                    file=sys.stderr, flush=True,
                )
        except RuntimeError:
            raise
        except Exception as e:
            print(f"  {page}: image-quality check skipped ({e})",
                  file=sys.stderr, flush=True)

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
