#!/usr/bin/env bash
# SPDX-License-Identifier: MIT OR Apache-2.0
#
# qemu_run.sh — boot a miniOS kernel in QEMU with the same NIC / CPU / memory
# wiring the Makefile uses, pipe arbitrary input at the CLI, and capture the
# serial log. Lets you exercise kernel features without a physical bus.
#
# Usage:
#   scripts/qemu_run.sh <platform> [seconds] [input_string]
#   scripts/qemu_run.sh <platform> [seconds] --stdin
#   scripts/qemu_run.sh <platform> [seconds] --gui [input_string]
#   scripts/qemu_run.sh <platform> [seconds] --display <backend> [input_string]
#   scripts/qemu_run.sh <platform> [seconds] --gui --no-hmi-forward
#   scripts/qemu_run.sh <platform> [seconds] --gui --no-ec-loopback
#
# Examples:
#   scripts/qemu_run.sh arm64 10 "help"
#   scripts/qemu_run.sh arm64 8 "ec_sdo_read 0x1018 0x01"
#   printf 'help\n' | scripts/qemu_run.sh arm64 8 --stdin
#   scripts/qemu_run.sh riscv64 6 "help"
#   scripts/qemu_run.sh riscv64 30 --gui
#   scripts/qemu_run.sh riscv64 --gui --no-hmi-forward
#   scripts/qemu_run.sh riscv64 --gui --no-ec-loopback
#
# Platforms:
#   arm64   — eth0 user/HMI plus eth1/eth2 cross-connected for EtherCAT loopback
#   riscv64 — eth0 user/HMI plus eth1/eth2 cross-connected for EtherCAT loopback
#
# By default the script runs QEMU with `-nographic` and prints the serial log to
# stdout. `--gui` auto-selects a conservative GUI display backend
# (`gtk,gl=off` preferred, then `sdl,gl=off`); `--display <backend>` lets you choose another QEMU
# display backend. `--no-hmi-forward` disables the guest
# UDP HMI host-forward rule. `--no-ec-loopback` removes the eth1/eth2 socket
# pair used for EtherCAT loopback. Exit code is 0 unless QEMU itself fails to launch;
# timing-out is treated as a successful run (the kernel is
# long-lived, so timeouts are expected).

set -u

usage() {
    sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
}

[[ $# -lt 1 ]] && usage
[[ "${1:-}" == "--help" || "${1:-}" == "-h" ]] && usage

PLATFORM="$1"; shift
SECONDS_ARG="10"
SECONDS_EXPLICIT=0
if [[ $# -gt 0 && "$1" =~ ^[0-9]+$ ]]; then
    SECONDS_ARG="$1"
    SECONDS_EXPLICIT=1
    shift
fi
INPUT_MODE="--no-input"
DISPLAY_MODE="nographic"
DISPLAY_BACKEND=""
DISPLAY_BACKEND_AUTO=0
HMI_FORWARD=1
EC_LOOPBACK=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --stdin)
            INPUT_MODE="--stdin"
            shift
            ;;
        --gui)
            DISPLAY_MODE="display"
            DISPLAY_BACKEND_AUTO=1
            shift
            ;;
        --display)
            [[ $# -lt 2 ]] && usage
            DISPLAY_MODE="display"
            DISPLAY_BACKEND="$2"
            shift 2
            ;;
        --no-gui)
            DISPLAY_MODE="nographic"
            DISPLAY_BACKEND=""
            shift
            ;;
        --no-hmi-forward)
            HMI_FORWARD=0
            shift
            ;;
        --no-ec-loopback)
            EC_LOOPBACK=0
            shift
            ;;
        --help|-h)
            usage
            ;;
        *)
            if [[ "$INPUT_MODE" != "--no-input" ]]; then
                usage
            fi
            INPUT_MODE="$1"
            shift
            ;;
    esac
done

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
HMI_GUEST_PORT="${QEMU_HMI_GUEST_PORT:-5000}"

choose_gui_backend() {
    local qemu_bin="$1"
    local backends
    backends="$("$qemu_bin" -display help 2>/dev/null || true)"
    if grep -qx 'gtk' <<<"$backends"; then
        echo "gtk,gl=off"
        return 0
    fi
    if grep -qx 'sdl' <<<"$backends"; then
        echo "sdl,gl=off"
        return 0
    fi
    return 1
}

choose_default_ports() {
    python3 - "$$" <<'PY'
import socket
import sys

pid = int(sys.argv[1])

def is_free(port: int, sock_type: int) -> bool:
    s = socket.socket(socket.AF_INET, sock_type)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False
    finally:
        s.close()

def find_free(start: int, sock_type: int, reserved: set[int]) -> int:
    for port in range(start, 65536):
        if port in reserved:
            continue
        if is_free(port, sock_type):
            return port
    raise SystemExit(f"no free port found from {start}")

base = 20000 + (pid % 1000) * 10
reserved: set[int] = set()
ec_port = find_free(base + 1, socket.SOCK_STREAM, reserved)
reserved.add(ec_port)
hmi_host_port = find_free(base + 2, socket.SOCK_DGRAM, reserved)
print(ec_port, hmi_host_port)
PY
}

AUTO_PORTS=()
if [[ -z "${QEMU_EC_PORT:-}" || -z "${QEMU_HMI_HOST_PORT:-}" ]]; then
    if ! read -r -a AUTO_PORTS < <(choose_default_ports 2>/dev/null); then
        PORT_BASE=$((20000 + ($$ % 1000) * 10))
        AUTO_PORTS=("$((PORT_BASE + 1))" "$((PORT_BASE + 2))")
    fi
fi

EC_PORT="${QEMU_EC_PORT:-${AUTO_PORTS[0]}}"
HMI_HOST_PORT="${QEMU_HMI_HOST_PORT:-${AUTO_PORTS[1]:-$HMI_GUEST_PORT}}"

case "$PLATFORM" in
    arm64)
        ELF="$ROOT/build/arm64/miniOS_kernel_arm64.elf"
        QEMU=qemu-system-aarch64
        QEMU_ARGS=(
            -M virt -cpu max -smp 4 -m 128M
            -global virtio-mmio.force-legacy=false
            -netdev user,id=n0
            -device virtio-net-device,netdev=n0
            -device virtio-gpu-device
            -device virtio-keyboard-device
            -device virtio-tablet-device
            -device qemu-xhci,id=xhci
            -kernel "$ELF"
        )
        ;;
    riscv64)
        ELF="$ROOT/build/riscv64/miniOS_kernel_riscv64.elf"
        QEMU=qemu-system-riscv64
        QEMU_ARGS=(
            -M virt -cpu rv64 -smp 4 -m 128M -bios none
            -global virtio-mmio.force-legacy=false
            -netdev user,id=n0
            -device virtio-net-device,netdev=n0
            -device virtio-gpu-device
            -device virtio-keyboard-device
            -device virtio-tablet-device
            -device qemu-xhci,id=xhci
            -kernel "$ELF"
        )
        ;;
    *)
        echo "unknown platform: $PLATFORM (expected arm64 or riscv64)" >&2
        exit 2
        ;;
esac

if [[ "$HMI_FORWARD" -eq 1 ]]; then
    QEMU_ARGS=("${QEMU_ARGS[@]/-netdev user,id=n0/-netdev user,id=n0,hostfwd=udp::"$HMI_HOST_PORT"-:"$HMI_GUEST_PORT"}")
fi

if [[ "$EC_LOOPBACK" -eq 1 ]]; then
    EC_ARGS=(
        -netdev socket,id=n1,listen=:"$EC_PORT"
        -device virtio-net-device,netdev=n1
        -netdev socket,id=n2,connect=127.0.0.1:"$EC_PORT"
        -device virtio-net-device,netdev=n2
    )
    QEMU_ARGS+=("${EC_ARGS[@]}")
fi

if [[ "$DISPLAY_MODE" == "display" ]]; then
    if [[ "$DISPLAY_BACKEND_AUTO" -eq 1 ]]; then
        if ! DISPLAY_BACKEND="$(choose_gui_backend "$QEMU")"; then
            echo "no usable QEMU GUI backend found for $QEMU; try --display <backend> or install gtk/sdl support" >&2
            exit 3
        fi
        echo "qemu_run.sh: using display backend '$DISPLAY_BACKEND'" >&2
    fi
    QEMU_ARGS=(-display "$DISPLAY_BACKEND" "${QEMU_ARGS[@]}")
else
    QEMU_ARGS=(-nographic "${QEMU_ARGS[@]}")
fi

if [[ ! -f "$ELF" ]]; then
    echo "ELF not found: $ELF — building TARGET=$PLATFORM" >&2
    make -C "$ROOT" TARGET="$PLATFORM" >/dev/null
fi

# Pick the input source.
FEED_INPUT=""
case "$INPUT_MODE" in
    --no-input) ;;
    --stdin)    FEED_INPUT="$(cat)" ;;
    *)          FEED_INPUT="$INPUT_MODE" ;;
esac

RUNNER=()
if command -v stdbuf >/dev/null 2>&1; then
    RUNNER=(stdbuf -o0 -e0)
fi

TIMEOUT_PREFIX=()
if [[ "$DISPLAY_MODE" == "display" && "$SECONDS_EXPLICIT" -eq 0 ]]; then
    :
else
    TIMEOUT_PREFIX=(timeout --preserve-status "${SECONDS_ARG}s")
fi

# QEMU consumes stdin as the serial port. Use a FIFO-backed feeder instead of a
# shell pipe so command timing is less sensitive and QEMU stdout remains a
# direct stream.
if [[ -n "$FEED_INPUT" ]]; then
    FIFO="$(mktemp -u)"
    mkfifo "$FIFO"
    cleanup_fifo() {
        rm -f "$FIFO"
        if [[ -n "${FEEDER_PID:-}" ]]; then
            kill "$FEEDER_PID" 2>/dev/null || true
            wait "$FEEDER_PID" 2>/dev/null || true
        fi
    }
    trap cleanup_fifo EXIT

    bash -c '
        fifo="$1"
        payload="$2"
        exec >"$fifo"
        sleep 2
        while IFS= read -r line; do
            printf "%s\n" "$line"
            sleep 1
        done <<<"$payload"
        while true; do printf " \n"; sleep 1; done
    ' _ "$FIFO" "$FEED_INPUT" &
    FEEDER_PID=$!
    "${TIMEOUT_PREFIX[@]}" "${RUNNER[@]}" "$QEMU" "${QEMU_ARGS[@]}" <"$FIFO"
    rc=$?
    cleanup_fifo
    trap - EXIT
else
    "${TIMEOUT_PREFIX[@]}" "${RUNNER[@]}" "$QEMU" "${QEMU_ARGS[@]}" </dev/null
    rc=$?
fi

# QEMU is long-lived, so a timeout is the expected successful case when one is
# requested.
# Preserve real launch/runtime failures instead of masking them.
if [[ "${rc:-0}" -eq 124 ]]; then
    exit 0
fi
exit "${rc:-0}"
