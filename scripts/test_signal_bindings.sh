#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT="${QEMU_EC_PORT:-$((25000 + ($$ % 1000)))}"
LOG="$(mktemp)"
trap 'rm -f "$LOG"' EXIT

QEMU_EC_PORT="$PORT" bash "$ROOT/scripts/qemu_run.sh" arm64 8 "hmi" >"$LOG"
grep -q "net-role hmi: eth0 -> core0" "$LOG"
grep -q "hmi: eth0 service" "$LOG"
grep -q "miniOS CLI ready" "$LOG"
grep -Eq "hmi: eth0 .* ip=.* mask=.* gw=.* udp=.* dhcp=.*" "$LOG"
cat "$LOG"
