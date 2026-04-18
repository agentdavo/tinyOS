#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
EC_PORT="${QEMU_EC_PORT:-$((26000 + ($$ % 1000) * 10))}"
HMI_PORT="${QEMU_HMI_HOST_PORT:-$((EC_PORT + 1))}"
LOG="$(mktemp)"
DISCOVER_LOG="$(mktemp)"
STATUS_LOG="$(mktemp)"
cleanup() {
    rm -f "$LOG" "$DISCOVER_LOG" "$STATUS_LOG"
    if [[ -n "${QEMU_WRAPPER_PID:-}" ]]; then
        kill "$QEMU_WRAPPER_PID" 2>/dev/null || true
        wait "$QEMU_WRAPPER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

QEMU_EC_PORT="$EC_PORT" QEMU_HMI_HOST_PORT="$HMI_PORT" \
    bash "$ROOT/scripts/qemu_run.sh" arm64 16 "hmi" >"$LOG" 2>&1 &
QEMU_WRAPPER_PID=$!

for _ in $(seq 1 20); do
    if grep -q "hmi: eth0 service" "$LOG"; then
        break
    fi
    sleep 1
done

grep -q "hmi: eth0 service" "$LOG"
sleep 2

DISCOVER_OK=0
for _ in $(seq 1 8); do
    if python3 "$ROOT/scripts/hmi_udp_probe.py" --host 127.0.0.1 --port "$HMI_PORT" --timeout 1.0 discover >"$DISCOVER_LOG"; then
        DISCOVER_OK=1
        break
    fi
    sleep 1
done
[[ "$DISCOVER_OK" -eq 1 ]]
python3 "$ROOT/scripts/hmi_udp_probe.py" --host 127.0.0.1 --port "$HMI_PORT" --timeout 2.0 status >"$STATUS_LOG"

grep -q "miniOS hmi ip nic=eth0" "$DISCOVER_LOG"
grep -Eq "status .*ip=.*mask=.*gw=.*page=" "$STATUS_LOG"

cat "$DISCOVER_LOG"
cat "$STATUS_LOG"
cat "$LOG"
