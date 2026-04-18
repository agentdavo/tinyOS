#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PORT_BASE="${QEMU_EC_PORT_BASE:-$((24000 + ($$ % 1000) * 10))}"

run_and_check() {
    local platform="$1"
    local port="$2"
    local log
    log="$(mktemp)"
    trap 'rm -f "$log"' RETURN
    QEMU_EC_PORT="$port" bash "$ROOT/scripts/qemu_run.sh" "$platform" 6 >"$log"
    grep -q "net-role hmi: eth0 -> core0" "$log"
    grep -q "net-role ec_a: eth1 -> core2" "$log"
    grep -q "eth2 -> core3" "$log"
    grep -q "topology servo bindings applied" "$log"
    cat "$log"
    rm -f "$log"
    trap - RETURN
}

run_and_check arm64 "$((PORT_BASE + 1))"
run_and_check riscv64 "$((PORT_BASE + 11))"
