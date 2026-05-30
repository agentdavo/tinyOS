#!/bin/bash
# rv64 boot reliability harness. Sequential (never races the build), captures
# per-run pass/fail plus the first trap line for triage. Usage:
#   scripts/rv64_boot_loop.sh [N]   (default 20)
set -u
ELF=build/riscv64/miniOS_kernel_riscv64.elf
N=${1:-20}
OUT=${RV64_LOOP_OUT:-/tmp/rv64_loop}
mkdir -p "$OUT"
reach=0; trap=0
for i in $(seq 1 "$N"); do
  log="$OUT/run_$i.log"
  timeout 18 qemu-system-riscv64 -M virt -cpu max -smp 4 -m 128M -bios none \
    -nographic -kernel "$ELF" > "$log" 2>&1
  r=$(grep -c "miniOS CLI ready" "$log")
  t=$(grep -c "rv64 TRAP" "$log")
  [ "$r" -gt 0 ] && reach=$((reach+1))
  [ "$t" -gt 0 ] && trap=$((trap+1))
  first_trap=$(grep -m1 "rv64 TRAP" "$log" | sed 's/^.*\(mcause=[^ ]*\).*\(name=.*\)$/\1 \2/')
  printf "run %2d: cli=%s trap=%s  %s\n" "$i" "$r" "$t" "$first_trap"
done
echo "SUMMARY: reached CLI $reach/$N, traps $trap/$N"
