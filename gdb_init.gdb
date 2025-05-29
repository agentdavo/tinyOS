# gdb_init.gdb (Focus on first constructor call inside call_constructors)

set confirm off
set pagination off
echo "[GDB_SCRIPT] Initial GDB settings applied.\n"

echo "[GDB_SCRIPT] Attempting to connect to target remote localhost:1234\n"
target remote localhost:1234
echo "[GDB_SCRIPT] Connected to target.\n"

set confirm on
set architecture auto 
echo "[GDB_SCRIPT] Architecture set to auto.\n"

echo "[GDB_SCRIPT] Setting Breakpoint 1: cpu_arm64.S:142\n"

break cpu_arm64.S:142

commands 1
  silent
  frame 0
end

continue