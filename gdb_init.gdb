# gdb_init.gdb (Modified to stop before call_constructors)

set confirm off
set pagination off
echo "[GDB_SCRIPT] Initial GDB settings applied.\n"

echo "[GDB_SCRIPT] Attempting to connect to target remote localhost:1234\n"
target remote localhost:1234
echo "[GDB_SCRIPT] Connected to target.\n"

set confirm on
set architecture auto 
echo "[GDB_SCRIPT] Architecture set to auto.\n"

# Breakpoint just BEFORE 'bl call_constructors' in cpu_arm64.S
# You'll need to find the exact line number for 'bl call_constructors' in your cpu_arm64.S
# Let's assume it's line 115 for this example (UPDATE THIS)
echo "[GDB_SCRIPT] Setting Breakpoint 1: At 'bl call_constructors' in _start\n"
break cpu_arm64.S:115
commands 1
  silent
  echo "\n[TEST EXEC] ==> Hit Breakpoint 1: About to call 'call_constructors'\n"
  echo "               Location: "
  frame 0 
  echo "               SP: "; print /x $sp
  echo "               x0 (core_id): "; print /x $x0
  echo "\n[TEST EXEC] Now at 'bl call_constructors'. Use 'stepi' to enter 'call_constructors',"
  echo "               then 'stepi' through its loop, watching x2 (function pointer)."
  echo "               Use 'step' (not stepi) when you are at 'blr x2' to step into the C++ constructor."
end

echo "[GDB_SCRIPT] Continuing execution from _start to hit Breakpoint 1...\n"
continue