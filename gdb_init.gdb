# gdb_init.gdb (Simplified for observing UART prints from assembly)

set confirm off
set pagination off
echo "[GDB_SCRIPT] Initial GDB settings applied.\n"

echo "[GDB_SCRIPT] Attempting to connect to target remote localhost:1234\n"
target remote localhost:1234
echo "[GDB_SCRIPT] Connected to target.\n"

set confirm on
set architecture auto 
echo "[GDB_SCRIPT] Architecture set to auto.\n"

# Optional: you can still set a breakpoint late in kernel_main if you expect it to get that far
# break kernel::kernel_main
# commands
#   echo "Reached kernel_main C++ entry"
#   continue
# end

echo "[GDB_SCRIPT] Continuing execution. Observe serial_core0.log for ASM_DEBUG prints...\n"
continue

# Script ends. GDB will let QEMU run.
# If it hangs, Ctrl+C in GDB should now pause QEMU without terminating it.