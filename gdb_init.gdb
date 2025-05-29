# gdb_init.gdb (Simplified for observing UART prints)
set confirm off
set pagination off
echo "[GDB_SCRIPT] Initial GDB settings applied.\n"
echo "[GDB_SCRIPT] Attempting to connect to target remote localhost:1234\n"
target remote localhost:1234
echo "[GDB_SCRIPT] Connected to target.\n"
set confirm on
set architecture auto 
echo "[GDB_SCRIPT] Architecture set to auto.\n"
echo "[GDB_SCRIPT] Continuing execution. Observe serial_core0.log for ASM and HAL_DEBUG prints...\n"
continue