# gdb_init.gdb

set confirm off
set pagination off

echo [GDB_SCRIPT] Initial GDB settings applied.\n

# Set architecture explicitly
set architecture aarch64
echo [GDB_SCRIPT] Architecture set to aarch64.\n

# Connect to QEMU
echo [GDB_SCRIPT] Attempting to connect to target remote localhost:1234\n
target remote localhost:1234
echo [GDB_SCRIPT] Connected to target.\n

# Set breakpoints
echo [GDB_SCRIPT] Setting Breakpoint 1: kernel_main\n
break kernel_main
commands 1
  silent
  print kernel::g_platform
  info registers
  continue
end

echo [GDB_SCRIPT] Setting Breakpoint 2: unhandled_exception_loop\n
break unhandled_exception_loop
commands 2
  silent
  echo [GDB_SCRIPT] Data abort detected\n
  backtrace
  print /x $far_el1
  x/10x $x0
  disassemble $x30, $x30+0x10
  info address _ZTVN3hal14qemu_virt_arm6420PlatformQEMUVirtARM64E
  continue
end

echo [GDB_SCRIPT] Setting Breakpoint 3: early_init_platform\n
break hal::qemu_virt_arm64::PlatformQEMUVirtARM64::early_init_platform
commands 3
  silent
  echo [GDB_SCRIPT] Entered early_init_platform\n
  continue
end

# Start execution
continue