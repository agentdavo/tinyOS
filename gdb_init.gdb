# gdb_init.gdb

set confirm off
set pagination off

echo [GDB_SCRIPT] Initial GDB settings applied.\n

# Set architecture explicitly
set architecture aarch64

echo [GDB_PROGRESS] Progress script loaded.\n

# Connect to QEMU (modify as needed)
target remote localhost:1234
echo [GDB_PROGRESS] Connected to QEMU target.\n

# Key progress milestones
break hal::get_platform
commands
  silent
  echo [GDB_PROGRESS] => At get_platform()\n
  continue
end

break kernel_main
commands
  silent
  echo [GDB_PROGRESS] => Entered kernel_main()\n
  print kernel::g_platform
  continue
end

break hal::qemu_virt_arm64::PlatformQEMUVirtARM64::early_init_platform
commands
  silent
  echo [GDB_PROGRESS] => Early platform init\n
  continue
end

break init_scheduler
commands
  silent
  echo [GDB_PROGRESS] => Scheduler initialized!\n
  continue
end

break start_userland
commands
  silent
  echo [GDB_PROGRESS] => Userland starting!\n
  continue
end

# Optional: Watch a key variable for change (e.g., g_platform status)
#watch kernel::g_platform.status
#commands
#  silent
#  echo [GDB_PROGRESS] g_platform.status changed: 
#  print kernel::g_platform.status
#  continue
#end

# Exception or panic handler
break unhandled_exception_loop
commands
  silent
  echo [GDB_PROGRESS] !!! Exception loop entered!\n
  backtrace
  info registers
  continue
end

# Run to first breakpoint
continue
