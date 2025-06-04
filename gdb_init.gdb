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
# Break on kernel entry to check early state
tbreak _start
commands
  silent
  echo [GDB_PROGRESS] => At _start\n
  # Show stack pointer and current core ID
  info registers sp
  p/x $mpidr_el1
  continue
end

# Break when global constructors are invoked. Use this spot to inspect
# the HAL platform instance before its constructor runs so we know the
# object is still zeroed out.
tbreak call_constructors
commands
  silent
  echo [GDB_PROGRESS] => call_constructors\n
  info registers sp
  printf "[DEBUG] g_platform_instance at %p\n", &hal::qemu_virt_arm64::g_platform_instance
  printf "[DEBUG] vtable before ctor: %p\n", *(void**)&hal::qemu_virt_arm64::g_platform_instance
  continue
end

break hal::get_platform
commands
  silent
  echo [GDB_PROGRESS] => At get_platform()\n
  continue
end

# After constructors run, stop in kernel_main to verify that the
# platform instance was initialized (vtable should be non-zero).
break kernel_main
commands
  silent
  echo [GDB_PROGRESS] => Entered kernel_main()\n
  print kernel::g_platform
  printf "[DEBUG] g_platform_instance vtable now: %p\n", *(void**)&hal::qemu_virt_arm64::g_platform_instance
  continue
end

# Platform initialization points
break hal::qemu_virt_arm64::PlatformQEMUVirtARM64::early_init_platform
commands
  silent
  echo [GDB_PROGRESS] => Early platform init\n
  info registers sp
  p/x $x0
  continue
end

break hal::qemu_virt_arm64::PlatformQEMUVirtARM64::early_init_core
commands
  silent
  echo [GDB_PROGRESS] => Early core init\n
  info registers sp
  p/x $x0
  continue
end

# Scheduler start
break kernel::core::Scheduler::start_core_scheduler
commands
  silent
  echo [GDB_PROGRESS] => Scheduler started\n
  info registers sp
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

# Exception or panic handlers
break hal::qemu_virt_arm64::PlatformQEMUVirtARM64::panic
commands
  silent
  echo [GDB_PROGRESS] !!! PANIC called !!!\n
  backtrace
  info registers
  continue
end

# Catch unexpected exceptions
break unhandled_exception_entry_point
commands
  silent
  echo [GDB_PROGRESS] !!! Unhandled exception !!!\n
  backtrace
  info registers
  continue
end

# Run to first breakpoint
continue
