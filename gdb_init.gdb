# gdb_init.gdb - GDB initialization and test script for miniOS kernel boot

# --- Initial GDB Setup ---
set confirm off
set pagination off
set print pretty on
set print object on

echo "[GDB_SCRIPT] Attempting to connect to target remote localhost:1234\n"
target remote localhost:1234
echo "[GDB_SCRIPT] Connected to target.\n"

set confirm on
# Set architecture (usually auto-detected correctly)
set architecture auto 
echo "[GDB_SCRIPT] Architecture set to auto.\n"

# --- Test Breakpoints and Commands ---

# Breakpoint 1: Start of kernel_main 
echo "[GDB_SCRIPT] Setting Breakpoint 1: kernel::kernel_main\n"
break kernel::kernel_main
commands 1
  silent
  echo "\n[TEST EXEC] ==> Hit Breakpoint 1: kernel::kernel_main\n"
  echo "               Location: "
  frame 0
  echo "               Verifying initial state...\n"
  echo "               DAIF (Interrupt Flags before kernel_main body): "
  # Try printing DAIF more robustly if $daif gives 'void'
  # maintenance print "$DAIF" might work, or examining SPSR_EL1 if in EL1
  print /x $daif 
  echo "               SP (Stack Pointer): "
  print /x $sp
  echo "\n[TEST EXEC] Setting up further breakpoints and continuing kernel_main setup...\n"
  
  # Breakpoint 2: At the 'asm volatile("msr daifclr, #2");' instruction.
  # For YOUR core.cpp (kernel_main starts 391), this is line 422.
  echo "[GDB_SCRIPT] Setting Breakpoint 2: core.cpp:422 (at IRQ enable instruction)\n"
  break core.cpp:422
  commands 2
    silent
    echo "\n[TEST EXEC] ==> Hit Breakpoint 2: AT 'msr daifclr, #2' (core.cpp:422)\n"
    echo "               Location: "
    frame 0
    echo "               DAIF (Interrupt Flags BEFORE 'msr daifclr, #2'): "
    print /x $daif 
    echo "[TEST EXEC] Stepping over 'msr daifclr, #2' instruction...\n"
    stepi # Execute the single 'msr daifclr, #2' instruction
    echo "               DAIF (Interrupt Flags AFTER 'msr daifclr, #2'): "
    print /x $daif
    eval "set $i_bit_masked = ($daif >> 7) & 1" # Check I-bit (bit 7)
    if $i_bit_masked
      echo "               WARNING: IRQs still masked in DAIF! (I-bit is 1)\n"
    else
      echo "               SUCCESS: IRQs appear unmasked in DAIF (I-bit is 0).\n"
    end
    echo "[TEST EXEC] Continuing, expecting timer interrupt...\n"
    disable 2 # Disable this breakpoint now
    
    # Set breakpoints for interrupt path
    echo "[GDB_SCRIPT] Setting Breakpoint 3 (IRQ Stub): current_el_spx_irq_entry\n"
    break current_el_spx_irq_entry
    commands 3
      silent
      echo "\n[TEST EXEC] ==> Hit Breakpoint 3: current_el_spx_irq_entry (cpu_arm64.S)\n"
      echo "               Location: "; frame 0
      echo "               ELR_EL1: "; print /x $elr_el1; echo " SPSR_EL1: "; print /x $spsr_el1
      echo "[TEST EXEC] Continuing into C++ IRQ handler...\n"
      continue
    end

    echo "[GDB_SCRIPT] Setting Breakpoint 4 (C++ IRQ Handler): kernel::hal::hal_irq_handler\n"
    break kernel::hal::hal_irq_handler
    commands 4
      silent
      echo "\n[TEST EXEC] ==> Hit Breakpoint 4: kernel::hal::hal_irq_handler\n"
      echo "               Location: "; frame 0 
      echo "               Argument core_id (from x0): "; print $x0 
      continue
    end

    echo "[GDB_SCRIPT] Setting Breakpoint 5 (Scheduler Tick): kernel::core::Scheduler::preemptive_tick\n"
    break kernel::core::Scheduler::preemptive_tick
    commands 5
      silent
      echo "\n[TEST EXEC] ==> Hit Breakpoint 5: kernel::core::Scheduler::preemptive_tick\n"
      echo "               Location: "; frame 0
      echo "               Argument core_id: "; print core_id 
      echo "\n[TEST EXEC] SUCCESS: Timer IRQ path to scheduler tick verified."
      echo "[TEST EXEC] System will continue. Next step: implement full context switch.\n"
      # Disable breakpoints after test sequence completes
      disable 1 
      disable 4 
      disable 5 
      # Keep Breakpoint 3 (IRQ stub) enabled if you want to see it fire repeatedly
      # disable 3 
    end
    continue # This continue is for Breakpoint 2's command list
  end

  # This continue is for Breakpoint 1's command list.
  continue 
end

# --- Initial GDB Execution Flow ---
echo "[GDB_SCRIPT] Continuing execution from _start to hit first breakpoint (kernel_main)...\n"
continue