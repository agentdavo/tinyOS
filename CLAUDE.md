# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project identity

The project is **miniOS** — a freestanding, bare-metal C++20 RTOS that runs as a QEMU `virt` kernel. The working directory is named `tinyOS` but all code, branding, and artifacts use `miniOS`. Build artifacts live under `build/<target>/` (for example `build/arm64/miniOS_kernel_arm64.elf`). Source headers and Makefile are v1.7.

**Boot status**: arm64 boots cleanly through SMP bring-up, virtio-gpu scanout configuration, FAT32 mount (when `sdcard.img` is attached), and into the UI + CLI + HMI services on core 0. rv64 now does the same: in a 10-run soak (QEMU 11.1, GCC 15.2) every cold boot reached `miniOS CLI ready`, echoed `ci-ok`, logged `[ec0] cycle=250us` and passed `test all` with `Tests completed: 11, 0 failed`. One earlier run hit a single unexplained fault (see open items below).

### rv64 boot work

**Goal:** rv64 reaches the CLI banner and accepts input as reliably as arm64 — both arches must satisfy the arch-parity contract below.

**rv64 needs `-cpu rva23s64` with Ubuntu 26.04's cross GCC.** Its `riscv64-linux-gnu` libgcc is built for the RVA23 profile (Zcb, Zb*, V), and the kernel pulls in `__udivti3` (`hal_qemu_rv64.cpp`) and `__floatuntidf` (`cnc/interpreter.cpp`) via 128-bit integer maths. On the Makefile's `-cpu rv64` those take an illegal-instruction trap (`mcause=2`, e.g. `c.lbu`) before the scheduler starts. The MSYS2 `riscv64-unknown-elf` toolchain can't link the kernel at all: its libgcc multilibs are `medlow` and can't reach `0x80000000`.

**Context-switch bugs fixed in the rv64 hang work** (all in `cpu_rv64.S` unless noted). Symptom: the banner printed, then the `ui` / `hmi` threads re-printed their startup lines on every tick and the system wedged.
1. `trap_entry` set `g_irq_in_progress[hart] = 1` and then skipped the TCB save whenever that flag was set, which was always. The interrupted state was never saved, so trap exit restored `current_thread` from its stale TCB and every tick rewound threads to their last voluntary switch point. The skip is gone. `cpu_context_switch_rv64` runs with MIE=0 end to end, so no interrupt can land mid-switch.
2. Trap exit cleared `g_irq_in_progress` *after* restoring GPRs, using `t0`–`t2` as scratch, which corrupted them in the interrupted thread on every trap. The flag is now cleared before the GPR restore.
3. `cpu_context_switch_rv64` saved and restored only the callee-saved registers. A thread last switched out by a trap resumed with the yielding thread's `ra`/`t*`/`a*`. It now saves and restores all 31 GPRs, like arm64's `cpu_context_switch_impl`.
4. Both mret paths mask `MIE` in the `mstatus` value they write, so an IRQ can't nest mid-restore; `mret` sets MIE from MPIE.
5. `ethercat/master.cpp` had three `yield(0)` calls. `schedule()` switches the *calling* CPU, so a cross-core id requeued another core's running thread and saved this core's registers into its TCB. `Scheduler::yield` (shared `core.cpp`) now always uses the calling core's id.
6. `trap_entry` also set `g_irq_in_progress` *before* saving GPRs, using `t0`–`t2`, so every trap corrupted them on entry as well as on exit. The flag is now set after the frame save. This was found by the `test fp` self-test below, and is the likely cause of the mismatched-context `motion` fault listed under open items.

**FP/SIMD state is context-switched on both arches.** `TCB.fp_regs[64]` / `TCB.fp_ctrl[2]` (byte offsets 272 / 784, pinned by `static_assert`s in `core.hpp`) hold q0–q31 + FPCR/FPSR on arm64 and f0–f31 + fcsr on rv64. `SAVE_FP` / `RESTORE_FP` macros in each `.S` run on the trap/IRQ path and on the voluntary switch. The saves are eager, because GCC uses q-registers for ordinary struct zeroing and memcpy. `test fp` loads patterns into FP registers and spins for 50 ms while `g_fp_scrub_in_irq[core]` makes that core's IRQ handler overwrite every FP register (`fp_scrub_registers()`). It fails if any register changes or if no IRQ landed on the core, since a run without an IRQ proves nothing. Negative control: with the macros disabled it fails with mask `0xf3` on arm64 and `0xc3` on rv64.

**arm64 had no timer preemption until the FP work.** `TimerDriver::init_core_timer_interrupt` wrote `CNTP_CTL_EL0 = 7`, but bit 1 is IMASK, so the scheduler tick was enabled *and masked*. No IRQ ever reached `hal_irq_handler` on any core, and arm64 ran purely cooperatively. It now writes `1`. `rt_wait.hpp` already did.

**Fixed earlier, during the EDF-adoption regression:**

1. Secondary harts could read `ready_qs_[N]` without the per-core lock and grab a partially-initialised TCB, faulting in `safe_strcpy` on a name pointer that hadn't propagated. `start_core_scheduler` now holds the lock around `pop_highest_priority_ready_task` (commit `188b223`).
2. Voluntary `cpu_context_switch_rv64` was using `ret` instead of `mret`, leaving a window where a timer IRQ could fire mid-switch and the trap entry would spill CPU state into the new thread's TCB, corrupting the in-flight switch. Now uses `mret` to atomically restore PC + mstatus from the new TCB — same idiom as arm64's `eret`-based switch (commit `72965c6`).
3. `PLICDriver::enable_core_irqs` no longer flips `mstatus.MIE` — it only enables per-source `mie` bits. The CPU stays IRQ-masked from boot until `mret` atomically restores MIE from the new thread's pstate. Mirrors arm64's `GICDriver::enable_core_irqs` (commit `72965c6`).
4. Idle thread's `tcb.pstate` was unconditionally `0x05` (arm64 SPSR for EL1h with IRQs unmasked) — meaningless on rv64 mstatus (means MPP=U, MIE=0, MPIE=0). Now wrapped in `__aarch64__` (commit `72965c6`).
5. `early_uart_puts` is arch-aware. The arm64 PL011 base `0x09000000` was hardcoded; any shared code that called it (Master::run_loop banner, FakeSlave halt, dc_drift periodic log) faulted on rv64. Now writes to NS16550 at `0x10000000` on rv64 (commit `72965c6`).
6. `uart_io_entry`'s `flush_line` uses the captured `uart` parameter rather than the static `cli::io::g_uart` — some path on rv64 was leaving `g_uart` apparently NULL'd by the time the flush ran (vtable[3]=NULL → jumped to address 0). Function parameter is on uart_io's stack and stable (commit `8efe5a0`).

**Still open:**
- In one run out of 14 after the fixes above, `motion` on hart 1 took `mcause=5` (load access fault, `mtval=0x88000000`) in `run_sync_arbiter` with `ra` inside `safety_status()`. The PC came from one context and the registers from another. Not seen since the `trap_entry` `t0`–`t2` fix (item 6), which is the likely cause; keep watching for it.
- A scheduler hazard on both arches, not yet fixed. (The other one, the timer IRQ deadlocking on `placement::Service::snapshot`'s lock, is fixed: that lock is now a `ScopedISRLock`.) `thread_bootstrap` marks an exiting thread `ZOMBIE` before switching off its stack, so `create_thread` on another core can reuse the TCB and stack while the switch is still in flight.
- Some rv64 PCI/MMIO paths (occasional traps on ec_a / fake_slave threads) may still need arch-aware fixes — not chased yet.

**Useful diagnostic tools added during this work:**
- The rv64 trap dump prints `mtval`, `ra` (regs[0]), `sp` (regs[1]), and the current_thread's `name` in a single locked puts() so concurrent harts don't fragment it. Decode a trap PC with `riscv64-linux-gnu-addr2line -f -e build/riscv64/miniOS_kernel_riscv64.elf <addr>`. Walk the stack by reading words at `sp+offset` (frame layouts visible in `objdump -d`).

Historical bugs fixed here (so they don't regress):
1. `-mno-outline-atomics` in the Makefile stops libgcc from emitting its `init_have_lse_atomics` ctor, which read the (nonexistent) ELF aux vector and faulted (arm64).
2. `cpu_arm64.S:call_constructors` saves/restores `x0` around each `blr` — previously the callee's return value clobbered the init_array cursor (arm64).
3. `_secondary_start` sets SP before any `bl` to a C function — PSCI leaves SP undefined and the old code crashed on the first function prologue push (arm64).
4. `cbz x21` tests the preserved core_id in `_start`'s primary-vs-secondary dispatch. The old code tested `x1` which is caller-save and happened-to-be-zero only by accident (arm64).
5. EDF `select_next_task` does a two-pass scan that prefers a non-caller task on pass 1; fixed-deadline cooperative yield used to re-pick the yielder every time (shared `core.cpp`, benefits both arches).
6. Early-UART output is lock-atomic at line granularity: `boot_log_core_phase_c` / `boot_log_reg64_c` hold the lock across the whole line, and `UARTDriver::puts` on arm64 shares the same lock (rv64's PL011-analog `UARTDriver::puts` has its own `UartLockGuard`). Output from concurrent cores no longer shreds.

**CLI**: `cli.cpp` is the only CLI implementation and is linked into every build — two threads on core 0 (`uart_io` owning the UART + SPSC queues, `cli` doing the line editor + dispatch). There is no `cli_minimal.cpp`; prior references to it in this file and README.md were stale. A successful boot prints `miniOS CLI ready. Type 'help'.` from `cli.cpp:1045`. If the banner is absent from the serial log, the scheduler didn't reach the `cli` thread — that's Phase 0 territory, not a missing subsystem.

**Master cycle**: 250 µs (`ethercat/master.cpp`) — the ClearPath-EC minimum. `devices/clearpath_ec.tsv` 0x60C2 is held to (25, −5) to match. The CI workflow parses the `[ecN] cycle=<us>us` banner emitted from `Master::run_loop` entry and fails if any master logs <250 µs or a non-multiple of 250 µs; keep that banner's format stable if you touch the cycle-time logging.

## Build & run

Cross-compiled bare-metal kernel — do not try to build with the host compiler.

```bash
make                       # build build/arm64/miniOS_kernel_arm64.elf
make clean
make run                   # boot in qemu-system-aarch64, serial goes to serial_core0.log
make debug                 # launches QEMU paused on :1234, then gdb-multiarch with gdb_init.gdb
make docs                  # doxygen -> docs/
make ENABLE_TRACE=0        # compile out the tracing subsystem (TRACE_DEFAULT_ENABLED macro)
```

The build emits `build/$(TARGET)/miniOS_kernel_$(TARGET).elf` — so `build/arm64/miniOS_kernel_arm64.elf` or `build/riscv64/miniOS_kernel_riscv64.elf`. There is no unqualified `miniOS_kernel.elf`; if you see one on disk it's stale from an older build. Always pass the build-dir ELF to qemu manually, or use `make run` (which resolves `$(ELF)` for you).

`make run` is non-interactive — kernel stdout/stdin is redirected via `-chardev file` to `serial_core0.log`. To interact with the CLI, invoke QEMU manually with `-nographic` and a tty chardev, or pipe input like CI does:

```bash
echo -e "test\n" | qemu-system-aarch64 -M virt -cpu max -smp 4 -m 128M -nographic -kernel build/arm64/miniOS_kernel_arm64.elf
```

For repo-local QEMU launches, prefer `scripts/qemu_run.sh`. The old fixed-mode
`scripts/run*.sh` wrappers are gone.

For UI capture, prefer the guest-driven path instead of gdb framebuffer pokes:

```bash
bash scripts/qemu_dump_ui_pages.sh /tmp/ui_captures
```

That script talks to the CLI over serial, issues `ui_page <id>` and `ui_dump <scale>`, and writes page `.ppm` / `.png` files. The relevant CLI commands live in `cli.cpp`; `ui_page` and `ui_dump` both force `kernel::ui::render_ui_once()` so the framebuffer reflects the requested page immediately.

### Target selection

The Makefile supports two targets and both build + boot to the UI + CLI + HMI services:

- **`TARGET=arm64`** (default) — QEMU virt arm64. `-cpu max`, PL011 UART, GICv2 (MMIO distributor/CPU-interface driver + `ITARGETSR` affinity — not GICv3 system-register), PSCI HVC SMP bring-up, virtio-mmio at `0x0A000000`/stride `0x200`/32 slots.
- **`TARGET=riscv64`** — QEMU virt rv64 in M-mode (`-bios none`). NS16550 UART, PLIC+CLINT, spin-table SMP bring-up (hart 0 writes each secondary's entry address into `g_secondary_entry[]`, kicks via MSIP), virtio-mmio at `0x10001000`/stride `0x1000`/8 slots. Also mounts FAT32 via virtio-blk when `sdcard.img` is attached.

Both arches link the same `core.cpp` / `cli.cpp` / `ui/` / `ethercat/` / `motion/` / shared virtio drivers / FAT32. The only legitimate divergence is below the HAL boundary — see "Arch parity contract" below.

### Arch parity contract

The shared kernel is arch-neutral above the HAL. When adding a feature:

- **Goes in `kernel/main.cpp` (one place)**: TSV loading, VFS init, net-role routing, thread-creation sequence. Exposed as `kernel::boot::load_runtime_tsvs` / `init_vfs` / `log_and_route_net_roles` / `create_boot_services(create_thread_fn)` / `create_runtime_services(create_thread_fn)`. Both `kernel_main()` (arm64) and `kernel_main_rv64()` (rv64) call these.
- **Stays split per-arch**: CPU register save/restore asm, PSCI vs spin-table SMP bring-up, IRQ controller (GICv3 vs PLIC+CLINT), UART MMIO base, timer CSRs, linker script, MMIO bus geometry.

Concrete "is this an accidental divergence?" checklist:
1. If a boot step needs adding to one side, does it also need to run on the other? If yes, add it to `kernel::boot::*` and have both sides call it. Don't duplicate.
2. If a driver ships under `hal/shared/`, both arches link it. Missing a shared driver from one arch's `HAL_CPP` is a regression.
3. TCB layout is a single type: `kernel::core::TCB`. Byte offsets for `regs[31]` / `sp` / `pc` / `pstate` are fixed — both `cpu_arm64.S` and `cpu_rv64.S` use the same offsets. rv64 stores `mstatus` in the `pstate` slot.
4. Both arches now share `kernel::core::Scheduler` + `EDFPolicy` from `core.cpp`. The arch boundary is the context-switch primitive (`cpu_context_switch_impl` — arm64 in `hal.cpp`, rv64 in `rv64_stubs.cpp`) plus the per-arch trap dispatcher's `preemptive_tick` hand-off. rv64's `rv64_sched.cpp` is empty — kept around so existing build-system references don't break.

New MMIO drivers go behind `hal::Platform`; concrete impls in `hal/arm64/hal_qemu_arm64.cpp` and `hal/riscv64/hal_qemu_rv64.cpp`.

### Tests

There is no host-runnable test binary. The CLI's `test` command runs a small built-in suite defined directly in `cli.cpp::cmd_test`. Subtests: status, motion, ec, devices, chain, mtl, fake_sdo, tcp, pallet, jobs, ui, fp. `test all` runs them all. CI greps the serial log for `Tests completed:.*0 failed`. To add a subtest, edit the dispatch in `cmd_test`.

### Static analysis (matches CI)

```bash
bear -- make clean && make           # produces compile_commands.json
clang-tidy -p compile_commands.json <file>
```

## Architecture

Three-layer stack, each layer depends only on the ones below it:

1. **`core.hpp`/`core.cpp`** — pure kernel types: `TCB`, `Scheduler` (SMP-aware EDF, up to `MAX_CORES=4`), `Spinlock`/`ScopedLock`/`ScopedISRLock`, `FixedMemoryPool`, `PerCPUData`. No HAL dependency.
2. **`hal.hpp`** — abstract `Platform` plus `*Ops` interfaces (`UARTDriverOps`, `IRQControllerOps`, `TimerDriverOps`, `DMAControllerOps`, `I2SDriverOps`, `MemoryOps`, `NetworkDriverOps`, `PowerOps`, `GPIODriverOps`, `WatchdogOps`). Pure virtuals only.
3. **`hal_qemu_arm64.{hpp,cpp}` + `cpu_arm64.S`** (arm64) or **`hal_qemu_rv64.{hpp,cpp}` + `cpu_rv64.S` + `rv64_stubs.cpp` + `rv64_sched.cpp`** (rv64) — per-arch concrete platforms. Boot/context-switch is in the `.S`; MMIO drivers are in the `.cpp`.

`kernel_globals.cpp` defines the singletons (`g_platform`, `g_scheduler_ptr`, trace/IRQ locks). arm64's `kernel_main()` and rv64's `kernel_main_rv64()` each instantiate their concrete `Platform`, assign `g_platform`, then hand off to the shared `kernel::boot::*` helpers for the rest of bring-up. To port to new hardware: new `hal_*` pair, new `cpu_*.S`, a linker script, and a new `kernel_main_<arch>` that follows the same sequence. For an arm64-SBC port specifically, see `HARDWARE_PORT.md` at the repo root — it walks through what's QEMU-only, what's real-hw-ready, and a 15-step bring-up checklist.

### Freestanding constraints

Kernel is built with `-ffreestanding -fno-exceptions -fno-rtti -nostartfiles -nostdlib` linked against `-lgcc` only. `cpp_runtime_stubs.cpp` and `freestanding_stubs.cpp` provide the C++ runtime bits (guards, pure-virtual handler, etc.) and libc-ish stubs. New code must not pull in libstdc++ or libc headers that require runtime support — use `<cstdint>`, `<span>`, `<array>`, `<atomic>`, `<string_view>` etc., which are header-only.

### CLI

Single implementation: `cli.cpp` (tab completion, history, full command table). Linked into every build. Adds commands by registering into the table in `cli.cpp`; no shadow-copy to keep in sync.

### Subsystems not currently linked

The active kernel links core, hal, util, trace, klog, cli, kernel_globals, the freestanding/runtime stubs, the EtherCAT stack, motion, config/tsv, devices, diag, rt/base_thread, ui (fb + splash + display + operator_api + ui_builder_tsv), automation (macro/ladder/probe runtimes), machine (toolpods/topology/placement/motion_wiring), hmi service, cnc interpreter + MDI, fs/vfs + fat32 + fs_fat32, and the per-arch HAL (`cpu_<arch>.S`, `hal_qemu_<arch>.cpp`, plus shared drivers: `virtio_net`, `virtio_gpu`, `virtio_blk`, `virtio_input`, `e1000`, `pci`, `xhci`, `sdcard`). arm64 additionally links `fake_slave.cpp` when `FAKE_SLAVE=1`. Adding a subsystem means appending its `.cpp` to `CORE_CPP` in the Makefile and calling its init from `kernel::boot::*` (so both arches pick it up).

Historical orphans (`audio.cpp`, `dsp.cpp`, `fs.cpp`, `gpio.cpp`, `net.cpp`, `demo_cli_dsp.cpp`, `test_framework.cpp` and their headers except `audio.hpp`) were removed when audit confirmed they had no callers in the active build. `audio.hpp` stays because `hal/arm64/hal_qemu_arm64.cpp`'s I2SDriver uses the `kernel::audio::AudioBuffer` POD type. Git history preserves the deleted code if any of it needs to be revived.

### Memory layout

Linker scripts `hal/arm64/linker.ld` / `hal/riscv64/linker.ld` fix the load base per arch (arm64 `RAM` origin `0x40000000`, rv64 `RAM` origin `0x80000000`, both 128 MB). CI asserts the arm64 layout via readelf/objdump; changing it will break that check.

## Conventions worth knowing

- Two namespaces carry all kernel code: `kernel::core::` (types) and `kernel::hal::` (interfaces/drivers). Subsystems get their own sub-namespace (`kernel::audio`, `kernel::dsp`, …).
- Global kernel constants (`MAX_THREADS=16`, `MAX_CORES=4`, `DEFAULT_STACK_SIZE=4096`, etc.) live in `core.hpp` — change them there, not in subsystem headers.
- Lock discipline: use `ScopedISRLock` inside IRQ context, `ScopedLock` elsewhere. `g_irq_handler_lock` wraps the dispatch in `hal_irq_handler`; `g_trace_lock` guards the legacy global trace buffer.
- License header (`SPDX-License-Identifier: MIT OR Apache-2.0`) goes at the top of every new source/header.
