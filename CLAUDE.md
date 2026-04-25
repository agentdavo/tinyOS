# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project identity

The project is **miniOS** — a freestanding, bare-metal C++20 RTOS that runs as a QEMU `virt` kernel. The working directory is named `tinyOS` but all code, branding, and artifacts use `miniOS`. Build artifacts live under `build/<target>/` (for example `build/arm64/miniOS_kernel_arm64.elf`). Source headers and Makefile are v1.7.

**Boot status**: both arm64 and rv64 boot cleanly through SMP bring-up, virtio-gpu scanout configuration, FAT32 mount (when `sdcard.img` is attached), and into the UI + CLI + HMI services on core 0 / hart 0. Historical bugs fixed here (so they don't regress):
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

- **`TARGET=arm64`** (default) — QEMU virt arm64. `-cpu max`, PL011 UART, GICv3, PSCI HVC SMP bring-up, virtio-mmio at `0x0A000000`/stride `0x200`/32 slots.
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
4. Residual: rv64 still has its own `rv64_sched.cpp` (FIFO round-robin) alongside arm64's `core.cpp` EDF scheduler. TCB type is unified but the ready queues + scheduler entry points are still split — medium-risk refactor, tracked separately. `cpu_context_switch_impl` is the shared seam (arm64 in `hal.cpp`, rv64 in `rv64_stubs.cpp`).

New MMIO drivers go behind `hal::Platform`; concrete impls in `hal/arm64/hal_qemu_arm64.cpp` and `hal/riscv64/hal_qemu_rv64.cpp`.

### Tests

There is no host-runnable test binary. Tests live in `test_framework.cpp` and are invoked by typing `test` at the running kernel's CLI. CI greps the serial log for `Tests completed:.*0 failed`. To run a single test, modify `test_framework.cpp` (there is no test-name filter).

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

`kernel_globals.cpp` defines the singletons (`g_platform`, `g_scheduler_ptr`, trace/IRQ locks). arm64's `kernel_main()` and rv64's `kernel_main_rv64()` each instantiate their concrete `Platform`, assign `g_platform`, then hand off to the shared `kernel::boot::*` helpers for the rest of bring-up. To port to new hardware: new `hal_*` pair, new `cpu_*.S`, a linker script, and a new `kernel_main_<arch>` that follows the same sequence.

### Freestanding constraints

Kernel is built with `-ffreestanding -fno-exceptions -fno-rtti -nostartfiles -nostdlib` linked against `-lgcc` only. `cpp_runtime_stubs.cpp` and `freestanding_stubs.cpp` provide the C++ runtime bits (guards, pure-virtual handler, etc.) and libc-ish stubs. New code must not pull in libstdc++ or libc headers that require runtime support — use `<cstdint>`, `<span>`, `<array>`, `<atomic>`, `<string_view>` etc., which are header-only.

### CLI

Single implementation: `cli.cpp` (tab completion, history, full command table). Linked into every build. Adds commands by registering into the table in `cli.cpp`; no shadow-copy to keep in sync.

### Subsystems not currently linked

`audio.cpp`, `dsp.cpp`, `fs.cpp`, `gpio.cpp`, `net.cpp` (and `demo_cli_dsp.cpp`, `test_framework.cpp`) exist but are **not** in `CORE_CPP`. The active kernel links core, hal, util, trace, cli, kernel_globals, the freestanding/runtime stubs, the EtherCAT stack, motion, config/tsv, devices, diag, rt/base_thread, ui (fb + splash + display + operator_api + ui_builder_tsv), automation (macro/ladder/probe runtimes), machine (toolpods/topology/placement/motion_wiring), hmi service, cnc interpreter + MDI, fs/vfs + fat32 + fs_fat32, and the per-arch HAL (`cpu_<arch>.S`, `hal_qemu_<arch>.cpp`, plus shared drivers: `virtio_net`, `virtio_gpu`, `virtio_blk`, `virtio_input`, `e1000`, `pci`, `xhci`, `sdcard`). arm64 additionally links `fake_slave.cpp` when `FAKE_SLAVE=1`. Adding a subsystem means appending its `.cpp` to `CORE_CPP` in the Makefile and calling its init from `kernel::boot::*` (so both arches pick it up).

### Memory layout

Linker scripts `hal/arm64/linker.ld` / `hal/riscv64/linker.ld` fix the load base per arch (arm64 `RAM` origin `0x40000000`, rv64 `RAM` origin `0x80000000`, both 128 MB). CI asserts the arm64 layout via readelf/objdump; changing it will break that check.

## Conventions worth knowing

- Two namespaces carry all kernel code: `kernel::core::` (types) and `kernel::hal::` (interfaces/drivers). Subsystems get their own sub-namespace (`kernel::audio`, `kernel::dsp`, …).
- Global kernel constants (`MAX_THREADS=16`, `MAX_CORES=4`, `DEFAULT_STACK_SIZE=4096`, etc.) live in `core.hpp` — change them there, not in subsystem headers.
- Lock discipline: use `ScopedISRLock` inside IRQ context, `ScopedLock` elsewhere. `g_irq_handler_lock` wraps the dispatch in `hal_irq_handler`; `g_trace_lock` guards the legacy global trace buffer.
- License header (`SPDX-License-Identifier: MIT OR Apache-2.0`) goes at the top of every new source/header.
