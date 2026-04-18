# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project identity

The project is **miniOS** — a freestanding, bare-metal C++20 RTOS that runs as a QEMU `virt` kernel. The working directory is named `tinyOS` but all code, branding, and artifacts use `miniOS`. Build artifacts live under `build/<target>/` (for example `build/arm64/miniOS_kernel_arm64.elf`). Source headers and Makefile are v1.7.

**Boot status**: kernel boots cleanly to the idle loop on all 4 cores. The two historical bugs that blocked it are both fixed:
1. `-mno-outline-atomics` in the Makefile stops libgcc from emitting its `init_have_lse_atomics` ctor, which read the (nonexistent) ELF aux vector and faulted.
2. `cpu_arm64.S:call_constructors` now saves/restores `x0` around each `blr` — previously the callee's return value clobbered the init_array cursor, so only the first ctor ever ran.

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

The Makefile supports two targets and both build:

- **`TARGET=arm64`** (default) — full kernel. Core/HAL/scheduler/CLI/EtherCAT/motion all linked. `QEMU_SYS=qemu-system-aarch64`, `-M virt -cpu max`, dual virtio-net NICs cross-connected via socket backends for EtherCAT loopback.
- **`TARGET=riscv64`** — partial port. HAL + `cpu_rv64.S` + minimal stubs build and boot under `qemu-system-riscv64 -M virt -cpu rv64`, but `CORE_CPP` is empty on the rv64 branch (no scheduler, no CLI, no EtherCAT yet — it's a skeleton for bring-up). NIC setup is SLIRP-only (`-netdev user`), which won't carry 0x88A4 frames, so EtherCAT won't work there even once core is linked.

When adding kernel features, keep them arch-neutral in `core.cpp` / `cli.cpp` / `ethercat/` / `motion/` so the rv64 skeleton can eventually pick them up. Anything new that needs an MMIO driver goes behind `hal::Platform` and gets a concrete impl in `hal/arm64/hal_qemu_arm64.cpp` and `hal/riscv64/hal_qemu_rv64.cpp`.

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
3. **`hal_qemu_arm64.{hpp,cpp}` + `cpu_arm64.S`** — the one concrete platform in `KERNEL_OBJ`. Boot/context-switch is in the `.S` file; MMIO drivers are in the `.cpp`.

`kernel_globals.cpp` defines the singletons (`g_platform`, `g_scheduler_ptr`, trace/IRQ locks). `kernel_main` instantiates the concrete `Platform` and assigns `g_platform`; everything else in the kernel dereferences that pointer. When porting to new hardware the only files that change are a new `hal_*` pair, a new `cpu_*.S`, a linker script, and the `g_platform = new ...` line (see README.md "Porting" section).

### Freestanding constraints

Kernel is built with `-ffreestanding -fno-exceptions -fno-rtti -nostartfiles -nostdlib` linked against `-lgcc` only. `cpp_runtime_stubs.cpp` and `freestanding_stubs.cpp` provide the C++ runtime bits (guards, pure-virtual handler, etc.) and libc-ish stubs. New code must not pull in libstdc++ or libc headers that require runtime support — use `<cstdint>`, `<span>`, `<array>`, `<atomic>`, `<string_view>` etc., which are header-only.

### CLI

Single implementation: `cli.cpp` (tab completion, history, full command table). Linked into every build. Adds commands by registering into the table in `cli.cpp`; no shadow-copy to keep in sync.

### Subsystems not currently linked

`audio.cpp`, `dsp.cpp`, `fs.cpp`, `gpio.cpp`, `net.cpp` (and `demo_cli_dsp.cpp`, `test_framework.cpp`) exist but are **not** in `CORE_CPP`. The active kernel links core, hal, util, trace, cli, kernel_globals, the freestanding/runtime stubs, the EtherCAT stack, motion, config/tsv, devices, diag, rt/base_thread, and the arm64 HAL (`cpu_arm64.S`, `hal_qemu_arm64.cpp`, `hal/shared/virtio_net.cpp`), plus `fake_slave.cpp` when `FAKE_SLAVE=1`. Adding back a subsystem means appending its `.cpp` to `CORE_CPP` in the Makefile and calling its init from `kernel_main`.

### Memory layout

Linker script `ld/linker_arm64.ld` places `.text` at `0x40000000` and `.data` at `0x40100000` — CI asserts both via readelf/objdump. Changing these will break the CI ELF-integrity check.

## Conventions worth knowing

- Two namespaces carry all kernel code: `kernel::core::` (types) and `kernel::hal::` (interfaces/drivers). Subsystems get their own sub-namespace (`kernel::audio`, `kernel::dsp`, …).
- Global kernel constants (`MAX_THREADS=16`, `MAX_CORES=4`, `DEFAULT_STACK_SIZE=4096`, etc.) live in `core.hpp` — change them there, not in subsystem headers.
- Lock discipline: use `ScopedISRLock` inside IRQ context, `ScopedLock` elsewhere. `g_irq_handler_lock` wraps the dispatch in `hal_irq_handler`; `g_trace_lock` guards the legacy global trace buffer.
- License header (`SPDX-License-Identifier: MIT OR Apache-2.0`) goes at the top of every new source/header.
