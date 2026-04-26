# Hardware Port Checklist

A practical guide for taking miniOS off QEMU and onto a real arm64 single-board
computer (SBC). Today the kernel is exercised exclusively against the
`qemu-system-aarch64 -M virt` machine model. This document describes what
a board-bring-up integrator has to change, what they get for free, and what to
verify at each step.

## 1. Audience and outcome

This guide is for an integrator with kernel/board-bring-up experience who has
chosen an arm64 SBC and wants to run miniOS on it as the primary OS (no host
Linux underneath). "Ported" means, concretely:

1. The kernel boots from the board's bootloader, prints its banner over the
   board's UART, and reaches `miniOS CLI ready. Type 'help'.`
   (`cli.cpp:1045`).
2. All `MAX_CORES` (default 4 — `core.hpp`) come up and execute under the
   shared EDF scheduler.
3. The operator HMI renders to the board's display (HDMI, parallel-LCD,
   MIPI-DSI — whatever framebuffer the SoC exposes).
4. At least one EtherCAT master is bound to a physical NIC and the
   `[ec0] cycle=250us` banner appears (parsed by CI; see
   `ethercat/master.cpp` / `CLAUDE.md` master-cycle note).
5. A jog command from the CLI moves a real servo through CSP without
   tripping the deadline-fault safety latch.

If you reach step 5 with a 30-minute soak, the port is real.

## 2. Target board criteria

miniOS deliberately does **not** endorse a specific SBC. The criteria below
let you score candidates yourself.

**Mandatory**

- arm64 (ARMv8-A or later). Cortex-A53/A55/A72/A76 class cores all work; the
  build defaults to `-mcpu=cortex-a53 -march=armv8-a+simd+lse`
  (`Makefile:67`) which is conservative enough to run on later cores.
- GICv3 interrupt controller. The HAL assumes a distributor + redistributor
  layout (`hal/arm64/hal_qemu_arm64.cpp`, `IRQController`). GICv2 boards
  would need a new IRQ driver.
- ≥ 4 cores. `MAX_CORES = 4` in `core.hpp`; the scheduler is dimensioned
  for that. Boards with fewer cores work after lowering the constant, but
  thread placement (HMI on core 0, masters on cores 2/3) gets cramped.
- ≥ 128 MB RAM. The linker script reserves a 128 MB region
  (`hal/arm64/linker.ld:2`); ESI blobs and PDO arenas can push that
  upward in the future.
- Mainline-Linux support is the strongest single proxy for "this SBC has
  documented PSCI, an FDT, and accessible peripheral docs". You will not
  necessarily run mainline Linux on the board, but mainline-supported
  silicon almost always means the bring-up information you need is public.

**Strongly recommended**

- Two MAC-capable Ethernet interfaces. One carries the HMI/management
  network, the other carries EtherCAT. This matches the runtime placement
  profile baked into the QEMU configuration (`Makefile:88-91`). A single-NIC
  board can boot but cannot exercise EtherCAT and HMI traffic
  simultaneously.
- Display output (HDMI, MIPI-DSI, or RGB-parallel LCD). The operator UI
  expects to call `Framebuffer::data()` on a linear RGB(A) framebuffer.
- USB host (optional for the first cut). Used today only for setup-image
  load via xHCI/PCIe (`hal/shared/xhci.cpp`); the first port can ignore it.

**Tradeoffs**

- "GbE-rich" SBCs with PCIe-attached NICs are easier for EtherCAT (PCIe
  e1000-class drivers are already in `hal/shared/e1000.cpp`).
- "Single-NIC + USB-Ethernet" SBCs (e.g. typical small consumer SBCs) are
  awkward — USB-Ethernet is an entire driver stack you would have to add.
- Boards with a vendor-specific MAC IP (DesignWare GMAC, mvneta, NXP ENET,
  etc.) need a new shared NIC driver. Plan a few weeks for that.

## 3. What is QEMU-specific today and must change

Every item below is a hard-coded assumption that needs to move.

| Concern | Where it lives | What changes |
|---|---|---|
| UART base address `0x09000000`, IRQ 33 (PL011) | `hal/arm64/hal_qemu_arm64.hpp:43,61` | Real SBC has its own UART base + IRQ. PL011 is reusable on PL011-derived UARTs; otherwise replace `UARTDriver::putc/getc` MMIO. |
| GIC distributor `0x08000000`, CPU interface `0x08010000` | `hal/arm64/hal_qemu_arm64.hpp:44-45` | Read from FDT on real hardware. Today hard-coded. The redistributor base differs per SoC. |
| virtio-mmio bus at `0x0A000000`, stride `0x200`, 32 slots | `hal/arm64/hal_qemu_arm64.hpp:47-49` | Does not exist on real silicon. Net/GPU/blk/input drivers backing onto virtio (`hal/shared/virtio_*`) need replacing with real-device drivers. |
| Net via virtio-net (and PCIe e1000 fallback) | `PlatformQEMUVirtARM64::early_init_platform` discovers virtio-net (`hal_qemu_arm64.cpp:726-749`) | A real port adds at least one platform-NIC driver under `hal/shared/<phy>.cpp`. The existing `e1000.cpp` is the closest template. |
| FAT32 root from virtio-blk | `init_block_device()` (`hal_qemu_arm64.cpp:756`) | Real SBC reads from eMMC/SD/NOR. Need a `dwmmc`/`sdhci` shared driver behind the same `fs::BlockReader` adapter. |
| PSCI HVC for SMP bring-up | `psci_cpu_on_impl` (`kernel/main.cpp:400-410`) | The instruction is `hvc #0`. Some bootloaders/SBCs require `smc #0` (firmware-mediated PSCI) or use the older spin-table mechanism (write secondary CPU release-address into a memory cell). Choosing wrong yields a hang at `release_secondary_cores` (`kernel/main.cpp:346`). |
| RAM origin `0x40000000` | `hal/arm64/linker.ld:2` | Real SBC RAM origin is whatever the bootloader places at — typically read from the FDT `/memory` node. CI asserts the QEMU layout via `readelf`/`objdump`; loosen that check before changing. |
| `MINIOS_USE_WFI` is **off** | `hal/arm64/hal_qemu_arm64.cpp:237-242` | Disabled because WFI is unreliable under WSL2/Hyper-V. Production builds on real hardware should re-enable it (`-DMINIOS_USE_WFI`) so idle cores actually idle. |
| xHCI probe gated on a discovered PCI host bridge | `hal_qemu_arm64.cpp:540-543` (commit `eb775a8`) | This guard exists because QEMU's USB+virtio-blk combo deadlocked. Real silicon has a real PCI/USB controller; the guard is harmless but the underlying probe path is exercised for the first time. |
| Timer frequency assumed 62.5 MHz when CNTFRQ_EL0 reads 0 | `hal_qemu_arm64.cpp:191` | Real SoCs program CNTFRQ_EL0 at boot. Trust the register value; the QEMU-only fallback should not fire. |

## 4. What is real-hardware-ready today

These subsystems were designed to be portable and are exercised hard on
QEMU. They should light up on a real SBC with no source changes:

- The shared SMP-aware EDF scheduler (`core.cpp`, used by both arches),
  including the two-pass tie-breaker that prefers a non-caller task
  (`CLAUDE.md` boot-status note 5), priority inheritance on `Spinlock`
  (commit `8b1a6d6`), and IRQ-safe per-core scheduler locks
  (commit `d8c6e0d`).
- Deadline-fault → QuickStop. The fault path triggers an SDO 0x6040
  QuickStop on the affected drive — designed and tested but only really
  exercised once a physical EtherCAT slave is on the bus.
- DC-Sync drift detection with consecutive-miss latch
  (commit `eb08f54`). On QEMU the timestamps are virtual; on real silicon
  this becomes a meaningful safety signal.
- The operator HMI (`ui/`). It renders into a linear framebuffer; bring up
  any display, point `Framebuffer::data()` at it, and the UI appears.
- ESI device DB. The Phase B vendor-XML ingest (commit `aad2d45`) reads
  ESI blobs at boot and supplies per-slave PDO defaults. Vendor neutral —
  any conformant ESI XML works.
- The full TSV-driven UI builder, machine builder, kinematic chain. None
  of these touch MMIO; they are pure data + scheduler-thread code.

## 5. Step-by-step bring-up checklist

Each step has a "verify" you can read off the serial console.

1. **Build with the right CPU flags.** Edit `Makefile:67` so `CPU_FLAGS`
   matches your SoC (`-mcpu=cortex-a72`, `-mcpu=cortex-a55`, etc.). Build
   with `make TARGET=arm64`. *Verify:* `build/arm64/miniOS_kernel_arm64.elf`
   exists; `aarch64-elf-readelf -h` reports the right Machine and entry
   point.
2. **Decide load address.** Read your bootloader's documentation for the
   kernel-load region in DRAM. Update `hal/arm64/linker.ld` `RAM` origin
   accordingly and rebuild. *Verify:* the ELF entry point matches what
   your bootloader will jump to.
3. **Flash a bootloader.** U-Boot or equivalent, configured to load the
   ELF (or a flat binary you derive from it via `objcopy -O binary`) at
   the chosen address and to pass an FDT pointer in `x0` (Linux boot
   protocol — what `_start` already expects, see `cpu_arm64.S:_start`).
   *Verify:* the bootloader log reaches `Starting kernel ...` (or the
   equivalent for your loader).
4. **Get the early UART working.** Change `UART_BASE` and `IRQ_UART0` in
   `hal/arm64/hal_qemu_arm64.hpp` to your board's values. If the UART is
   not PL011-compatible, replace the `UARTDriver` MMIO routines as well.
   *Verify:* the early-boot prints from `boot_log_core_phase_c`
   (`CLAUDE.md` note 6) appear over serial.
5. **Re-enable WFI.** Add `-DMINIOS_USE_WFI` to `CXXFLAGS` for production
   builds (see §3). *Verify:* idle CPU draw drops; the boot log is
   unchanged.
6. **GICv3 base addresses.** Update `GIC_DISTRIBUTOR_BASE` and
   `GIC_CPU_INTERFACE_BASE` in `hal/arm64/hal_qemu_arm64.hpp` to the
   values from your SoC's TRM (or, eventually, parse them from the FDT).
   *Verify:* the system timer interrupt fires (cycle banner appears in
   step 8).
7. **SMP bring-up.** Try `psci_cpu_on_impl` as-is (HVC) first. If
   secondaries never enter `_secondary_start`, switch the inline asm
   in `kernel/main.cpp:405` from `hvc #0` to `smc #0`. If your bootloader
   provides spin-table release addresses instead, replace
   `release_secondary_cores()` with a write to each cell.
   *Verify:* `[SMP] PSCI_CPU_ON core N rc=0` for each secondary, followed
   by `[boot] core N online`.
8. **Generic timer.** No code change usually needed if the bootloader
   programmed CNTFRQ_EL0. *Verify:* the `[ecN] cycle=250us` banner
   appears, confirming the timer is firing at the expected rate.
9. **Real Ethernet.** Add `hal/shared/<phy>.cpp` mirroring
   `hal/shared/e1000.cpp`. Implement `init_interface`, `send_packet`,
   and the RX callback path defined in `kernel::hal::net::NetworkDriverOps`
   (`hal.hpp`). Wire it into `PlatformQEMUVirtARM64::get_net_ops`.
   *Verify:* `eth0` carries pings to/from the HMI network host.
10. **EtherCAT on the real NIC.** Bind master 0 to your EtherCAT-side NIC
    via the existing net-role routing (`kernel::boot::log_and_route_net_roles`
    in `kernel/main.cpp`). *Verify:* `[ec0] cycle=250us` is steady; slaves
    move to OP within a few seconds.
11. **Real storage for FAT32.** Add a `dwmmc`/`sdhci` shared driver and
    bind it to the existing `fs::BlockReader` adapter (see the
    `VirtioBlkReader` template in `hal_qemu_arm64.hpp:296-304`). *Verify:*
    boot log reports the FAT32 mount succeeding from your eMMC/SD.
12. **Framebuffer.** Bring up your display controller far enough to expose
    a linear RGB(A) framebuffer. Point the existing `DisplayOps`
    implementation at it. *Verify:* the splash screen and operator UI
    render on the panel.
13. **Operator UI smoke test.** From the CLI: `ui_page main`, `ui_dump 1`.
    *Verify:* the page renders without scheduler stalls on the serial log.
14. **Motion smoke test.** Jog one axis from the CLI; run a probing cycle
    against a real probe. *Verify:* deadline-fault counter stays at 0 in
    `ec status`.
15. **30-minute soak.** Continuous CSP cycle, a representative workload.
    *Verify:* p99 cycle time stays well under 1 ms (typically <500 µs on
    real silicon vs. the QEMU CI gate of 5 ms — see §6); the deadline-fault
    counter remains 0.

## 6. Known QEMU compromises to watch on hardware

- **WFI**: see §3 / §5 step 5. Re-enable on real silicon.
- **Cycle-time CI gate**: the current p99 threshold is generous because
  QEMU under WSL2 has wide variance. Real hardware should comfortably hit
  sub-500 µs p99; tighten the gate after you have a baseline.
- **xHCI / PCI probe guard** (commit `eb775a8`): exists because the
  combination of QEMU virtio-blk and qemu-xhci deadlocked. Real silicon
  has a real PCI host bridge, so the guard becomes a no-op and the
  underlying probe runs for the first time. Expect to debug the actual
  xHCI controller path here.
- **Vendor PDO sizes**: Phase B ESI ingest (commit `aad2d45`) lit up the
  per-slave PDO sizing path. On QEMU it is exercised against a fake
  slave; on real hardware it will be exercised against whatever drives
  you connect, so expect to find ESI corner cases the fake never hit.
- **DC-Sync feedback loop**: validated structurally on QEMU; first real
  observation of drift behaviour will happen on hardware. The
  consecutive-miss latch is conservative — review thresholds for your
  topology.

## 7. What is still missing for a production deployment

These are honest gaps, not bugs. None of them block a development
deployment, and all of them want explicit decisions before a fielded
machine.

- **SIL-rated safety chain.** Today the deadline-fault path issues a
  master-driven QuickStop. A safety-rated deployment uses a dedicated
  safety-logic slave (Safety-over-EtherCAT) that is independent of the
  application master. Plan to integrate one before fielded use.
- **Hardware watchdog.** The `WatchdogOps` interface exists but is a
  stub on QEMU. Wire it to your SoC's watchdog and bite if the master
  cycle stalls.
- **Field-replaceable storage.** Operator-friendly setup save/restore
  via a USB stick (and not just SD removal) is not implemented.
- **Multi-language operator UI.** The HMI string table is single-locale
  today.
- **Mains-power management.** E-stop integration with contactor
  drop-out is the integrator's responsibility — miniOS does not yet
  have a coordinated power-down sequence.
- **Field calibration toolchain.** Encoder velocity dump for Bode
  plotting, current-loop tuning UI, etc. are not in the operator UI.

---

If a step here is wrong for your specific SBC, the right fix is almost
always to add a board-flag (e.g. `BOARD=` make variable) that selects
between the QEMU and your-board values, rather than to delete the QEMU
defaults. Both targets are tested in CI and we want to keep that path
green.
