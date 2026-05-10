# miniOS

miniOS is a freestanding C++20 CNC controller kernel that boots on QEMU `virt`
for both `arm64` and `riscv64`. It pairs a small SMP RTOS with an
EtherCAT-oriented motion stack, a TSV-driven operator UI, and a G-code
interpreter.

> The repo directory is `tinyOS`; the project, ELFs, and branding are all
> `miniOS`.

For a rendered gallery of every operator page see **[UI.md](./UI.md)** —
refreshed automatically by the
[UI screenshots](./.github/workflows/ui-screenshots.yml) workflow.

## What's in the tree

- **Kernel** — preemptive SMP scheduler (EDF, up to `MAX_CORES=4`), per-arch
  HALs for QEMU `virt`, FAT32-backed VFS for `setup.cfg` persistence, klog
  ring buffer, freestanding C++20 (`-fno-exceptions -fno-rtti`, `-lgcc` only).
- **EtherCAT** — one or two masters, ESM walk, expedited + segmented SDO
  upload, queued (non-blocking) SDO upload/download, PDO mapping, DC SYNC0
  with closed-loop drift compensation, fake-slave for QEMU bring-up.
- **Motion** — 32-axis kernel, two channels (mill / lathe), CiA-402 drive
  state machine, S-curve trajectories, depth-N look-ahead with
  chord-error-based arc segmentation, per-channel junction-deviation knob,
  load-side feedback (BiSS-C / EL5042), barriers / electronic gearing /
  synchronous moves, power-skiving CSV mode.
- **G-code** — interpreter with mid-program restart (modal-state replay),
  in-RAM MDI line queue driving a second channel, F-word as combined-axis
  path velocity, M62/M63 deferred to motion-queue end, **5-axis TCP**
  (head-kinematics tool-tip → spindle-ref transform).
- **UI** — TSV-driven 1080×1920 operator surface (DRO dashboard, Jog, MDI,
  Program, Offsets, Macros, Probe, Service, network setup, axis detail,
  tool-change wizard, comp surfaces). Software GLES1 renderer with z-buffer,
  back-face cull, perspective-correct interpolation, near-plane clip;
  Wavefront OBJ + MTL and binary/ASCII STL importers. Browser TSV editor at
  [`tools/ui_editor/`](./tools/ui_editor/).
- **HMI** — `eth0` ARP/IPv4/UDP/DHCP service exposing
  `discover` / `status` / `symbol_get` / `symbol_set` over UDP and the
  legacy raw L2 (0x88B5) protocol.
- **Automation** — TSV macro runtime (`M300..M399`), TSV ladder runtime,
  machine-symbol registry shared by macros, ladder, CLI, UI, and HMI.

## Boot status

| Target    | QEMU command                                       | Status                                                        |
| --------- | -------------------------------------------------- | ------------------------------------------------------------- |
| `arm64`   | `qemu-system-aarch64 -M virt -cpu max -smp 4`      | Reaches CLI reliably; primary reference platform.             |
| `riscv64` | `qemu-system-riscv64 -M virt -cpu max -bios none`  | Reaches CLI intermittently (~1 in 10 cold runs); see CLAUDE.md "rv64 boot work in progress". |

Both arches link the same shared kernel above the HAL boundary. The
divergence is intentional and documented as the "arch-parity contract" in
`CLAUDE.md`.

## Build & run

Toolchain (Debian/Ubuntu names):

```bash
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
                 binutils-aarch64-linux-gnu \
                 gcc-riscv64-linux-gnu g++-riscv64-linux-gnu \
                 qemu-system-arm qemu-system-misc gdb-multiarch
```

Build:

```bash
make                       # build/arm64/miniOS_kernel_arm64.elf
make TARGET=riscv64        # build/riscv64/miniOS_kernel_riscv64.elf
make run                   # boot in QEMU, serial -> serial_core0.log
make debug                 # QEMU paused on :1234 + gdb-multiarch
make docs                  # doxygen -> docs/
```

`make run` is non-interactive. To talk to the CLI:

```bash
echo -e "status\n" | qemu-system-aarch64 -M virt -cpu max -smp 4 -m 128M \
    -nographic -kernel build/arm64/miniOS_kernel_arm64.elf
```

For local QEMU launches with networking + sdcard, use
`scripts/qemu_run.sh`.

### UI page capture

```bash
bash scripts/qemu_dump_ui_pages.sh /tmp/ui_captures
```

Drives the CLI over serial, issues `ui_page <id>` + `ui_dump <scale>`, and
writes one `.ppm` and one `.png` per page. The page list is read from
`devices/embedded_ui.tsv`. `scripts/generate_ui_md.py` regenerates `UI.md`
from the captures; the GitHub Actions workflow does both on push to `main`.

## Architecture

Three layers, each depending only on those below:

1. **`core.{hpp,cpp}`** — scheduler, TCBs, per-core ready queues, spinlocks,
   trace, stack accounting. No HAL dependency.
2. **`hal.hpp`** — abstract `Platform` plus `*Ops` interfaces (UART, IRQ,
   timer, DMA, GPIO, watchdog, network, display, input, storage, USB).
3. **Per-arch HAL** — `hal/arm64/` + `cpu_arm64.S` or `hal/riscv64/` +
   `cpu_rv64.S`. Plus shared MMIO drivers under `hal/shared/` (virtio-net /
   gpu / blk / input, e1000, pci, xhci, sdcard).

Above the HAL the controller stack is layered:

- `ethercat/` — raw 0x88A4 framing, ESM, mailbox SDO, PDO, DC, cyclic LRW
- `devices/` — TSV-loaded device DB, PDO maps, OD defaults, embedded blobs
- `motion/` + `cnc/` — drive-facing motion kernel, channels, sync
  primitives, interpreter, MDI service
- `machine/` + `automation/` — symbol registry, macro/ladder runtimes,
  toolpods, probing, compensation
- `hmi/` — `eth0` ARP / IPv4 / UDP / DHCP / HMI control protocol
- `ui/` + `render/` — framebuffer, display backend, input, TSV UI, GLES1
- `fs/` — VFS + FAT32 (read/write — `setup.cfg` survives a reboot)

Boot is bracketed by the `BootPhase` RAII helper so each step prints
`begin` / `ok` markers to the serial log.

## Configuration model (TSV)

Machine config is split across TSV domains — one concern per file:

| File                                | Role                                                                      |
| ----------------------------------- | ------------------------------------------------------------------------- |
| `devices/<slave>.tsv`               | EtherCAT slave identity, SDO init, PDO entries, OD defaults               |
| `devices/embedded_topology.tsv`     | Slave-slot → motion-axis binding per master                               |
| `devices/embedded_signals.tsv`      | Named machine signals over IO / EtherCAT-facing symbols                   |
| `devices/embedded_hmi.tsv`          | `eth0` IP policy (DHCP, static fallback, UDP port)                        |
| `devices/embedded_placement.tsv`    | Thread placement and arm64/rv64 NIC IRQ routing                           |
| `devices/kinematic_*.tsv`           | Render-side axis graph, axis type, parentage, motion binding              |
| `devices/embedded_toolpods.tsv`     | Toolpod geometry, stations, physical/virtual tool IDs                     |
| `devices/embedded_macros.tsv`       | TSV macro sequences (`M300..M399`)                                        |
| `devices/embedded_ladder.tsv`       | Continuous-scan interlocks and derived feedback                           |
| `devices/embedded_ui.tsv`           | Operator pages, widgets, actions, bindings                                |

Configuring a new machine usually means: add device TSVs → bind topology →
name machine signals → describe kinematics → write macros / ladder for
interlocks and tool-change → build the operator workflow in
`embedded_ui.tsv`.

## Runtime placement

Defined in `devices/embedded_placement.tsv`. Default arm64 profile:

- `core0` — boot, CLI, UART I/O, UI thread, interpreter, macro / ladder /
  probe runtimes, bus-config
- `core1` — motion kernel
- `core2` — `ec_a`
- `core3` — `ec_b` or `fake_slave`

NIC routing (arm64):

- `eth0` HMI → core 0
- `eth1` ec_a → core 2
- `eth2` ec_b / fake_slave → core 3

Boot prints the effective NIC-role map, e.g. `net-role ec_a: eth1 -> core2`.
A core only becomes a tickless dedicated RT core when its hosted `ec_*`
thread doesn't share with general-purpose threads — moving `ec_a` onto
`core0` for diagnostics will *not* turn the UI/CLI core dedicated.

On rv64 the PLIC driver applies per-source hart affinity by programming
each source's per-context enable bits.

The placement keys are: `ui_core`, `cli_core`, `uart_io_core`,
`motion_core`, `gcode_core`, `macro_core`, `ladder_core`, `probe_core`,
`bus_config_core`, `ec_a_core`, `ec_b_core`, `fake_slave_core`,
`hmi_irq_core`, `hmi_nic`, `ec_a_nic`, `ec_b_nic`, `fake_slave_nic`.

## EtherCAT master

- Layer-2 raw `0x88A4`, ESM walking Init → PreOp → SafeOp → Op.
- CoE mailbox: expedited SDO download/upload + segmented upload (covers
  0x1008 device name, 0x100A firmware, 0x10F3 diag history, etc.).
  SDO upload is now a non-blocking request queue; tracked SDO download
  has async confirm/abort with timeout.
- One `bus_config` thread per active master (logs are master-qualified —
  `[bus_config m0]`, `[bus_config m1]` — so dual-segment bring-up is
  readable on one console).
- Per-slave bring-up: probe identity (0x1018) → look up in TSV DB → consult
  `embedded_topology.tsv` for role → push TSV `sdo_init` block → program
  per-mode PDO pair (`configure_cia402_axis_for_mode`) → arm DC SYNC0 if
  `dc_assign != 0` → sum E-bus current downstream of EK1100 (refuse SafeOp
  > 2000 mA) → `allow_safeop(true)` only when every check passes.
- DC drift: closed-loop phase-shift compensation per cycle.
- 250 µs servo cycle (CI gate enforces `cycle_us % 250 == 0 && >= 250`).
- In-tree `fake_slave` emulates a ClearPath-EC enough that the whole
  bring-up (including segmented SDO + tuning + 0x10F3 diag) runs in QEMU.

### Topology TSV

`devices/embedded_topology.tsv` is the bridge between a heterogeneous
EtherCAT bus and the motion kernel. Each `binding` record names:

- `role`: `servo` / `coupler` / `din` / `dout` / `ain` / `aout` /
  `encoder` / `probe` / `sensor`
- `master`: `0` / `1` or `ec_a` / `ec_b`
- `slave`: discovery-order slot on that master
- `axis`: logical motion axis for servo-bound slots
- optional `axis_name`, `channel`, `name`

```tsv
binding role=coupler master=0 slave=0 name=main_coupler
binding role=din     master=0 slave=1 name=machine_din
binding role=servo   master=0 slave=5 axis=0  axis_name=X channel=0 name=x_servo
binding role=servo   master=1 slave=0 axis=16 axis_name=C channel=1 name=main_spindle
```

### Supported devices

Drop a TSV in `devices/`, list it in `devices/embedded_blob.S`, and tag
`type=servo|coupler|encoder|digital_input|digital_output|analog_input|analog_output`.

- **Teknic ClearPath-EC** — CiA-402 servo. Default target -Exxx
  (51200 cpr); -Rxxx (12800 cpr) also supported.
- **Beckhoff EK1100/1101/1110/1122 + EP1122** couplers — passive ESM walk,
  skip SDO/PDO/DC.
- **Beckhoff EL5042** BiSS-C 2-channel encoder — load-side feedback.
  Pairs with ClearPath at `AssignActivate=0x0300` so TxPDO samples are
  captured at the same SYNC0 edge.
- **Beckhoff EL1809** 16-channel 24 V DI (no-mailbox terminal).
- **Beckhoff EL2809** 16-channel 24 V / 0.5 A DO (no-mailbox terminal).
- **Beckhoff EL3162** 2-channel 0..10 V 16-bit AI (CoE-capable).
- **EL7211-9014 / EL5001 / EL5151 / EL5112 / EL5122 / EL5131** — TSVs
  shipped under `devices/`.

## Motion kernel

- 32 axes (`MAX_AXES=32`), two channels (`mill` axes 0..15, `lathe`
  axes 16..31). `Kernel::set_topology` rewrites bindings at runtime
  (validated: no axis in two channels, no empty channel).
- CiA-402 drive state machine with per-mode controlword masking and a
  two-step servo-on handshake guard.
- S-curve trajectory generator (integer maths in counts/s, /s², /s³),
  per-axis tunable `vmax_cps`, `accel_cps2`, `jerk_cps3`.
- **Look-ahead** — depth-N chain ring (was depth-1). Arc dispatcher fills
  the chain instead of one segment per tick. Arc segmentation is by
  chord-error tolerance, not arc length. Per-channel junction-deviation
  knob.
- **5-axis TCP** — head-kinematics transform from tool tip → spindle ref.
  F-word measures tool-tip path velocity when TCP is on. Tool length is
  an XYZ vector (not Z-only scalar).
- Composable homing (direction / edge / trigger / backoff). Torque-threshold
  trigger is first-class, so ClearPath hardstop methods (−1 / −2) compose
  naturally. Post-homing auto-push of `0x607D` software limits.
- Stop matrix — five independent stop actions (QuickStop / Disable / Halt /
  FaultReaction / Abort), each mapping to a configurable StopKind;
  brake-engage latency honoured.
- Load-side feedback (`Axis::LoadFeedback`): int64 raw position, rational
  scale/sign/offset, integer PI with anti-windup, slew-limited decay on
  invalidity. EL5042 errors propagate into `Axis::fault_latched`. Faults
  surface `0x603F` + decoded text via `cia402::error_text` /
  `cia402::diag_text` (52-entry TextId table from the Teknic ESI).
- Power skiving — differential CSV mode (`set_axis_velocity`), spindle
  indexer for tooth counting, feed-per-tooth.

### Sync primitives

Three integer-maths primitives, each same-cycle sample → command for
sub-count phase-lock:

- **Barrier rendezvous** — per-channel stable-cycle convergence check vs
  `|cmd − actual| ≤ tolerance_counts`, atomic cross-channel release.
- **Electronic gearing** — `follower.cmd = follower_base + (k_num *
  (leader.actual − leader_base)) / k_den`, ramped engage/disengage,
  int64 intermediate math.
- **Synchronous coordinated move** — computes `T_final` from the slowest
  participating axis, rescales every other axis's effective `vmax` so they
  arrive together, auto-posts a barrier across every channel that owns a
  participating axis.

Per-channel feedhold and override (feed / rapid / spindle permille) are
isolated by default. Cross-channel fault propagation is **explicit**, only
via an active sync primitive.

### Mid-program restart

`cnc::interp::Runtime::restart_at_line(channel, line_no)` reloads the
program on `channel`, then does a **motion-free** scan of lines
`0..line_no-1` to reconstruct modal state (absolute / inch / plane,
motion mode, feed, spindle direction+speed intent, active WCS, active
tool, tool-length comp, coolant, axis targets). Axis motion, homing,
barriers, macros, dwells, and physical M3/4/5 are deliberately skipped.
The channel is left in `Ready` at `line_no`; the operator presses Cycle
Start to begin.

This matches the two-pass "sequence-number search / block restart"
contract used by Fanuc / Siemens / Haas. The Program page exposes it as a
`GOTO LINE` input + an `AT BLOCK` readback.

### MDI backend

`cnc/mdi.{hpp,cpp}` is an in-RAM MDI service with a 32-slot history ring
and 8-slot execution FIFO. `Service::submit(line)` enqueues a line;
`Service::tick()` drains the head by writing a throwaway program (line +
`M30`) and calling `cnc::interp::Runtime::start(1)` on channel 1. That
reuses the full G-code parser and motion pipeline. Status transitions
Queued → Running → Idle automatically when channel 1 completes.
Bind keys: `mdi:input`, `mdi:last`, `mdi:status`, `mdi:message`,
`mdi:depth`. Actions: `mdi:submit`, `mdi:clear`, `mdi:abort`.

## UI framework

- **Framebuffer** (`ui/fb.{hpp,cpp}`) — 1080×1920 RGBA, 8×16 bitmap font.
  `draw_text_scaled` blits each glyph pixel as a `scale`×`scale` block
  (used for DRO hero digits).
- **TSV builder** (`ui/ui_builder_tsv.cpp`) — pages, widgets, focus,
  bindings, actions, `include page=<id>` templates, `scale=<1..4>`,
  `active_if=<bind>:<int>` conditional highlight.
- **Display backend** (`ui/display.cpp`, `hal/shared/virtio_gpu.*`) —
  virtio-gpu scanout, deferred present (skip when fb is clean,
  heartbeat every 1 s).
- **Input** (`hal/shared/virtio_input.*`) — touch, pointer, keyboard,
  wheel.
- **Operator API** (`ui/operator_api.cpp`) — machine snapshot (cmd / act /
  dtg / homed per axis, WCS / units / block / runtime / parts / selected
  axis / jog increment), MDI helpers, mid-program-restart entry points.
- **Software GLES1 renderer** (`render/`) — Phong lighting, Z-buffer,
  back-face cull, viewport AABB cull, perspective-correct attribute
  interpolation, Sutherland-Hodgman near-plane clip. OBJ + MTL
  importer with per-`usemtl` material colour; binary + ASCII STL
  auto-dispatched by file shape.

The default operator surface (in `devices/embedded_ui.tsv`):

- **`bottom_nav`** — 6-tile persistent nav included on every page
  (Jog · MDI · Program · Offsets · Macros · Service).
- **`dashboard`** — 4-axis DRO with scale-3 commanded digits + per-axis
  `act` / `dtg` / `homed`. Cycle cluster (START / HOLD / RESET / HOME)
  + isolated oversized E-STOP. Status strip with alarms / mode / WCS /
  units / runtime.
- **`jog`** — manual surface, axis selector, jog-increment selector
  (0.001 / 0.01 / 0.1 / 1 mm), feed-override slider, JOG-/JOG+, full
  axis-status panel.
- **`mdi`** — editable G-code line, SUBMIT / CLEAR / ABORT, status /
  depth / last / message strip, history panel.
- **`program`** — preview, PREV / NEXT / REBUILD / CYCLE, GOTO LINE
  driving `restart_at_line`.
- **`offsets`** — 6 WCS × X/Y/Z/A and 8 tools × length/radius/wear, with
  `active_if` highlight on the current WCS / tool.
- **`probe`** — Renishaw cycles (stylus qualify, Z surface, X/Y edge,
  bore XY, 3D pocket, reference sphere). ACCEPT writes into the WCS.
- **`macros`** — macro browser/runner.
- **`service`** — alarms, setup load/save, diagnostics, PROBE CYCLES.
- **Comp surfaces** — PEC / Geometry / Volumetric.
- **Network setup**, **axis detail**, **tool-change wizard**.

The UI is pinned to `core0` by default. Only EtherCAT/NIC placement is
meant to be adjusted per deployment.

### TSV UI editor

Hosted at <https://agentdavo.github.io/tinyos/> (built from `main` by
[`.github/workflows/pages.yml`](./.github/workflows/pages.yml)).
`tools/ui_editor/index.html` is a single-file browser app (vanilla JS,
no build step, works offline) for editing `devices/embedded_ui.tsv`:

- 1080×1920 portrait canvas, snap-to-grid (10 px), zoom 25 %–200 %.
- Drag-drop palette (label / button / panel / container / slider /
  input / progress / image / graph).
- Inspector with grouped `bind=` / `action=` dropdowns sourced from the
  kernel binder enum, plus dedicated editors for `scale=` and
  `active_if=`.
- File System Access API (with a blob-download fallback for Firefox /
  Safari), `localStorage` auto-save, 50-step undo / redo.
- Round-trip fidelity: opening and saving back produces a byte-identical
  file (field order, comments, blank lines, alias names all preserved).

Bind / action / shortcut catalogues: see
[`tools/ui_editor/README.md`](./tools/ui_editor/README.md).

## Network

- **virtio-net** — default QEMU NIC for EtherCAT transport / loopback.
- **e1000** — Intel NIC for passthrough; up to 3 NICs via
  `init_e1000_nic(idx, mmio_base, irq)`.
- The masters consume `NetworkDriverOps` directly. The legacy `net.cpp`
  socket layer has been removed; the controller datapath does not go
  through it.

### Controller network split

Default three-NIC layout:

- `eth0` — HMI / diagnostics
- `eth1` — EtherCAT master `ec_a`
- `eth2` — EtherCAT master `ec_b` or in-kernel `fake_slave`

`eth0` runs the HMI service ([`hmi/hmi_service.cpp`](./hmi/hmi_service.cpp)):
ARP, IPv4, UDP, DHCP client with static fallback from
`devices/embedded_hmi.tsv`, plus the original raw L2 (`0x88B5`) protocol.
The HMI control surface exposes `discover` / `status` / `symbol_get` /
`symbol_set` — everything goes through the same machine-symbol layer used
by macros, ladder, and the operator UI.

```tsv
hmi key=dhcp_enable      value=1
hmi key=static_ip        value=10.0.2.15
hmi key=netmask          value=255.255.255.0
hmi key=gateway          value=10.0.2.2
hmi key=udp_port         value=5000
hmi key=dhcp_timeout_ms  value=3000
```

`scripts/qemu_run.sh` forwards the HMI UDP port from guest to host —
host port `QEMU_HMI_HOST_PORT` (default = guest port), guest port
`QEMU_HMI_GUEST_PORT` (default `5000`). `scripts/hmi_udp_probe.py` and
`scripts/test_hmi_udp.sh` exercise the live UDP round-trip.

### EtherCAT signal binding

`devices/embedded_signals.tsv` supports rich sources, not just coarse
`ec.di` / `ec.do`:

- `ec.statusword`, `ec.controlword`
- `ec.position_actual`, `ec.velocity_actual`
- `ec.error_code`, `ec.drive_mode`
- `ec.digital_inputs`, `ec.digital_outputs`
- `ec.tx_bit` / `ec.rx_bit`
- `ec.tx_s16` / `ec.tx_u16` / `ec.tx_s32` / `ec.tx_u32`
- `ec.rx_s16` / `ec.rx_u16` / `ec.rx_s32` / `ec.rx_u32`
- `ec.tx_pin` / `ec.rx_pin` — resolves against device-DB PDO `pin=` names

```tsv
signal name=x_scale_raw      source=ec.tx_pin          master=0 slave=4 pin=ch1_position_compact value_type=int
signal name=x_servo_status   source=ec.statusword      master=0 slave=5 value_type=int
signal name=main_spindle_pos source=ec.position_actual master=1 slave=0 value_type=int
signal name=x_servo_outputs  source=ec.digital_outputs master=0 slave=5 value_type=int writable=1
```

## TSV macros & ladder

Three layers, each with a TSV interface:

- **Symbol registry** — typed named values (`door_closed`,
  `drawer_open_cmd`, `probe_tripped`, `cycle_inhibit`, `axis_x_pos`).
  Built-ins live in [`machine/machine_registry.cpp`](./machine/machine_registry.cpp).
- **Macro runtime** — sequenced "set output, wait for feedback, move
  axis, fault on timeout" in [`automation/macro_runtime.cpp`](./automation/macro_runtime.cpp).
  `M300..M399` are reserved for TSV macros; the interpreter waits in
  `WaitingMacro` until the macro completes or faults.
- **Ladder runtime** — continuous-scan logic for interlocks and derived
  feedback in [`automation/ladder_runtime.cpp`](./automation/ladder_runtime.cpp).

Step types: `set` / `wait` / `delay` / `move` / `alarm`.
Compare ops: `eq` / `ne` / `lt` / `le` / `gt` / `ge`.

```tsv
# devices/embedded_macros.tsv
symbol name=drawer_open_cmd type=bool initial=0 writable=1
symbol name=drawer_open_fb  type=bool initial=0 writable=1

macro id=drawer_open title=Drawer Open mcode=300
step  type=set    symbol=drawer_close_cmd value_type=bool value=0
step  type=set    symbol=drawer_open_cmd  value_type=bool value=1
step  type=wait   symbol=drawer_open_fb   value_type=bool compare=eq value=1 timeout_ms=1000
```

```tsv
# devices/embedded_ladder.tsv
rung id=door_interlock     type=bool lhs=door_closed     compare=eq rhs=0 out=cycle_inhibit  true=1 false=0
rung id=drawer_open_feedback type=bool lhs=drawer_open_cmd compare=eq rhs=1 out=drawer_open_fb true=1 false=0
```

Use ladder for continuous safety logic (door interlocks, cycle inhibit,
feedback synthesis, latched state). Use macros for sequenced flows
(drawer / clamp / tool change / probe). Always wait on a feedback symbol
with a timeout, never assume an output succeeded.

CLI verbs: `macro_ls`, `macro_run <id>`, `macro_stop`, `symbols`,
`symbol_set <name> <value>`, `ladder_ls`.

## CLI reference

One-shot dashboard:

- `status` — masters, slaves, DB, channels, barriers, gears, motion
  stats, axis positions, fake_slave snapshot.

EtherCAT:

- `ec` / `ec_slaves` / `ec_hist` — master state / slave list / latency
  histograms
- `ec_scan [start=0x1001] [count=8]` — probe identity over a range
- `ec_abort` — broadcast AL=Init (emergency bring-down)
- `ec_watchdog <timeout_ms> [station]` — program SM watchdog
- `ec_sdo_read <idx> <sub> [station]` — expedited SDO upload
- `ec_sdo_read_str <idx> <sub> [station]` — segmented SDO upload
- `ec_identity` / `ec_encoder` / `ec_dc` / `ec_safety` / `ec_tuning` /
  `ec_diag_dump` `[station]`
- `ec_probe` / `ec_probe_arm <mask>` — touch-probe capture
- `ec_home_methods` / `ec_home_params` / `ec_home_run` — homing
- `ec_sw_limits <neg> <pos>` — `0x607D` soft limits
- `ec_foldback` — Move Done Torque Foldback
- `ec_eeprom_save` — `0x1010:01 ← 'save'` (persist NVM)
- `ec_allow_safeop` — manual override for the PreOp→SafeOp gate (bench
  tool)

Motion:

- `channels` — list channels + axis ownership
- `topology <name>=<ax,ax,...> [...]` — rewrite axis-to-channel binding
- `move <axis> <pos>` / `mpos` — single-axis move + per-axis dump
- `motion_enable <axis>` / `motion_disable <axis>`
- `sync_move <mask> <ax> <tgt> [...]` — cross-channel coordinated move
- `fault_reset <axis>` / `fault_inject` — recovery / fake-slave injection
- `feedhold <ch> <0|1>` / `override <ch> <feed|rapid|spindle> <permille>`
- `barriers` / `barrier_arrive <ch> <token> <parts>`
- `gears` / `gear_engage` / `gear_disengage`
- `axis_spin <axis> <cps>` — simulated leader for gear-harness testing
- `axis_load_configure` / `axis_load_sample` / `axis_load_calibrate`
- `axis_sw_limits` / `axis_autodetect` / `axis_tune`
- `set_torque_limit <axis> <permille>` — CiA-402 0x6072

Power skiving:

- `skiving_config <ts1> <teeth> <c1_cpr> [offset]`
- `skiving_engage <c1_rpm> <ts1_rpm> <z_axis> <teeth>`
- `feed_per_tooth <diameter> <teeth>`
- `c1_index_sync` / `multipass` / `din6`

Diagnostics / infra:

- `version` — kernel + git hash + build stamp
- `meminfo` — pool usage + per-thread stack watermarks
- `klog` — recent UART output ring
- `setup_save` / `setup_load` — persist machine setup to FAT32
  `setup.cfg`
- `save_cfg` — dump current operator state as replayable CLI commands
- `trace` / `stats` / `top` / `ttop` — scheduler / trace buffer
- `rt` / `rt_reset` / `budget` — RT jitter telemetry
- `devices` — TSV-loaded device DB inventory
- `ui_page <id>` / `ui_dump [scale]` — switch / capture an operator page
- `test` — built-in subtest suite (status / motion / ec / devices)

## Persistent storage

QEMU usage:

```bash
-device sd-card,id=sd0,file=sdcard.img
```

`fs/fat32` is read/write so `setup.cfg` survives a reboot. The CLI verbs
are `setup_save` / `setup_load`.

## Memory layout

Linker scripts `hal/arm64/linker.ld` / `hal/riscv64/linker.ld` place
`.text` at `0x40000000` (arm64) or `0x80200000` (rv64), `.data` adjacent.
CI asserts the arm64 addresses via readelf; changing them breaks the
ELF-integrity check.

## Verification

CI greps the serial log for:

- `miniOS CLI ready` — boot reached the CLI thread
- `Tests completed:.*0 failed` — built-in suite passed
- `[ecN] cycle=<us>us` — each master at exactly 250 µs

Smoke scripts:

- `bash scripts/test_multi_nic_smoke.sh`
- `bash scripts/test_signal_bindings.sh`
- `bash scripts/test_hmi_udp.sh`

Static analysis (matches CI):

```bash
bear -- make clean && make
clang-tidy -p compile_commands.json <file>
```

## Hardware bring-up

For a real arm64 SBC port, see [HARDWARE_PORT.md](./HARDWARE_PORT.md) —
it walks through what's QEMU-only, what's real-hw-ready, and a 15-step
bring-up checklist.

## License

`SPDX-License-Identifier: MIT OR Apache-2.0` on every source file.
Contributions under the same dual licence.
