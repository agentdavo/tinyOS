# miniOS

miniOS is a bare-metal CNC controller kernel in C++20. The current tree contains:

- a preemptive SMP scheduler and arch HALs for QEMU `virt` on `arm64` and `riscv64`
- one or two EtherCAT masters, CiA-402 drive models, PDO / SDO / DC bring-up, and a device database loaded from TSV
- a 32-axis motion kernel with channels, barriers, gearing, homing, compensation scaffolding, and probe / toolpod / macro / ladder runtimes
- a G-code interpreter with mid-program restart (motion-free modal-state replay) and an in-RAM MDI line queue that drives a second interpreter channel
- a TSV-driven operator UI (portrait 1080×1920) with a DRO-hero dashboard, persistent 6-tile bottom nav, MDI page, Offsets table, Jog surface with axis/increment selectors, and an E-STOP front and centre on the cycle cluster
- a standalone browser-based TSV UI editor under `tools/ui_editor/` with drag-drop, live validation against the kernel's bind/action enums, and byte-for-byte round-trip of `devices/embedded_ui.tsv`
- a software GLES1 renderer for machine / toolpath preview, touch / keyboard / mouse plumbing, and serial CLI control

The repo directory is `tinyOS`, but the project and generated ELFs are `miniOS`.

## Build status

Both main targets build. The `arm64` path is still the primary reference platform, but `riscv64` now boots the shared UI / CLI / motion / EtherCAT-oriented stack rather than a standalone scheduler demo.

```bash
make
make TARGET=riscv64
make run
```

## Controller architecture

At a high level the controller is built from seven layers:

1. `core.{hpp,cpp}`: scheduler, TCBs, per-core ready queues, spinlocks, trace, and stack accounting.
2. `hal.hpp` plus `hal/arm64/` or `hal/riscv64/`: UART, timer, IRQ routing, NIC discovery, display/input drivers, and CPU bring-up.
3. `ethercat/`: raw 0x88A4 framing, AL state machine, mailbox SDO upload/download, PDO mapping, DC configuration, and per-master cyclic LRW exchange.
4. `devices/` + `devices/device_db.*`: TSV-loaded device descriptions, PDO maps, SDO init blocks, OD defaults, and embedded config blobs.
5. `motion/` + `cnc/`: drive-facing motion kernel, channels / sync primitives, offsets, program store, interpreter (with motion-free mid-program restart), and the MDI line queue (`cnc/mdi.cpp`).
6. `machine/` + `automation/`: machine signals, custom machine logic, automation, pod/tool storage, probing, and compensation.
7. `hmi/`: `eth0` HMI/IP service with ARP, IPv4, UDP, DHCP/static config, plus the machine-registry-backed HMI control protocol.
8. `ui/` + `render/`: framebuffer, display backend, input integration, TSV UI, and 2D/3D operator rendering.

## Runtime placement

Runtime placement is now configured in `devices/embedded_placement.tsv` rather than
being hardcoded in the boot path. The default arm64 profile is:

- `core0`: boot, CLI, UART I/O, UI thread, interpreter, macro runtime, ladder runtime, probe runtime, bus-config
- `core1`: motion kernel
- `core2`: `ec_a`
- `core3`: `ec_b` or `fake_slave`
- `eth0`: HMI / user-network NIC
- `eth1`: EtherCAT master A NIC
- `eth2`: EtherCAT master B or fake-slave NIC

The default NIC IRQ routing on arm64 matches that placement:

- `eth0` / virtio-net index 0 -> core 0
- `eth1` / virtio-net index 1 -> core 2
- `eth2` / virtio-net index 2 -> core 3

The authoritative knobs are:

- `devices/embedded_placement.tsv` for runtime thread and NIC IRQ placement
- [`machine/runtime_placement.cpp`](/home/djs/tinyOS/machine/runtime_placement.cpp:1) for the parser / defaults
- [`kernel/main.cpp`](/home/djs/tinyOS/kernel/main.cpp:203) for the shared boot-path application of placement
- [`hal/arm64/hal_qemu_arm64.cpp`](/home/djs/tinyOS/hal/arm64/hal_qemu_arm64.cpp:652) for arm64 NIC IRQ routing support

The current placement keys are:

- `ui_core`, `cli_core`, `uart_io_core`
- `motion_core`, `gcode_core`, `macro_core`, `ladder_core`, `probe_core`
- `bus_config_core`, `ec_a_core`, `ec_b_core`, `fake_slave_core`
- `hmi_irq_core`
- `hmi_nic`, `ec_a_nic`, `ec_b_nic`, `fake_slave_nic`

At boot the kernel now prints the effective NIC-role map, for example:

- `net-role hmi: eth0 -> core0`
- `net-role ec_a: eth1 -> core2`
- `net-role ec_b: eth2 -> core3`

Important detail: dedicated tickless EC-core behavior is also placement-driven now. A
core only becomes a dedicated RT core when it hosts `ec_a` / `ec_b` / `fake_slave`
and no general-purpose runtime threads share it. Moving `ec_a` onto `core0` for a
diagnostic profile will therefore move the thread and NIC IRQ there, but it will not
incorrectly turn the UI/CLI core into a tickless dedicated core.

On rv64 the thread affinity is placement-driven too, and the PLIC driver now applies
per-source hart affinity by programming each source's per-context enable bits. The boot
log reports PLIC distributor setup, per-hart context setup, and final NIC routing such
as `"[rv64] plic route nic0 irq=0x8 hart_mask=0x4"`.

## Configuration model

Machine configuration is intentionally split across TSV domains rather than one monolithic file:

- `devices/*.tsv`: EtherCAT slave identity, SDO init, PDO entries, OD defaults
- `devices/embedded_topology.tsv`: explicit EtherCAT role ownership and servo-to-axis binding per master/slot
- `devices/embedded_signals.tsv`: named machine signals mapped onto IO / EtherCAT-facing symbols
- `devices/embedded_hmi.tsv`: HMI NIC policy such as DHCP enable, static fallback IP, netmask, gateway, and UDP port
- `hmi/hmi_service.cpp`: `eth0` HMI transport, ARP/IPv4/UDP/DHCP handling, and the registry-backed HMI control protocol
- `devices/embedded_placement.tsv`: runtime thread placement and arm64 NIC IRQ routing
- `devices/embedded_hmi.tsv`: `eth0` IP policy and UDP HMI port
- `devices/kinematic_*.tsv`: render-side axis graph, axis type, direction, parentage, and motion-axis binding
- `devices/embedded_toolpods.tsv`: toolpod geometry, stations, physical tool IDs, virtual tool IDs
- `devices/embedded_macros.tsv`: custom machine sequences and `M300..M399` macros
- `devices/embedded_ladder.tsv`: continuous interlocks and derived logic
- `devices/embedded_ui.tsv`: operator pages, widgets, actions, bindings

That split is deliberate:

- EtherCAT device TSV configures the bus and PDO image.
- Topology TSV decides which discovered slave slot owns which logical motion axis.
- Signal TSV gives stable machine names to IO and control bits.
- The HMI service reads and writes those same named symbols, so UI/automation/network control share one contract.
- Macro / ladder TSV defines machine behavior.
- Kinematic / toolpod / UI TSV defines what the operator sees and how the machine is presented.

## How to configure a machine

For a new machine, the usual order is:

1. Add or update device TSVs in `devices/` for every EtherCAT slave on the bus.
2. Ensure the device gets embedded and loaded at boot.
3. Define bus ownership in `devices/embedded_topology.tsv` so couplers, IO slices, encoders, and servos are no longer inferred from slave order.
4. Bind machine-facing names in `devices/embedded_signals.tsv`.
5. Define the machine render model in `devices/kinematic_*.tsv` and, if needed, tool storage in `devices/embedded_toolpods.tsv`.
6. Add macros and ladder logic for interlocks, tool change, probing, drawer/door/clamp handling, and custom M-codes.
7. Build the operator workflow in `devices/embedded_ui.tsv`.

In practice the most important files are:

- `devices/clearpath_ec.tsv`, `devices/beckhoff_ek.tsv`, and other slave TSVs
- `ethercat/bus_config.cpp`
- `devices/embedded_topology.tsv`
- `machine/machine_registry.cpp`
- `devices/embedded_signals.tsv`
- `devices/embedded_placement.tsv`
- `devices/embedded_macros.tsv`
- `devices/embedded_ladder.tsv`
- `devices/embedded_toolpods.tsv`
- `devices/embedded_ui.tsv`

## HAL extension points

The HAL still uses ops tables, so platform support is added by implementing the relevant interface in `hal/shared/` or in an arch-specific platform:

- input: `InputOps`
- storage: `StorageOps`
- display: `DisplayOps`
- USB host: `USBHostControllerOps`
- networking: `net::NetworkDriverOps`
- timers / IRQ / DMA / GPIO / watchdog in `hal.hpp`

## Build & run

```
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu \
                 binutils-aarch64-linux-gnu \
                 gcc-riscv64-linux-gnu g++-riscv64-linux-gnu \
                 qemu-system-arm qemu-system-misc gdb-multiarch
make                                   # builds build/arm64/miniOS_kernel_arm64.elf
make TARGET=riscv64                    # builds build/riscv64/miniOS_kernel_riscv64.elf
make run                               # boot in QEMU, serial to serial_core0.log
```

Build outputs live under `build/<target>/`, so the default ARM64 ELF is
`build/arm64/miniOS_kernel_arm64.elf` and the rv64 ELF is
`build/riscv64/miniOS_kernel_riscv64.elf`.

Use `scripts/qemu_run.sh` as the canonical direct QEMU runner for both
targets. The older one-off `scripts/run*.sh` wrappers have been removed.

`make run` is non-interactive — serial goes to `serial_core0.log`. For CLI interaction use `-nographic` and pipe input:
```bash
echo -e "status\n" | qemu-system-aarch64 -M virt -cpu max -smp 4 -m 128M -nographic -kernel build/arm64/miniOS_kernel_arm64.elf
```

### UI page capture

The TSV UI can be switched and dumped from the guest CLI. The current page
catalogue is `dashboard`, `jog`, `mdi`, `program`, `offsets`, `service`,
`macros`, `probe`, plus the shared `bottom_nav` template included on every
real page.

```bash
ui_page dashboard
ui_page mdi
ui_page offsets
ui_dump 6
```

- `ui_page <id>` switches the active TSV page and forces a one-shot render.
- `ui_dump [scale]` emits a binary `P6` PPM stream framed by `UI_DUMP_BEGIN ...` / `UI_DUMP_END`.
- `scripts/qemu_dump_ui_pages.sh <out-dir>` boots QEMU with `-nographic`, drives those CLI commands over serial, and writes one `.ppm` and one `.png` per page. The `PAGES` tuple at the top of that script was updated to match the current operator surface.

This is the preferred screenshot path for design review. It avoids poking the renderer through gdb and captures exactly what the guest framebuffer contains.

For design iteration without booting the guest, use the browser editor
under `tools/ui_editor/` (see the "TSV UI editor" section below).

## Kernel

- C++20, freestanding, `-fno-exceptions -fno-rtti`, links `-lgcc` only.
- Three-layer stack: `core.{hpp,cpp}` (scheduler, spinlocks, TCBs), `hal.hpp` (abstract `Platform`), arch HAL (`hal/arm64/` or `hal/riscv64/`).
- SMP-aware EDF scheduler, up to `MAX_CORES=4`. Per-core ready queues, priority 0..15, cooperative yield.
- 250 µs servo cycle (CI gate enforces `cycle_us % 250 == 0 && >= 250`).
- Stack painting + `meminfo` CLI surface watermarks per thread.
- `version` CLI carries the git hash (Makefile `GIT_HASH` + `+dirty` suffix).

## EtherCAT master

- Layer-2 raw 0x88A4 framing, ESM state machine walking Init→PreOp→SafeOp→Op.
- CoE mailbox: **expedited SDO download + upload** (single-slot blocking) and **segmented SDO upload** (multi-segment with toggle; covers 0x1008 device name, 0x100A firmware, 0x10F3:N diag entries, etc.).
- One `bus_config` thread per active master between PreOp and SafeOp:
  - logs are master-qualified (`[bus_config m0]`, `[bus_config m1]`) so dual-segment bring-up is readable over one serial console
  - probes identity via 0x1018 (`probe_slave_identity`) and looks the device up in the TSV DB by observed VID/PID
  - consults `devices/embedded_topology.tsv` to decide whether the discovered slot is a servo axis, coupler, encoder, or IO slice
  - dispatches by `DeviceType` — Coupler / Servo / EncoderInput / DigitalInput / DigitalOutput / AnalogInput / AnalogOutput. No-mailbox terminals skip SDO.
  - re-hooks servo slots onto their configured logical axis instead of assuming `slave index == axis index`
  - pushes the TSV `sdo_init` block (fault reaction 0x605E, following error 0x6065/6066, overspeed 0x231A, brake delay 0x2170, ITP 0x60C2, torque limits 0x6072/60E0/60E1)
  - programs the per-mode PDO pair via `configure_cia402_axis_for_mode` — CSP→0x1601/0x1A01 per ESI
  - arms Distributed Clocks SYNC0 via 4 FPWRs to 0x0980 / 0x09A0 / 0x09A4 / 0x09A8 when the device's `dc_assign` is non-zero
  - sums E-bus current draw downstream of EK1100 and refuses SafeOp if it exceeds 2000 mA
  - flips `allow_safeop(true)` only when every check passes; otherwise the bus stays at PreOp with `identity_mismatch` set
- `cycle_lrw` pack/unpack is offset-driven from the `PdoLayout` computed for each slave's active mapping. 0x60FD / 0x60FE:1 digital I/O carried as u32 per spec. Fault code 0x603F visible to motion.
- `Master::broadcast_init_state`, `configure_sm_watchdog`, `run_homing_sequence`, `push_software_limits`, `configure_dc_sync0`, `push_homing_params`, `probe_homing_methods`, `probe_encoder_resolution`, `upload_sdo_segmented` all exposed as CLI handles.
- In-tree `fake_slave` emulates enough of a ClearPath-EC — mailbox SDO (expedited + segmented), CiA-402 FSA, CSP actual-position echo, DI/O mirror, canned responses for 0x1008 / 0x1018 / 0x608F / 0x60E3 / 0x1C32 / 0x1C33 / tuning block / 0x10F3 diag entry — so the whole bring-up is exercisable in QEMU.

### Topology TSV

`devices/embedded_topology.tsv` is the bridge between a heterogeneous EtherCAT bus and the motion kernel. Each `binding` record names:

- `role`: `servo`, `coupler`, `din`, `dout`, `ain`, `aout`, `encoder`, `probe`, `sensor`
- `master`: `0` / `1` or `ec_a` / `ec_b`
- `slave`: discovery-order slot on that master
- `axis`: logical motion axis for servo-bound slots
- optional `axis_name`, `channel`, and `name`

Example:

```tsv
binding	role=coupler	master=0	slave=0	name=main_coupler
binding	role=din	master=0	slave=1	name=machine_din
binding	role=servo	master=0	slave=5	axis=0	axis_name=X	channel=0	name=x_servo
binding	role=servo	master=1	slave=0	axis=16	axis_name=C	channel=1	name=main_spindle
```

This is now the authoritative way to express "mixed bus, but only these slots are motion axes".

## Supported devices (TSV-driven)

- **Teknic ClearPath-EC** — CiA-402 servo. Default target variant is **-Exxx (51200 cpr)**; -Rxxx (12800 cpr) also supported.
- **Beckhoff EK1100 / EK1101 / EK1110 / EK1122 / EP1122** couplers — passive, walk the ESM, skip SDO/PDO/DC config.
- **Beckhoff EL5042 BiSS-C** 2-channel encoder interface — load-side feedback for dual-loop motion control. Pairs with ClearPath at `AssignActivate=0x0300` so TxPDO samples are captured at the same SYNC0 edge.
- **Beckhoff EL1809** — 16-channel 24 V digital input, 3 ms filter. No-mailbox terminal (fixed PDO 0x1A00..0x1A0F).
- **Beckhoff EL2809** — 16-channel 24 V / 0.5 A digital output. No-mailbox terminal (fixed PDO 0x1600..0x160F).
- **Beckhoff EL3162** — 2-channel 0..10 V 16-bit analog input. CoE-capable, sdo_init block configures per-channel filter + scaling (0x8000 / 0x8010).
- **Additional devices** (TSV files in `devices/`):
  - **EL7211-9014** — 4.5A OCT servo drive
  - **EL5001** — SSI encoder interface
  - **EL5151** — Incremental encoder
  - **EL5112** — 2-channel incremental encoder
  - **EL5122** — 2-channel TTL encoder
  - **EL5131** — Incremental encoder with 24V output

Adding a new terminal is a matter of dropping a TSV in `devices/`, listing it in `devices/embedded_blob.S`, and tagging `type=servo|coupler|encoder|digital_input|digital_output|analog_input|analog_output` in the device line.

## Kinematic Configuration (TSV-driven)

Machine visualization uses TSV config files to define axis chains:

- **devices/kinematic_mill3.tsv** - 3-axis mill (X, Y, Z, spindle)
- **devices/kinematic_millturn.tsv** - mill-turn with 2 channels (X,Y,Z on ch0, C,B,spindle on ch1)

**TSV Format:**
```
name,type,parent,dir_x,dir_y,dir_z,off_x,off_y,off_z,min,max,mesh,channel
X,Linear,base,1,0,0,0,0,0,0,1000,box,0
C,Rotary,Z,0,0,1,0,0.1,0,0,360,table,1
```

Each axis can reference an OBJ file via the `obj_file` field in the kinematic model.

## UI Framework

- **Framebuffer** (`ui/fb.hpp/cpp`): 1080x1920 RGBA with 8x16 bitmap font. `draw_text_scaled(x, y, s, fg, bg, scale)` blits each glyph pixel as a `scale`×`scale` block, used for the DRO hero digits on the dashboard.
- **Widgets** (`ui/ui.hpp`): Widget, Container, Label, Button, ProgressBar, BarGraph, ScreenManager.
- **TSV builder** (`ui/ui_builder_tsv.cpp`): page/widget DSL, bindings, actions, focus, form editing, include templates for shared page fragments, `scale=<1..4>` integer pixel scaling for labels and buttons, and `active_if=<bind>:<int>` conditional highlight on buttons (brightens bg and draws a white outline when the named binder value matches).
- **Display backend** (`ui/display.cpp`): virtio-gpu integration.
- **UI bootstrap** (`ui/splash.cpp`): TSV-only boot/render path with `render_ui_once()` for deterministic page capture.

### Operator surface

The current `devices/embedded_ui.tsv` ships eight operator pages plus a
shared navigation template:

- **`bottom_nav`** — 6-tile persistent nav (Jog · MDI · Program · Offsets · Macros · Service) included on every real page via `include page=bottom_nav` so tile positions stay identical across pages.
- **`dashboard`** — home / hero page. 4-axis DRO with scale=3 commanded digits and a scale=1 `act`/`dtg`/`homed` detail row per axis. Cycle cluster on the right: START / HOLD / RESET / HOME and an isolated oversized E-STOP. Status strip up top surfaces alarms, mode, active WCS, units, and a live runtime clock. Program and tool/work strips below the DRO.
- **`jog`** — manual motion surface. Full-size DRO hero, axis selector (X/Y/Z/A), jog-increment selector (0.001 / 0.01 / 0.1 / 1 mm — kernel counts: 1/10/100/1000), feed-override slider, JOG- / JOG+ action buttons, CYCLE / HOLD / RESET / HOME row, and a full axis-status panel at the bottom with per-axis cmd/act/dtg/homed plus load bar, spindle RPM, and mode.
- **`mdi`** — manual data input. Editable G-code line (`bind=mdi:input`), SUBMIT / CLEAR / ABORT, live status/depth/last/message strip bound to `cnc::mdi::Service`, and a history panel.
- **`program`** — program browser with preview image, PREV / NEXT / REBUILD / CYCLE buttons, and a GOTO LINE input that drives `cnc::interp::Runtime::restart_at_line` (see "Mid-program restart" below).
- **`offsets`** — tabular view of all 6 work coordinate systems (G54..G59 × X/Y/Z/A) and 8 tools (T1..T8 × length/radius/wear). WCS and tool rows carry `active_if=wcs:<N>` / `active_if=active_tool:<N>` so the currently selected slot is visually unmistakable.
- **`service`** — alarm recovery, setup load/save, diagnostics, plus a big PROBE CYCLES button that routes to the Probe page.
- **`macros`** — macro browser / runner.
- **`probe`** — Renishaw cycles: stylus qualify, Z surface, X/Y edge, bore XY, 3D pocket, reference sphere.

### MDI backend

`cnc/mdi.{hpp,cpp}` implements an in-RAM MDI service with a 32-slot
history ring and an 8-slot execution FIFO. `Service::submit(line)`
enqueues a line; `Service::tick()` (driven from `ui_builder::tick()`)
drains the head of the FIFO by writing a throwaway program named
`__mdi__` (line + `M30`) and calling `cnc::interp::Runtime::start(1)` on
channel 1. That reuses the full G-code parser and motion pipeline
without duplicating `tick_channel`. Status transitions Queued →
Running → Idle automatically when channel 1 completes, and faults
surface as Error with a message. History is not persisted.

Bind keys exposed: `mdi:input`, `mdi:last`, `mdi:status`, `mdi:message`,
`mdi:depth`. Actions: `mdi:submit`, `mdi:clear`, `mdi:abort`.

## Developing TSV-backed M-codes

Machine-side custom M-codes are now intended to be developed in TSV, not hardcoded into the interpreter.

The runtime is split into three layers:

- **Machine symbol registry**: typed named values such as `door_closed`, `drawer_open_cmd`, `probe_tripped`, `cycle_inhibit`, `axis_x_pos`
- **Macro runtime**: procedural sequences like "set output, wait for feedback, move axis, fault on timeout"
- **Ladder runtime**: continuous scan logic for interlocks and derived feedback

The relevant files are:

- [devices/embedded_macros.tsv](/home/djs/tinyOS/devices/embedded_macros.tsv:1)
- [devices/embedded_ladder.tsv](/home/djs/tinyOS/devices/embedded_ladder.tsv:1)
- [machine/machine_registry.cpp](/home/djs/tinyOS/machine/machine_registry.cpp:1)
- [automation/macro_runtime.cpp](/home/djs/tinyOS/automation/macro_runtime.cpp:1)
- [automation/ladder_runtime.cpp](/home/djs/tinyOS/automation/ladder_runtime.cpp:1)

### How M-codes dispatch

- `M300..M399` are reserved for TSV macros.
- The interpreter calls the macro runtime when it sees one of those codes.
- The interpreter then waits in `WaitingMacro` until the macro completes or faults.

That means a G-code program can do things like:

```gcode
M300
G4 P500
M303
```

Where `M300` might open a drawer and `M303` might run a tool-change macro.

### Step 1: Define symbols

Add writable or read-only symbols at the top of `devices/embedded_macros.tsv`:

```tsv
symbol	name=drawer_open_cmd	type=bool	initial=0	writable=1
symbol	name=drawer_open_fb	type=bool	initial=0	writable=1
symbol	name=tool_measure_request	type=bool	initial=0	writable=1
```

Guidelines:

- Use `bool` for commands, interlocks, sensors, and status bits.
- Use `int` for counts, raw analog values, positions, and timers.
- Use `float` only when you actually need engineering-unit math.
- Keep the core C++ machine structs fixed; expose new machine behavior through symbols instead of adding arbitrary struct fields.

Built-in symbols are registered in `machine/machine_registry.cpp`. That is the right place to bind a symbol to real motion state, fake IO, EtherCAT data, or a derived condition.

### Step 2: Define a macro

Macros live in `devices/embedded_macros.tsv` as one `macro` record followed by `step` records:

```tsv
macro	id=drawer_open	title=Drawer Open	mcode=300
step	type=set	symbol=drawer_close_cmd	value_type=bool	value=0
step	type=set	symbol=drawer_open_cmd	value_type=bool	value=1
step	type=wait	symbol=drawer_open_fb	value_type=bool	compare=eq	value=1	timeout_ms=1000
```

Current step types:

- `set`: write a symbol
- `wait`: poll a symbol until a compare passes or timeout expires
- `delay`: wait for `timeout_ms`
- `move`: issue `move_to` on `X/Y/Z/A`
- `alarm`: stop the macro and raise a macro fault

Current compare operators:

- `eq`, `ne`, `lt`, `le`, `gt`, `ge`

### Step 3: Add ladder interlocks or feedback

Ladder TSV runs every scan and is the right place for door interlocks, feedback synthesis, and inhibit logic:

```tsv
rung	id=door_interlock	type=bool	lhs=door_closed	compare=eq	rhs=0	out=cycle_inhibit	true=1	false=0
rung	id=drawer_open_feedback	type=bool	lhs=drawer_open_cmd	compare=eq	rhs=1	out=drawer_open_fb	true=1	false=0
```

Use ladder for:

- cycle-start inhibit when a door is open
- deriving `drawer_open_fb` / `drawer_closed_fb`
- resetting `tool_measure_done`
- future timed or latched machine interlocks

Use macros for:

- drawer open/close sequences
- tool change sequences
- tool measure/probe sequences
- clamp/unclamp flows
- any ordered "do this, wait for that" procedure

### Step 4: Bind symbols to real machine behavior

If a symbol should reflect real hardware or kernel state, bind it in `machine/machine_registry.cpp`.

Typical examples:

- map `di_*` / `do_*` to EtherCAT digital IO
- expose analog inputs like `ai_uni_0`
- expose live axis positions like `axis_x_pos`
- derive `cycle_allowed` from `cycle_inhibit`, `door_closed`, and `macro_busy`

This is the bridge between TSV automation and the real controller.

### Step 5: Rebuild and test

```bash
make TARGET=arm64
make TARGET=riscv64
```

Useful CLI commands:

```bash
macro_ls
macro_run 0
macro_stop
symbols
symbol_set drawer_open_fb 1
ladder_ls
```

Useful UI path:

- open the `macros` page in the TSV UI
- select a macro
- run or abort it
- watch active step and status

### Design guidance

- Prefer one macro per machine action, not one giant macro for everything.
- Put continuous safety logic in ladder, not in macros.
- Make macros wait on feedback symbols instead of assuming outputs succeeded.
- Use a timeout on every external action that depends on IO or operator response.
- Keep `M300..M399` stable and machine-specific; they become part of the shop-floor contract.

### Typical example: tool drawer open

1. Add writable command symbols like `drawer_open_cmd`.
2. Bind or derive feedback symbols like `drawer_open_fb`.
3. Add a ladder rung to inhibit cycle start if the door is open.
4. Add a macro on `M300` that sets the command and waits for feedback.
5. Call `M300` from G-code or trigger it from the `macros` UI page.

## 3D Rendering (GLES1 Software Renderer)

- **render/gles1.hpp/cpp**: Phong lighting, solid mesh rendering, matrix transforms
- **render/machine_model.hpp/cpp**: Box/cylinder mesh primitives
- **render/kinematic_model.hpp/cpp**: Kinematic chain with forward kinematics
- **render/obj_importer.hpp/cpp**: Wavefront OBJ loader (for complex meshes)

## SD Card Driver

- **hal/shared/sdcard.hpp/cpp**: SPI SD card interface
- QEMU usage: `-device sd-card,id=sd0,file=sdcard.img`

## Motion kernel

- 32 axes (`MAX_AXES=32`) and 2 motion channels (`mill`, `lathe`) in the default topology.
- Intended controller model: heterogeneous EtherCAT bus with servos, DIO, AIO, encoder/scales, couplers, and other process-data terminals, with only servo-class devices bound into motion axes.
- Current arm64 boot wiring still assumes axis `i` maps to `g_master_a.slave(i)` in discovery order; that is an implementation limitation, not the intended architecture.
- CiA-402 drive state machine (`DriveStateMachine`) with per-mode controlword masking (`ControlwordBuilder::sanitize`) and two-step servo-on handshake guard.
- **S-curve trajectory generator** with jerk limiting (integer maths, counts/s, counts/s², counts/s³). Per-axis tunable: `vmax_cps`, `accel_cps2`, `jerk_cps3`.
- Composable homing engine — direction, edge, trigger, backoff; torque-threshold trigger is first-class so ClearPath hardstop homing (methods −1 / −2) composes naturally. Post-homing auto-push of 0x607D software limits via `arm_post_homing_limits`.
- Stop matrix — five independent stop actions (QuickStop / Disable / Halt / FaultReaction / Abort) each mapping to a configurable StopKind; brake-engage latency honoured.
- Load-side feedback path (`Axis::LoadFeedback`): int64 raw position, rational scale/sign/offset, integer PI with anti-windup integrator, bounded trim cap, slew-limited decay on invalidity, cycle-counter watchdog. EL5042 Error bits propagate into the same `Axis::fault_latched` path as drive-side faults. Faults surface `0x603F` code + decoded string via `cia402::error_text` and `cia402::diag_text` (52-entry TextId table extracted from the Teknic ESI DiagMessages block).
- **Power skiving support**: differential speed CSV mode (`set_axis_velocity`), spindle indexer for tooth counting, feed-per-tooth calculation.

## Channels and synchronisation (mill-turn)

Two channels are live at boot: `ch0 mill` (axes 0–15) and `ch1 lathe` (axes 16–31). `Kernel::set_topology(specs, count)` rewrites the binding at runtime after validation (no axis in two channels, no empty channel).

Three kernel-level sync primitives, all integer maths, all same-cycle sample → command so sub-count phase-lock is achievable:

- **Barrier rendezvous** (`arrive_at_barrier`): named `barrier_token` (u16 at runtime), per-channel stable-cycle convergence check against `|commanded − actual| ≤ tolerance_counts`, atomic cross-channel release. `SyncTimeout` fault on `max_wait_cycles` expiry.
- **Electronic gearing** (`engage_gear` / `disengage_gear`): `follower.cmd = follower_base + (k_num * (leader.actual − leader_base)) / k_den`, same-cycle leader-sample / follower-commit. Ramped engage/disengage over `ramp_cycles`. Int64 intermediate math — no float drift, exact over long runs. Target use cases: threading, rigid-tap, cross-spindle hobbing, power-skiving.
- **Synchronous coordinated move** (`sync_move`): computes T_final from the slowest participating axis, rescales every other axis's effective vmax so they arrive together, and auto-posts a barrier across every channel that owns a participating axis.

Per-channel feedhold and override (feed / rapid / spindle permille) are isolated by default. Cross-channel fault propagation is **explicit**, only via an active sync primitive.

Gear-quality loop:

```
ClearPath -Exxx (51200 cpr, motor inner loop)
          └── CSP target = commanded_position + outer_trim
                                                ├── outer PI trimming from
Renishaw BiSS-C / EL5042 (sub-nm load feedback)     load_position_counts
```

Both samples latched at the same SYNC0 edge. **DIN 5 / DIN 6 gear quality is reachable** at the kernel level — with 0.0001° resolution (3,600,000 counts/rev), each count is 0.098 μm at the pitch circle, giving ~100 counts per tooth for a 90-tooth MOD 1.25 internal gear.

## Network drivers

- **VirtIO-Net**: default QEMU NIC path for EtherCAT frame transport and loopback/simulation.
- **e1000**: Intel NIC driver for passthrough or non-virtio experiments. Supports up to 3 NICs through `init_e1000_nic(idx, mmio_base, irq)`.
- The EtherCAT masters consume the HAL `NetworkDriverOps` interface directly. The old higher-level `net.cpp` socket layer is not the controller datapath and should be treated as legacy / non-authoritative until it is reconciled or removed.

## UI framework

The UI path is now a first-class controller subsystem, not just a splash screen:

- **Framebuffer** (`ui/fb.hpp/cpp`): 1080x1920 portrait RGBA framebuffer with text and primitive drawing. Text drawing has a `draw_text_scaled` path used for DRO hero digits.
- **Display backend** (`ui/display.cpp`, `hal/shared/virtio_gpu.*`): virtio-gpu scanout on QEMU arm64
- **Input** (`hal/shared/virtio_input.*`): touch, pointer, keyboard, and wheel events through virtio-input
- **TSV UI builder** (`ui/ui_builder_tsv.cpp`): pages, widgets, focus, bindings, actions, forms, `scale=` and `active_if=` attributes, `include page=<id>` page templates, and embedded macros
- **Operator API** (`ui/operator_api.cpp`): machine snapshot with commanded vs actual vs distance-to-go per axis, homed state, WCS / units / block / runtime / parts / selected axis / jog increment, plus helpers for MDI queue and mid-program restart
- **MDI service** (`cnc/mdi.cpp`): in-RAM line queue that drives the interpreter on channel 1
- **Software GLES1 renderer** (`render/`): machine scene, toolpath preview, kinematic model, and pod-aware overlays

The UI is currently pinned to `core0` by default. That is intentional for the operator path; only the EtherCAT/NIC placement is meant to be adjusted per deployment.

### Mid-program restart

`cnc::interp::Runtime::restart_at_line(channel, line_no)` reloads the
selected program on `channel`, then does a motion-free scan of lines
0..line_no-1 to reconstruct modal state (absolute / inch / plane /
motion-mode, feed, spindle direction+speed intent, active WCS, active
tool, tool-length compensation, coolant, axis targets). Axis motion,
homing, barriers, macros, dwells, M3/4/5 spindle commands, and physical
tool changes are deliberately skipped so the machine doesn't move during
the scan. The channel is left in `State::Ready` at `line_no`; the
operator calls Cycle Start separately to begin execution from the
reconstructed state.

This matches the two-pass "sequence-number search / block restart"
contract used by Fanuc, Siemens, and Haas — the controller recovers the
program's modal state but still expects the operator to verify that the
physical machine (tool in spindle, spindle direction/RPM, coolant) is in
a compatible state before resuming.

The Program page exposes this as a `GOTO LINE` input plus an `AT BLOCK`
readback bound to `block_current`. The operator types a line number,
presses Enter, then Cycle Start.

### TSV UI editor

`tools/ui_editor/index.html` is a single-file browser app (vanilla JS,
no build step, works offline) for editing `devices/embedded_ui.tsv`:

- 1080x1920 portrait canvas with snap-to-grid (10 px), zoom 25%-200%.
- Drag-drop palette (label / button / panel / container / slider / input / progress / image / graph).
- Right-side inspector with grouped `bind=` and `action=` dropdowns sourced directly from the kernel binder enum, plus dedicated editors for `scale=` and `active_if=<bind>:<int>`.
- File open / save via the File System Access API, with a blob-download fallback for Firefox/Safari.
- `localStorage` auto-save on every edit, 50-step undo/redo, keyboard nudge.
- Round-trip fidelity: opening `devices/embedded_ui.tsv` and saving back produces a byte-identical file (field order, comments, blank lines, and `title=`/`src=` aliases all preserved via a per-record `__order` / `__textKey` marker).

Bind catalogue, action catalogue, and keyboard shortcuts are documented
in `tools/ui_editor/README.md`.

## CLI reference

One-shot dashboard:
- `status` — masters, slaves, DB, channels, barriers, gears, motion stats, axis positions, fake_slave snapshot

Bus / SDO:
- `ec` / `ec_slaves` / `ec_hist` — master status / slave list / latency histograms
- `ec_scan [start=0x1001] [count=8]` — probe identity over a station range
- `ec_abort` — broadcast AL=Init (emergency bring-down)
- `ec_watchdog <timeout_ms> [station]` — program SM watchdog
- `ec_sdo_read <idx> <sub> [station]` — expedited SDO upload
- `ec_sdo_read_str <idx> <sub> [station]` — segmented SDO upload (strings, diag entries)
- `ec_identity [station]` — 0x1018 vendor/product/rev/serial
- `ec_encoder [station]` — 0x608F counts/rev
- `ec_dc [station]` — 0x1C32/0x1C33 DC telemetry
- `ec_safety [station]` — 0x605E / 0x6065 / 0x6066 / 0x231A / 0x2170
- `ec_probe` / `ec_probe_arm <mask>` — touch-probe capture
- `ec_tuning [station]` — drive tuning block (Kr/Kv/Kp/Ki/Kfv/…)
- `ec_home_methods [station]` — 0x60E3 supported methods
- `ec_home_params <method> <fast> <slow> <accel> <off> <torque> [station]` — push params
- `ec_home_run <method> <fast> <slow> <accel> <off> <torque> [timeout_ms] [station]` — full homing sequence
- `ec_sw_limits <neg> <pos> [station]` — 0x607D:1/:2 software position limits
- `ec_foldback <0|1> [torque_0.1%] [tc_ms] [station]` — Move Done Torque Foldback
- `ec_eeprom_save [station]` — 0x1010:01 ← 'save' (persist NVM)
- `ec_diag_dump [station]` — walk 0x10F3 DiagHistory + decode TextId
- `ec_allow_safeop` — manual override for the PreOp→SafeOp gate (bench tool)

Motion:
- `channels` — list channels + axis ownership
- `topology <name>=<ax,ax,...> [...]` — rewrite axis-to-channel binding
- `move <axis> <pos>` / `mpos` — single-axis move + per-axis dump
- `motion_enable <axis>` / `motion_disable <axis>` — explicit servo on/off
- `sync_move <mask> <ax> <tgt> [...]` — cross-channel coordinated move (9.6)
- `fault_reset <axis>` / `fault_inject` — operator recovery / fake-slave injection
- `feedhold <ch> <0|1>` / `override <ch> <feed|rapid|spindle> <permille>`
- `barriers` / `barrier_arrive <ch> <token> <parts>` — rendezvous primitive (9.4)
- `gears` / `gear_engage <leader> <follower> <ch> <k_num> <k_den> [ramp]` / `gear_disengage <follower>`
- `axis_spin <axis> <cps>` — simulated continuous-rotation leader for gear-harness testing
- `axis_load_configure <axis> <scale_num> <scale_den> <sign> …` — enable outer PI
- `axis_load_sample <axis> <raw> [error]` — push load sample
- `axis_load_calibrate <axis>` — capture offset so `load_following_error ≈ 0`
- `axis_sw_limits <axis> <neg> <pos>` — arm post-homing 0x607D push
- `axis_autodetect <axis>` — manual encoder-cpr re-probe
- `axis_tune <axis> [vmax] [accel] [jerk]` — per-axis motion tuning

Power skiving:
- `skiving_config <ts1> <teeth> <c1_cpr> [offset]` — configure tool spindle
- `skiving_engage <c1_rpm> <ts1_rpm> <z_axis> <teeth>` — differential speed skiving
- `feed_per_tooth <diameter> <teeth>` — compute feed/tooth
- `c1_index_sync <c1> <enable> <teeth>` — index pulse trigger
- `multipass <depth> <passes> [doc]` — rough/finish passes
- `din6 <cmd> <axis> [args]` — DIN6 axis indexing (index/sync/tooth/config)

Diagnostics / infra:
- `version` — kernel + build info (git hash + build stamp)
- `meminfo` — FixedMemoryPool usage + per-thread stack watermarks
- `save_cfg` — dump current operator state as replayable CLI commands
- `trace` / `stats` / `top` / `ttop` — scheduler / trace buffer
- `rt` / `rt_reset` / `budget` — RT jitter telemetry
- `devices` — TSV-loaded device DB inventory

## Target selection

- **`TARGET=arm64`** (default): primary reference target. Boots the full stack under `qemu-system-aarch64 -M virt -cpu max`, including UI, CLI, motion, EtherCAT masters, device DB loading, macros, ladder, toolpods, probe runtime, and display/input backends.
- **`TARGET=riscv64`**: secondary bring-up target. Boots under `qemu-system-riscv64 -M virt -bios none`, releases secondary harts through the spin-table, enters the scheduler on all harts, loads the shared UI / CLI / motion / macro / probe path, and can run the EtherCAT-oriented runtime. It still differs from arm64 in HAL maturity, especially IRQ routing and peripheral support.

## PCI passthrough (e1000)

For real EtherCAT hardware testing, use Intel e1000 NICs with PCI passthrough:
```bash
# QEMU with 3 e1000 NICs passed through
qemu-system-aarch64 -M virt -cpu max -smp 4 -m 128M \
  -device pci-hostdev,host=0000:03:00.0,id=e1000_0 \
  -device pci-hostdev,host=0000:04:00.0,id=e1000_1 \
  -device pci-hostdev,host=0000:05:00.0,id=e1000_2
```
Find PCI addresses with `lspci | grep -i ether`.

## Memory layout

Linker scripts `hal/arm64/linker.ld` / `hal/riscv64/linker.ld` place `.text` at `0x40000000` (arm64) or `0x80200000` (rv64) and `.data` adjacent. CI asserts the arm64 addresses via readelf; changing them breaks the ELF-integrity check.

## Licence

`SPDX-License-Identifier: MIT OR Apache-2.0` on every source file. Contributions under the same dual licence.
## Controller network contract

The default controller network split is now explicit:

- `eth0`: HMI and diagnostics
- `eth1`: EtherCAT master `ec_a`
- `eth2`: EtherCAT master `ec_b` or the in-kernel `fake_slave`

`eth0` does not run through the legacy `net.cpp` path. It now hosts a small HMI
network service implemented in [`hmi/hmi_service.cpp`](/home/djs/tinyOS/hmi/hmi_service.cpp:1):

- ARP
- IPv4
- UDP
- DHCP client with static fallback from `devices/embedded_hmi.tsv`
- the original raw L2 HMI protocol (`ethertype 0x88B5`) for direct bench tooling

The HMI control surface itself exposes:

- `discover`: identify the controller, NIC role, and build hash
- `status`: master state, slave counts, cycle counters, active TSV page, and current IP config
- `symbol_get`: read a machine-registry symbol by name
- `symbol_set`: write a writable machine-registry symbol by name

That keeps HMI control aligned with TSV macros, ladder, and the operator UI:
everything crosses the same machine-registry symbol layer.

The embedded HMI config TSV currently accepts:

```tsv
hmi	key=dhcp_enable	value=1
hmi	key=static_ip	value=10.0.2.15
hmi	key=netmask	value=255.255.255.0
hmi	key=gateway	value=10.0.2.2
hmi	key=udp_port	value=5000
hmi	key=dhcp_timeout_ms	value=3000
```

On QEMU `virt`, `eth0` uses the normal `-netdev user` uplink, so DHCP can hand
out an address that reaches the outside world. If DHCP does not answer in time,
the HMI service falls back to the static TSV configuration and continues serving
the UDP/raw HMI protocol locally.

For host-side probing in QEMU, `scripts/qemu_run.sh` now forwards the HMI UDP
port from the guest to the host:

- host UDP port: `QEMU_HMI_HOST_PORT` (default: same as the guest port)
- guest UDP port: `QEMU_HMI_GUEST_PORT` (default: `5000`)

That makes the guest HMI reachable on `127.0.0.1:$QEMU_HMI_HOST_PORT` during a
QEMU run, while `eth1`/`eth2` remain reserved for EtherCAT loopback.

Two helper scripts are included:

- `scripts/hmi_udp_probe.py` sends `discover` and `status` requests over UDP
- `scripts/test_hmi_udp.sh` boots QEMU, waits for CLI readiness, and validates a
  live UDP HMI round-trip from the host

## EtherCAT signal binding

`devices/embedded_signals.tsv` is no longer limited to coarse `ec.di` /
`ec.do` / `ec.ai_*` bindings. The registry now supports richer sources:

- `ec.statusword`
- `ec.controlword`
- `ec.position_actual`
- `ec.velocity_actual`
- `ec.error_code`
- `ec.drive_mode`
- `ec.digital_inputs`
- `ec.digital_outputs`
- `ec.tx_bit` / `ec.rx_bit`
- `ec.tx_s16` / `ec.tx_u16` / `ec.tx_s32` / `ec.tx_u32`
- `ec.rx_s16` / `ec.rx_u16` / `ec.rx_s32` / `ec.rx_u32`
- `ec.tx_pin` / `ec.rx_pin`

`ec.tx_pin` / `ec.rx_pin` resolve against the device DB's PDO `pin=` names, so
heterogeneous terminals such as scales, analog terminals, and servo-specific
PDOs can be exposed to macros, ladder, CLI, and HMI without hard-coding byte
offsets in multiple places.

Example:

```tsv
signal  name=x_scale_raw         source=ec.tx_pin          master=0 slave=4 pin=ch1_position_compact value_type=int
signal  name=x_servo_status      source=ec.statusword      master=0 slave=5 value_type=int
signal  name=main_spindle_pos    source=ec.position_actual master=1 slave=0 value_type=int
signal  name=x_servo_outputs     source=ec.digital_outputs master=0 slave=5 value_type=int writable=1
```

## Verification

The repo now includes dedicated smoke scripts for the controller topology:

- `bash scripts/test_multi_nic_smoke.sh`
- `bash scripts/test_signal_bindings.sh`

These exercise the three-NIC layout, boot-role logging, and the richer signal
binding surface through the QEMU harness.
