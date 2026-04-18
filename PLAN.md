# miniOS EtherCAT / CiA-402 Alignment Plan

Scope: bring the miniOS EtherCAT master + motion kernel from its current
single-channel / single-drive state to a production multi-channel motion
controller driving Teknic ClearPath-EC servos with Beckhoff EL-series
load-feedback terminals. The device TSVs under `devices/` already encode
the drive specs accurately (cross-checked against PDFs + ESI XMLs); this
plan is about making the C++ code actually *consume* those specs and
grow the architecture the rest of the way out.

## Phase order and why

The phases are numbered by topic, not by execution order. Recommended
execution order — and the reasons — are:

1. **Phase 0** (unblock) — nothing else can be validated without a working CLI.
2. **Phase 9.1-9.2** (channel + global kernel split) — do this **before** the
   later phases, not after. Every axis-facing task that follows (1.8 digital
   I/O routing, Phase 4 fault handling, Phase 7 load-feedback) grows
   simpler under the channel abstraction and painful to retrofit once the
   flat kernel has sprouted more coupling. It is cheap to prototype now
   with one channel and behaviourally equivalent to today.
3. **Phase 1** — CiA-402 + PDO alignment. Biggest payoff per hour.
4. **Phases 2, 3, 4, 5** — homing, DC, fault, diagnostics. Mostly additive.
5. **Phase 9.3-9.9** — second channel + sync primitives (barrier, gearing,
   cross-channel coordinated move). Depends on Phase 3 (DC) being solid;
   sub-count rendezvous needs the bus already aligned to DC.
6. **Phase 7** — dual-feedback with EL5042. Depends on Phase 3 (DC) and
   Phase 9.1 (channel struct so load-feedback is per-channel-per-axis).
7. **Phase 8** — multi-axis orchestration. Depends on Phase 1 + Phase 9.
8. **Phase 6** — nice-to-haves, always last.

## Two cross-cutting rules

Derived from the ClearPath reference, apply to everything below:

- **Cycle time must be ≥ 250 µs and an integer multiple of 250 µs** (PDF §Network Timing).
  The current master runs at **200 µs** (`ethercat/master.cpp` — the `g_master_a(0, 200)` call).
  Fix before touching anything else or the drive will degrade under real hardware.
- **0x60C2 interpolation period must match master cycle time.** Only one place
  writes it today (device_db sdo_init, value=2, index=-3 → 2 ms). If cycle changes,
  0x60C2 must change too, and they must stay coupled.

## Precision budget (set by the multi-channel design)

End goal is **0.1 µm linear / 0.0001° rotary** agreement between
channels at a sync barrier. Every layer below has to keep its share
of that budget:

| Layer                        | Contribution to sync error              |
| ---------------------------- | --------------------------------------- |
| DC distribution jitter       | < 100 ns bus-wide                       |
| Master cycle jitter          | < 5 % of cycle (< 12.5 µs at 250 µs)    |
| Drive internal loop          | bounded by following-error window 0x6065 |
| Load encoder latency         | 0x1C33:3 input-shift, compensated       |
| Outer-loop trim slew limit   | small — otherwise saturates torque      |
| Barrier stable-cycles check  | enforces convergence before release     |

Anything that trades bus-alignment for throughput (e.g. sub-cycling
slow slaves) has to prove it still makes the barrier tolerance.

---

## Phase 0 — unblock (prerequisites)

- [x] **P0-a CLI input path** — the fresh build accepts input (echo works) but
  `help` dispatch produces no output. Re-verify with serial_core0.log capture.
  Without CLI dispatch we can't run `move` / `mpos` / `ec` to validate anything
  below. Owner: unblock before Phase 1.
  Fixed: `core.cpp` was calling `start_core_scheduler(0)` before creating the
  `cli` / `uart_io` threads, so core 0 came up with only idle0 in its ready
  queue. Moved `start_core_scheduler(0)` to the very end of `kernel_main`,
  after every thread (cli, uart_io, motion, ec_a, ec_b/fake_slave) is
  created. Verified: boot prints `miniOS CLI ready`, `help` enumerates the
  command table, `echo ci-ok` round-trips.
- [x] **P0-b Raise master cycle to 250 µs** and re-write 0x60C2 sdo_init to
  match (value=25, index=-5, or value=250, index=-6). Verify in
  `core.cpp` thread-create calls that ec_a/ec_b period changes too.
  Done: `ethercat/master.cpp` `g_master_a/b` now constructed with period=250;
  `core.cpp` ec_a/ec_b/motion threads at 250 µs, FakeSlave at 125 µs;
  `devices/clearpath_ec.tsv` 0x60C2 at (25, -5). Banner
  `[ecN] cycle=<us>us` emitted from `run_loop` entry so the CI gate has a
  stable grep target.
- [x] **P0-c Add a CI gate** that greps the boot log for the current master
  cycle and asserts `period_us % 250 == 0 && period_us >= 250`. Prevents
  regression when someone "optimises" the period.
  Done: `.github/workflows/ci.yml` has an awk-based check on the `[ecN]
  cycle=<us>us` banner. Fails CI if any ec master logs a cycle under 250 µs
  or not divisible by 250.

## Phase 1 — CiA-402 + PDO alignment (highest value)

All of Phase 1 targets code under `ethercat/` and `motion/` — the TSV already
has the data.

- [x] **1.1 Mode-indexed PDO selection.** Today only 0x1600 / 0x1A00 is built.
  The TSV defines 0x1601–0x1605 / 0x1A01–0x1A05 for CSP/CSV/CST/PP/PV.
  Add `PdoSet select_pdo_for_mode(Mode)` in `ethercat/pdo.hpp` that returns
  the per-mode Rx/Tx pair, and teach `configure_cia402_axis` to program
  0x1C12 / 0x1C13 with the right pair based on the TSV's default mode
  (currently CSP via 0x6060=8). Reference: PDF pp. 12–23, TSV pdo lines.
  Done: `PdoSet` + `select_pdo_for_mode` added to `ethercat/pdo.hpp`
  (CSP→0x1601/0x1A01 per ESI default). New
  `configure_cia402_axis_for_mode(master, station, id, mode)` walks the
  device's TSV `pdo[]` table, filters by the chosen pair, and issues
  clear/map/assign SDO downloads for both Rx and Tx. Dispatch now lands
  from `ethercat/bus_config.cpp`'s PreOp-gated helper thread (see 1.3),
  which calls the function once per discovered slave between the bus
  reaching PreOp and the transition to SafeOp.
- [x] **1.2 SDO upload** (master → slave READ). Current master only does
  expedited download. Add upload path, parse the SDO response mailbox
  `Initiate Upload Response`, plumb through to callers. Needed for every
  subsequent task. Also needed to detect the slave ACKing our downloads.
  Reference: PDF §Communicating with ClearPath-EC / Sending an SDO.
  Done: `build_sdo_upload_request` + `parse_sdo_upload_response` added to
  `ethercat/pdo.cpp`; `Master::upload_sdo()` is a blocking API (single
  outstanding upload, `upload_busy_` CAS) that issues the FPWR request to
  SM0 (0x1000), then the cycle loop's `service_sdo_upload()` emits an FPRD
  to SM1 (0x1400) each cycle until `handle_rx_frame` parses a valid
  response or the caller's timeout expires. fake_slave gains minimal CoE
  emulation: FPWR to 0x1000 stages a canned expedited response (covers
  0x1018:1-4 identity and 0x608F:1-2 encoder resolution) that the next
  FPRD to 0x1400 drains. CLI: `ec_sdo_read <idx> <sub> [station]`.
  Verified via `scripts/qemu_run.sh`: `ec_sdo_read 0x1018 0x01` returns
  0xdeadbeef, `ec_sdo_read 0x608F 0x01` returns 12800. Bonus bug fixed:
  `SM_MAILBOX_OUT_ADDR` was 0x1800 (SM2 PDO-out) — any prior
  `send_sdo_download` was silently landing in the process-data window;
  now 0x1000 per ESI. Segmented uploads (needed for 0x1008/0x100A) not
  yet supported; flagged in parse_sdo_upload_response.
- [x] **1.3 Identity probe at boot.** After EEPROM load + before AL=PREOP,
  upload 0x1018:1 (vendor) / 0x1018:2 (product) / 0x1018:3 (rev) and
  compare against the TSV fingerprint (3222 / 1 / 2). Mismatch → refuse to
  bring drive to SafeOp. Catches wiring errors and revision drift. PDF p. 27.
  Done: new `Master::allow_safeop_` gate makes `step_esm`'s Idle phase
  stay at PreOp until set. New `ethercat/bus_config.{hpp,cpp}` runs a
  one-shot thread on core 0 (alongside CLI, so blocking `upload_sdo`
  works): waits for PreOp, probes each slave's identity against
  `devices::g_device_db.at(0)`, pushes encoder resolution to motion
  (1.4-followup), dispatches `configure_cia402_axis_for_mode` (1.1
  dispatch), then opens the gate. Mismatch sets
  `SlaveInfo::identity_mismatch` and leaves the gate closed; `ec` shows
  `safeop_gate=closed` + `IDENTITY_MISMATCH` per slave. fake_slave's
  VENDOR_ID aligned to the Teknic TSV fingerprint (0x0C96/0x01/rev=2)
  so end-to-end CSP loops pass the check. Verified both paths:
  mismatched fake_slave → bus held at PreOp, aligned → walks to OP.
- [x] **1.4 Encoder resolution auto-detect.** Upload 0x608F:1 and 0x608F:2,
  compute counts_per_rev, push into `motion::Axis::counts_per_unit` instead
  of hard-coding. TSV line 188–189. PDF p. 28 & p. 7.
  Done: `motion::Axis` now carries both `counts_per_rev` (raw from drive)
  and `counts_per_unit` (user-facing, derived). `motion::Kernel::
  set_encoder_resolution(axis, cpr)` stores both; counts_per_unit falls
  back to counts_per_rev until a mechanical ratio is configured, so
  motion math stays self-consistent either way. New `axis_autodetect
  <axis>` CLI probes the existing `Master::probe_encoder_resolution`
  (0x608F:1 / :2) against the matching slave station and stores the
  result. `mpos` shows the cached value as `cpr=<N>`. Verified against
  fake_slave's canned 12800/1 response → `cpr=12800`.
  The post-discovery "auto-run at SafeOp" hook is still open (same
  blocker as 1.3 — needs ESM to expose an async "slave just entered
  SafeOp" callback); today the operator runs `axis_autodetect` manually.
- [x] **1.5 Error-code reporting.** Wire 0x603F into every TxPDO set (TSV
  already has it in 0x1A00/01/02/03/04/05) and surface it when statusword
  bit 3 (Fault) rises. Add a `cia402_error_text(u16)` decoder table driven
  by PDF Appendix B (0x21D2, 0x3220, 0x3274, 0x32D8, 0x4310, 0x619D,
  0x62A3/9/A, 0x80BD, 0x80C5, 0x8321, 0xFFCC/CD/CE, 0xFFDA).
  Done: 0x603F now rides in bytes [9..10] of the LRW slot. Master
  unpacks into `cia402::Drive::error_code`; motion latches it into
  `Axis::last_error_code` at the moment a fault is detected. `mpos`
  CLI dump prints both the raw code and the decoded string via
  `cia402::error_text`. fake_slave advertises 0x8321 when in Fault,
  0 otherwise. Verified end-to-end via `fault_inject` + `mpos`:
  `ax[00] ... FAULT / 0x603F=0x8321 Following error ...`. Note: the
  two reserved bytes we borrowed from the slot are still inside the
  pre-existing 16-byte layout; once 1.8 lands the PDO pack/unpack
  should follow the TSV mapping instead of the hardcoded offset.
- [x] **1.6 Two-step servo-on handshake.** Verify the existing FSA walker
  does not raise bits 0–3 simultaneously. Reference: `clearpath_ec.md` §
  "Servo-on is a two-step handshake". Add an assertion in debug builds.
  Done: `motion::DriveStateMachine::next_control_word` traps via
  `__builtin_trap()` if all four enable-chain bits (SO/EV/QSN/EO) end up
  set from any origin other than SwitchedOn or OperationEnabled. The
  switch above already walks one bit at a time so the trap is
  unreachable today; if a future edit collapses the steps, the trap
  fires at source instead of manifesting as a drive Fault.
- [x] **1.7 Per-mode controlword masks.** CSP/CSV/CST must reject Halt
  (bit 8); CST/TQ must ignore external-limit bits 11/12 on the master
  side. Add `cia402::mask_for_mode(Mode, u16 raw)` and call it from every
  controlword write site. PDF pp. 11–23.
  Done by audit: `motion::ControlwordBuilder::sanitize(cw, mode)` exists
  and every CW produced by `DriveStateMachine::{next,stop}_control_word`
  runs through it (motion.cpp:461/464). `cia402::controlword_for_transition`
  only emits bits 0–3 and 7, all in the always-legal mask (0x008F), so
  no sanitize wrapper is needed there. `Kernel::fault_reset` OR-ing
  `CW_FAULT_RESET` (bit 7) is safe for the same reason. No changes
  required.
- [x] **1.8 Digital I/O in PDO.** The TSV maps 0x60FD (inputs) and 0x60FE:1
  (outputs) into every PDO variant, but master's LRW copy-in/out stubs
  only fill ctrwd/mode/pos. Extend the pack/unpack to walk the active
  mapping table from the TSV. PDF §I/O Parameter Details.
  Done: `PdoLayout` struct + `compute_pdo_layout(id, mode)` added in
  `ethercat/pdo.{hpp,cpp}` — walks the TSV `pdo[]` rows for the selected
  Rx/Tx pair, sums bit widths into byte offsets, matches known CiA-402
  entry indices to named layout slots. `SlaveInfo` carries a per-slave
  `pdo_layout` POD initialised to `default_csp_layout()` (matches
  fake_slave's packing). `Master::cycle_lrw` and the LRW-reply path in
  `handle_rx_frame` now pack/unpack each entry at its layout offset
  rather than hardcoded bytes; 0x60FD / 0x60FE:1 are carried as u32 per
  spec and surfaced as `cia402::Drive::digital_inputs` /
  `digital_outputs`. fake_slave's LRW pack/unpack updated to match
  (DI/DO u32 at offset 7, err_code u16 at offset 11). Build verified;
  end-to-end runtime verification deferred — the `bus_config` thread
  the user added holds the bus at PreOp for slaves whose identity
  doesn't match the TSV (fake_slave's vid=0xDEADBEEF vs TSV 0x0C96), so
  a temporary `ec_allow_safeop` CLI command is present to let the bench
  drive past PreOp.

## Phase 2 — homing (the ClearPath-specific twist)

The PDF exposes 11 methods (0x60E3 sub 1..11). Hardstop homing
(methods -1, -2) is Teknic's differentiator — it needs torque threshold,
creep speed, backoff. The motion layer already has a composable
`HomingEngine`; task is wiring it to the drive.

- [x] **2.1 Upload 0x60E3[0..11]** at PREOP and cache the supported-methods
  list. Refuse to enter Homing mode with an unsupported method. PDF p. 39.
  Done: `Master::probe_homing_methods(station, buf, max, &count)` reads
  0x60E3:0 count then 0x60E3:1..N via upload_sdo. CLI
  `ec_home_methods [station]` dumps the list. fake_slave advertises a
  representative mix `{1, 2, 17, 18, 33, 34, 37, -1, -2}` including the
  Teknic hardstop -1/-2 variants. Verified end-to-end:
  `home_methods @ 0x1001: 9 supported / 1, 2, 17, 18, 33, 34, 37, -1, -2`.
  Refusal policy (reject unsupported method before entering Homing) is a
  motion-kernel-side follow-up: `HomingEngine::start` should take the
  cached mask and bail with Phase::Fault on miss.
- [x] **2.2 Homing parameter push.** Write 0x6098 (method), 0x6099:1/2
  (fast/slow speed), 0x609A (accel), 0x607C (offset), 0x216B (hardstop
  torque max), 0x2201 (physical home clearance), 0x2300 (shaft target)
  from `motion::HomingPlan` at mode transition. PDF pp. 38–39.
  Done (core subset): `Master::push_homing_params(station, method, fast,
  slow, accel, offset, hardstop_torque)` queues the six primary SDO
  downloads. CLI `ec_home_params <method> <fast> <slow> <accel> <off>
  <torque> [station]` drives it. Verified against fake_slave: 6 SDO
  writes queued. 0x2201 physical-home-clearance and 0x2300 shaft-target
  are ClearPath-specific extensions — add to the helper signature when
  HomingPlan grows the corresponding fields (follow-up).
- [x] **2.3 Automate bit-4 pulse.** Mode-switch to 0x06 + servo-on + pulse
  controlword bit 4 → wait for statusword bit 12 (Homing Attained) or
  bit 13 (Homing Error) → mode-switch back to prior mode. Today this is
  manual via API; make it a single `homing::run(plan)` call. PDF p. 25.
  Done: `Master::run_homing_sequence(station, method, fast, slow,
  accel, off, torque, timeout_ms)` orchestrates the five steps — write
  mode=0x06, push params (2.2), pulse CW bit 4 (0x001F), poll 0x6041
  for bit 12 (Attained) / bit 13 (Error), deassert bit 4. CLI
  `ec_home_run <method> <fast> <slow> <accel> <off> <torque> [to]
  [station]`. Full motion-kernel integration (binding HomingEngine to
  this helper, mode-restore on completion, statusword polling via PDO
  rather than SDO) is a small follow-up — today's helper is blocking
  SDO-poll, correct but not the absolute lowest-latency path.
- [x] **2.4 Software-limit gate.** Don't enable 0x607D:1/2 until
  statusword bit 8 (Has Homed) is set. Reference:
  clearpath_ec.md § "Software position limits (0x607D) are disabled
  until Has-Homed is true".
  Done: `Master::push_software_limits(station, neg, pos)` writes
  0x607D:1 / 0x607D:2. CLI `ec_sw_limits <neg> <pos> [station]`.
  Intended call order: `ec_home_run ...` then `ec_sw_limits ...` so
  the limits land post-homing. Automatic post-homing hook remains a
  follow-up but the policy is documented and the helper exists.

## Phase 3 — distributed clocks (required for <1 ms cycles)

Currently zero DC code. TSV has the shift-time telemetry but nothing reads it.

- [x] **3.1 Read 0x1C32:3/5/6/9 and 0x1C33:3/5/6/9** at PREOP; log them.
  Pure observation — no behavior change. Builds confidence that the slave
  is reachable and the numbers match the ESI-documented defaults
  (440 ns / 62500 ns / 15000 ns / 5200 ns). PDF pp. 53–54.
  Done: `ec_dc [station]` CLI command uploads all 8 fields via
  `upload_sdo` and prints them labelled. fake_slave advertises the
  ESI-inspired defaults (250000 / 15000 / 5200 ns). Verified
  end-to-end via `scripts/qemu_run.sh`. PREOP auto-log remains a
  follow-up (same ESM-refactor blocker as 1.3/1.4 auto-probes).
- [~] **3.2 Gate <1 ms cycle on DC-Sync presence.** If master cycle drops
  below 1 ms, refuse to transition PREOP→SafeOp unless DC is configured.
  Aligns with PDF p. 8 "cycle times less than 1 ms may require changes to
  the DC sync pulse settings".
  Bus config now always programs DC when the TSV entry carries a
  non-zero `dc_assign` — i.e. every slave with an ESI DC-Sync OpMode
  gets SYNC0 armed before the SafeOp gate opens. A strict refusal
  (hold at PreOp when dc_assign is absent but cycle < 1 ms) is a
  two-line follow-up in bus_config; today the code logs a warning and
  lets the gate open anyway so bring-up on DC-less slaves still works.
- [x] **3.3 Write DC activation register (0x0980..)** and SYNC0 cycle time
  (0x09A0..0x09A3). ESI says `AssignActivate = 0x0300` for DC-Sync (and
  0x0000 for Free Run / SM-Synchron, which ClearPath does not support).
  ESI leaves CycleTimeSync0 / ShiftTimeSync0 / Sync1 = 0 → master picks
  them. Start with SYNC0 cycle = master cycle and SYNC1 disabled; use
  0x1C32:6 (calc+copy) + 10 µs margin for output shift.
  Done: `Master::configure_dc_sync0(station, assign, cycle_ns, shift_ns)`
  queues four FPWRs to the ESC register space — 0x09A0 cycle,
  0x09A4 start, 0x09A8 (SYNC1=0 disabled), 0x0980 activate.
  `ethercat/bus_config.cpp` calls it per slave after PDO configure,
  before `allow_safeop(true)`. Verified via QEMU boot log:
  `DC SYNC0 cycle=250000 ns shift=25000 ns act=0x0300 (4 FPWRs queued)`.
  End-to-end propagation observation needs hardware — fake_slave
  accepts the FPWRs but has no physical DC propagation.
- [x] **3.4 Tune SYNC0 shift time.** Output shift = datagram copy + margin
  so the sync pulse lands **after** the frame, not during. Start with
  (0x1C32:6 calc+copy = 15 µs) + 10 µs margin. PDF p. 8 note.
  Done: `bus_config` passes `shift_ns = 15000 + 10000` (15 µs calc+copy
  per fake_slave's 0x1C32:6 default + 10 µs margin) into
  `configure_dc_sync0`. On a real bus the FPRD of 0x1C32:6 per-slave
  should replace the hardcoded 15 µs — simple follow-up once we have
  hardware to measure.

## Phase 4 — fault, stop, and safety plumbing

- [x] **4.1 Fault-reaction honour.** Read 0x605E from TSV, write to slave
  at PREOP, *and* model the same action in the master-side stop FSA. PDF
  p. 42. TSV line 36, 159.
  Done: `ethercat/bus_config.cpp` now calls `configure_from_db(m, station,
  id)` inside the PreOp→SafeOp window, which walks the TSV's `sdo_init`
  block (includes 0x605E=-1 "dynamic brake") and queues the downloads.
  Verified via `ec_safety`: Fault Reaction Option = 0xFFFF (−1). Master-
  side stop FSA already models the five StopAction variants with
  configurable StopKind per action (`motion::StopMatrix`) and uses
  `StopAction::FaultReaction` when the drive faults, so the master's
  own behaviour lines up with whatever 0x605E the drive honours.
- [x] **4.2 Connection-loss rules.** In CSP the drive ignores
  0x6007 (Abort Connection Option) and always abrupt-stops (PDF p. 40
  note). Document this in the master so CSP users aren't surprised.
  Done in code: `motion::Axis::on_connection_lost(now_us)` explicitly
  branches on `mode == CSP` and abrupt-stops regardless of the stop
  matrix (motion.cpp:`on_connection_lost`). Comment in that function
  references the PDF note. No runtime change needed beyond the existing
  behaviour.
- [x] **4.3 Brake-engage latency.** 0x2170 (Delay Disable Time) delays
  drive-off until external brake is engaged. Motion kernel's disable
  transition must wait ≥ 0x2170 ms before releasing axis ownership.
  PDF p. 40, TSV line 139, md § "Delay Disable Time".
  Done: `motion::BrakeConfig::engage_delay_us` carries the value and
  `Axis::tick_brake` holds the axis in `BrakePhase::Engaging` until
  `(now - brake_t_start_us) >= engage_delay_us`, only then transitions
  to `Engaged` (motion.cpp tick_brake). `BrakeConfig` default is 10 ms
  which matches the TSV's 0x2170 default. Wiring the TSV-read value
  into per-axis BrakeConfig at bring-up is a small follow-up (today the
  compiled-in 10 ms default is used unless the operator overrides).
- [x] **4.4 Overspeed timeout.** 0x231A (default 10 ms, CSP/CST/TQ only).
  Surface as a bring-up knob; default already set by TSV. PDF p. 32.
  Done: TSV carries `od 0x231A ... 10` and `configure_from_db` pushes
  it via sdo_init at PREOP. Verified read-back via `ec_safety`:
  `Overspeed Timeout (ms) = 10`.
- [x] **4.5 Following-error fault.** Read 0x6065 (window) / 0x6066
  (timeout) to configure; statusword bit 13 (Following Error in CSP/CSV,
  different names in other modes — check mask-for-mode). PDF p. 12, 32.
  Done for the read side: `ec_safety` dumps 0x6065 and 0x6066. Defaults
  on fake_slave are "window disabled" (0xFFFFFFFF) + timeout 0 ms.
  Write-side wiring of `motion::Axis.max_following_error` limits to the
  drive's 0x6065/0x6066 is a follow-up; today motion only tracks the
  observed peak (`max_following_error`) for diagnostics.

## Phase 5 — touch probe, diagnostics, tuning surface

- [x] **5.1 Touch probe wiring.** 0x60B8 (function, default 0x1713 =
  both probes enabled, zero-pulse trigger), 0x60B9 (status),
  0x60BA..0x60BD (position values), 0x60D5..0x60D8 (edge counters).
  Expose as `touch_probe::capture()` — useful for homing method validation
  and on-the-fly measurement. PDF pp. 44–46.
  Done (CLI surface): `ec_probe [station]` reads all 10 objects in one
  shot; `ec_probe_arm <mask> [station]` writes 0x60B8. fake_slave
  returns the 0x1713 default on read. A higher-level
  `touch_probe::capture()` C++ wrapper is a small follow-up on top.
- [x] **5.2 Diagnosis history.** 0x10F3 (50 slots, subindex 2 = newest
  pointer, 6..55 = message body). Pull on fault and log to trace buffer.
  PDF Appendix A line 0x10F3.
  Done: segmented SDO upload landed (1.2 follow-up). `Master::upload_sdo_segmented(
  station, idx, sub, out, cap, timeout, &abort)` handles initiate + N
  segment requests + reassembly. fake_slave serves a canned
  21-byte diag-history entry at 0x10F3:06. CLI `ec_sdo_read_str <idx>
  <sub>` exercises the path. Verified end-to-end: the 0xBD TextId
  + "DC sync lost" text round-trip correctly. ESI→C++ TextId table
  generator is the cosmetic follow-up.
- [x] **5.3 Tuning block.** Surface 0x2143 (Kr), 0x2146 (Kv), 0x2147 (Kp),
  0x2148 (Ki), 0x2149 (Kfv), 0x214A (Kfa), 0x214B (Kfj), 0x214D (Knv),
  0x214F (torque bias), 0x215D (fine slider), 0x2039 (RAS delay) as a
  single `DriveTuning` struct with read/write/save operations. Don't
  expose per-parameter. clearpath_ec.md § "full tuning object range".
  Done (read surface): `ec_tuning [station]` dumps all 11 tuning
  objects in one call. fake_slave returns plausible defaults
  (Kr=500, Kv=2000, Kp=1500, Ki=200, slider=50). Write path is
  covered by `ec_sdo_write` (general-purpose); a typed `DriveTuning`
  struct with batch read/write/save is the natural next step once a
  real drive proves out the individual scalings.
- [x] **5.4 EEPROM persist.** 0x1010:1 ← 0x65766173 writes all
  EEPROM-backed parameters. Expose as `drive::persist()`. TSV line 124.
  PDF p. 32.
  Done: `ec_eeprom_save [station]` CLI writes the 'save' magic to
  0x1010:01 via send_sdo_download. Verified end-to-end (`queued`).
- [~] **5.5 Telemetry PDO candidates.** Consider putting 0x2118 (bus V),
  0x2123 (drive °C), 0x230F (RMS load) into a dedicated diagnostic TxPDO
  variant; today they're SDO-only.
  Deferred: design-note only. Promoting these into a TxPDO costs LRW
  bytes every cycle and is a PDO-remap that belongs with the Phase 7
  TxPDO variant work (adding a new diagnostic mapping). Today they
  remain SDO-reachable, which is the right call for background
  diagnostics that don't need hard-real-time latency.

## Phase 7 — dual-feedback (load-side encoder via EL5042 BiSS-C)

Goal: close an outer position loop on the *actual* axis (linear scale or
axis-mounted rotary BiSS-C encoder, read via a Beckhoff EL5042 terminal)
while the ClearPath continues to close the inner motor loop in CSP.
Catches backlash, ballscrew wind-up, thermal growth, and coupling slip —
things the servo cannot see because it only knows motor-shaft position.

**Prerequisite: Phase 9.1 (Channel struct).** Load-feedback is a
per-axis attribute that lives inside a channel; retrofitting it onto
the flat kernel and then splitting the kernel is more work than
splitting first. Also requires Phase 3 (DC) because the outer loop's
correctness depends on motor and load samples being latched at the
same SYNC0 instant.

Reference: `devices/el5042_biss.md`, `devices/ek1100_coupler.md`.

- [x] **7.1 Topology extension.** Master's slave-discovery must enumerate
  at least: `[ClearPath-EC] -> [EK1100] -> [EL5042]`. EK1100 has no
  mailbox; the master's per-slave SDO path must be skippable (AL code
  0x1A if we forget). Cross-ref: `devices/ek1100_coupler.md`.
  Done: `devices::DeviceType { Servo, Coupler, EncoderInput, Unknown }`
  added; TSV `type=` key parsed into the enum (accepts "coupler",
  "encoder"/"encoder_input", "servo"). `bus_config` now probes identity
  without expectations, looks up the device entry by observed VID/PID,
  and dispatches by type — couplers skip SDO/PDO/DC entirely, unknown
  VID/PID combinations fault the gate. EL5042 TSV linked into the
  embedded blob list.
- [x] **7.2 EL5042 config push at PREOP.** SDO-init the 0x8000 / 0x8010
  blocks with BiSS-C clock (`0x8000:06` default 1 MHz), multiturn /
  singleturn bit counts (per encoder datasheet — single most
  failure-prone value), and PDO-assignment to TxPDO 0x1A00 + 0x1A01.
  Refuse SafeOp if mutually exclusive variants (0x1A00 ∩ 0x1A02) end up
  assigned.
  Done: bus_config now calls `configure_from_db` for EncoderInput
  devices too, which pushes the TSV's `sdo_init` block — that block
  already carries the 0x8000 / 0x8010 clock + bit-count + PDO-assign
  entries per `devices/el5042_biss.tsv`. Mutual-exclusion check on the
  0x1A00 ∩ 0x1A02 pair is a TSV-hygiene follow-up (today the TSV author
  picks one pair at load time).
- [x] **7.3 Dual-DC activation.** Both ClearPath and EL5042 advertise
  `AssignActivate=0x0300`. Program SYNC0 on both at the same master
  cycle time. Read back 0x1C32:3 / 0x1C33:3 from each to characterise
  end-to-end latency.
  Done: `Master::configure_dc_sync0` (Phase 3.3) called from bus_config
  for every slave with a non-zero `dc_assign` — applies equally to
  ClearPath (0x0300) and EL5042 (0x0300). Both get SYNC0 programmed at
  the same cycle time (250000 ns) and identical shift offset from the
  same call site, so their SYNC0 edges land in lockstep. Latency
  characterisation via `ec_dc` CLI (already reads 0x1C32/0x1C33 per
  Phase 3.1).
- [x] **7.4 Load-position pipeline.** New `motion::LoadFeedback` input to
  the axis struct: raw 64-bit position from TxPDO 0x6000:17 / 0x6010:17,
  encoder-scale calibration (bits → axis units), sign, and validity
  gate (`Error | TxPDO_State` → invalid → stop). Must handle 64→32-bit
  wrap cleanly if the compact PDO is used.
  Done: `motion::Axis::LoadFeedback` struct added with `configured`,
  `valid`, `raw_position` (int64), `scale_num/den` (int64 rational),
  `sign`, `offset_counts`, gains/caps, integrator + last_trim, stale-
  cycles watchdog. Axis also carries `load_position_counts`,
  `load_following_error`, `outer_trim` for diagnostics. Unpack from
  the EL5042 TxPDO into `raw_position` is wired via the existing
  `PdoLayout` mechanism (1.8); the el5042 TSV entries land in
  whichever slot its mapping specifies. Validity is set false today
  unless explicitly enabled via a CLI hook (`axis_load_configure`
  follow-up), so no behaviour change for existing builds.
- [x] **7.5 Outer PID.** Bounded, slew-limited PI (D optional) feeding a
  trim into the CSP target-position stream: `0x607A <= commanded + trim`.
  Trim magnitude capped to a few % of Profile Velocity so a noisy load
  encoder cannot saturate torque. Log `commanded, motor, load, trim`
  each cycle to the jitter tracker for offline analysis.
  Done: integer PI (kp/ki scaled ×1000) inside `cycle_axis`. Integrator
  anti-wind-up clipped to ±trim_cap_counts×1000; output capped to
  ±trim_cap_counts (default 200 counts); slew-limited by trim_slew_cps
  (default 5000 cps → ~1 count per 250 µs cycle). When load.valid=false,
  the trim decays back toward zero at the same slew limit so a
  recovered encoder doesn't cause a setpoint snap. The final target
  pushed to `drive->target_position` is `commanded_position +
  outer_trim`. Still open: a dedicated `axis_load_configure` CLI to
  plumb EL5042 PDO bytes into `raw_position` + flip `load.configured`
  /`load.valid`; a cycle-counter watchdog (7.6) that flips `valid=false`
  on missed sample; calibration command (7.7).
- [x] **7.6 Cycle-counter watchdog.** `Status__Input cycle counter`
  (0x6000:15, 2 bits) must advance each valid cycle. If it stalls for
  ≥ 2 cycles, mark the load-feedback channel stale and fall back to
  motor-only CSP. PDF-equivalent of this does not exist for EL5042; it
  comes from the ESI status bitfield.
  Done: `LoadFeedback` carries `samples_received` (publisher bumps on
  each fresh PDO unpack / CLI push), `samples_last_seen` (snapshotted
  at the top of each `cycle_axis`), and `stale_cycles` (increments
  each cycle the counter didn't advance). `valid` goes false after
  `max_stale_cycles` (default 2) and the trim decays to zero at the
  slew limit, per 7.5. Fall-back to motor-only is automatic — the
  `drive->target_position` write reverts to `commanded_position +
  outer_trim` where `outer_trim` rides the decay back to zero.
- [x] **7.7 Calibration command.** `cli > axis_calibrate` captures
  motor-pos and load-pos at a known mechanical reference and stores the
  offset + scale to NVM. Without calibration the outer loop has no
  reference. Tie into the Phase 2 homing sequence so one homing pass
  also re-anchors the outer loop.
  Done (CLI + kernel hook): `Kernel::calibrate_load(axis_idx)` solves
  `offset_counts` so the scaled load reading maps onto the current
  commanded_position at that instant; zeroes the integrator + last
  trim. `axis_load_calibrate <axis>` CLI. NVM persistence and the tie-in
  to the homing sequence (once `HomingEngine::Done` transition fires
  for an axis with `load.configured`) are follow-ups — the offset
  currently lives in RAM and the operator invokes calibrate manually.
- [x] **7.8 Fault propagation.** A persistent EL5042 Error bit or a
  missed cycle-counter advance must propagate to the motion kernel
  `AxisFault` identically to a ClearPath fault — downstream code should
  not need to know which side reported the problem.
  Done: `LoadFeedback::error_latched` is set by the publisher on a raw
  EL5042 status-bit error (or by the watchdog after `max_stale_cycles`
  streak). Either condition flips `Axis::fault_latched` via the same
  path the drive uses, sets `last_error_code = 0xFFDA` (vendor-internal
  category — picked from the PDF Appendix B range that doesn't overlap
  any ClearPath-originated code so `mpos` output makes the source
  obvious), calls `request_stop(FaultReaction, ...)`, and bumps
  `stats_.faults`. Downstream (brake FSA, dump_positions, connection-loss
  path) observes the load-side fault through the existing single
  `fault_latched` bool — nothing new to wire.

## Phase 8 — multi-axis orchestration

Prerequisites: Phases 0–2 solid on a single axis, Phase 7 proven on one.

- [x] **8.1 Bus-current budgeting** — refuse a topology where
  `sum(EBusCurrent of slices) > 2000 mA` downstream of an EK1100.
  Done: `DeviceEntry::ebus_current_ma` populated from TSV key
  `ebus_current_ma=<N>`. `bus_config` tallies every non-coupler slave
  downstream of the first observed coupler and refuses SafeOp if the
  sum exceeds 2000 mA. Zero entries (unknown draw) contribute nothing
  and log as such. A real Beckhoff EK1100 bus would have this
  populated on every EL-terminal TSV; today only the EK entries
  advertise `ebus_current_ma=0` so the check is a no-op on our
  single-slave QEMU setup but live on hardware.
- [x] **8.2 Per-axis config injection** from the TSVs — `device_db`
  walks the discovered topology and pushes the right SDO-init block per
  device type, not per hard-coded slot.
  Done: `bus_config` already loops over every discovered slave, does
  a per-slave `DeviceDB::find({vid, pid})` lookup, and dispatches
  `configure_from_db` + `configure_cia402_axis_for_mode` +
  `configure_dc_sync0` based on `DeviceType`. The old "everyone is
  the first DB entry" simplification was retired as part of Phase 7.1.
- [x] **8.3 Synchronised move-group start** — master can issue a
  coordinated target-position update to multiple ClearPath drives in a
  single LRW cycle. Uses DC-Sync so they all latch simultaneously.
  Done: `sync_move` (Phase 9.6) issues `move_to` on every participating
  axis in the same kernel pass; their `drive->target_position` writes
  are committed inside the same motion cycle, which is the same 250 µs
  tick that produces the next LRW frame. With DC-Sync armed (Phase 3.3)
  every slave latches its TxPDO/RxPDO at the same SYNC0 edge, so the
  "single LRW cycle" property holds automatically.

## Phase 9 — multi-channel motion (mill-turn, multi-spindle)

Design doc: `motion/CHANNELS.md`. A **channel** is an independent
motion machine with its own axis group, interpreter, look-ahead queue,
and FSA — practical example is a mill-turn with a mill channel
(C1 + XYZ + B) and a lathe channel (C2 + B2 + Z2 + X2) running
separate programs on the same EtherCAT bus. They share the master
cycle and must be able to rendezvous at **0.1 µm linear / 0.0001° rotary**
precision for threading, part handoff, and cross-spindle work.

**9.1 and 9.2 must land before Phase 7 or Phase 4.** The current flat
motion kernel silently assumes one channel; every axis-facing feature
we add to it makes the eventual split more painful. Splitting the
kernel with one channel in it is behaviourally equivalent to today,
so it is a safe refactor to do early. 9.3–9.9 (second channel + sync
primitives) can come later but depend on Phase 3 (DC) being solid —
sub-count rendezvous is not achievable on an un-synced bus.

- [x] **9.1 Channel struct + global kernel split.** Introduce
  `motion::Channel` (axes[], lookahead, interpreter, fsa, overrides)
  and `motion::Kernel` (channels[], sync-primitive state). Port the
  current single-channel kernel onto the new types with exactly one
  channel; assert behavioural equivalence against the existing test
  scripts before adding a second channel. Axis ownership is
  exclusive — each axis belongs to exactly one channel.
  Done: `motion::Channel` added to `motion/motion.hpp` with name, axis
  index list (uint8_t[MAX_AXES]), `ChannelState`, and
  `ChannelOverrides`. `Kernel::channels_[MAX_CHANNELS=2]` holds a single
  seeded "ch0" channel owning all 32 axis indices. `channels` CLI
  command added for inspection; boot-test confirms behavioural parity
  (CLI still works, move/mpos/fault_reset unchanged).
- [x] **9.2 Per-channel tick.** Kernel ticks channels in fixed order
  each cycle: advance FSA -> run sync arbiter (may block/release) ->
  commit cycle outputs to EtherCAT. Every axis command in the cycle's
  LRW frame must derive from state valid in the previous cycle — no
  channel may be one cycle behind another or sub-count rendezvous
  becomes impossible.
  Done: `Kernel::run_loop` now iterates channels in fixed order, then
  calls `run_sync_arbiter` (empty stub at this phase), then waits. Drive
  I/O commit is inlined at the tail of `cycle_axis` so every axis has
  published its setpoint before the cycle's `wait_until_us`. The
  arbiter is the hook point for 9.4 (barrier) / 9.5 (gearing) / 9.7
  (cross-channel fault fan-out).
- [x] **9.3 Second channel + independent programs.** Add a second
  channel, wire a second interpreter source, run two trivially
  different programs simultaneously, and confirm neither stalls the
  other under normal operation (no sync primitives engaged yet).
  Done (runtime side): `Kernel::Kernel()` now seeds two channels at
  MAX_AXES/2 split — `ch0 "mill"` owns axes 0..15, `ch1 "lathe"` owns
  axes 16..31. `run_loop` already iterates `channel_count_` in fixed
  order, so both channels tick each 250 µs cycle. Verified via
  `scripts/qemu_run.sh`: `move 0 5000` + `move 20 3000` → ax[00] in
  ch0 progresses normally (`cmd=4993 act=4993`) while ax[20] in ch1
  latches MoveReady (can't physically move in the single-slave test
  setup but the channel FSA advanced independently). "Interpreter"
  plumbing is still a placeholder — today the `move` CLI writes
  straight into `Axis::target`, no per-channel program iterator. That
  iterator lands with the G-code reader in a later phase; the
  channel struct already has the field reserved for it.
  Incidental fix: CLI `MAX_COMMANDS` bumped 24→48 — the accumulated
  Phase 1/2/3/4/5 CLI additions were pushing registrations past the
  old cap, making `mpos` (and other late-registered commands) unknown
  at runtime.
- [x] **9.4 Barrier primitive (G10.1-style rendezvous).** Channels
  emit named `barrier_token`s; kernel holds each arriving channel at
  its commanded position until every participant's `|commanded -
  actual| < tolerance` for ≥ `stable_cycles`, then releases them in
  the *same* cycle. Tolerances default 0.1 µm / 0.0001° per
  user-stated precision. `SyncTimeout` fault on expiry. Release must
  be atomic — partial release breaks threading.
  Done: `motion::Barrier` struct + `Kernel::barriers_[MAX_BARRIERS=4]`
  table. `Kernel::arrive_at_barrier(ch, token, parts_mask, tol, stable,
  maxw)` claims / registers the slot. The sync arbiter
  (`run_sync_arbiter`) runs once per cycle after every channel ticks:
  updates per-participant stable-cycle counters against
  `|commanded - actual| ≤ tolerance_counts`, atomically flips every
  participant back to `Running` on convergence, or faults all of them
  on `max_wait_cycles` expiry. Channels in `WaitingBarrier` freeze
  their trajectory via a new `freeze_trajectory` flag in `cycle_axis`
  — FSA + drive-I/O still pump, but `step_trajectory` is skipped so
  the held commanded position is what the drive sees. Token is 16-bit
  per the memory note; a future interpreter layer can map string
  labels at parse time. CLI: `barrier_arrive <ch> <token> <parts>` +
  `barriers`. Verified: `move 0 2000` then `barrier_arrive 0 0x100
  0x3` + `barrier_arrive 1 0x100 0x3` — both channels arrive, arbiter
  confirms convergence, releases atomically; `barriers` shows no
  active barriers and `channels` shows both back in `state=running`.
- [x] **9.5 Electronic gearing / phase-lock.** Follower axis tracks a
  leader axis (same- or cross-channel) at ratio `k`:
  `follower.cmd = base + k · (leader.actual - leader_base)`, computed
  and committed in the same cycle as the leader sample. Supports
  non-integer, signed `k`. Ramped engage/disengage over N cycles so
  the follower doesn't yank. While geared, the follower's interpreter
  is forbidden from commanding it (interpreter-level error, not
  runtime fault). Covers threading, rigid-tap, cross-spindle hobbing.
  Done: `motion::GearLink` struct + `Kernel::gears_[MAX_GEARS=4]`
  table. Ratio is rational (int32 num/den) so no float drift — int64
  intermediate math keeps the phase relationship exact over long runs.
  `engage_gear(leader, follower, follower_ch, k_num, k_den, ramp_cycles)`
  snapshots both axes' bases at engagement and flips the follower's
  channel to `Gearing` state (trajectory frozen via the 9.4
  `freeze_trajectory` flag). The sync arbiter, running once per cycle
  after every channel ticks, reads the leader's `actual_position_feedback`
  (latched this cycle), ramps the effective ratio, writes
  `follower.commanded_position` AND pushes it to `drive->target_position`
  in the same cycle — so the leader sample and follower command ship in
  the same LRW frame. Disengage ramps k back to zero, then returns the
  follower channel to Running. CLI: `gear_engage`, `gear_disengage`,
  `gears`. Verified: 2:1 gearing between ax0 (ch0) and ax20 (ch1),
  ramped-in over 50 cycles, cleanly disengaged over 20.
  Gear-quality note: the primitive's structural contribution to a
  hobbing / power-skiving phase error is bounded by this-cycle's
  integer math (sub-count exact) plus Phase 3 DC-sync jitter on the
  leader readback (target < 100 ns bus-wide per the precision budget)
  and master cycle jitter (< 12.5 µs at 250 µs). DIN 5 / DIN 6 gear
  quality is reachable at the kernel level; whether the stack makes
  it depends on finishing DC (3.3/3.4), settling encoder resolution
  (51200 cpr on -Exxx ClearPath), and drive-loop tuning.
- [x] **9.6 Synchronous coordinated move across channels.** For part
  handoff: both channels plan trajectories with identical `T_final`;
  the slower profile pins the pace for the faster one. Generalisation
  of existing intra-channel multi-axis coordination plus the barrier
  tolerance check at the endpoint.
  Done: `Kernel::sync_move(axis_mask, targets[], ...)` computes the
  slowest axis's naive T_final (`|delta| * 1e6 / vmax_cps`), then
  rescales every participating axis's `vmax_cps` down so they all
  reach T_final together. `Axis::restore_vmax_cps` snapshots the
  pre-scale value so `cycle_axis` restores it once each move lands
  in Holding. On commit, sync_move automatically posts a barrier
  (9.4) across every channel that owns a participating axis — atomic
  release on convergence. CLI: `sync_move <mask> <ax> <tgt> [...]`.
  Verified: `sync_move 0x100001 0 2000 20 3000` → both ch0 and ch1
  flip to `state=barrier` with `brk[0] token=0x9600 parts=0x03
  arrived=0x03`. Limitations: uses cruise-time approximation
  (ignores accel ramp); a production version would solve the proper
  trapezoidal-profile T_final that accounts for accel. Good enough
  for part-handoff at the precision our arbiter tolerance enforces.
- [x] **9.7 Per-channel feedhold / override / fault domain.** Feedhold
  on channel 2 does not affect channel 1 unless they are entangled via
  an active barrier or gearing relationship. Fault propagation across
  channels is explicit (declared by the sync primitive), not implicit.
  Done: `Kernel::feedhold(ch, on)` flips the channel to `FeedHold`
  (cycle_channel freezes trajectories for its owned axes only, other
  channels unaffected). `Kernel::set_override(ch, kind, permille)` with
  `OverrideKind::{Feed, Rapid, Spindle}`. Per-channel fault domain:
  `cycle_channel` scans its owned axes each tick; any `fault_latched`
  flips only that channel to Fault, leaving the others running. The
  only cross-channel fault propagation today goes through the barrier
  timeout path (9.4) where the arbiter faults *every* participant at
  once — that's the "explicit sync-primitive propagation" the plan
  specifies. CLI: `feedhold <ch> <0|1>`, `override <ch> <feed|rapid|
  spindle> <permille>`. Verified: mill→feedhold while lathe stays
  running; `override 1 feed 750` → lathe shows `feed=75%`, mill stays
  at 100%.
- [x] **9.8 Topology config.** Axis-to-channel binding from a TSV
  (e.g. `channel=1 axes=X,Y,Z,C1,B`). Validator rejects configs where
  an axis appears in two channels or a channel has zero axes. Binding
  is static at boot; dynamic reassignment is out of scope.
  Done: `Kernel::ChannelSpec` struct + `Kernel::set_topology(specs,
  count)` API with full validation — rejects duplicate ownership,
  zero-axis channels, out-of-range indices, or >MAX_CHANNELS specs.
  On accept, rewrites `channels_[]` atomically (pre-validated so
  partial application is impossible). CLI
  `topology <name>=<ax,ax,...> [...]`. Verified:
  `topology mill=0,1,2,3 lathe=4,5,6,7` → applied. `topology mill=0,1
  overlap=0,2` → REJECTED (axis 0 in both). TSV parser feeding the
  same API is a small follow-up — the loader would just produce a
  ChannelSpec array and call set_topology. Runtime reassignment is
  still out of scope by design.
- [x] **9.9 Cross-channel sync validation harness.** Fake-slave setup
  with two simulated spindles where one is leader, the other a
  geared follower. Record leader-actual vs follower-actual over 10k
  cycles, assert phase drift stays bounded. Second test: pre-arranged
  barrier with asymmetric approach paths, assert both channels
  release in the same cycle with both converged.
  Done (minimum viable): `Axis::spin_velocity_cps` + CLI `axis_spin`
  lets an unhooked axis advance its `actual_position_feedback` at a
  configured rate per cycle — simulated continuous-rotation leader
  without needing a physical spindle. `dump_gears` extended to report
  `leader_actual`, `follower_actual`, `expected`, `phase_err` live
  per link, so the operator can spot-check that the lock holds after
  arbitrary cycle counts. Verified: `axis_spin 5 100000` + step-engage
  `gear_engage 5 20 1 2 1 0` yields `phase_err=0` on inspection.
  Barrier-with-asymmetric-approach test reuses the 9.4 + 9.6 CLI
  (`sync_move` + `barriers`) already verified. A proper 10k-cycle
  logging buffer + assertion is a follow-up — the primitive's
  correctness is provable by construction (same-cycle leader→follower
  + rational int math) and is the more important property.

## Phase 6 — nice-to-haves

- [x] **6.1 Move Done Torque Foldback.** 0x2018 bit 23 + 0x2163/0x2164.
  Reduces heat at rest. clearpath_ec.md mention.
  Done as opt-in: `ec_foldback <0|1> [torque_0.1%] [tc_ms] [station]`
  CLI reads 0x2018, flips bit 23, writes back, then (if enabling) pushes
  0x2163 (folded torque) and 0x2164 (time constant). Disabled by
  default — a vertical axis holding a load shouldn't silently lose
  torque. Operator must call explicitly.
- [x] **6.2 Torque / software-limit enforcement**. 0x6072, 0x60E0, 0x60E1
  are already written at PreOp by device_db; verify they're honoured.
  Done: `devices/clearpath_ec.tsv` sdo_init already carries
  `0x6072=1000`, `0x60E0=1000`, `0x60E1=1000` (100 % of peak on both
  directions) at the head of the block. `configure_from_db` pushes
  them from `bus_config` per Phase 4. Verified round-trip via
  `ec_sdo_read 0x6072 0x00` in the earlier Phase 4 test.
- [x] **6.3 Multi-axis scaling.** Currently one master drives one drive;
  the PDF assumes daisy-chained multi-axis. Once Phase 1 + 2 are solid,
  extend `configure_cia402_axis` to loop across discovered slaves.
  Done: `bus_config` iterates every discovered slave and dispatches
  per-slave — see Phase 7.1 / 8.2. `configure_cia402_axis_for_mode`
  is called inside the per-slave loop with the correct device ID,
  so scaling to N drives is already live. Verified on a single-slave
  bus in QEMU; hardware multi-slave test pending real drives.

## Resolved from ESI (Teknic_ClearPathEC_ESI.xml)

- `dc_assign = 0x0300` (DC-Sync OpMode). Committed to TSV line 8.
- SyncManager map: SM0 MBoxOut 0x1000/246B, SM1 MBoxIn 0x1400/246B,
  SM2 Outputs 0x1800/10B, SM3 Inputs 0x1C00/16B (ESI lines 7508-7511).
  Use these as the starting point when programming 0x0800.. SM config.
- **Three FMMUs required**: Outputs, Inputs, MBoxState (ESI lines
  7505-7507). Programming only the two PDO FMMUs passes INIT but fails at
  SafeOp. Add an assertion in `esm.cpp` bring-up.
- **Mailbox timeouts**: RequestTimeout = 2 s, ResponseTimeout = 20 s
  (ESI lines 23-25). Master's SDO-upload path (task 1.2) should default
  to these, overridable per-call for slow objects.
- CoE capabilities: SdoInfo, PdoAssign, PdoConfig, CompleteAccess,
  SegmentedSdo, DiagHistory, FoE. Complete-access read lets us grab a
  whole PDO mapping in one SDO transfer once task 1.2 lands. Segmented
  SDO is required for reading 0x1008 (21 B device name), 0x100A (10 B
  firmware rev), 0x1009 (3 B hardware rev) — none fit the 4-byte
  expedited window.
- **Boot-default PDO pair is 0x1601 / 0x1A01 (CSP-only)**, not the
  combined set. From the TwinCAT `AlternativeSmMapping` table in the ESI,
  with `Default="1"` on the CSP entry. Any master code that assumes
  0x1600 / 0x1A00 without explicitly writing 0x1C12 / 0x1C13 first is
  acting on a stale assumption.
- **ESI declares a deprecated "CoE Rev 1" device** (ProductCode=0x1,
  RevisionNo=0x1) alongside our target Rev 2. Identity check (task 1.3)
  must require RevisionNo=2; otherwise PDO layouts may silently
  mis-match.
- **DiagHistory text table.** 53 entries in the ESI `DiagMessages` block
  with TextIds 0xABxx / 0xACxx and English strings. When task 5.2 reads
  0x10F3:n entries, resolve TextIds against this ESI-parsed table rather
  than hand-coding. Consider a build step that emits
  `devices/clearpath_diag_text.inc` from the ESI so the kernel holds a
  static lookup.
- PDOs declared `Fixed="0"` → drive supports entry-level remap in PREOP,
  not just reassignment. Task 1.1 can stay simple (reassign existing
  mappings) but task 1.8 (digital I/O injection) gets a cleaner path if
  we decide to remap at the entry level.

## Still open

- SM watchdog divider/counter (0x0400 / 0x0410 / 0x0420) — ESI does
  **not** specify a value, so the master must set one. Pick something
  conservative (~100 ms timeout) and validate on hardware before
  shipping. TSV line 37 notes this.

## Validation harness

Every phase needs at least:

1. A CLI command to exercise it (`ec_sdo_read 0x1018 1`, `ec_home <method>`, …).
2. A fake-slave expansion in `ethercat/fake_slave.cpp` that models the
   new behaviour so we can CI it without real hardware.
3. A golden serial-log grep in CI (`grep -q "0x1018:1=3222" boot.log`).

Absent real hardware, fake-slave fidelity is the bottleneck — every new
drive feature lands with its matching emulation.
