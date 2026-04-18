# Beckhoff EL5042 — 2-channel BiSS-C encoder interface

**Fingerprint**: VendorId=2 (Beckhoff), ProductCode=0x13B23052, RevisionNo≥0x00100000, Profile 5001 (measuring terminal), 2 channels. Type name `EL5042 2Ch. BiSS-C Encoder`. ESI source: `ethercat/esi/Beckhoff EL5xxx.xml` lines 106671–~113000.

**Role in miniOS**: secondary position sensor on the *load* side of the axis. The ClearPath-EC servo closes the motor loop internally (0x6064 = position actual, derived from the motor-shaft encoder + ballscrew ratio). The EL5042 sits next to the mechanical axis (linear glass scale, SICK/Heidenhain rotary BiSS-C absolute encoder, etc.) and reports the *true* load position. The motion controller runs an outer loop that compares load-position to commanded-position and trims the target-position stream flowing into the ClearPath via CSP. This is a textbook dual-loop / master-slave cascade and it catches everything the servo cannot see: backlash, ballscrew wind-up, thermal growth, coupling slip.

## Why it pairs cleanly with ClearPath-EC

- **Same DC sync scheme**. EL5042 `AssignActivate=0x0300` (DC-Sync OpMode) matches ClearPath's `0x0300`. Both terminals latch their measurement at the SYNC0 edge, so a pair of samples arriving in the same TxPDO frame correspond to the *same* instant within ~tens of nanoseconds. Without DC-Sync, the two positions would be offset by the cycle-time jitter (hundreds of µs) and the outer loop would chase that jitter instead of real error.
- **64-bit absolute position**. BiSS-C supports multiturn absolute encoders; EL5042 reports `0x6000:17 / 0x6010:17 Position : ULINT (64 bit)` in the Standard TxPDO so there is no wrap-around bookkeeping in the master. Compact TxPDO falls back to 32-bit UDINT if bandwidth matters.
- **Input-shift time under DC-Sync** is published in 0x1C33:3 like any EtherCAT device — the master can read it, subtract it from the host timestamp, and obtain the corrected sample instant.

## Mailbox / SM / FMMU / DC (from ESI)

- Mailbox timeouts: RequestTimeout = 100 ms, ResponseTimeout = 10000 ms (much tighter than the ClearPath 2000 / 20000 ms; keeps input-only terminals snappy).
- SM0 MBoxOut 0x1000 / 128 B / ctrl 0x26; SM1 MBoxIn 0x1080 / 128 B / ctrl 0x22; SM2 Outputs 0x1100 / 0 B / ctrl 0x04 (disabled — input-only terminal); SM3 Inputs 0x1180 / 20 B / ctrl 0x20.
- Two FMMUs: `Inputs`, `MBoxState`. No output FMMU.
- DC OpModes: `Synchron` (AssignActivate=0x0) and `DC` (AssignActivate=0x0300). Use DC for dual-loop; Synchron is acceptable for diagnostics only.

## PDO catalogue

All four TxPDOs are `Fixed="1"` — Beckhoff chose not to expose entry-level remapping here. Pick the variant at SM assignment time via 0x1C13, same pattern as ClearPath.

| PDO   | Name                          | Size | Contents                                                                 | Use                          |
| ----- | ----------------------------- | ---- | ------------------------------------------------------------------------ | ---------------------------- |
| 0x1A00 | FB Inputs Channel 1 (default) | 10 B | Status bitfield (16 b) + 64-bit position (0x6000:17)                     | Primary axis-load feedback   |
| 0x1A01 | FB Inputs Channel 2 (default) | 10 B | Status + 64-bit position (0x6010:17)                                     | Second axis on same terminal |
| 0x1A02 | Channel 1 Compact             |  6 B | Status + 32-bit position (0x6000:17)                                     | Bandwidth-constrained bus    |
| 0x1A03 | Channel 2 Compact             |  6 B | Status + 32-bit position (0x6010:17)                                     | Bandwidth-constrained bus    |

`0x1A00` and `0x1A02` mutually exclude each other (`<Exclude>#x1a02</Exclude>`); same for 0x1A01 / 0x1A03. The TwinCAT default in the ESI is *2 Ch. Standard* = 0x1A00 + 0x1A01 (both channels, 64-bit). We should default to that.

## Status bitfield (0x6000 / 0x6010 for channel 1 / 2)

Bits 0–15 of the TxPDO header, decoded per ESI:

- bit 1 `Warning` — encoder reports warning (typically dirty/weak signal but still usable).
- bit 2 `Error` — encoder reports hard fault; position word is not trustworthy.
- bit 3 `Ready` — encoder has completed its first valid read after power-up.
- bits 4–12 reserved.
- bit 13 `Diag` — terminal-level diag (mailbox has an entry).
- bit 14 `TxPDO State` — 1 = terminal failed to pack this cycle's PDO (stale data).
- bits 15–16 `Input cycle counter` (2 bits) — rolling counter, increments each valid read; the master can detect *stuck data* if this doesn't advance cycle-over-cycle.

Any of `Error | TxPDO State` must invalidate the sample in the outer loop; `Warning | Diag` should raise a telemetry flag but not gate motion.

## Configurable parameters (0x8000 = Ch1, 0x8010 = Ch2)

These are the SDO writes the master must push in PREOP before SafeOp. Defaults from the ESI `DefaultData` blocks:

| SDO          | Name                 | Default | Meaning                                                                     |
| ------------ | -------------------- | ------- | --------------------------------------------------------------------------- |
| 0x8000:0D    | Disable Status Bits  | 0       | Strips status fields from the PDO word — leave 0.                            |
| 0x8000:0E    | CRC Invert           | 0       | Some older BiSS encoders invert CRC. Leave 0 unless the scale datasheet says otherwise. |
| 0x8000:0F    | CRC Polynomial       | 0x43    | Standard BiSS-C polynomial. Only change if encoder vendor specifies.         |
| 0x8000:11    | Clock Frequency      | `1 MHz` | Enum: 10/5/3.33/2.5/2/1/0.5/0.25 MHz. Must be set per encoder — long BiSS cables or some scales top out at 1 MHz. |
| 0x8000:12    | Multiturn [Bit]      | varies  | Multiturn bit count (0–25) reported by encoder. Master sets this from encoder datasheet. |
| 0x8000:13    | Singleturn [Bit]     | varies  | Singleturn bit count (typ. 25 for 25-bit SSI, 22 for Heidenhain ECN). Same as above. |

`0x8000:01 Device Type = 0 (BiSS-C)` / `1 (SSI)` — leave 0 for BiSS-C. `0x8000:05 Supply Voltage = 5.0 V or 9.0 V` — set per encoder datasheet.

Multiturn + Singleturn bit counts are the *single most important thing to get right*. The terminal packs the raw BiSS frame into a 64-bit word; if bit counts are wrong the master reads a left-shifted / masked position that drifts relative to the mechanical truth. When the outer loop compares to ClearPath, a miscounted singleturn bit width shows up as "the axis and the motor disagree by exactly N encoder counts per rev" — easy to spot, easy to correct.

## Things the master has to do (beyond normal PDO wiring)

1. **Match clock rate to cable length and encoder.** 1 MHz is safe for ~10 m; 10 MHz needs short, well-shielded cable. Read this from the encoder datasheet and write 0x8000:11 before SafeOp.
2. **Honour the encoder's own fault bits** — BiSS-C drivers set their Warning / Error flags independent of the terminal's own ready bit. Both must be green before the outer loop engages.
3. **Tune SYNC0 input-shift**. 0x1C33:3 gives the terminal's own input-shift (latency between SYNC0 and the instant BiSS actually latches). The master should read it and apply it when timestamping samples.
4. **TxPDO-State watchdog**. If bit 14 stays high for more than ~2 cycles, the terminal is failing to read — escalate to a motion fault (it is safer to stop than to servo against stale load position).
5. **Inter-channel exclusion**. 0x1A00 and 0x1A02 are mutually exclusive via `<Exclude>`. Picking an illegal pair in the 0x1C13 assignment will fail at PREOP→SafeOp. Validate before writing.

## Outer-loop sketch (for PLAN.md Phase 7)

```
 per cycle (SYNC0):
   sample_motor     = RxPDO[ClearPath].position_actual   (0x6064, 32b, counts/rev from 0x608F)
   sample_load      = TxPDO[EL5042 ch1].position          (0x6000:17, 64b, counts from scale)
   sample_load_err  = TxPDO[EL5042 ch1].status & ERROR_MASK
   if sample_load_err: stop_axis(); log(); continue
   load_pos_in_cmd_units = apply_calibration(sample_load)    # scale bits -> axis units
   position_error        = commanded_pos - load_pos_in_cmd_units
   trim                  = outer_pid.step(position_error)
   next_target_position  = commanded_pos + trim            # what goes into ClearPath 0x607A
```

The trim must be small (bounded, slew-limited) — the inner loop is already stable; the outer loop is only compensating for the slow, structural errors the motor cannot see. Unbounded trim plus a noisy load encoder is how you saturate torque into the ballscrew.
