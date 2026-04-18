# Beckhoff EK1100 / EK1101 — EtherCAT bus couplers

**Fingerprint**: VendorId=2 (Beckhoff). EK1100 ProductCode=0x044C2C52, EK1101 ProductCode=0x044D2C52. Multiple revisions in the ESI (0x00100000 through 0x00120000 for current parts); the hardware is forward/backward compatible within a major revision. ESI source: `ethercat/esi/Beckhoff EK11xx.xml`.

**Role in miniOS**: first node in any Beckhoff EL-terminal chain. The coupler converts 100BASE-TX (RJ45, the network side) to E-Bus (the short-range LVDS backplane that actually carries frames between the slice terminals: EL5042, EL1xxx digital inputs, EL2xxx digital outputs, etc.). Without the coupler, the EL terminals have no way to talk to the master. It is not optional — one EK1100 (or variant) per Beckhoff block.

## What the coupler does NOT do

- **No PDOs** on EK1100. It is fully transparent: frames go in X1, get repeated into the E-Bus going downstream, and come back to X2 on the return path. Zero process data.
- **No SDO, no mailbox.** The coupler responds to DL (EtherCAT register) reads so the master can enumerate it, but has no CoE service layer.
- **No DC propagation logic the master has to configure.** The coupler's E-Bus repeater is part of the DC chain automatically.

EK1101 is slightly different — it carries a configurable station ID (DIP switch or similar), and so has one tiny TxPDO (`0x1A00`, single UINT ID at `0x6000:01`, 16 bits, mapped to SM0) plus an `IdentificationAdo=#x1000` field. The master can read the switch setting to distinguish otherwise-identical slices. Still no RxPDO.

## What the master must do

1. **Topology-discover it.** Part of the BRD(0x0000) AL-status scan at boot; the coupler answers like any other slave. Use the ProductCode + RevisionNo to confirm.
2. **Map the port labels.** Three ports per variant: `X1 IN` (MII upstream from master), `EBUS` (downstream to first EL terminal), `X2 OUT` (MII further downstream — next coupler on the wire, or loopback if it's the tail). The master's auto-close-ring logic relies on these labels being read correctly from the ESI so it knows where redundant links should be closed.
3. **Budget the E-Bus current.** ESI `<Electrical><EBusCurrent>-2000</EBusCurrent>` means *the coupler supplies 2 A of E-Bus current to the slices downstream*. Each EL terminal has a positive `EBusCurrent` (EL5042 draws 120 mA). Sum of downstream draws must not exceed 2 A or the block browns out. Master should refuse a topology that violates this.
4. **Handle the no-mailbox case.** SDO init/upload/download must be skipped for EK1100 — the terminal will ABORT with AL_CODE 0x1A "Mailbox not yet in use" if the master queues any CoE traffic. Our device_db `sdo_init` section must be empty for EK1100.
5. **For EK1101, read `0x6000:01` in the first post-INIT cycle** to learn the station ID. Useful for identifying which of several physically identical blocks this one is.

## Position in miniOS's planned topology

```
miniOS master (virtio-net)
    |
    +-- ClearPath-EC #1          (CiA-402 servo, 0x1600-0x1A00 PDO family)
    |
    +-- ClearPath-EC #2          (second axis, if dual-drive)
    |
    +-- EK1100 / EK1101          (coupler -- no PDOs, just pass-through + E-Bus bridge)
    |     |
    |     +-- EL5042             (BiSS-C load encoder, TxPDO 0x1A00+0x1A01)
    |     |
    |     +-- EL1xxx             (limit / home switches as DI, optional)
    |     |
    |     +-- EL2xxx             (external brake or signalling DO, optional)
    |
    +-- (ring closes back to master)
```

The coupler is therefore the *mandatory* second-to-last stop before we can use any EL-series terminal, including the EL5042 dual-feedback encoder.

## Variants we are likely to see

| Part         | PID         | Delta vs EK1100                                       |
| ------------ | ----------- | ----------------------------------------------------- |
| EK1100       | 0x044C2C52  | Baseline coupler, 2 A E-Bus, RJ45 IN + RJ45 OUT.      |
| EK1100-0008  | 0x044C2C52 (rev …0008) | M8-connector variant for wet/vibration environments.  |
| EK1100-0030  | 0x044C2C52 (rev …001E) | 4 kV galvanic isolation between upstream MII and E-Bus. |
| EK1101       | 0x044D2C52  | Adds a 2-digit rotary station-ID switch + tiny 0x1A00 PDO reporting it. |

For our purposes, treat them as a single class: "coupler — just topology, no PDOs, no SDO." The extra ID PDO on EK1101 is optional telemetry.
