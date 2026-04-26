# `esi_to_blob.py` — ESI XML to miniOS device blob

Phase A host tool. Walks a directory of EtherCAT Slave Information (ESI)
XML files, extracts the bits the kernel needs to bring a slave from INIT
to OP, and emits a compact little-endian binary descriptor blob. The
kernel side (Phase B) is not yet wired — the blob just sits on disk as
an artefact other tooling can consume.

## Why

Today miniOS reads device descriptions from hand-written TSV files
(`devices/clearpath_ec.tsv`). That works for one or two slaves but does
not scale to a vendor catalogue that ships ~20 MB of ESI XML covering
hundreds of products. This tool ingests the vendor XML directly and
boils it down to ~178 KB of fixed-layout records the kernel can mmap and
index in O(1). TSV stays as the runtime override / dev-loop format;
the blob covers everything else.

## Usage

Standalone — Python 3 stdlib only, no `pip install` needed.

```
python3 tools/esi_to_blob.py \
    [--xml-dir ethercat/esi] \
    [--output build/esi_payload.bin] \
    [--vendor-filter "Teknic, Inc."] \
    [--max-devices N]
```

Arguments:

- `--xml-dir` — directory of `*.xml` ESI files. Default `ethercat/esi`.
- `--output` — destination for the binary blob. Default
  `build/esi_payload.bin`. Parent directory is created if missing.
- `--vendor-filter` — case-insensitive `<Vendor><Name>` match. May be
  repeated to allow several vendors. Omit to ingest everything.
- `--max-devices` — cap the emitted device count (testing / quick-look).

The tool prints a summary to stdout: device count, blob byte size, top
vendors by device count, device-type breakdown, and the list of XML
files that were skipped (with the reason).

## Tests

```
python3 tools/esi_to_blob_test.py
```

Covers integer-dialect parsing, UTF-8-safe name truncation, the Teknic
ClearPath round-trip (parse → encode → reparse → cross-check fields),
graceful handling of malformed XML, and a CLI smoke test that exercises
`--max-devices` end-to-end.

## Blob format

All fields are little-endian.

```
Header (16 bytes):
  [0..3]   Magic         "ESI\x01"
  [4..5]   Format ver    0x0001
  [6..7]   Device count  uint16
  [8..15]  Reserved      zero

Index table:
  For each device: uint32 byte offset of its device record, relative to
  the start of the file.

Device records (one per device, padded to 8-byte alignment):
  [0..3]    VID
  [4..7]    PID
  [8..11]   Revision
  [12..43]  Name (32 bytes, NUL-padded UTF-8, truncated on a UTF-8
            boundary so partial multi-byte codepoints are dropped)
  [44]      Device type enum (1 byte; see below)
  [45]      SM count (0..4)
  [46..47]  Reserved
  [48..51]  Mailbox request timeout ms
  [52..55]  Mailbox response timeout ms
  [56..57]  DC AssignActivate
  [58..63]  Reserved
  For each SM (sm_count entries, 16 bytes each):
    [0..1]   StartAddress
    [2..3]   DefaultSize
    [4]      ControlByte
    [5]      Enable
    [6..15]  Type string (10 bytes, NUL-padded; "MBoxOut", "MBoxIn",
             "Outputs", "Inputs", ...)
  PDO count (uint16, capped at 64)
  For each PDO entry (10 bytes):
    [0..1]   PDO index (e.g. 0x1A00)
    [2..3]   Entry index (e.g. 0x6041)
    [4]      SubIndex
    [5]      Bit length
    [6]      Direction (0=Rx, 1=Tx)
    [7]      SM number this PDO is assigned to
    [8..9]   Reserved
```

### Device type enum

| Value | Name             | Heuristic                                          |
|-------|------------------|----------------------------------------------------|
| 0     | Generic          | Default when nothing else matches                  |
| 1     | Servo            | PDO entries include both 0x6040 and 0x6041         |
| 2     | DigitalInput     | TxPDO-only, entries in 0x6000–0x60FF range         |
| 3     | DigitalOutput    | RxPDO-only, entries in 0x7000–0x70FF range         |
| 4     | AnalogInput      | Entry in 0x6400–0x64FF range                       |
| 5     | Encoder          | Name contains "encoder", or 0x6004 entry present   |
| 6     | Coupler          | No PDOs and no mailbox                             |
| 7     | MailboxGateway   | Mailbox declared but no PDOs                       |

Heuristics are best-effort. Misclassification is non-fatal — the kernel
will still see the SM/PDO/DC payload exactly as encoded.

### DC AssignActivate selection

The tool walks `<Dc><OpMode>` entries and picks the first one marked
`Default="1"`. If none, it falls back to the first `Fixed="1"`, then to
the first non-zero `AssignActivate`, then to whatever comes first.
Many ESIs (Teknic included) declare a `Synchron`/`FreeRun` entry with
`AssignActivate=0` next to the real DC entry; that ordering trick is
why the non-zero fallback matters.

## Robustness

- XML files that fail to parse are reported in the skipped list and the
  tool moves on.
- Devices missing a `ProductCode` attribute are silently dropped.
- Names are truncated at a UTF-8 boundary so the 32-byte field never
  ends mid-codepoint.
- PDO entries are capped at 64 per device. No real device declares more
  in its default mapping; if you hit the cap the device is interesting
  enough to inspect by hand.
- Encoded record length is padded to an 8-byte boundary so the index
  table always points at aligned addresses.

## Phase B — kernel integration

Shipped. The blob is linked into every kernel image (both arm64 and
rv64) via `devices/embedded_esi.S`, which `.incbin`s
`build/esi_payload.bin` into `.rodata`. The Makefile target rebuilds the
blob automatically when `tools/esi_to_blob.py` changes, so a fresh
checkout produces the same artefact CI sees. Vendor XML filenames
contain spaces (Make can't represent those in dependency lists), so
runtime XML edits need a manual `touch tools/esi_to_blob.py` to force
regeneration; the python tool itself is stdlib-only and will run on any
host that already builds the kernel.

Boot flow (`devices/embedded.cpp::load_all_embedded`):

1. The operator-curated TSV blobs are parsed first into `g_device_db`.
2. `DeviceDB::load_esi_blob(_binary_esi_payload_start, _end, ...)`
   walks the binary, validates magic / version / bounds, and inserts
   each device record into the database.
3. Any record whose `{VID, PID}` already exists in the table from the
   TSV pass is skipped — TSV is authoritative for hand-curated devices
   so its SDO inits and PDO remapping survive.
4. The boot log emits a single line:
   `[devices] ESI blob: <N> loaded, <M> skipped (TSV duplicate)`.

`MAX_DEVICES` in `devices/device_db.hpp` is set to 600 (571 vendor
devices + headroom for TSV entries on top). Bump it if a future vendor
catalog grows past that — the loader logs a clean overflow rather than
trampling memory.

The format is versioned (`Format ver` field in the header) so the
kernel can refuse a blob it does not understand. The kernel parser
mirrors the layout constants documented above; bumping the version is
required if any field shifts. Adding an entirely new section at the end
of a device record without changing existing offsets does not need a
bump as long as the kernel ignores trailing bytes.

## License

SPDX-License-Identifier: MIT OR Apache-2.0
