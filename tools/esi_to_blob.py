#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""
esi_to_blob.py - miniOS ESI XML -> binary device descriptor blob.

Phase A host tool. Walks an ESI XML directory, extracts identity / Sync
Manager / mailbox / PDO / DC info per device, and emits a compact
little-endian binary the kernel will eventually consume in place of
the current TSV-only device DB. No third-party dependencies.

Blob format (little-endian throughout):

  Header (16 bytes):
    [0..3]   Magic         "ESI\\x01"
    [4..5]   Format ver    0x0001
    [6..7]   Device count
    [8..15]  Reserved      zero

  Index table:
    For each device: uint32 byte offset of device record (relative to
    start of the file).

  Device records (one per device, packed, padded to 8-byte alignment):
    [0..3]    VID
    [4..7]    PID
    [8..11]   Revision
    [12..43]  Name (32 bytes, NUL-padded UTF-8)
    [44]      Device type enum (1 byte)
    [45]      SM count (0..4)
    [46..47]  Reserved
    [48..51]  Mailbox request timeout ms
    [52..55]  Mailbox response timeout ms
    [56..57]  DC AssignActivate
    [58..63]  Reserved
    For each SM (16 bytes per entry, sm_count entries):
      [0..1]   StartAddress
      [2..3]   DefaultSize
      [4]      ControlByte
      [5]      Enable
      [6..15]  Type string (10 bytes, NUL-padded)
    PDO count (uint16), capped at PDO_ENTRY_CAP
    For each PDO entry (10 bytes):
      [0..1]   PDO index
      [2..3]   Entry index
      [4]      SubIndex
      [5]      Bit length
      [6]      Direction (0=Rx, 1=Tx)
      [7]      SM number
      [8..9]   Reserved
"""

from __future__ import annotations

import argparse
import os
import struct
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from dataclasses import dataclass, field
from typing import Iterable, Optional


MAGIC = b"ESI\x01"
FORMAT_VERSION = 0x0001
NAME_LEN = 32
SM_TYPE_LEN = 10
SM_RECORD_LEN = 16
PDO_RECORD_LEN = 10
PDO_ENTRY_CAP = 64
HEADER_LEN = 16
DEV_FIXED_LEN = 64  # bytes [0..63] before SM array

DEV_TYPE_GENERIC = 0
DEV_TYPE_SERVO = 1
DEV_TYPE_DIGITAL_INPUT = 2
DEV_TYPE_DIGITAL_OUTPUT = 3
DEV_TYPE_ANALOG_INPUT = 4
DEV_TYPE_ENCODER = 5
DEV_TYPE_COUPLER = 6
DEV_TYPE_MAILBOX_GATEWAY = 7

DEV_TYPE_NAMES = {
    DEV_TYPE_GENERIC: "Generic",
    DEV_TYPE_SERVO: "Servo",
    DEV_TYPE_DIGITAL_INPUT: "DigitalInput",
    DEV_TYPE_DIGITAL_OUTPUT: "DigitalOutput",
    DEV_TYPE_ANALOG_INPUT: "AnalogInput",
    DEV_TYPE_ENCODER: "Encoder",
    DEV_TYPE_COUPLER: "Coupler",
    DEV_TYPE_MAILBOX_GATEWAY: "MailboxGateway",
}


def _parse_int(value: Optional[str]) -> Optional[int]:
    """Parse the ESI integer dialect: '#x300', '0x300', '768', '768d'."""
    if value is None:
        return None
    s = value.strip()
    if not s:
        return None
    try:
        if s.startswith("#x") or s.startswith("#X"):
            return int(s[2:], 16)
        if s.startswith("0x") or s.startswith("0X"):
            return int(s[2:], 16)
        if s.endswith("d") or s.endswith("D"):
            return int(s[:-1], 10)
        return int(s, 10)
    except ValueError:
        return None


def _text(el: Optional[ET.Element]) -> str:
    if el is None or el.text is None:
        return ""
    return el.text.strip()


def _truncate_utf8(s: str, max_bytes: int) -> bytes:
    """Encode and truncate at a safe UTF-8 boundary."""
    raw = s.encode("utf-8", errors="replace")
    if len(raw) <= max_bytes:
        return raw
    # Walk back until the truncation point sits on a UTF-8 boundary.
    cut = max_bytes
    while cut > 0 and (raw[cut] & 0xC0) == 0x80:
        cut -= 1
    return raw[:cut]


@dataclass
class SMEntry:
    start_address: int = 0
    default_size: int = 0
    control_byte: int = 0
    enable: int = 0
    sm_type: str = ""


@dataclass
class PDOEntry:
    pdo_index: int = 0
    entry_index: int = 0
    sub_index: int = 0
    bit_len: int = 0
    direction: int = 0  # 0=Rx, 1=Tx
    sm_number: int = 0


@dataclass
class Device:
    vendor_id: int = 0
    product_code: int = 0
    revision: int = 0
    name: str = ""
    dev_type: int = DEV_TYPE_GENERIC
    mailbox_req_ms: int = 0
    mailbox_rsp_ms: int = 0
    dc_assign_activate: int = 0
    sms: list = field(default_factory=list)
    pdos: list = field(default_factory=list)
    source_file: str = ""
    vendor_name: str = ""


def _classify_device(pdos: Iterable[PDOEntry], has_mailbox: bool, name: str = "") -> int:
    """Heuristic classifier from CoE indices in the default PDO list."""
    pdos = list(pdos)
    indices = {p.entry_index for p in pdos}
    name_lc = name.lower()
    if 0x6040 in indices and 0x6041 in indices:
        return DEV_TYPE_SERVO
    if "encoder" in name_lc or 0x6004 in indices:
        return DEV_TYPE_ENCODER
    if not pdos and not has_mailbox:
        return DEV_TYPE_COUPLER
    if any(0x6400 <= i <= 0x64FF for i in indices):
        return DEV_TYPE_ANALOG_INPUT
    has_rx = any(p.direction == 0 for p in pdos)
    has_tx = any(p.direction == 1 for p in pdos)
    if has_tx and not has_rx and any(0x6000 <= i <= 0x60FF for i in indices):
        return DEV_TYPE_DIGITAL_INPUT
    if has_rx and not has_tx and any(0x7000 <= i <= 0x70FF for i in indices):
        return DEV_TYPE_DIGITAL_OUTPUT
    if has_mailbox and not pdos:
        return DEV_TYPE_MAILBOX_GATEWAY
    return DEV_TYPE_GENERIC


def _parse_sms(device_el: ET.Element) -> list:
    sms = []
    for sm in device_el.findall("Sm"):
        entry = SMEntry()
        entry.start_address = _parse_int(sm.get("StartAddress")) or 0
        entry.default_size = _parse_int(sm.get("DefaultSize")) or 0
        entry.control_byte = _parse_int(sm.get("ControlByte")) or 0
        entry.enable = _parse_int(sm.get("Enable")) or 0
        entry.sm_type = (sm.text or "").strip()
        sms.append(entry)
        if len(sms) >= 4:
            break
    return sms


def _parse_pdos(device_el: ET.Element) -> list:
    pdos: list = []
    for tag, direction in (("RxPdo", 0), ("TxPdo", 1)):
        for pdo in device_el.findall(tag):
            pdo_index = _parse_int(_text(pdo.find("Index"))) or 0
            sm_no = _parse_int(pdo.get("Sm")) or 0
            for entry in pdo.findall("Entry"):
                entry_index = _parse_int(_text(entry.find("Index"))) or 0
                if entry_index == 0:
                    # Padding entry; skip but keep PDO record valid.
                    continue
                p = PDOEntry()
                p.pdo_index = pdo_index
                p.entry_index = entry_index
                p.sub_index = _parse_int(_text(entry.find("SubIndex"))) or 0
                p.bit_len = _parse_int(_text(entry.find("BitLen"))) or 0
                p.direction = direction
                p.sm_number = sm_no
                pdos.append(p)
                if len(pdos) >= PDO_ENTRY_CAP:
                    return pdos
    return pdos


def _parse_dc_assign_activate(device_el: ET.Element) -> int:
    dc = device_el.find("Dc")
    if dc is None:
        return 0
    default_val = None
    fixed_val = None
    first_val = None
    for op in dc.findall("OpMode"):
        aa = _parse_int(_text(op.find("AssignActivate")))
        if aa is None:
            continue
        if first_val is None:
            first_val = aa
        if op.get("Default") == "1" and default_val is None:
            default_val = aa
        if op.get("Fixed") == "1" and fixed_val is None:
            fixed_val = aa
    if default_val is not None:
        return default_val
    if fixed_val is not None:
        return fixed_val
    # Many ESIs (Teknic included) declare a benign Synchron entry first
    # with AssignActivate=0 and the DC entry second; prefer the first
    # non-zero AssignActivate as a last resort.
    for op in dc.findall("OpMode"):
        aa = _parse_int(_text(op.find("AssignActivate")))
        if aa:
            return aa
    return first_val or 0


def _device_name(device_el: ET.Element) -> str:
    """Pick a human-readable device name (Type element text first)."""
    type_el = device_el.find("Type")
    if type_el is not None:
        text = (type_el.text or "").strip()
        if text:
            return text
    # Beckhoff style: Type element is empty, real label lives in Name.
    for name_el in device_el.findall("Name"):
        text = (name_el.text or "").strip()
        if text:
            return text
    return ""


def parse_xml_file(path: str, vendor_filter: Optional[set]) -> list:
    devices: list = []
    try:
        tree = ET.parse(path)
    except (ET.ParseError, OSError) as exc:
        raise RuntimeError(f"parse error: {exc}") from exc

    root = tree.getroot()
    vendor_el = root.find("Vendor")
    vendor_id = _parse_int(_text(vendor_el.find("Id"))) if vendor_el is not None else 0
    vendor_name = _text(vendor_el.find("Name")) if vendor_el is not None else ""
    if vendor_id is None:
        vendor_id = 0

    if vendor_filter and vendor_name and vendor_name.lower() not in vendor_filter:
        return devices

    for device_el in root.iter("Device"):
        type_el = device_el.find("Type")
        if type_el is None:
            continue
        product_code = _parse_int(type_el.get("ProductCode"))
        if product_code is None:
            continue
        revision = _parse_int(type_el.get("RevisionNo")) or 0

        name = _device_name(device_el)
        sms = _parse_sms(device_el)
        pdos = _parse_pdos(device_el)
        dc_aa = _parse_dc_assign_activate(device_el)

        mailbox_el = device_el.find("Info/Mailbox")
        if mailbox_el is None:
            mailbox_el = device_el.find("Mailbox")
        req_ms = 0
        rsp_ms = 0
        if mailbox_el is not None:
            tmo = mailbox_el.find("Timeout")
            if tmo is not None:
                req_ms = _parse_int(_text(tmo.find("RequestTimeout"))) or 0
                rsp_ms = _parse_int(_text(tmo.find("ResponseTimeout"))) or 0

        dev = Device(
            vendor_id=vendor_id,
            product_code=product_code,
            revision=revision,
            name=name,
            mailbox_req_ms=req_ms,
            mailbox_rsp_ms=rsp_ms,
            dc_assign_activate=dc_aa,
            sms=sms,
            pdos=pdos,
            source_file=os.path.basename(path),
            vendor_name=vendor_name,
        )
        dev.dev_type = _classify_device(
            pdos, has_mailbox=mailbox_el is not None, name=name
        )
        devices.append(dev)
    return devices


def _encode_device(dev: Device) -> bytes:
    name_bytes = _truncate_utf8(dev.name, NAME_LEN).ljust(NAME_LEN, b"\x00")
    sm_count = min(len(dev.sms), 4)
    pdo_count = min(len(dev.pdos), PDO_ENTRY_CAP)

    head = struct.pack(
        "<III32sBBHIIH6s",
        dev.vendor_id & 0xFFFFFFFF,
        dev.product_code & 0xFFFFFFFF,
        dev.revision & 0xFFFFFFFF,
        name_bytes,
        dev.dev_type & 0xFF,
        sm_count & 0xFF,
        0,
        dev.mailbox_req_ms & 0xFFFFFFFF,
        dev.mailbox_rsp_ms & 0xFFFFFFFF,
        dev.dc_assign_activate & 0xFFFF,
        b"\x00" * 6,
    )
    assert len(head) == DEV_FIXED_LEN, f"head len {len(head)}"

    sm_chunks = []
    for sm in dev.sms[:sm_count]:
        sm_type = _truncate_utf8(sm.sm_type, SM_TYPE_LEN).ljust(SM_TYPE_LEN, b"\x00")
        sm_chunks.append(
            struct.pack(
                "<HHBB10s",
                sm.start_address & 0xFFFF,
                sm.default_size & 0xFFFF,
                sm.control_byte & 0xFF,
                sm.enable & 0xFF,
                sm_type,
            )
        )
    sm_blob = b"".join(sm_chunks)
    assert len(sm_blob) == sm_count * SM_RECORD_LEN

    pdo_blob = struct.pack("<H", pdo_count)
    for p in dev.pdos[:pdo_count]:
        pdo_blob += struct.pack(
            "<HHBBBBH",
            p.pdo_index & 0xFFFF,
            p.entry_index & 0xFFFF,
            p.sub_index & 0xFF,
            p.bit_len & 0xFF,
            p.direction & 0xFF,
            p.sm_number & 0xFF,
            0,
        )
    assert len(pdo_blob) == 2 + pdo_count * PDO_RECORD_LEN

    record = head + sm_blob + pdo_blob
    pad = (-len(record)) & 7
    return record + b"\x00" * pad


def build_blob(devices: list) -> bytes:
    encoded = [_encode_device(d) for d in devices]
    n = len(devices)
    header_and_index = HEADER_LEN + 4 * n
    offsets = []
    cursor = header_and_index
    for blob in encoded:
        offsets.append(cursor)
        cursor += len(blob)

    out = bytearray()
    out += MAGIC
    out += struct.pack("<HH", FORMAT_VERSION, n & 0xFFFF)
    out += b"\x00" * 8
    assert len(out) == HEADER_LEN
    for off in offsets:
        out += struct.pack("<I", off)
    for blob in encoded:
        out += blob
    return bytes(out)


def read_blob_summary(data: bytes) -> dict:
    """Tiny reverse-parser used by the round-trip test and `--verify`."""
    if len(data) < HEADER_LEN or data[:4] != MAGIC:
        raise ValueError("bad magic / truncated blob")
    version, count = struct.unpack_from("<HH", data, 4)
    offsets = list(struct.unpack_from(f"<{count}I", data, HEADER_LEN))
    devices = []
    for off in offsets:
        (vid, pid, rev) = struct.unpack_from("<III", data, off)
        name = data[off + 12 : off + 12 + NAME_LEN].split(b"\x00", 1)[0]
        dev_type, sm_count = struct.unpack_from("<BB", data, off + 44)
        req_ms, rsp_ms = struct.unpack_from("<II", data, off + 48)
        (dc_aa,) = struct.unpack_from("<H", data, off + 56)
        sm_off = off + DEV_FIXED_LEN
        sms = []
        for i in range(sm_count):
            base = sm_off + i * SM_RECORD_LEN
            sa, ds, cb, en = struct.unpack_from("<HHBB", data, base)
            stype = data[base + 6 : base + 16].split(b"\x00", 1)[0].decode(
                "utf-8", "replace"
            )
            sms.append(
                {
                    "start_address": sa,
                    "default_size": ds,
                    "control_byte": cb,
                    "enable": en,
                    "type": stype,
                }
            )
        pdo_off = sm_off + sm_count * SM_RECORD_LEN
        (pdo_count,) = struct.unpack_from("<H", data, pdo_off)
        pdos = []
        for i in range(pdo_count):
            base = pdo_off + 2 + i * PDO_RECORD_LEN
            pi, ei, si, bl, dr, sm_no, _ = struct.unpack_from("<HHBBBBH", data, base)
            pdos.append(
                {
                    "pdo_index": pi,
                    "entry_index": ei,
                    "sub_index": si,
                    "bit_len": bl,
                    "direction": dr,
                    "sm_number": sm_no,
                }
            )
        devices.append(
            {
                "vid": vid,
                "pid": pid,
                "revision": rev,
                "name": name.decode("utf-8", "replace"),
                "dev_type": dev_type,
                "mailbox_req_ms": req_ms,
                "mailbox_rsp_ms": rsp_ms,
                "dc_assign_activate": dc_aa,
                "sms": sms,
                "pdos": pdos,
            }
        )
    return {"version": version, "count": count, "devices": devices}


def _print_summary(devices: list, blob_size: int, skipped: list) -> None:
    print(f"Devices ingested: {len(devices)}")
    print(f"Blob size: {blob_size} bytes")
    by_vendor = Counter(d.vendor_name or f"VID 0x{d.vendor_id:08x}" for d in devices)
    print("Top vendors:")
    for vendor, count in by_vendor.most_common(5):
        print(f"  {count:5d}  {vendor}")
    by_type = Counter(DEV_TYPE_NAMES[d.dev_type] for d in devices)
    print("Device types:")
    for tname, count in by_type.most_common():
        print(f"  {count:5d}  {tname}")
    if skipped:
        print(f"Skipped XMLs ({len(skipped)}):")
        for path, reason in skipped:
            print(f"  {path}: {reason}")


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(description="ESI XML -> miniOS device blob")
    ap.add_argument("--xml-dir", default="ethercat/esi",
                    help="Directory holding *.xml ESI files")
    ap.add_argument("--output", default="build/esi_payload.bin",
                    help="Path for the emitted binary blob")
    ap.add_argument("--vendor-filter", action="append", default=None,
                    help="Only ingest devices whose Vendor/Name matches "
                         "(case-insensitive, may repeat)")
    ap.add_argument("--max-devices", type=int, default=None,
                    help="Cap emitted device count (testing aid)")
    args = ap.parse_args(argv)

    xml_dir = args.xml_dir
    if not os.path.isdir(xml_dir):
        print(f"error: --xml-dir {xml_dir!r} is not a directory", file=sys.stderr)
        return 2

    vendor_filter = (
        {v.lower() for v in args.vendor_filter} if args.vendor_filter else None
    )

    devices: list = []
    skipped: list = []

    files = sorted(
        os.path.join(xml_dir, f)
        for f in os.listdir(xml_dir)
        if f.lower().endswith(".xml")
    )
    for path in files:
        try:
            devs = parse_xml_file(path, vendor_filter)
        except RuntimeError as exc:
            skipped.append((os.path.basename(path), str(exc)))
            continue
        except Exception as exc:  # pragma: no cover - defensive
            skipped.append((os.path.basename(path), f"unhandled: {exc}"))
            continue
        devices.extend(devs)

    if args.max_devices is not None:
        devices = devices[: args.max_devices]

    blob = build_blob(devices)
    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "wb") as fh:
        fh.write(blob)

    _print_summary(devices, len(blob), skipped)
    return 0


if __name__ == "__main__":
    sys.exit(main())
