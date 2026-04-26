#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
"""Smoke + round-trip tests for esi_to_blob.

Run from the repo root:
    python3 tools/esi_to_blob_test.py
"""

from __future__ import annotations

import io
import os
import sys
import tempfile
import textwrap
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import esi_to_blob as eib  # noqa: E402

TEKNIC_XML = os.path.join(REPO, "ethercat", "esi", "Teknic_ClearPathEC_ESI.xml")


class ParserTests(unittest.TestCase):
    def test_int_dialect(self):
        self.assertEqual(eib._parse_int("#x300"), 0x300)
        self.assertEqual(eib._parse_int("0x10"), 16)
        self.assertEqual(eib._parse_int("256"), 256)
        self.assertIsNone(eib._parse_int(None))
        self.assertIsNone(eib._parse_int(""))
        self.assertIsNone(eib._parse_int("garbage"))

    def test_truncate_utf8_boundary(self):
        # Three-byte UTF-8 codepoint, truncated mid-sequence, must drop it.
        s = "x" + "€" * 5  # x + 5 euros (each 3 bytes)
        self.assertEqual(eib._truncate_utf8(s, 5), b"x\xe2\x82\xac")
        self.assertEqual(eib._truncate_utf8(s, 100), s.encode("utf-8"))


@unittest.skipUnless(os.path.exists(TEKNIC_XML), "Teknic ESI XML not present")
class TeknicTests(unittest.TestCase):
    def test_teknic_parse_and_roundtrip(self):
        devs = eib.parse_xml_file(TEKNIC_XML, None)
        # The Teknic ESI ships two device entries: Rev 2 (active) and Rev 1
        # (deprecated).
        self.assertEqual(len(devs), 2)
        rev2 = next(d for d in devs if d.revision == 2)
        self.assertEqual(rev2.vendor_id, 0x0C96)
        self.assertEqual(rev2.product_code, 0x1)
        self.assertEqual(rev2.dc_assign_activate, 0x0300)
        self.assertEqual(rev2.mailbox_req_ms, 2000)
        self.assertEqual(rev2.mailbox_rsp_ms, 20000)
        self.assertEqual(rev2.dev_type, eib.DEV_TYPE_SERVO)
        self.assertEqual(len(rev2.sms), 4)
        self.assertEqual(rev2.sms[0].start_address, 0x1000)
        self.assertEqual(rev2.sms[0].sm_type, "MBoxOut")
        self.assertEqual(rev2.sms[2].start_address, 0x1800)
        self.assertEqual(rev2.sms[3].start_address, 0x1C00)

        blob = eib.build_blob(devs)
        parsed = eib.read_blob_summary(blob)
        self.assertEqual(parsed["count"], 2)
        round_rev2 = next(d for d in parsed["devices"] if d["revision"] == 2)
        self.assertEqual(round_rev2["vid"], 0x0C96)
        self.assertEqual(round_rev2["pid"], 0x1)
        self.assertEqual(round_rev2["dc_assign_activate"], 0x0300)
        self.assertEqual(round_rev2["mailbox_req_ms"], 2000)
        self.assertEqual(round_rev2["mailbox_rsp_ms"], 20000)
        self.assertEqual(round_rev2["sms"][0]["start_address"], 0x1000)
        self.assertEqual(round_rev2["sms"][0]["type"], "MBoxOut")
        # Spot-check that controlword (0x6040) survived encoding.
        entry_indices = {p["entry_index"] for p in round_rev2["pdos"]}
        self.assertIn(0x6040, entry_indices)
        self.assertIn(0x6041, entry_indices)


class MalformedXMLTests(unittest.TestCase):
    def test_skips_bad_xml(self):
        with tempfile.TemporaryDirectory() as d:
            bad = os.path.join(d, "bad.xml")
            with open(bad, "w", encoding="utf-8") as f:
                f.write("<EtherCATInfo><Vendor><Id>1<")  # truncated
            with self.assertRaises(RuntimeError):
                eib.parse_xml_file(bad, None)

    def test_skips_devices_without_product_code(self):
        with tempfile.TemporaryDirectory() as d:
            xml = os.path.join(d, "noprod.xml")
            with open(xml, "w", encoding="utf-8") as f:
                f.write(textwrap.dedent("""\
                    <?xml version="1.0"?>
                    <EtherCATInfo>
                      <Vendor><Id>123</Id><Name>Acme</Name></Vendor>
                      <Descriptions><Devices>
                        <Device>
                          <Type>BogusNoProductCode</Type>
                        </Device>
                        <Device>
                          <Type ProductCode="#x42" RevisionNo="#x1">Real</Type>
                        </Device>
                      </Devices></Descriptions>
                    </EtherCATInfo>
                """))
            devs = eib.parse_xml_file(xml, None)
            self.assertEqual(len(devs), 1)
            self.assertEqual(devs[0].product_code, 0x42)
            self.assertEqual(devs[0].vendor_id, 123)


class CLITests(unittest.TestCase):
    def test_max_devices_and_vendor_filter(self):
        with tempfile.TemporaryDirectory() as out_dir:
            blob_path = os.path.join(out_dir, "blob.bin")
            rc = eib.main([
                "--xml-dir", os.path.join(REPO, "ethercat", "esi"),
                "--output", blob_path,
                "--max-devices", "3",
            ])
            self.assertEqual(rc, 0)
            with open(blob_path, "rb") as f:
                data = f.read()
            parsed = eib.read_blob_summary(data)
            self.assertEqual(parsed["count"], 3)


if __name__ == "__main__":
    # Quiet unittest banner output for the CLI summary stream.
    unittest.main(verbosity=2)
